#include "SkeletonFinder/skeleton_finder_3D.h"

#include <chrono>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;


// verifyFrontier: raycasting and checking if the frontier is valid
// processFrontier: initializing new node and adding it to the skeleton


bool SkeletonFinder::initSkeleton(Eigen::Vector3d start_pt, Eigen::Vector3d start_dir, const PointCloud::Ptr map_pcl) {
    // pre-computed values for further computations
    genSamplesOnUnitSphere();
    identifyBwFacets();

    Eigen::Vector3d target_pos = start_pt + 10 * start_dir.normalized();
    kdtreeForRawMap.setInputCloud(map_pcl);
    double valid_distance = rayFlyingValidLenInverse(start_pt, target_pos, 
                                                    _base_radius, _min_hit_ratio);
    cout << "First valid distance: " << valid_distance << endl;
    
    if (valid_distance > 4.0) {
        cout << "Initial location not found" << endl;
        return false;
    } 

    Eigen::Vector3d initial_location = start_pt + valid_distance * start_dir;
    cout << "Initial location: " << initial_location << endl;

    // initialize the start node
    NodePtr start = new Node(initial_location, NULL);
    // initialize vertices for the start node
    initBlackAndWhiteVertices(start);
    // add node to the skeleton
    recordNode(start);
    return true;
}

void SkeletonFinder::updateSkeleton(const PointCloud::Ptr update_map_pcl) {
    // check if the update map is empty
    if (update_map_pcl->points.size() == 0) {
        cout << "Update map pointcloud is empty!" << endl;
        return;
    }
    // point cloud map update
    kdtreeForRawMap.setInputCloud(update_map_pcl);
        
    cout << "Updating skeleton with " << NodeList.size() << " nodes..." << endl;
    // 1. update relevant node vertices based on new map
    // 2. update known frontiers based on new map
    // 3. update verify known frontiers based on new map
    // 4. if frontier is valid, process frontier 

    // loop through all nodes in the skeleton
    for (NodePtr node: NodeList) {
        // gate nodes are not updated
        if (node->isGate) {
            cout << "Gate node." << endl;
            continue;
        }

        // check for relevant unconfirmed frontiers
        // too far away, no need to update
        if (radiusSearchOnRawMap(node->coord) > _max_ray_length + _max_expansion_ray_length) {
            cout << "Node too very far away." << endl;
            continue;
        }
            
        for (FrontierPtr ftr: node->frontiers) {
            // if frontier is confirmed, no need to update
            if (ftr->confirmed)
                continue;
            // find latest valid point on the frontier
            Eigen::Vector3d ftr_valid = ftr->proj_center +
                         ftr->outwards_unit_normal * (2*_search_margin + ftr->max_valid_len);
            // if frontier is too far away, no need to update
            if (radiusSearchOnRawMap(ftr_valid) < _max_ray_length) {
                pending_frontiers.push_back(ftr);
                cout << "Relevant frontier found." << endl;
            }
                
        }

        // if node is confirmed, no need to update
        if (checkNodeConfirmed(node)) {
            cout << "Node confirmed." << endl;
            // TODO: post process confirmed nodes
            continue;
        }

        
        // node not relevant for map segment
        if (radiusSearchOnRawMap(node->coord) > _max_ray_length) {
            cout << "Node too far away." << endl;
            continue;
        }

        
        // update node vertices, facets, frontiers
        cout << "Updating node..." << endl;
        updateNode(node);
    }
    
    // loop through all frontiers
    while (!pending_frontiers.empty()) {
        FrontierPtr cur_frontier = pendingFrontiersPopFront();
        // verify if frontier is valid, mark unknown ones
        if (verifyFrontierVariations(cur_frontier)) {
            // process frontier if valid, creating new nodes
            // these also update the skeleton
            processFrontier(cur_frontier);
        }
    }

    cout << "Expansion finished. Number of nodes: " << NodeList.size() << endl;
}


void SkeletonFinder::initBlackAndWhiteVertices(NodePtr nodePtr) {
    vector<Eigen::Vector3d>::iterator it;
    for (it = sample_directions.begin(); it != sample_directions.end(); it++) {
        int index = nodePtr->sampling_directions.size();
        nodePtr->sampling_directions.push_back(vec3((*it)(0), (*it)(1), (*it)(2)));

        // TODO: ez nem biztos
        Eigen::Vector3d newVertex = nodePtr->coord;
        //newVertex += (*it) * _max_ray_length;
        VertexPtr new_unknown_vertex = new Vertex(newVertex, (*it), UNKNOWN);
        new_unknown_vertex->sampling_dire_index = index;
        nodePtr->unknown_vertices.push_back(new_unknown_vertex);
    }
}


void SkeletonFinder::updateBlackAndWhiteVertices(NodePtr node) {
    // vertices: white, black, unknwon
    vector<VertexPtr> confirmed_vertices;
    // loop through unknown vertices
    // check for clear, valid path from node to unknown vertex
    // on aerial: should distinguish between valid and safe path
    // on ground: valid path = safe path
    // to simplify: valid length cannot be greater than clear length
    for (VertexPtr v: node->unknown_vertices) {
        // check last validated length
        Eigen::Vector3d source = node->coord + v->dire_unit_sphere * v->max_valid_len;
        Eigen::Vector3d target = node->coord + v->dire_unit_sphere * _max_ray_length;
        pair<double, double> stats = checkPathClearAndValidLength(source, target);
        double clearence = stats.first;
        double valid = stats.second;

        // up to this point, the path is clear and valid
        v->max_valid_len = valid;

        cout << "Vertex clearence: " << clearence << ", valid: " << valid << endl;
        // path is not valid, stays unknown
        if (clearence > valid && clearence < _max_ray_length)
            continue;
        
        cout << "New vertex confirmed." << endl;
        // path is valid, where reachable, create new vertex
        v->confirmed = true;
        v->coord = source + v->dire_unit_sphere * clearence;
        confirmed_vertices.push_back(v);

        // cannot reach the end, make it a black vertex
        if (clearence < _max_ray_length) {
            // create a new black vertex
            v->type = BLACK;
            node->black_vertices.push_back(v);
        } else {
            // create a new white vertex
            v->type = WHITE;
            node->white_vertices.push_back(v);
        }
    }

    // remove confirmed vertices from unknown vertices
    for (VertexPtr v: confirmed_vertices) {
        node->unknown_vertices.erase(
            remove(node->unknown_vertices.begin(), node->unknown_vertices.end(), v), 
            node->unknown_vertices.end()
        );
    }
}


bool SkeletonFinder::checkNodeConfirmed(NodePtr node) {
    // check if all vertices are confirmed
    return node->unknown_vertices.empty();
}


bool SkeletonFinder::initNode(NodePtr curNodePtr) {
    // only need to call upon node creation
    initBlackAndWhiteVertices(curNodePtr);
    identifyFacets(curNodePtr);

    if (!checkWithinBbx(curNodePtr->coord))
        return false;

    if (!checkFloor(curNodePtr))
        return false;

    if (!curNodePtr->isGate)
        if (!updateNode(curNodePtr))
            return false;

    recordNode(curNodePtr);
    return true;
}


bool SkeletonFinder::updateNode(NodePtr curNodePtr) {
    updateBlackAndWhiteVertices(curNodePtr);

    if (curNodePtr->black_vertices.size() < 4) {
        return false;
    }

    // check if all vertices are confirmed
    if (checkNodeConfirmed(curNodePtr)) {
        centralizeNodePos(curNodePtr);

        // TODO: update nodes_pcl
        
        // remove node if it is too small
        double radius = getNodeRadius(curNodePtr);
        if (radius < _min_node_radius && curNodePtr->white_vertices.empty()) {
            // TODO: should we remove this node? or mark it rollbacked?
            return false;
        }

        // Absorb node inside node
        if (curNodePtr->seed_frontier != NULL) {
            bool diff_index = false;
            int index = curNodePtr->black_vertices.at(0)->collision_node_index;
            for (VertexPtr v : curNodePtr->black_vertices) {
                if (v->collision_node_index != index) {
                    diff_index = true;
                    break;
                }
            }
            if (!diff_index) {
                // TODO: should we remove this node? or mark it rollbacked?
                return false;
            }
        }

    }

    // add relevant frontiers from previous updates
    for (FrontierPtr ftr: curNodePtr->frontiers) {
        if (!ftr->confirmed) {

            if (radiusSearchOnRawMap(ftr->proj_center) < _max_ray_length) {
                pending_frontiers.push_back(ftr);
            }
        }

    }
    
    // TODO: only check for confirmed black vertices
    // TODO: how to handle flowback to unknown vertices?
    findFlowBack(curNodePtr);
    // identify new frontiers
    identifyFrontiers(curNodePtr);
    // add new facets to the point cloud
    addFacetsToPcl(curNodePtr);
    return true;
}

// 0: not confirmed
// 1: fully confirmed
// < 0: confirmation level for variations
// valid: frontier is valid in a specific direction
bool SkeletonFinder::verifyFrontierVariations(FrontierPtr cur_frontier) {
    // frontier is already confirmed
    if (cur_frontier->confirmed > 0) 
        return cur_frontier->valid;
    
    // confirmation level -1
    // default projection center is the center of the facet
    if (cur_frontier->confirmed > -1 && verifyFrontierExploration(cur_frontier)) {
        cur_frontier->confirmed = cur_frontier->valid ? 1 : -1;
        if (cur_frontier->valid) return true;
    } else return false;

    Eigen::Vector3d prev_proj_center = cur_frontier->proj_center;
    Eigen::Vector3d prev_normal = cur_frontier->outwards_unit_normal;

    // confirmation level -2
    // try projecting to the facet center
    cur_frontier->proj_center = cur_frontier->proj_facet->center;
    if (cur_frontier->confirmed > -2 && verifyFrontierExploration(cur_frontier)) {
        cur_frontier->confirmed = cur_frontier->valid ? 1 : -2;
        if (cur_frontier->valid) return true;
    } else return false;

    // confirmation level -3
    // try projecting to the facet normal
    cur_frontier->proj_center = prev_proj_center;
    cur_frontier->outwards_unit_normal = cur_frontier->proj_facet->outwards_unit_normal;
    if (cur_frontier->confirmed > -3 && verifyFrontierExploration(cur_frontier)) {
        cur_frontier->confirmed = cur_frontier->valid ? 1 : -3;
        if (cur_frontier->valid) return true;
    } else return false;

    // confirmation level -4
    // try projecting to the facet center using facet normal
    cur_frontier->proj_center = cur_frontier->proj_facet->center;
    if (cur_frontier->confirmed > -4 && verifyFrontierExploration(cur_frontier)) {
        cur_frontier->confirmed = cur_frontier->valid ? 1 : -4;
        if (cur_frontier->valid) return true;
    } else return false;

    // confirmation level -5
    // try projecting to the neighbor facets
    for (FacetPtr candidate_facet: cur_frontier->proj_facet->nbhd_facets) {
        // NOTE: added this line to the loop, was outside in the original code
        cur_frontier->outwards_unit_normal = prev_normal;
        cur_frontier->proj_center = candidate_facet->center;
        if (verifyFrontierExploration(cur_frontier))
            if (cur_frontier->valid) break;

        cur_frontier->outwards_unit_normal = candidate_facet->outwards_unit_normal;
        if (verifyFrontierExploration(cur_frontier))
            if (cur_frontier->valid) break;
    }
    bool valid = cur_frontier->valid;
    cur_frontier->confirmed = valid ? 1 : -5;
    return valid;
}

// returns true if the frontier is confirmed
bool SkeletonFinder::verifyFrontierExploration(FrontierPtr ftr_ptr) {
    Eigen::Vector3d center = ftr_ptr->proj_center;
    Eigen::Vector3d normal = ftr_ptr->outwards_unit_normal;
    

    Eigen::Vector3d source = center + normal * (2*_search_margin + ftr_ptr->max_valid_len);
    Eigen::Vector3d target = normal * _max_expansion_ray_length;

    // check if the frontier is too near to the obstacle
    pair<double, int> rs_result = radiusSearch(source);
    if (rs_result.first < _search_margin) {
        ftr_ptr->valid = false;
        return true;
    }

    pair<double, double> stats = checkPathClearAndValidLength(source, target);
    double clearence = stats.first;
    double valid = stats.second;

    // if valid < max_ray_length, only confirmed if clearence >= valid
    // if valid >= max_ray_length, confirmed if clearence >= max_ray_length
    
    // up to this point, the path is clear and valid
    ftr_ptr->max_valid_len = valid;
    // we dont know if the path is valid
    if (clearence - valid > 1e-3  && valid < _max_ray_length) 
        return false;
    // otherwise its confirmed
    ftr_ptr->confirmed = true;

    // no hit (free space), create a new node candidate
    if (clearence >= _max_ray_length) {
        Eigen::Vector3d node_pos = center + 
                    0.5 * _max_expansion_ray_length * normal;
        if (checkWithinBbx(node_pos))  {
            ftr_ptr->valid = true;
            ftr_ptr->next_node_pos = node_pos;
        }
    // hit, create a new node in the midpoint
    } else if (clearence > _frontier_creation_threshold) {
        Eigen::Vector3d hit_on_pcl = source + clearence * normal;
        ftr_ptr->valid = true;
        ftr_ptr->next_node_pos = (center + hit_on_pcl) / 2;;
    }
    return true;
}
