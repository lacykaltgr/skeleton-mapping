typedef pcd::PointCloud<pcl::PointXYZ> PointCloud;
typedef chrono::high_resolution_clock clock;
typedef chrono::duration<double> duration;


// verifyFrontier: raycasting and checking if the frontier is valid
// processFrontier: initializing new node and adding it to the skeleton


void SkeletonFinder::initSkeleton(Eigen::Vector3d start_pt) {
    // pre-computed values for further computations
    genSamplesOnUnitSphere();
    identifyBwFacets();

    // initialize the start node
    NodePtr start = new Node(start_pt, NULL);
    // initialize vertices for the start node
    initBlackAndWhiteVertices(start);
    // add node to the skeleton
    recordNode(curNodePtr);
}

void SkeletonFinder::updateSkeleton(const PointCloud::Ptr update_map_pcl) {
    // check if the update map is empty
    if (update_map_pcl->points.size() == 0) {
        cout << "Update map pointcloud is empty!" << endl;
        return;
    }
    // point cloud map update
    kdtreeForRawMap.setInputCloud(update_map_pcl);
        
    cout << "Updating skeleton..." << endl;
    // 1. update relevant node vertices based on new map
    // 2. update known frontiers based on new map
    // 3. update verify known frontiers based on new map
    // 4. if frontier is valid, process frontier 

    // loop through all nodes in the skeleton
    for (NodePtr node: NodeList) {
        // gate nodes are not updated
        if (node->isGate) 
            continue;

        // if node is confirmed, no need to update
        if (checkNodeConfirmed(node))
            // TODO: post process confirmed nodes
            continue
        
        // node not relevant for map segment
        if (radiusSearchOnRawMap(node->coord) > max_ray_length)
            continue;
        
        // update node vertices, facets, frontiers
        // also considers unknown frontiers from previous updates
        updateNode(node);
    }
    
    // loop through all frontiers added in updateNode
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

        newVertex += (*it) * _max_ray_length;
        VertexPtr new_unknown_vertex = new Vertex(newVertex, (*it), WHITE);
        new_unknown_vertex->sampling_dire_index = index;
        nodePtr->unknown_vertices.push_back(new_unknown_vertex);
    }
}

// TODO: nehéz
void SkeletonFinder::updateBlackAndWhiteVertices(NodePtr nodePtr) {
  // vector<VertexPtr> black;
  vector<Eigen::Vector3d>::iterator it;
  for (it = sample_directions.begin(); it != sample_directions.end(); it++) {
    int index = nodePtr->sampling_directions.size();
    nodePtr->sampling_directions.push_back(vec3((*it)(0), (*it)(1), (*it)(2)));

    pair<Vector3d, int> raycast_result =
        raycast(nodePtr->coord, *it, _max_ray_length);
    Eigen::Vector3d newVertex = raycast_result.first;
    if (raycast_result.second == -2) {
      newVertex += (*it) * _max_ray_length;
      VertexPtr new_white_vertex = new Vertex(newVertex, (*it), WHITE);
      new_white_vertex->sampling_dire_index = index;
      nodePtr->white_vertices.push_back(new_white_vertex);
    } else {
      VertexPtr new_black_vertex;
      new_black_vertex = new Vertex(newVertex, (*it), BLACK);
      new_black_vertex->collision_node_index = raycast_result.second;
      new_black_vertex->sampling_dire_index = index;
      new_black_vertex->dis_to_center =
          getDis(new_black_vertex->coord, nodePtr->coord);
      nodePtr->black_vertices.push_back(new_black_vertex);
    }
  }
}


bool SkeletonFinder::checkNodeConfirmed(NodePtr node) {
    // check if all vertices are confirmed
    for (VertexPtr v: node->black_vertices) {
        if (!v->confirmed) {
            return false;
        }
    }
    for (VertexPtr v: node->white_vertices) {
        if (!v->confirmed) {
            return false;
        }
    }
    return true;
}


void SkeletonFinder::initNode(NodePtr curNodePtr) {
    initBlackAndWhiteVertices(curNodePtr);
    // only need to call upon node creation
    identifyFacets(curNodePtr);
    updateNode(curNodePtr);
    recordNode(curNodePtr);
}


void SkeletonFinder::updateNode(NodePtr curNodePtr) {
    // Similar to initNode, refines the node
    // removed checkWithinBbx, checkFloor: these were already computed in initNode

    updateBlackAndWhiteVertices(curNodePtr);

    if (curNodePtr->black_vertices.size() < 4) {
        return;
    }

    // check if all vertices are confirmed
    if (checkNodeConfirmed(curNodePtr)) {
        centralizeNodePos(curNodePtr);

        // TODO: update nodes_pcl
        
        // remove node if it is too small
        double radius = getNodeRadius(curNodePtr);
        if (radius < _min_node_radius && curNodePtr->white_vertices.empty()) {
            // TODO: should we remove this node? or mark it rollbacked?
            return;
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
                return;
            }
        }

    }

    // add relevant frontiers from previous updates
    for (FrontierPtr ftr: curNodePtr->frontiers) {
        if (!ftr->confirmed)
            if (radiusSearchOnRawMap(ftr->proj_center) < _max_ray_length) {
                pending_frontiers.push_back(ftr);
            }
    }
    
    // TODO: only check for confirmed black vertices
    // TODO: how to handle flowback to unknown vertices?
    findFlowBack(curNodePtr);
    // identify new frontiers
    identifyFrontiers(curNodePtr);
    // add new facets to the point cloud
    addFacetsToPcl(curNodePtr);
}

bool SkeletonFinder::verifyFrontierVariations(FrontierPtr cur_frontier) {
    // default projection center is the center of the facet
    verifyFrontierExploration(cur_frontier);

    if (cur_frontier->valid) {
        return true;
    }

    Eigen::Vector3d prev_proj_center = cur_frontier->proj_center;
    Eigen::Vector3d prev_normal = cur_frontier->outwards_unit_normal;

    // try projecting to the facet center
    cur_frontier->proj_center = cur_frontier->proj_facet->center;
    verifyFrontierExploration(cur_frontier);

    if (cur_frontier->valid) {
        return true;
    }

    // try projecting to the facet normal
    cur_frontier->proj_center = prev_proj_center;
    cur_frontier->outwards_unit_normal = cur_frontier->proj_facet->outwards_unit_normal;
    verifyFrontierExploration(cur_frontier);

    if (cur_frontier->valid) {
        return true;
    }

    // try projecting to the facet center using facet normal
    cur_frontier->proj_center = cur_frontier->proj_facet->center;
    verifyFrontierExploration(cur_frontier);

    if (cur_frontier->valid) {
        return true;
    }

    cur_frontier->outwards_unit_normal = prev_normal;
    for (FacetPtr candidate_facet: cur_frontier->proj_facet->nbhd_facets) {
        // try projecting to the neighbor facet center
        cur_frontier->proj_center = candidate_facet->center;
        verifyFrontierExploration(cur_frontier);

        if (cur_frontier->valid) {
            return true;
        }
        else {
            cur_frontier->outwards_unit_normal = candidate_facet->outwards_unit_normal;
            verifyFrontierExploration(cur_frontier);

            if (cur_frontier->valid) {
                return true;
            }
        }
    }

    return false;
}


void SkeletonFinder::verifyFrontierExploration(FrontierPtr ftr_ptr) {
    verifyFrontier(ftr_ptr);

    Eigen::Vector3d raycast_start_pt =
        ftr_ptr->proj_center + 2.0 * ftr_ptr->outwards_unit_normal *
                                    _search_margin;


    // ground robots: RayWalking
    // aerials: check if difference in distance to map does not exceed threshold

    // check from max achieved length
    // if check OK: 
    //      mark valid, confirmed
    // else: 
    //      save maximum length and direction of the ray (between variations?)
    //      mark invalid

}


bool SkeletonFinder::processFrontier(FrontierPtr curFtrPtr) {
    // Gate node: midpoint of frontier
    NodePtr gate;
    if (curFtrPtr->gate_node == NULL) {
        gate = new Node(curFtrPtr->proj_center, curFtrPtr, true);
        curFtrPtr->gate_node = gate;
    } else {
        gate = curFtrPtr->gate_node;
    }

    bool floor = checkFloor(gate);
    // TODO: bbx?
    // bool bbx = checkWithinBbx(gate->coord);

    if (!floor || !bbx) {
        gate->rollbacked = true;
        return false;
    }

    // TODO: add logic for the unconfirmed nodes

    // Center node
    NodePtr new_node = new Node(curFtrPtr->next_node_pos, curFtrPtr);
    bool init_success = initNode(new_node);
    if (init_success) {
        initNode(gate);
        curFtrPtr->master_node->connected_Node_ptr.push_back(gate);
        gate->connected_Node_ptr.push_back(curFtrPtr->master_node);
        gate->connected_Node_ptr.push_back(new_node);
        new_node->connected_Node_ptr.push_back(gate);
    } else {
        new_node->rollbacked = true;
        if (gate->connected_Node_ptr.empty()) {
            gate->rollbacked = true;
            curFtrPtr->valid = false;
            for (auto f : curFtrPtr->facets) {
                f->valid = false;
            }
        }
    }

    return init_success;
}