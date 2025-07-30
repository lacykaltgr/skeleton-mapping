

void findFlowBack_E(NodePtr node) {
  if (node->seed_frontier == NULL)
    return;

  int size = loop_candidate_frontiers.size();
  vector<vector<Eigen::Vector3d>> flow_back_frontier_log(size);
  vector<FrontierPtr> frontiers(size);
  vector<int> pending_frontier_index;

  // Count number of contact black vertices on each frontiers
  for (VertexPtr v : node->black_vertices) {

    // only process vertices collide with other polyhedrons
    if (v->collision_node_index < 0)
      continue;

    // hit_on_pcl is on seed_frontier which is already connected
    if (checkPtOnFrontier(node->seed_frontier, v->coord))
      continue;

    FrontierPtr loop_ftr = findFlowBackFrontier(v->coord, v->collision_node_index);

    // cannot find loop frontier
    if (loop_ftr == NULL)
      continue;

    flow_back_frontier_log.at(loop_ftr->index).push_back(v->coord);
    frontiers.at(loop_ftr->index) = loop_ftr;
  }

  // Sort decreasingly: flowback frontiers with more hits first
  for (int i = 0; i < size; i++) {
    if (flow_back_frontier_log.at(i).empty())
      continue;

    int n_vertices = (int) flow_back_frontier_log.at(i).size();
    int radius_vertices = getVerticesRadius(flow_back_frontier_log.at(i));
    if (n_vertices < _min_flowback_creation_threshold 
        && radius_vertices < _min_flowback_creation_radius_threshold) 
        continue

    if (pending_frontier_index.empty())
      pending_frontier_index.push_back(i);
    else {
      vector<int>::iterator it;
      for (it = pending_frontier_index.begin();
           it != pending_frontier_index.end(); it++) {
        if (flow_back_frontier_log.at(*it).size() <
            flow_back_frontier_log.at(i).size()) {
          pending_frontier_index.insert(it, i);
          break;
        }
        pending_frontier_index.push_back(i);
        break;
      }
    }
  }

  // Start flowback
  int size_pending_frontier = pending_frontier_index.size();
  vector<Eigen::Vector3d> connected_node_pos;
  if (node->seed_frontier->gate_node->connected_Node_ptr.empty()) {
    connected_node_pos.push_back(node->seed_frontier->master_node->coord);
  } else {
    for (NodePtr gate_con_node : node->seed_frontier->gate_node->connected_Node_ptr) {
      connected_node_pos.push_back(gate_con_node->coord);
    }
  }

  for (int i = 0; i < size_pending_frontier; i++) {
    int ind = pending_frontier_index.at(i);
    FrontierPtr flowback_frontier = frontiers.at(ind);

    // Unsafe loop: loop seg is not obstacle-free
    Eigen::Vector3d end_pt_on_frontier;
    if (flowback_frontier->gate_node == NULL)
      end_pt_on_frontier = flowback_frontier->proj_center;
    else
      end_pt_on_frontier = flowback_frontier->gate_node->coord;

    if (_exploration_mode) {
      if (!checkPathClear(node->coord, end_pt_on_frontier) ||
          !checkPathClear(flowback_frontier->master_node->coord, end_pt_on_frontier)) {
        // ROS_ERROR("Flowback seg not clear!");
        continue;
      }
    } else {
      if (!checkSegClear(node->coord, end_pt_on_frontier) ||
          !checkSegClear(flowback_frontier->master_node->coord, end_pt_on_frontier)) {
        // ROS_ERROR("Flowback seg not clear!");
        continue;
      }
    }

    // Bad loop: loop only contains 4 nodes
    if (_bad_loop) {
      bool bad_loop = false;
      for (Eigen::Vector3d pos : connected_node_pos) {
        if (flowback_frontier->gate_node == NULL ||
            flowback_frontier->gate_node->rollbacked) {
          if (isSamePos(pos, flowback_frontier->master_node->coord)) {
            bad_loop = true;
          }
        } else {
          for (NodePtr frontier_con_node :
               flowback_frontier->gate_node->connected_Node_ptr) {
            if (isSamePos(pos, frontier_con_node->coord)) {
              bad_loop = true;
              break;
            }
          }
        }
        if (bad_loop)
          break;
      }
      if (bad_loop) {
        continue;
      }
    }

    // Create flowback: new gate node if necessary
    if (flowback_frontier->gate_node == NULL) {
      NodePtr new_gate =
          new Node(flowback_frontier->proj_center, flowback_frontier, true);
      if (initNode(new_gate)) {
        flowback_frontier->gate_node = new_gate;
        flowback_frontier->master_node->connected_Node_ptr.push_back(new_gate);
        new_gate->connected_Node_ptr.push_back(flowback_frontier->master_node);
        node->connected_Node_ptr.push_back(new_gate);
        new_gate->connected_Node_ptr.push_back(node);
      } else
        continue;
    } else if (flowback_frontier->gate_node->rollbacked) {
      flowback_frontier->gate_node->rollbacked = false;
      recordNode(flowback_frontier->gate_node);
      flowback_frontier->master_node->connected_Node_ptr.push_back(
          flowback_frontier->gate_node);
      flowback_frontier->gate_node->connected_Node_ptr.push_back(
          flowback_frontier->master_node);
      node->connected_Node_ptr.push_back(flowback_frontier->gate_node);
      flowback_frontier->gate_node->connected_Node_ptr.push_back(node);
    } else {
      node->connected_Node_ptr.push_back(flowback_frontier->gate_node);
      flowback_frontier->gate_node->connected_Node_ptr.push_back(node);
    }

    for (NodePtr frontier_con_node :
         flowback_frontier->gate_node->connected_Node_ptr) {
      connected_node_pos.push_back(frontier_con_node->coord);
    }
  }
}