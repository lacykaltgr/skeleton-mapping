#include "SkeletonFinder/skeleton_finder_3D.h"
#include "SkeletonFinder/data_type_3D.h"
#include <iostream>
#include <vector>
#include <algorithm>
#include <unordered_map>
#include <unordered_set>

using namespace std;

// Helper function to perform DFS and mark components
void Graph::dfs(int node, vector<bool>& visited, vector<int>& component) {
    visited[node] = true;
    component.push_back(node);

    for (int neighbor : getNeighbors(node)) {
        if (!visited[neighbor]) {
            dfs(neighbor, visited, component);
        }
    }
}

// Function to get disconnected components without Boost
vector<vector<int>> SkeletonFinder::getDisconnectedComponents(vector<NodePtr> nodes) {
    int numNodes = nodes.size();
    Graph graph(numNodes);

    // Assign index to each node
    for (int i = 0; i < numNodes; ++i) {
        nodes[i]->index = i;
    }

    // Add edges to the graph based on node connections
    for (int i = 0; i < numNodes; ++i) {
        for (auto& neighbor : nodes[i]->connected_Node_ptr) {
            int neighborIndex = neighbor->index;
            graph.addEdge(i, neighborIndex);
        }
    }

    // Find all connected components using DFS
    vector<bool> visited(numNodes, false);
    vector<vector<int>> components;

    for (int i = 0; i < numNodes; ++i) {
        if (!visited[i]) {
            vector<int> component;
            graph.dfs(i, visited, component);
            components.push_back(component);
        }
    }

    // Sort components by size in descending order
    sort(components.begin(), components.end(), [](const vector<int>& a, const vector<int>& b) {
        return a.size() > b.size();
    });

    return components;
}
