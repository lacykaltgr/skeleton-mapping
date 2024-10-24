#include <iostream>
#include <vector>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/connected_components.hpp>
#include <algorithm>

using namespace std;
using namespace boost;

// Define the graph structure using Boost's adjacency list
typedef adjacency_list<vecS, vecS, undirectedS> Graph;

// Function to get disconnected components using Boost
vector<vector<int>> getDisconnectedComponents(const vector<NodePtr>& nodes) {
    int numNodes = nodes.size();
    Graph graph(numNodes);

    // Add edges to the graph based on the node connections
    for (int i = 0; i < numNodes; ++i) {
        for (auto& neighbor : nodes[i]->connected_Node_ptr) {
            int neighborIndex = neighbor->index;
            add_edge(i, neighborIndex, graph);
        }
    }

    // Vector to store the component index for each node
    vector<int> component(numNodes);
    int numComponents = connected_components(graph, &component[0]);

    // Group nodes by component
    vector<vector<int>> components(numComponents);
    for (int i = 0; i < numNodes; ++i) {
        components[component[i]].push_back(i);
    }

    // Sort components by size in descending order
    sort(components.begin(), components.end(), [](const vector<int>& a, const vector<int>& b) {
        return a.size() > b.size();
    });

    return components;
}
