#include "reader.h"

int main()
{
    adjacency_list_t adj_list = parse_file("t.txt");
    visited_set_t visited;
    //display_adjacency_list(adj_list);
   

if (bfs(adj_list, 0)) {
    std::cout << "The graph is connected starting from node 0." << std::endl;
} else {
    std::cout << "The graph is not connected starting from node 0." << std::endl;
}
 if ( dfs_visit1(adj_list, 0,  visited)) {
    std::cout << "The graph is connected starting from node 0." << std::endl;
} else {
    std::cout << "The graph is not connected starting from node 0." << std::endl;
}

int start_node,end_node;
std::cout<<"insert the start node\n";
std::cin>>start_node;
std::cout<<"insert the end node\n";
std::cin>>end_node;
auto result = dijkstra2(adj_list, start_node, end_node);
weight_t shortest_distance = result.first;
    std::vector<node_id_t> shortest_path = result.second;

    // Print the results
    if (!shortest_path.empty()) {
        std::cout << "Shortest distance from node " << start_node << " to node " << end_node << " is " << shortest_distance << std::endl;
        std::cout << "Shortest path: ";
        for (node_id_t node : shortest_path) {
            std::cout << node << " ";
        }
        std::cout << std::endl;
    } else {
        std::cout << "Node " << start_node << " is not reachable from node " << end_node << std::endl;
    }

    return 0;
}