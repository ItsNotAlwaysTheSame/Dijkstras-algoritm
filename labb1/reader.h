#ifndef READER_H
#define READER_H

#include <iostream>
#include "algorithm"
#include <map>
#include <unordered_set>
#include <queue>


#include <queue>
#include <limits>

using node_id_t = int;
using weight_t = double;
using meta_t = std::map<node_id_t, std::string>;
using visited_set_t = std::unordered_set<node_id_t>;
/*
struct Node {
    node_id_t id;
    weight_t weight;
    bool operator>(const Node& other) const {
        return weight > other.weight;
    }
};*/


struct edge{
    node_id_t n1;
    node_id_t n2;
    weight_t weight;
    std::string description;
    bool operator>(const edge& other) const {
        return weight > other.weight;
    }
};
using node_map_t = std::map<node_id_t, std::vector<edge>>;

using edge_list_t = std::vector<edge>;

using adjacency_list_t = std::pair<meta_t, edge_list_t>;

adjacency_list_t parse_file(std::string filename);

void display_adjacency_list(adjacency_list_t adj_list);

bool dfs_visit(const adjacency_list_t& adj_list, node_id_t node, visited_set_t& visited);
bool dfs_visit1(const adjacency_list_t& adj_list, node_id_t node, visited_set_t& visited);

bool is_graph_connected(const adjacency_list_t& adj_list, node_id_t start_node);


bool bfs(const adjacency_list_t& adj_list, node_id_t start_node);

std::pair<weight_t, std::vector<node_id_t>> dijkstra2(const adjacency_list_t& adj_list, node_id_t start_node, node_id_t end_node);

#endif