#include "reader.h"

#include "istream"
#include "fstream"
#include "ostream"
#include "iostream"
#include <stack>





enum token{
    COMMENT, META, EDGE, END_OF_FILE
};

token get_line_type(std::istream& is){
    switch(is.peek()){
        case std::istream::traits_type::eof():      return END_OF_FILE;
        case '#':                                   return COMMENT;
        case 'M':                                   return META;
    };
    return EDGE;
}

meta_t meta;

edge read_edge(std::istream& is){
    edge e;
    is >> e.n1 >> e.n2 >> e.weight;
    std::getline(is, e.description);
    return e;
}

void read_meta(std::istream& is){
    char discard;
    node_id_t vertex_id;
    std::string name;
    is >> discard >> vertex_id;
    std::getline(is, name);
    meta[vertex_id] = name;
}

adjacency_list_t parse_file(std::string filename){
    std::ifstream in(filename);
    token l;
    edge_list_t edge_list;
    while((l = get_line_type(in)) != END_OF_FILE){
        edge e;
        switch(l){
            case token::EDGE:
                e = read_edge(in);
                edge_list.push_back(e);
                break;
            case token::META:
                read_meta(in);
                break;
            default:
                std::string comment;
                std::getline(in, comment);
        }                
    }
    return adjacency_list_t{meta, edge_list};
}

void display_adjacency_list(adjacency_list_t adj_list) {
    std::cout << "Meta:" << std::endl;
    for (auto& meta_pair : adj_list.first) {
        std::cout << meta_pair.first << " " << meta_pair.second << std::endl;
    }
    std::cout << std::endl << "Edges:" << std::endl;
    for (auto const& edge : adj_list.second) {
        std::cout << edge.n1 << " " << edge.n2 << " " << edge.weight << " " << edge.description << std::endl;
    }
}

bool dfs_visit(const adjacency_list_t& adj_list, node_id_t node, visited_set_t& visited) {
    visited.insert(node);
    for (const auto& edge : adj_list.second) {
        if (edge.n1 == node && visited.find(edge.n2) == visited.end()) {
            if (!dfs_visit(adj_list, edge.n2, visited)) {
                return false;
            }
        } else if (edge.n2 == node && visited.find(edge.n1) == visited.end()) {
            if (!dfs_visit(adj_list, edge.n1, visited)) {
                return false;
            }
        }
    }
    return true;
}

bool dfs_visit1(const adjacency_list_t& adj_list, node_id_t node, visited_set_t& visited) {
    visited.insert(node);
    std::stack <node_id_t> s;
    for (const auto& edge : adj_list.second) {
        if (edge.n1 == node && visited.find(edge.n2) == visited.end()) {
            if (!dfs_visit(adj_list, edge.n2, visited)) {
                s.push(edge.n2);
                visited.insert(edge.n2);
            }
        } else if (edge.n2 == node && visited.find(edge.n1) == visited.end()) {
            if (!dfs_visit(adj_list, edge.n1, visited)) {
                s.push(edge.n1);
                visited.insert(edge.n1);
                            }
        }
    }
    return visited.size()==adj_list.first.size();
}


bool bfs(const adjacency_list_t& adj_list, node_id_t start_node) {
    visited_set_t visited;
    std::queue<node_id_t> q;
    q.push(start_node);
    visited.insert(start_node);
    while (!q.empty()) {
        node_id_t node = q.front();
        q.pop();
        for (const auto& edge : adj_list.second) {
            if (edge.n1 == node && visited.find(edge.n2) == visited.end()) {
                q.push(edge.n2);
                visited.insert(edge.n2);
            } else if (edge.n2 == node && visited.find(edge.n1) == visited.end()) {
                q.push(edge.n1);
                visited.insert(edge.n1);
            }
        }
    }
    return visited.size() == adj_list.first.size();
}
bool is_graph_connected(const adjacency_list_t& adj_list, node_id_t start_node) {
    visited_set_t visited;
    return dfs_visit(adj_list, start_node, visited) && visited.size() == adj_list.first.size();
}

std::pair<weight_t, std::vector<node_id_t>> dijkstra2(const adjacency_list_t& adj_list, node_id_t start_node, node_id_t end_node) {
    std::map<node_id_t, weight_t> dist;
    std::map<node_id_t, node_id_t> prev;
    for (const auto& meta_pair : adj_list.first) {
        dist[meta_pair.first] = std::numeric_limits<weight_t>::infinity();
        prev[meta_pair.first] = -1;
    }
    dist[start_node] = 0;

    std::priority_queue<edge, std::vector<edge>, std::greater<edge>> pq;
    pq.push({start_node, start_node});

    while (!pq.empty()) {
        edge curr = pq.top();
        pq.pop();

        

        if (curr.weight > dist[curr.n2]) {
            continue; //larger than the currect weight 
        }

        for (const auto& edge : adj_list.second) {
            if (edge.n1 == curr.n2 || edge.n2 == curr.n2) {
                node_id_t next_node = (edge.n1 == curr.n2) ? edge.n2 : edge.n1;
                weight_t new_dist = dist[curr.n2] + edge.weight;
                if (new_dist < dist[next_node]) {
                    dist[next_node] = new_dist;
                    prev[next_node] = curr.n2;
                    pq.push({curr.n2, next_node, new_dist});
                }
            }
        }
    }

    //keep track of other conditions
    // Build the path from start to end node
    std::vector<node_id_t> path;
    if (prev[end_node] != -1) {
        node_id_t curr_node = end_node;
        while (curr_node != start_node) {
            path.push_back(curr_node);
            curr_node = prev[curr_node];
        }
        path.push_back(start_node);
        std::reverse(path.begin(), path.end());
    }

    return {dist[end_node], path};
}

/*
std::pair<weight_t, std::vector<node_id_t>> dijkstra2(const adjacency_list_t& adj_list, node_id_t start_node, node_id_t end_node) {
    std::map<node_id_t, weight_t> dist;
    std::map<node_id_t, node_id_t> prev;
    for (const auto& _pair : adj_list.first) {
        dist[meta_pair.first] = std::numeric_limits<weight_t>::infinity();
        prev[meta_pair.first] = -1;
    }
    dist[start_node] = 0;

    std::priority_queue<Node, std::vector<Node>, std::greater<Node>> pq;
    pq.push({start_node, 0});

    while (!pq.empty()) {
        Node curr = pq.top();
        pq.pop();

        if (curr.id == end_node) {
            break; // Found the end node
        }

        if (curr.weight > dist[curr.id]) {
            continue;
        }

        for (const auto& edge : adj_list.second) {
            node_id_t next_node;
            if (edge.n1 == curr.id) {
                next_node = edge.n2;
            } else if (edge.n2 == curr.id) {
                next_node = edge.n1;
            } else {
                continue;
            }

            weight_t new_dist = dist[curr.id] + edge.weight;
            if (new_dist < dist[next_node]) {
                dist[next_node] = new_dist;
                prev[next_node] = curr.id;
                pq.push({next_node, new_dist});
            }
        }
    }

    // Build the path from start to end node
    std::vector<node_id_t> path;
    if (prev[end_node] != -1) {
        node_id_t curr_node = end_node;
        while (curr_node != start_node) {
            path.push_back(curr_node);
            curr_node = prev[curr_node];
        }
        path.push_back(start_node);
        std::reverse(path.begin(), path.end());
    }

    return {dist[end_node], path};
}*/
