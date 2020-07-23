#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = &m_Model.FindClosestNode(end_x, end_y);
}


// CalculateHValue method.
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    // Node objects have a distance method to determine the distance to another node.
    return node->distance(*end_node);
}


// AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    // Use FindNeighbors() method of current_node to populate current_node.neighbors vector with all the neighbors.
    current_node->FindNeighbors();

    // For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
    // Use CalculateHValue below to implement the h-Value calculation.
    // For each node in current_node.neighbors, add neighbor to open_list and set node's visited attribute to true.
    for (auto neighbor : current_node->neighbors) {
        neighbor->parent = current_node;
        neighbor->h_value = CalculateHValue(neighbor);
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
        neighbor->visited = true;
        open_list.push_back(neighbor);
    }
}


// NextNode method to sort the open list and return the next node.
RouteModel::Node *RoutePlanner::NextNode() {
    // Sort the open_list according to the sum of the h value and g value.
    std::sort(open_list.begin(), open_list.end(), [](auto a, auto b) {
        return a->g_value + a->h_value < b->g_value + b->h_value;
    });

    // Create a pointer to the node in the list with the lowest sum.
    auto next_node = open_list.front();

    // Remove lowest sum node from the open_list.
    open_list.erase(open_list.begin());

    // Return the pointer.
    return next_node;
}


// ConstructFinalPath method to return the final path found from A* search.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    distance = 0.0f;

    // Create path_found vector
    std::vector<RouteModel::Node> path_found;

    // Iteratively follow the chain of parents of nodes until the starting node is found.
    // For each node in the chain, add the distance from the node to its parent to the distance variable.
    while (true) {
        if ( current_node->parent == nullptr ) {
            path_found.push_back(*current_node);
            break;
        }
        distance += current_node->distance(*current_node->parent);
        path_found.push_back(*current_node);
        current_node = current_node->parent;
    }

    std::reverse(path_found.begin(), path_found.end());

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// A* Search algorithm.
void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;
    open_list.push_back(start_node);
    start_node->visited = true;
    
    while (open_list.size() > 0) {
        // Use the NextNode() method to sort the open_list and return the next node.
        current_node = NextNode();

        if (current_node->distance(*end_node) != 0) {
            // Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
            AddNeighbors(current_node);
        }
        else {
            // When the search has reached the end_node, use the ConstructFinalPath method to return the final path.
            // Store the final path in the m_Model.path attribute before the method exits. 
            // This path will then be displayed on the map tile.
            m_Model.path = ConstructFinalPath(current_node);
            return;
        }
    }
}