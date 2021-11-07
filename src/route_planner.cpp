#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    start_node = &(model.FindClosestNode(start_x, start_y));
    start_node->visited = true; // start_node is inherently visited so close it
    //start_node->g_value is initialized in the constructor (to 0.0)
    //start_node->parent is initialized in the constructor (to null)

    end_node = &(model.FindClosestNode(end_x, end_y));
}


float RoutePlanner::CalculateHValue(RouteModel::Node const * node) {
    return (node->distance(*end_node));
}


void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (auto current_neighbor : current_node->neighbors) {
        if(!(current_neighbor->visited)) {
            current_neighbor->parent = current_node;
            current_neighbor->h_value = CalculateHValue(current_neighbor);
            current_neighbor->g_value = (current_node->g_value) + (current_node->distance(*current_neighbor));
            open_list.emplace_back(current_neighbor);
            current_neighbor->visited = true;
        }
    }
}


// comparison function to sort the open_list of nodes
bool Compare(const RouteModel::Node * n1, const RouteModel::Node * n2) {
    return (n1->g_value + n1->h_value) > (n2->g_value + n2->h_value);
}


RouteModel::Node *RoutePlanner::NextNode() {
    std::sort(open_list.begin(), open_list.end(), Compare);
    RouteModel::Node * next_node = open_list.back();
    open_list.pop_back();
    return next_node;
}


std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    RouteModel::Node * cn = current_node;
    path_found.emplace_back(*cn);

    // iteratively add all nodes along the path to path_found
    RouteModel::Node * current_parent = cn->parent;
    while(current_parent) {
        distance += cn->distance(*current_parent);
        path_found.emplace_back(*current_parent);

        cn = current_parent;
        current_parent = (current_parent->parent);
    }

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    std::reverse(path_found.begin(), path_found.end()); // Reverse the path so it goes from start to end (instead of end to start which is the order the path was built)
    return path_found;
}


void RoutePlanner::AStarSearch() {
    RouteModel::Node * current_node = start_node;

    while( current_node != end_node ) {
        AddNeighbors(current_node); // add all of the neighbors of the current node to the open_list

        if( open_list.size() == 0 ) {
            break; // did not find a path
        }
        else {
            current_node = NextNode(); // sort the open_list and return the next node
        }
    }

    if( current_node == end_node ) {
        path_found = true;    
        m_Model.path = ConstructFinalPath(current_node); // populate m_Model.path so map can be rendered
    }
    else {
        std::cout << "AStarSearch() did not find a path from the start to the end.  Sorry.  Maybe use an airplane?" << std::endl;
    }
}
