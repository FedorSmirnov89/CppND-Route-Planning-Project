#include "route_planner.h"
#include <algorithm>
#include <iostream>

using std::vector;
using std::sort;
using std::cout;

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    this->start_node = &model.FindClosestNode(start_x, start_y);
    this->end_node = &model.FindClosestNode(end_x, end_y);
}



float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*(this->end_node));
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (RouteModel::Node *nodePtr : current_node->neighbors){
        nodePtr->parent = current_node;
        nodePtr->h_value = CalculateHValue(nodePtr);
        nodePtr->g_value = current_node->g_value + current_node->distance(*nodePtr);
        open_list.push_back(nodePtr);
        nodePtr->visited = true;
    }
}


bool compare(RouteModel::Node* first, RouteModel::Node* second){
    float fFirst = first->g_value + first->h_value;
    float fSecond = second->g_value + second->h_value;
    return fFirst > fSecond;
}

void sortList(vector<RouteModel::Node*> &openList){
    sort(openList.begin(), openList.end(), compare);
}


RouteModel::Node *RoutePlanner::NextNode() {
    sortList(open_list);
    RouteModel::Node* next = open_list.back();
    open_list.pop_back();
    return next;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    vector<RouteModel::Node> pathReverse;
    pathReverse.push_back(*current_node);

    while (current_node->parent != nullptr){
        auto parent = current_node->parent;
        pathReverse.push_back(*parent);
        distance += current_node->distance(*parent);
        current_node = parent;
        cout << "on path node " << current_node->x << " " << current_node->y << "\n";
        cout << "start node is " << this->start_node->x << " " << this->start_node->y << "\n";
    }

    for (int i = 0; i < pathReverse.size(); i++){
        path_found.push_back(pathReverse[pathReverse.size()-i-1]);
    }

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node;
    this->open_list.push_back(this->start_node);
    this->start_node->visited = true;
    
    while (open_list.size() != 0){
        current_node = this->NextNode();
        if(this->end_node == current_node){
            break;
        }
        this->AddNeighbors(current_node);
    }
    cout << "left loop\n";
    m_Model.path = ConstructFinalPath(current_node);
}