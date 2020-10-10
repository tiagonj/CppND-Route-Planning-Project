#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    this->start_node = &m_Model.FindClosestNode(start_x, start_y);
    this->end_node = &m_Model.FindClosestNode(end_x, end_y);
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    return node->distance(*(this->end_node));
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();

    for (auto neighbour : current_node->neighbors)
    {
        if (!neighbour->visited) // Ignore neighbours which have already been visited
        {
            neighbour->visited = true;
            neighbour->parent = current_node;
            neighbour->h_value = RoutePlanner::CalculateHValue(neighbour);
            neighbour->g_value = current_node->g_value + current_node->distance(*neighbour);
            this->open_list.push_back(neighbour);
        }
    }

}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

bool nodeFValueComparator(RouteModel::Node * n1, RouteModel::Node * n2) 
{
    return (n1->g_value + n1->h_value) < (n2->g_value + n2->h_value);
}

RouteModel::Node *RoutePlanner::NextNode() {
    // Sort open_list using a custom comparator function
    std::sort(open_list.begin(), open_list.end(), nodeFValueComparator);

    // Remove and return the first node in the sorted list (i.e. the one with the lowest f-value)
    RouteModel::Node * np = open_list[0];
    open_list.erase(open_list.begin());
    return np;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // Build up the path, starting from current node, through to its parent 
    // node and through all of the subsequent parent nodes, until a node is 
    // found which does not have a parent (which presumably is start_node)
    while (current_node)
    {
        // Insert node at the front of the vector, to get the correct order
        path_found.insert(path_found.begin(), *current_node);

        if (current_node->parent)
        {
            // Accumulate the distances between current_nodes and their parents
            distance += current_node->distance(*current_node->parent);
        }

        current_node = current_node->parent;
    }

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    this->distance = distance;

    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // Initialise start_node attributes
    this->start_node->parent = nullptr;
    this->start_node->visited = true;
    this->start_node->g_value = 0.0f;
    this->start_node->h_value = RoutePlanner::CalculateHValue(this->start_node);

    // Kick off A* search by adding start_node to the open_list
    open_list.push_back(this->start_node);

    // Keep trying to find path until open_list is empty
    while (open_list.size() > 0U)
    {
        // Remove the lowest-f-value node from the open_list
        current_node = RoutePlanner::NextNode();

        if (current_node == this->end_node)
        {
            // A path from start_node to end_node has been found
            m_Model.path = RoutePlanner::ConstructFinalPath(this->end_node);
            return;
        }

        // Path from start_node to end_node has not been found yet

        // Expand open_list with the not-yet-visited neighbours of current_node
        RoutePlanner::AddNeighbors(current_node);
    }

    std::cout << "Did not find a path from start to end node!\n";

}