#ifndef __TOPOLOGICAL_H
#define __TOPOLOGICAL_H

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <s8_utils/math.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>
#include <boost/foreach.hpp>
#include <cstdlib>

const std::string CONFIG_DOC =  "/catkin_ws/src/s8_mapper/parameters/parameters.json";

#define TOPO_NODE_FREE      1 << 1
#define TOPO_NODE_WALL      1 << 2
#define TOPO_NODE_OBJECT    1 << 3
#define TOPO_NODE_CURRENT   1 << 4

class Topological {
public:
    struct Node {
        Node *east;
        Node *north;
        Node *west;
        Node *south;
        double x;
        double y;
        int value;

        Node(double x, double y, int value) : x(x), y(y), value(value) {
            east = NULL;
            north = NULL;
            west = NULL;
            south = NULL;
        }
    };

private:
    Node *root;
    Node *last;
    std::vector<Node*> nodes;
    double same_nodes_max_dist;

public:
    Topological() {
        init(0, 0);
    }

    Topological(double x, double y) {
        add_params();

        init(x, y);

        //TODO remove below
        
        // Create initial node, TODO Have as part of initialization
        // add_node(0, 0.2, TOPO_NODE_WALL, false, false, false, false);
        // add_node(0, -0.2, TOPO_NODE_WALL, false, false, false, false);
        // add_node(-0.2, 0, TOPO_NODE_WALL, false, false, false, false);


        // Add map nodes

        add_node(0, 0, TOPO_NODE_FREE, false, false, false, false);
        add_node(1, 0, TOPO_NODE_FREE, false, false, false, false);
        add_node(1, 1, TOPO_NODE_FREE, false, false, false, false);
        add_node(0, 1, TOPO_NODE_FREE, false, false, false, false);
        add_node(0, 2, TOPO_NODE_FREE, false, false, false, false);
        add_node(0.5, 2, TOPO_NODE_FREE, false, false, false, false);
        add_node(2, 2, TOPO_NODE_FREE, false, false, false, false);
        add_node(2, 1, TOPO_NODE_FREE, false, false, false, false);
        add_node(1, 1, TOPO_NODE_FREE, false, false, false, false);

        auto path = dijkstra(root, [](Node * node){
            ROS_INFO("(%.2lf, %.2lf)", node->x, node->y);
            return s8::utils::math::is_equal(node->x, 0.5) && s8::utils::math::is_equal(node->y, 2);
        });

        // add_node(2.0, 0, TOPO_NODE_FREE, false, false, true, true);
        // add_node(2.0, 0.5, TOPO_NODE_FREE, true, false, false, true);
        // add_node(1.5, 0.5, TOPO_NODE_FREE, false, true, true, false);
        // add_node(1.5, 1.0, TOPO_NODE_FREE, true, true, false, false);
        // add_node(2.0, 1.0, TOPO_NODE_FREE, false, false, true, true);
        // add_node(2.0, 1.5, TOPO_NODE_FREE, true, false, false, true);
        // add_node(1.5, 1.5, TOPO_NODE_FREE, false, true, true, false);
        // add_node(1.5, 2.3, TOPO_NODE_FREE, true, false, false, false);
        // add_node(2.0, 2.3, TOPO_NODE_FREE, true, false, false, true);
        // add_node(2.0, 2.0, TOPO_NODE_FREE, false, false, true, true);
        // add_node(1.6, 2.0, TOPO_NODE_FREE, false, true, false, false);
        // add_node(1.6, 2.3, TOPO_NODE_FREE, true, false, false, false);
        // add_node(1.1, 2.3, TOPO_NODE_FREE, true, true, false, false);
        // add_node(1.1, 2.1, TOPO_NODE_FREE, false, false, true, true);
        // add_node(0.0, 2.1, TOPO_NODE_FREE, false, true, false, false);
        // add_node(0.0, 0.4, TOPO_NODE_FREE, false, true, true, false);
        // add_node(0.5, 0.4, TOPO_NODE_FREE, false, false, true, true);
        // add_node(0.5, 0.9, TOPO_NODE_FREE, true, true, false, true);

        // // Go back
        // add_node(0.5, 0.5, TOPO_NODE_FREE, false, false, true, true);
        // add_node(0.14, 0.4, TOPO_NODE_FREE, false, true, true, false);
        // add_node(0.06, 2.3, TOPO_NODE_FREE, true, true, false, false);
        // add_node(0.17, 2.3, TOPO_NODE_FREE, true, false, false, true);
        
    }

    ~Topological() {
        for(Node *n : nodes) {
            delete n;
        }
    }

    void add_node(double x, double y, int value, bool isWallNorth, bool isWallWest, bool isWallSouth, bool isWallEast) {
        if (value == TOPO_NODE_FREE || value == TOPO_NODE_CURRENT){
            if (!does_node_exist(x, y, TOPO_NODE_FREE, isWallNorth, isWallWest, isWallSouth, isWallEast)){
                Node* tmp = new Node(x, y, value);
                add(last, tmp);

                set_as_last_node(tmp);
                ROS_INFO("Changing current node to last node");
                add_walls(x, y, isWallNorth, isWallWest, isWallSouth, isWallEast);
            }
        }
        else{
            Node* tmp = new Node(x, y, value);
            add(last, tmp);            
        }
    }

    void render(std::vector<visualization_msgs::Marker> & markers) {
        auto add_marker = [&markers, this](int type, double x, double y, float a, float r, float g, float b) {
            visualization_msgs::Marker marker;

            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time();
            marker.ns = "s8";
            marker.id = markers.size();
            marker.type = type;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = x;
            marker.pose.position.y = y;
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0;
            marker.pose.orientation.y = 0;
            marker.pose.orientation.z = 0;
            marker.pose.orientation.w = 0;
            marker.scale.x = 0.06;
            marker.scale.y = 0.06;
            marker.scale.z = 0.06;
            marker.color.a = a;
            marker.color.r = r;
            marker.color.g = g;
            marker.color.b = b;

            markers.push_back(marker);
        };

        for(Node *n : nodes) {
            if(is_wall(n)) {
                add_marker(visualization_msgs::Marker::SPHERE, n->x, n->y, 1, 1, 0, 0);
            } else if(is_free(n)){
                add_marker(visualization_msgs::Marker::SPHERE, n->x, n->y, 1, 1, 0.5, 0);
            } else if(is_object(n)){
                add_marker(visualization_msgs::Marker::SPHERE, n->x, n->y, 1, 0, 0, 1);
            } else {
                add_marker(visualization_msgs::Marker::SPHERE, n->x, n->y, 1, 0, 1, 1);
            }
        }

        traverse(root, [&markers](Node * from, Node * to) {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time();
            marker.ns = "s8";
            marker.id = markers.size();
            marker.type = visualization_msgs::Marker::LINE_STRIP;
            marker.scale.x = 0.01;
            marker.color.a = 1;
            marker.color.r = 1;
            marker.color.g = 1;
            marker.color.b = 0;

            auto add_point = [&marker](Node *node) {
                geometry_msgs::Point point;
                point.x = node->x;
                point.y = node->y;
                marker.points.push_back(point);
            };



            add_point(from);
            add_point(to);

            markers.push_back(marker);
        });
    }

    bool does_node_exist(double x, double y, int value, bool isWallNorth, bool isWallWest, bool isWallSouth, bool isWallEast){
        std::vector<Node*>::iterator nodeIte;
        int j = 0;
        for (nodeIte = nodes.begin(); nodeIte != nodes.end(); nodeIte++){
            // loop through old vector and load into new vector.
            // Check for type and position.
            if ((nodes[j]->value == TOPO_NODE_FREE || nodes[j]->value == TOPO_NODE_CURRENT) && std::abs(nodes[j]->x - x) < same_nodes_max_dist && std::abs(nodes[j]->y - y) < same_nodes_max_dist){
                // Check for properties
                if (check_properties(isWallNorth, (*nodeIte)->north))
                    ROS_INFO("NORTH IS TRUE");
                if(check_properties(isWallWest, (*nodeIte)->west))
                    ROS_INFO("WEST IS TRUE");
                if(check_properties(isWallSouth, (*nodeIte)->south))
                    ROS_INFO("SOUTH IS TRUE");
                if(check_properties(isWallEast, (*nodeIte)->east))
                    ROS_INFO("EAST IS TRUE");
                if ( check_properties(isWallNorth, (*nodeIte)->north) && check_properties(isWallWest, (*nodeIte)->west) && check_properties(isWallSouth, (*nodeIte)->south) && check_properties(isWallEast, (*nodeIte)->east)){                    
                    
                    // TODO: link last and (*nodeIte).
                    // Think that the problem lies in the traverse rather than the linking.

                    link(last, *nodeIte);
                    set_as_last_node(*nodeIte);
                    return true;
                }
            }
            j++;
        }
        return false; 
    }

private:
    void traverse(Node *start, std::function<void(Node*,Node*)> func) {
        std::unordered_set<Node*> traversed_nodes;

        traverse(traversed_nodes, start, start->east, func);
        traverse(traversed_nodes, start, start->north, func);
        traverse(traversed_nodes, start, start->west, func);
        traverse(traversed_nodes, start, start->south, func);
    }

    void traverse(std::unordered_set<Node*> traversed_nodes, Node *current, Node *next, std::function<void(Node*,Node*)> func) {
        if(next == NULL || traversed_nodes.count(next) == 1) {
            return;
        }

        traversed_nodes.insert(next);

        //ROS_INFO("x: %lf, y: %lf", current->x, current->y);
        func(current, next);

        if(current != next->east) {
            std::unordered_set<Node*> branch_traversed_nodes;
            branch_traversed_nodes.insert(traversed_nodes.begin(), traversed_nodes.end());
            traverse(branch_traversed_nodes, next, next->east, func);
        }

        if(current != next->north) {
            std::unordered_set<Node*> branch_traversed_nodes;
            branch_traversed_nodes.insert(traversed_nodes.begin(), traversed_nodes.end());
            traverse(branch_traversed_nodes, next, next->north, func);
        }

        if(current != next->west) {
            std::unordered_set<Node*> branch_traversed_nodes;
            branch_traversed_nodes.insert(traversed_nodes.begin(), traversed_nodes.end());
            traverse(branch_traversed_nodes, next, next->west, func);
        }

        if(current != next->south) {
            std::unordered_set<Node*> branch_traversed_nodes;
            branch_traversed_nodes.insert(traversed_nodes.begin(), traversed_nodes.end());
            traverse(branch_traversed_nodes, next, next->south, func);
        }
    }

    std::vector<Node*> dijkstra(Node *start, std::function<bool(Node*)> target_node_evaluator) {
        std::list<Node*> node_list;
        std::map<Node*, Node*> previous;
        std::map<Node*, double> distance;

        auto get_closest_node = [&node_list, &distance]() {
            Node * lowest_node = node_list.front();
            for(auto n : node_list) {
                if(distance[n] < distance[lowest_node]) {
                    lowest_node = n;
                }
            }
            return lowest_node;
        };

        auto get_neighbors = [&node_list](Node * node) {
            std::vector<Node*> neighbors;

            if(node->east != NULL && std::find(node_list.begin(), node_list.end(), node->east) != node_list.end()) {
                neighbors.push_back(node->east);
            }
            if(node->north != NULL && std::find(node_list.begin(), node_list.end(), node->north) != node_list.end()) {
                neighbors.push_back(node->north);
            }
            if(node->west != NULL && std::find(node_list.begin(), node_list.end(), node->west) != node_list.end()) {
                neighbors.push_back(node->west);
            }
            if(node->south != NULL && std::find(node_list.begin(), node_list.end(), node->south) != node_list.end()) {
                neighbors.push_back(node->south);
            }

            return neighbors;
        };

        auto length = [](Node * from, Node * to) {
            double dx = from->x - to->x;
            double dy = from->y - to->y;
            return std::sqrt(dx*dx + dy*dy);
        };

        distance[start] = 0; //Distance from start to start is 0

        for(Node * n : nodes) {
            if(n != start) {
                distance[n] = std::numeric_limits<double>::infinity();
                previous[n] = NULL;
            }
            node_list.push_back(n);
        }

        Node * target = NULL;
        while(!node_list.empty()) {
            Node * closest_node = get_closest_node();
            node_list.erase(std::find(node_list.begin(), node_list.end(), closest_node));

            if(target_node_evaluator(closest_node)) {
                target = closest_node;
                break;
            }

            auto neighbors = get_neighbors(closest_node);
            for(Node * neighbor_node : neighbors) {
                double alt = distance[closest_node] + length(closest_node, neighbor_node);
                if(alt < distance[closest_node]) {
                    distance[neighbor_node] = alt;
                    previous[neighbor_node] = closest_node;
                }
            }
        }

        std::vector<Node *> path;
        Node * current = target;
        while(previous.count(current) == 1) {
            path.push_back(current);
            current = previous[current];
        }

        return path;
    }

    void init(double x, double y) {
        root = new Node(x, y, TOPO_NODE_FREE);
        last = root;
        nodes.push_back(root);
    }

    void add(Node *last_node, Node *new_node) {
        nodes.push_back(new_node);
        link(last_node, new_node);
    }

    void set_as_last_node(Node* new_last_node){
        last->value = TOPO_NODE_FREE;
        new_last_node->value = TOPO_NODE_CURRENT;
        last = new_last_node;
    }

    // TODO: MAKE THIS BETTER
    bool check_properties(bool property, Node* node){
        if (node == NULL){
            return true;
        } else if (property == true && node->value == TOPO_NODE_WALL) {
            return true;
        } else if (property == false) {
            return true;
        }
        return false;
    }

    void link(Node *from, Node *to) {
        //TODO Make this better
        if(std::abs(from->x - to->x) < std::abs(from->y - to->y)) {
            if(to->y > from->y) {
                from->north = to;
                to->south = from;
                //ROS_INFO("North - South");
            } else {
                from->south = to;
                to->north = from;
                //ROS_INFO("South - North");
            }
        } else {
            if(to->x > from->x) {
                from->east = to;
                to->west = from;
                //ROS_INFO("East - West");
            } else {
                from->west = to;
                to->east = from;
                //ROS_INFO("West - East");
            }
        }
    }

    void add_walls(double x, double y, bool isWallNorth, bool isWallWest, bool isWallSouth, bool isWallEast){
        if (isWallNorth == true){
            add_node(x, y+0.2, TOPO_NODE_WALL, false,false,false,false);
        }
        if (isWallWest == true){
            add_node(x-0.2, y, TOPO_NODE_WALL, false,false,false,false);
        }
        if (isWallSouth == true){
            add_node(x, y-0.2, TOPO_NODE_WALL, false,false,false,false);
        }
        if (isWallEast == true){
            add_node(x+0.2, y, TOPO_NODE_WALL, false,false,false,false);
        }
    }

    bool is_wall(Node *node) {
        return (node->value & TOPO_NODE_WALL) == TOPO_NODE_WALL;
    }
    bool is_free(Node *node) {
        return (node->value & TOPO_NODE_FREE) == TOPO_NODE_FREE;
    }
    bool is_object(Node *node) {
        return (node->value & TOPO_NODE_OBJECT) == TOPO_NODE_OBJECT;
    }
    bool is_wall_node(Node* node){
        if (node == NULL)
            return false;
        else if (node->value == TOPO_NODE_WALL)
            return true;
        else 
            return false;
    }

    void add_params()
    {
        const char * home = ::getenv("HOME");
        boost::property_tree::ptree pt;
        boost::property_tree::read_json(home + CONFIG_DOC, pt);
        // CIRCLE
        same_nodes_max_dist = pt.get<double>("same_nodes_max_dist");
    }
};

#endif
