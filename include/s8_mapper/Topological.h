#ifndef __TOPOLOGICAL_H
#define __TOPOLOGICAL_H

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <vector>

#define TOPO_NODE_FREE      1 << 1
#define TOPO_NODE_WALL      1 << 2
#define TOPO_NODE_OBJECT    1 << 3

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
    Node *tmp;
    std::vector<Node*> nodes;

public:
    Topological() {
        init(0, 0);
    }

    Topological(double x, double y) {
        init(x, y);

        //TODO remove below
        
        // Create initial node, TODO Have as part of initialization
        add_node(0, 0.2, TOPO_NODE_WALL, false, false, false, false);
        add_node(0, -0.2, TOPO_NODE_WALL, false, false, false, false);
        add_node(-0.2, 0, TOPO_NODE_WALL, false, false, false, false);
        
        // Add map nodes
        if (!does_node_exist(2.0, 0, TOPO_NODE_FREE, false, false, true, true))
            add_node(2.0, 0, TOPO_NODE_FREE, false, false, true, true);
        if (!does_node_exist(1.8, 0.5, TOPO_NODE_FREE, false, false, false, false))
            add_node(1.8, 0.5, TOPO_NODE_FREE, false, true, false, false);
        if (!does_node_exist(2.0, 1.0, TOPO_NODE_FREE, false, false, false, true))
            add_node(2.0, 1.0, TOPO_NODE_FREE, false, false, false, true);
        if (!does_node_exist(2.0, 2.0, TOPO_NODE_FREE, true, false, false, true));
            add_node(2.0, 2.0, TOPO_NODE_FREE, true, false, false, true);
        if (!does_node_exist(1.0, 2.0, TOPO_NODE_FREE, true, true, false, false))
            add_node(1.0, 2.0, TOPO_NODE_FREE, true, true, false, false);
        if (!does_node_exist(1.0, 1.0, TOPO_NODE_FREE, false, true, true, false))
            add_node(1.0, 1.0, TOPO_NODE_FREE, false, true, true, false);
        if (!does_node_exist(2.05, 1.05, TOPO_NODE_FREE, false, false, false, true)){
            add_node(2.3, 1.05, TOPO_NODE_FREE, false, false, false, true);
        //link(last, tmp);
        //set_as_last_node(tmp);
            //render();
        }
        /*
        if (!does_node_exist(1.5, 1.0, TOPO_NODE_FREE, false, false, false, true))
            add_node(1.5, 1.0, TOPO_NODE_FREE, true, false, false, false);
        */
    }

    ~Topological() {
        for(Node *n : nodes) {
            delete n;
        }
    }

    void add_node(double x, double y, int value, bool isWallNorth, bool isWallWest, bool isWallSouth, bool isWallEast) {
        Node* tmp = new Node(x, y, value);
        add(last, tmp);

        if (value == TOPO_NODE_FREE){
            last = tmp; 
            ROS_INFO("Changing current node to last node");

            if (isWallNorth == true){
                add_node(x, y+0.2, TOPO_NODE_WALL, false,false,false,false);
                ROS_INFO("NODE North");
            }
            if (isWallWest == true){
                add_node(x-0.2, y, TOPO_NODE_WALL, false,false,false,false);
                ROS_INFO("NODE West");
            }
            if (isWallSouth == true){
                add_node(x, y-0.2, TOPO_NODE_WALL, false,false,false,false);
                ROS_INFO("NODE South");
            }
            if (isWallEast == true){
                add_node(x+0.2, y, TOPO_NODE_WALL, false,false,false,false);
                ROS_INFO("NODE East");
            }
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
            } else {
                add_marker(visualization_msgs::Marker::SPHERE, n->x, n->y, 1, 0, 0, 1);
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
            // Check for type and position.
            //double x_tmp = (*nodeIte)->x;
            ROS_INFO("hahah");
            if (nodes[j]->value == value && std::abs(nodes[j]->x - x) < 0.15 && std::abs(nodes[j]->y - y) < 0.15){
                // Check for properties
                if ( check_properties(isWallNorth, (*nodeIte)->north) && check_properties(isWallWest, (*nodeIte)->west) && check_properties(isWallSouth, (*nodeIte)->south) && check_properties(isWallEast, (*nodeIte)->east)){
                    // TODO do it properly.
                }
            }
            j++;
        }
        return false; 

    }

private:
    void traverse(Node *start, std::function<void(Node*,Node*)> func) {
        traverse(start, start->east, func);
        traverse(start, start->north, func);
        traverse(start, start->west, func);
        traverse(start, start->south, func);
    }

    void traverse(Node *current, Node *next, std::function<void(Node*,Node*)> func) {
        if(next == NULL) {
            return;
        }

        //ROS_INFO("x: %lf, y: %lf", current->x, current->y);
        func(current, next);

        if(current != next->east) {
            traverse(next, next->east, func);
        }

        if(current != next->north) {
            traverse(next, next->north, func);
        }

        if(current != next->west) {
            traverse(next, next->west, func);
        }

        if(current != next->south) {
            traverse(next, next->south, func);
        }
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
        last = new_last_node;
    }
    void set_as_tmp_node(Node* new_tmp_node){
        tmp = new_tmp_node;
    }

    bool check_properties(bool property, Node* node){
        if (node == NULL){
            ROS_INFO("NULL");
            return true;
        } else if (property == true && node->value == TOPO_NODE_WALL) {
            ROS_INFO("TRUE");
            return true;
        } else if (property == false && node->value == TOPO_NODE_FREE) {
            ROS_INFO("FALSE");
            return true;
        }
        return false;
    }

    void link(Node *from, Node *to) {
        //TODO Make this better
        ROS_INFO("I'm In");
        if(std::abs(from->x - to->x) < std::abs(from->y - to->y)) {
            ROS_INFO("X less than Y");
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
            ROS_INFO("Y less than X");
            if(to->x > from->x) {
                ROS_INFO("to bigger than from");
                from->east = to;
                ROS_INFO("Almost at the end");
                to->west = from;
                //ROS_INFO("East - West");
            } else {
                ROS_INFO("from bigger than to");
                from->west = to;
                to->east = from;
                //ROS_INFO("West - East");
            }
        }
    }

    bool is_wall(Node *node) {
        return (node->value & TOPO_NODE_WALL) == TOPO_NODE_WALL;
    }
    bool is_free(Node *node) {
        return (node->value & TOPO_NODE_FREE) == TOPO_NODE_FREE;
    }
};

#endif
