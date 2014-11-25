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
    std::vector<Node*> nodes;

public:
    Topological() {
        init(0, 0);
    }

    Topological(double x, double y) {
        init(x, y);

        //TODO remove below
        add(root, new Node(0, 0.1, TOPO_NODE_WALL));
        add(root, new Node(0, -0.1, TOPO_NODE_WALL));
        add(root, new Node(-0.2, 0, TOPO_NODE_WALL));
    }

    ~Topological() {
        for(Node *n : nodes) {
            delete n;
        }
    }

    void add_node(double x, double y, int value) {
        add(last, new Node(x, y, value));
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
            marker.scale.x = 0.02;
            marker.scale.y = 0.02;
            marker.scale.z = 0.02;
            marker.color.a = a;
            marker.color.r = r;
            marker.color.g = g;
            marker.color.b = b;

            markers.push_back(marker);
        };

        for(Node *n : nodes) {
            if(is_wall(n)) {
                add_marker(visualization_msgs::Marker::SPHERE, n->x, n->y, 1, 1, 0, 0);
            } else {
                add_marker(visualization_msgs::Marker::SPHERE, n->x, n->y, 1, 0, 1, 0);
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

    void link(Node *from, Node *to) {
        //TODO Make this better
        double treshold = 0.2;
        if(std::abs(from->x - to->x) < treshold) {
            if(to->y > from->y) {
                from->north = to;
                to->south = from;
            } else {
                from->south = to;
                to->north = from;
            }
        } else {
            if(to->x > from->x) {
                from->east = to;
                to->west = from;
            } else {
                from->west = to;
                to->east = from;
            }
        }
    }

    bool is_wall(Node *node) {
        return (node->value & TOPO_NODE_WALL) == TOPO_NODE_WALL;
    }
};

#endif
