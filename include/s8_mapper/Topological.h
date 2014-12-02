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

#define TOPO_EAST                7 * M_PI / 4
#define TOPO_NORTH               M_PI / 4
#define TOPO_WEST                3 * M_PI / 4
#define TOPO_SOUTH               5 * M_PI / 4

class Topological {
public:
    struct Node {
        std::unordered_set<Node*> neighbors;
        double x;
        double y;
        int value;

        Node(double x, double y, int value) : x(x), y(y), value(value) {
            
        }
    };

private:
    Node *root;
    Node *last;
    std::unordered_set<Node*> nodes;
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
        auto check_properties = [](bool is_wall, std::unordered_set<Node*> nodes) {
            if(nodes.size() == 0) {
                return true;
            }

            for(auto n : nodes) {
                if(is_wall) {
                    if(n->value == TOPO_NODE_WALL) {
                        return true;
                    }
                } else {
                    if(n->value != TOPO_NODE_WALL) {
                        return true;
                    }
                }
            }

            return false;
        };

        for (auto n : nodes){
            // loop through old vector and load into new vector.
            // Check for type and position.
            if ((n->value == TOPO_NODE_FREE || n->value == TOPO_NODE_CURRENT) && std::abs(n->x - x) < same_nodes_max_dist && std::abs(n->y - y) < same_nodes_max_dist){

                if ( check_properties(isWallNorth, neighbors_in_heading(n, TOPO_NORTH)) && check_properties(isWallWest, neighbors_in_heading(n, TOPO_WEST)) && check_properties(isWallSouth, neighbors_in_heading(n, TOPO_SOUTH)) && check_properties(isWallEast, neighbors_in_heading(n, TOPO_EAST))){                    
                    
                    // TODO: link last and (*nodeIte).
                    // Think that the problem lies in the traverse rather than the linking.

                    link(last, n);
                    set_as_last_node(n);
                    return true;
                }
            }
        }
        return false;
    }

private:
    void traverse(Node *start, std::function<void(Node*,Node*)> func) {
        std::unordered_set<Node*> traversed_nodes;
        traversed_nodes.insert(start);

        for(Node * n : start->neighbors) {
            traverse(traversed_nodes, start, n, func);
        }
    }

    void traverse(std::unordered_set<Node*> traversed_nodes, Node *current, Node *next, std::function<void(Node*,Node*)> func) {
        if(next == NULL || traversed_nodes.count(next) == 1) {
            return;
        }

        traversed_nodes.insert(next);

        //ROS_INFO("x: %lf, y: %lf", current->x, current->y);
        func(current, next);

        for(Node * n : next->neighbors) {
            std::unordered_set<Node*> branch_traversed_nodes;
            branch_traversed_nodes.insert(traversed_nodes.begin(), traversed_nodes.end());
            traverse(branch_traversed_nodes, next, n, func);
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

            for(Node * neighbor_node : closest_node->neighbors) {
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
        nodes.insert(root);
    }

    void add(Node *last_node, Node *new_node) {
        nodes.insert(new_node);
        link(last_node, new_node);
    }

    void set_as_last_node(Node* new_last_node){
        last->value = TOPO_NODE_FREE;
        new_last_node->value = TOPO_NODE_CURRENT;
        last = new_last_node;
    }

    void link(Node *from, Node *to) {
        auto from_connected = neighbors_in_heading(from, heading_between_nodes(from, to));

        for(Node *n : from_connected) {
            to->neighbors.insert(n);
            n->neighbors.insert(to);
        }

        to->neighbors.insert(from);
        from->neighbors.insert(to);
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

    double angle_to_heading(double angle) {
        if(angle <= TOPO_NORTH || angle > TOPO_EAST) {
            return TOPO_EAST;
        }
        if(angle <= TOPO_WEST || angle > TOPO_NORTH) {
            return TOPO_NORTH;
        }
        if(angle <= TOPO_SOUTH || angle > TOPO_WEST) {
            return TOPO_WEST;
        }
        if(angle <= TOPO_EAST || angle > TOPO_SOUTH) {
            return TOPO_SOUTH;
        }
    }

    double angle_between_nodes(Node * from, Node * to) {
        double dx = to->x - from->x;
        double dy = to->y - from->y;
        return std::atan2(dy, dx);
    }

    double heading_between_nodes(Node * from, Node * to) {
        return angle_to_heading(angle_between_nodes(from, to));
    }

    std::unordered_set<Node *> neighbors_in_heading(Node * node, double heading) {
        std::unordered_set<Node *> neighbors;

        for(auto n : node->neighbors) {
            if(heading_between_nodes(node, n) == heading) {
                neighbors.insert(n);
            }
        }

        return neighbors;
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
