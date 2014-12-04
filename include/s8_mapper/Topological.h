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

#define TOPO_NODE_FREE              1 << 1
#define TOPO_NODE_WALL              1 << 2
#define TOPO_NODE_OBJECT            1 << 3
#define TOPO_NODE_CURRENT           1 << 4
#define TOPO_NODE_OBJECT_VIEWER     1 << 5

#define TOPO_EAST           -1 * M_PI / 4
#define TOPO_NORTH          1 * M_PI / 4
#define TOPO_WEST           3 * M_PI / 4
#define TOPO_SOUTH          -3 * M_PI / 4

class Topological {
public:
    struct Node {
        std::unordered_set<Node*> neighbors;
        double x;
        double y;
        int value;
        long id;

        Node(double x, double y, int value) : x(x), y(y), value(value) {
            id = rand();
        }

        Node(double x, double y, int value, long id) : x(x), y(y), value(value), id(id) {

        }
    };

private:
    Node *root;
    Node *last;
    Node *viewer;
    std::unordered_set<Node*> nodes;
    ros::ServiceClient* set_position_client;
    double same_nodes_max_dist;
    double same_nodes_euclidean_dist;

public:
    Topological() {
        srand(0); //Just want random in the lifespan of the map node execution.
        init(0, 0);
    }

    Topological(double x, double y, ros::ServiceClient* position_client) {
        add_params();
        root = NULL;
        last = NULL;
        set_position_client = position_client;

        //init(x, y);

        //TODO remove below
        
        // Create initial node, TODO Have as part of initialization
        // add_node(0, 0.2, TOPO_NODE_WALL, false, false, false, false);
        // add_node(0, -0.2, TOPO_NODE_WALL, false, false, false, false);
        // add_node(-0.2, 0, TOPO_NODE_WALL, false, false, false, false);


        // Add map nodes

//         add_node(0, 0, TOPO_NODE_FREE, true, true, false, false);
//         add_node(1, 0, TOPO_NODE_FREE, false, false, false, false);
//         add_node(1, 1, TOPO_NODE_FREE, false, false, false, false);

//   //      add_node(0, 0.15, TOPO_NODE_FREE, true, true, true, false);
// //        add_node(0.5, 1, TOPO_NODE_FREE, false, false, false, false);
//         add_node(0, 1, TOPO_NODE_FREE, false, false, false, false);
//    //     add_node(0, 1.5, TOPO_NODE_FREE, false, false, false, false);
//         add_node(0, 2, TOPO_NODE_FREE, false, false, false, false);
//         add_node(0.5, 2, TOPO_NODE_FREE, false, false, false, false);
//         add_node(2, 2, TOPO_NODE_FREE, false, false, false, false);
//         add_node(2, 1, TOPO_NODE_FREE, false, true, true, true);

        // const std::string home = ::getenv("HOME");
        // save_to_file(home + "/maps/map.json");

        const std::string home = ::getenv("HOME");
        load_from_file(home + "/maps/map_1417728019.json");
        //add_node(0, 0, TOPO_NODE_FREE, false, true, false, false);
        //add_node(1.5, 0, TOPO_NODE_FREE, false, false, false, false);

        // for(auto n : nodes) {
        //     std::string s = "Connections: ";
        //     for(auto nn : n->neighbors) {
        //         s += "(" + std::to_string(nn->x) + "," + std::to_string(nn->y) + ")";
        //     }
        //     ROS_INFO("(%lf %lf) %s", n->x, n->y, s.c_str());
        // }

        // // traverse(root, [](Node * from, Node * to){
        // //     ROS_INFO("(%.2lf, %.2lf) -> (%.2lf, %.2lf)", from->x, from->y, to->x, to->y);
        // // });

        // auto path = dijkstra(root, [](Node * node){
        //     ROS_INFO("(%.2lf, %.2lf)", node->x, node->y);
        //     return s8::utils::math::is_equal(node->x, 0.5) && s8::utils::math::is_equal(node->y, 2);
        // });

        // ROS_INFO("PATH");

        // for(Node * node : path) {
        //     ROS_INFO("(%.2lf, %.2lf)", node->x, node->y);
        // }

        // exit(0);

        // ROS_INFO("almost adding");
        // add_node(x, y, TOPO_NODE_FREE, true, true, true, false);
        // ROS_INFO("added");
        // add_node(2.0, 0.0, TOPO_NODE_FREE, true, false, true, true);
    }

    ~Topological() {
        for(Node *n : nodes) {
            delete n;
        }
    }

    void add_node(double x, double y, int value, bool isWallNorth, bool isWallWest, bool isWallSouth, bool isWallEast) {
        if (!is_root_initialized()){
            Node* tmp = new Node(x, y, value);
            add(last, tmp); 
            add_walls(x, y, isWallNorth, isWallWest, isWallSouth, isWallEast);
            ROS_INFO("INITIALIZING");
        }
        else if (value == TOPO_NODE_FREE || value == TOPO_NODE_CURRENT){
            if (!does_node_exist(x, y, TOPO_NODE_FREE, isWallNorth, isWallWest, isWallSouth, isWallEast)){
                Node* tmp = new Node(x, y, value);
                add(last, tmp);

                set_as_last_node(tmp);
                ROS_INFO("Changing current node to last node");
                add_walls(x, y, isWallNorth, isWallWest, isWallSouth, isWallEast);
            }
        }
        else if (value == TOPO_NODE_OBJECT_VIEWER){
            Node* tmp = new Node(x, y, value);
            viewer = tmp;
            link(last, viewer);
            nodes.insert(viewer);
        }
        else if (value == TOPO_NODE_OBJECT){
            Node* tmp = new Node(x, y, value);
            link(viewer, tmp);
            nodes.insert(tmp);
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
            } else if(is_object_viewer(n)){
                add_marker(visualization_msgs::Marker::SPHERE, n->x, n->y, 1, 0, 0.5, 1);
            } else{
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
                    //if(n->value != TOPO_NODE_WALL) {
                    //    return true;
                    //}
                    return true;
                }
            }
            return false;
        };

        for (auto n : nodes){
            // loop through old vector and load into new vector.
            // Check for type and position.
            if (n->value == TOPO_NODE_FREE || n->value == TOPO_NODE_CURRENT){

                // Check only based on distance in world coordinates
                if (std::abs(n->x - x) < same_nodes_max_dist && std::abs(n->y - y) < same_nodes_max_dist){
                    if ( check_properties(isWallNorth, neighbors_in_heading(n, TOPO_NORTH)) && check_properties(isWallWest, neighbors_in_heading(n, TOPO_WEST)) && check_properties(isWallSouth, neighbors_in_heading(n, TOPO_SOUTH)) && check_properties(isWallEast, neighbors_in_heading(n, TOPO_EAST))){                    
                        
                        // TODO: link last and (*nodeIte).
                        // Think that the problem lies in the traverse rather than the linking.
                        link(last, n);
                        set_as_last_node(n);
                        add_walls(n->x, n->y, isWallNorth, isWallWest, isWallSouth, isWallEast);
                        update_position(n);
                        return true;
                    }
                }
                // Check based on euclidean distance if traveling between two known nodes.
                if (std::abs( euclidian_distance(x, last->x, y, last->y) - euclidian_distance(n->x, last->x, n->y, last->y)) < same_nodes_euclidean_dist){
                    if (heading_between_nodes(last, x, y) == heading_between_nodes(last, n)){
                        if ( check_properties(isWallNorth, neighbors_in_heading(n, TOPO_NORTH)) && check_properties(isWallWest, neighbors_in_heading(n, TOPO_WEST)) && check_properties(isWallSouth, neighbors_in_heading(n, TOPO_SOUTH)) && check_properties(isWallEast, neighbors_in_heading(n, TOPO_EAST))){                    
                            if (n->neighbors.count(last)>0){
                                link(last, n);
                                set_as_last_node(n);
                                add_walls(n->x, n->y, isWallNorth, isWallWest, isWallSouth, isWallEast);
                                update_position(n);
                                return true;
                            }
                        } 
                    }  
                }
            }
        }
        return false;
    }

    bool is_root_initialized(){
        if (root == NULL)
            return false;
        else
            return true;
    }

    bool save_to_file(const std::string & filename) {
        boost::property_tree::ptree pt;
        boost::property_tree::ptree pt_nodes;

        for(Node * n : nodes) {
            boost::property_tree::ptree pt_node;
            pt_node.put("id", n->id);
            pt_node.put("x", n->x);
            pt_node.put("y", n->y);
            boost::property_tree::ptree pt_neighbors;
            for(Node * nn : n->neighbors) {
                boost::property_tree::ptree pt_neighbor;
                pt_neighbor.put("", nn->id);
                pt_neighbors.push_back(std::make_pair("", pt_neighbor));
            }

            pt_node.add_child("neighbors", pt_neighbors);
            pt_nodes.push_back(std::make_pair("", pt_node));
        }

        pt.add_child("nodes", pt_nodes);
        pt.put("root", root->id);
        write_json(filename, pt);

        return true;
    }

    bool load_from_file(const std::string & filename) {
        std::map<long, Node *> ids;
        boost::property_tree::ptree pt;
        boost::property_tree::read_json(filename, pt);

        long root_id = pt.get<long>("root");

        //Create nodes.
        BOOST_FOREACH(boost::property_tree::ptree::value_type &v, pt.get_child("nodes")) {
            auto node_pt = v.second;
            Node *n = new Node(node_pt.get<double>("x"), node_pt.get<double>("y"), node_pt.get<int>("value"), node_pt.get<long>("id"));
            nodes.insert(n);
            if (n->value == TOPO_NODE_CURRENT)
                n->value = TOPO_NODE_FREE;
            if(n->id == root_id) {
                root = n;
                last = root;
                last->value = TOPO_NODE_CURRENT;
                //set_as_last_node(n);
            }
            ids[n->id] = n;
        }

        //Link nodes.
        BOOST_FOREACH(boost::property_tree::ptree::value_type &v, pt.get_child("nodes")) {
            auto node_pt = v.second;
            Node *n = ids[node_pt.get<long>("id")];
            BOOST_FOREACH(boost::property_tree::ptree::value_type &v2, node_pt.get_child("neighbors")) {
                auto neighbor_pt = v2.second;
                long neighbor_id = neighbor_pt.get<long>("");
                Node *nn = ids[neighbor_id];
                n->neighbors.insert(nn);
            }
        }

        last = root;

        return true;
    }

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
        const double NODE_DISTANCE_PENALTY = 0.30;

        auto length = [&NODE_DISTANCE_PENALTY](Node * from, Node * to) {
            double dx = from->x - to->x;
            double dy = from->y - to->y;
            return std::sqrt(dx*dx + dy*dy) + NODE_DISTANCE_PENALTY;
        };

        return dijkstra(start, target_node_evaluator, length);
    }

    std::vector<Node*> dijkstra(Node *start, std::function<bool(Node*)> target_node_evaluator, std::function<double(Node*,Node*)> length) {
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

        distance[start] = 0; //Distance from start to start is 0

        for(Node * n : nodes) {
            if(n != start) {
                distance[n] = 1000;
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
                if(std::find(node_list.begin(), node_list.end(), neighbor_node) == node_list.end()) {
                    //already checked this node.
                    continue;
                }

                double alt = distance[closest_node] + length(closest_node, neighbor_node);

                if(alt < distance[neighbor_node]) {
                    distance[neighbor_node] = alt;
                    previous[neighbor_node] = closest_node;
                }
            }
        }

        double path_length = 0;
        std::vector<Node *> path;
        Node * current = target;
        path.push_back(current);
        while(previous.count(current) == 1) {
            path.push_back(previous[current]);
            path_length += length(previous[current], current);
            current = previous[current];
        }

        return path;
    }

    Node * get_last() {
        return last;
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

private:
    void init(double x, double y) {
        root = new Node(x, y, TOPO_NODE_FREE);
        last = root;
        nodes.insert(root);
    }

    void add(Node *last_node, Node *new_node) {
        if (!is_root_initialized()){
            root = new_node;
            last = root;
            ROS_INFO("initialization");
            nodes.insert(new_node);
        }
        else{
            nodes.insert(new_node);
            link(last_node, new_node);
        }
    }

    void set_as_last_node(Node* new_last_node){
        last->value = TOPO_NODE_FREE;
        new_last_node->value = TOPO_NODE_CURRENT;
        last = new_last_node;
    }

    void link(Node *from, Node *to) {
        if(from == to) {
            return;
        }

        auto from_connected = neighbors_in_heading(from, heading_between_nodes(from, to));

        // for(Node *n : from_connected) {
        //     ROS_INFO("form connected: %lf %lf", n->x, n->y);
        //     to->neighbors.insert(n);
        //     n->neighbors.insert(to);
        // }

        // Loop through neighbours and add connections to the new node.
        if (to->value == TOPO_NODE_FREE || to->value == TOPO_NODE_CURRENT){
            double heading = heading_between_nodes(from, to);
            for (Node * n : from->neighbors){
                if (n->value != TOPO_NODE_WALL && n->value != TOPO_NODE_OBJECT && heading == heading_between_nodes(from, n)){
                    to->neighbors.insert(n);
                    n->neighbors.insert(to);
                }
            }
            heading = heading_between_nodes(to, from);
            for (Node * n : to->neighbors){
                if (n->value != TOPO_NODE_WALL && n->value != TOPO_NODE_OBJECT && heading == heading_between_nodes(to, n)){
                    from->neighbors.insert(n);
                    n->neighbors.insert(from);
                }
            }
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
    bool is_object_viewer(Node *node) {
        return (node->value & TOPO_NODE_OBJECT_VIEWER) == TOPO_NODE_OBJECT_VIEWER;
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
        if(angle <= TOPO_NORTH && angle > TOPO_EAST) {
            return TOPO_EAST;
        }
        if(angle <= TOPO_WEST && angle > TOPO_NORTH) {
            return TOPO_NORTH;
        }
        if(angle <= TOPO_SOUTH || angle > TOPO_WEST) {
            return TOPO_WEST;
        }
        if(angle <= TOPO_EAST && angle > TOPO_SOUTH) {
            return TOPO_SOUTH;
        }
    }

    double angle_between_nodes(Node * from, Node * to) {
        double dx = to->x - from->x;
        double dy = to->y - from->y;
        return std::atan2(dy, dx);
    }

    double angle_between_nodes(Node * from, double x_to, double y_to) {
        double dx = x_to - from->x;
        double dy = y_to - from->y;
        return std::atan2(dy, dx);
    }

    double heading_between_nodes(Node * from, Node * to) {
        return angle_to_heading(angle_between_nodes(from, to));
    }

    double heading_between_nodes(Node * from, double x_to, double y_to) {
        return angle_to_heading(angle_between_nodes(from, x_to, y_to));
    }

    double euclidian_distance(double x_from, double x_to, double y_from, double y_to){
        double xd = std::abs(x_from - x_to);
        double yd = std::abs(y_from - y_to);

        return std::sqrt(xd*xd + yd*yd);
    }

    void add_params()
    {
        const char * home = ::getenv("HOME");
        boost::property_tree::ptree pt;
        boost::property_tree::read_json(home + CONFIG_DOC, pt);
        // CIRCLE
        same_nodes_max_dist = pt.get<double>("same_nodes_max_dist");
        same_nodes_euclidean_dist = pt.get<double>("same_nodes_euclidean_dist");
    }

    void update_position(Node* node){
        s8_pose::setPosition pn;
        pn.request.x = node->x;
        pn.request.y = node->y;
        if(!set_position_client->call(pn)) {
            ROS_FATAL("Failed to call set position node.");
        }
    }
};

#endif
