#include <ros/ros.h>
#include <s8_common_node/Node.h>
#include <s8_mapper/mapper_node.h>
#include <s8_ir_sensors/ir_sensors_node.h>
#include <s8_pose/pose_node.h>
#include <s8_pose/setPosition.h>
#include <s8_mapper/OccupancyGrid.h>
#include <s8_mapper/Topological.h>
#include <s8_mapper/Navigator.h>
#include <vector>
#include <signal.h>
#include <time.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <s8_msgs/IRDistances.h>
#include <nav_msgs/OccupancyGrid.h>
#include <s8_mapper/PlaceNode.h>
#include <s8_mapper/NavigateAction.h>
#include <actionlib/server/simple_action_server.h>

#define HZ                          10
#define RENDER_HZ                   10

#define PARAM_SIDE_LENGTH_NAME      "side_length"
#define PARAM_SIDE_LENGTH_DEFAULT   10.0
#define PARAM_RESOLUTION_NAME       "resolution"
#define PARAM_RESOLUTION_DEFAULT    0.02
#define PARAM_RENDER_NAME           "render"
#define PARAM_RENDER_DEFAULT        true
#define PARAM_ROBOT_WIDTH_NAME      "robot_width"
#define PARAM_ROBOT_WIDTH_DEFAULT   0.235
#define PARAM_ROBOT_LENGTH_NAME     "robot_length"
#define PARAM_ROBOT_LENGTH_DEFAULT  0.2

#define TOPIC_IR_DISTANCES          s8::ir_sensors_node::TOPIC_IR_DISTANCES
#define TOPIC_POSE                  s8::pose_node::TOPIC_POSE_SIMPLE
#define TRESHOLD_VALUE              s8::ir_sensors_node::TRESHOLD_VALUE
                
#define HEADING_NORTH               1
#define HEADING_WEST                2
#define HEADING_SOUTH               3
#define HEADING_EAST                4

#define ACTION_NAVIGATE             "/s8/navigate"

using namespace s8::mapper_node;
using namespace s8::map;
using namespace s8::utils::math;
using s8::ir_sensors_node::is_valid_ir_value;
using s8::pose_node::FrontFacing;
using s8::pose_node::SERVICE_SET_POSITION;

class Mapper : public s8::Node {
    double side_length;
    double resolution;
    ros::Subscriber robot_position_subscriber;
    ros::Subscriber ir_sensors_subscriber;
    ros::Subscriber pose_subscriber;
    double robot_x;
    double robot_y;
    double prev_robot_x;
    double prev_robot_y;
    double robot_width;
    double robot_length;

    double left_back_reading;
    double left_front_reading;
    double right_front_reading;
    double right_back_reading;

    bool render;
    long map_state;
    long map_state_rendered;
    ros::Publisher rviz_publisher;
    ros::Publisher markers_publisher;

    int render_frame_skips;
    const int render_frames_to_skip;

    OccupancyGrid occupancy_grid;
    Topological topological;
    RobotPose robot_pose;
    IRPositions ir_world_positions;
    IRPositions ir_robot_positions;
    IRReadings ir_readings;

    ros::ServiceServer place_node_service;
    ros::ServiceClient set_position_client;

    Navigator navigator;
    bool navigating;

    actionlib::SimpleActionServer<s8_mapper::NavigateAction> navigate_action_server;

public:
    Mapper() : navigating(false), left_back_reading(TRESHOLD_VALUE), left_front_reading(TRESHOLD_VALUE), right_front_reading(TRESHOLD_VALUE), right_back_reading(TRESHOLD_VALUE), render_frame_skips(0), render_frames_to_skip((HZ / RENDER_HZ) - 1), map_state(0), map_state_rendered(-1), robot_x(0.0), robot_y(0.0), prev_robot_x(0.0), prev_robot_y(0.0), robot_pose(0, 0, -90), navigator(&topological, std::bind(&Mapper::go_to_unexplored_place_callback, this, std::placeholders::_1)), navigate_action_server(nh, ACTION_NAVIGATE, boost::bind(&Mapper::action_execute_navigate_callback, this, _1), false) {
        init_params();
        print_params();

        ir_robot_positions.left_back = Coordinate(-0.0725, -0.065);
        ir_robot_positions.left_front = Coordinate(-0.0725, 0.07);
        ir_robot_positions.right_back = Coordinate(0.0825, -0.065);
        ir_robot_positions.right_front = Coordinate(0.0775, 0.07);

        if(render) {
            rviz_publisher = nh.advertise<nav_msgs::OccupancyGrid>(TOPIC_RENDER, 1, true);
            markers_publisher = nh.advertise<visualization_msgs::MarkerArray>(TOPIC_VISUALIZATION_MARKERS, 1, true);
        }

        occupancy_grid = OccupancyGrid(side_length, resolution, ir_robot_positions, robot_width, robot_length, &rviz_publisher);

        set_position_client = nh.serviceClient<s8_pose::setPosition>(SERVICE_SET_POSITION, true);
        ir_sensors_subscriber = nh.subscribe<s8_msgs::IRDistances>(TOPIC_IR_DISTANCES, 1, &Mapper::ir_distances_callback, this);
        pose_subscriber = nh.subscribe<geometry_msgs::Pose2D>(TOPIC_POSE, 1, &Mapper::pose_callback, this);

        place_node_service = nh.advertiseService(SERVICE_PLACE_NODE, &Mapper::place_node_callback, this);

        topological = Topological(0, 0, &set_position_client);

        navigate_action_server.start();
    }

    void update() {
        ir_world_positions.left_front = occupancy_grid.robot_coord_system_to_world_coord_system(ir_robot_positions.left_front);
        ir_world_positions.left_back = occupancy_grid.robot_coord_system_to_world_coord_system(ir_robot_positions.left_back);
        ir_world_positions.right_front = occupancy_grid.robot_coord_system_to_world_coord_system(ir_robot_positions.right_front);
        ir_world_positions.right_back = occupancy_grid.robot_coord_system_to_world_coord_system(ir_robot_positions.right_back);

        occupancy_grid.update(ir_readings, ir_world_positions, robot_pose);

        navigator.update(robot_pose.position.x, robot_pose.position.y, robot_pose.rotation, ir_readings);

        if(should_render() && topological.is_root_initialized()) {
            visualization_msgs::MarkerArray markerArray;

            occupancy_grid.render();
            render_robot(markerArray.markers);
            topological.render(markerArray.markers);

            markers_publisher.publish(markerArray);
        }
    }

    void render_robot(std::vector<visualization_msgs::Marker> & markers) {
        auto add_marker = [&markers, this](MapCoordinate coordinate, float a, float r, float g, float b) {
            visualization_msgs::Marker marker;

            marker.header.frame_id = "map";
            marker.header.stamp = ros::Time();
            marker.ns = "s8";
            marker.id = markers.size();
            marker.type = visualization_msgs::Marker::CUBE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = coordinate.i * resolution - side_length / 2;
            marker.pose.position.y = coordinate.j * resolution - side_length / 2;
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0;
            marker.pose.orientation.y = 0;
            marker.pose.orientation.z = 0;
            marker.pose.orientation.w = 0;
            marker.scale.x = resolution;
            marker.scale.y = resolution;
            marker.scale.z = resolution;
            marker.color.a = a;
            marker.color.r = r;
            marker.color.g = g;
            marker.color.b = b;

            markers.push_back(marker);    
        };

        auto render_sensor = [this, add_marker](Coordinate sensor_world_position, double r, double g, double b) {
            MapCoordinate map_coordinate = occupancy_grid.cartesian_to_grid(sensor_world_position);
            add_marker(map_coordinate, 1.0, r, g, b);
        };

        auto render_robot = [this, add_marker, robot_pose](double r, double g, double b) {
            MapCoordinate mc = occupancy_grid.cartesian_to_grid(robot_pose.position);
            add_marker(mc, 1.0, r, g, b);
        };

//        ROS_INFO("left_front: %s", to_string(ir_world_positions.left_front).c_str());

        render_sensor(ir_world_positions.left_front, 1, 1, 0);
        render_sensor(ir_world_positions.left_back, 1, 0, 1);
        render_sensor(ir_world_positions.right_back, 0, 0, 1);
        render_sensor(ir_world_positions.right_front, 0, 1, 0);

        render_robot(0, 0, 0);
    }

    void save() {
        if(!topological.is_root_initialized()) {
            return;
        }

        std::string home = ::getenv("HOME");
        std::string filename = home + "/maps/map_" + std::to_string(time(NULL)) + ".json";

        ROS_INFO("Saving topological map to file: %s", filename.c_str());

        topological.save_to_file(filename);

        ROS_INFO("Done");
    }

private:
    void action_execute_navigate_callback(const s8_mapper::NavigateGoalConstPtr & navigate_goal) {
        ROS_INFO("Starting navigating");
        navigator.go_to_object_place();

        navigating = true;
        ros::Rate rate(10);
        int ticks = 0;
        while(ros::ok() && navigating) {
            rate.sleep();
            ros::spinOnce();
        }

        s8_mapper::NavigateResult navigate_action_result;
        navigate_action_result.reason = 0;
        ROS_INFO("PREEMPTED: Navigate action succeded.");
        navigate_action_server.setSucceeded(navigate_action_result);
    }

    void go_to_unexplored_place_callback(GoToUnexploredResult result) {
        ROS_INFO("Callback %d", result);
        navigating = false;
    }

    bool place_node_callback(s8_mapper::PlaceNode::Request& request, s8_mapper::PlaceNode::Response& response) {
        ROS_INFO("Placing node %lf %lf", request.x, request.y);

        int current_heading = get_current_heading();
        ROS_INFO("Current Heading: %d", current_heading );
        bool isWallNorth = is_wall_north(current_heading, request.isWallLeft, request.isWallForward, request.isWallRight);
        bool isWallWest = is_wall_west(current_heading, request.isWallLeft, request.isWallForward, request.isWallRight);
        bool isWallSouth = is_wall_south(current_heading, request.isWallLeft, request.isWallForward, request.isWallRight);
        bool isWallEast = is_wall_east(current_heading, request.isWallLeft, request.isWallForward, request.isWallRight);
        double related_x = std::cos(degrees_to_radians(robot_pose.rotation)-request.theta)*(request.dist);
        double related_y = std::sin(degrees_to_radians(robot_pose.rotation)-request.theta)*(request.dist);

        response.placed = topological.add_node(robot_pose.position.x+related_x, robot_pose.position.y+related_y, request.value, isWallNorth, isWallWest,isWallSouth,isWallEast);
        return true;
    }

    void ir_distances_callback(const s8_msgs::IRDistances::ConstPtr & ir_distances) {
        ir_readings.left_back = ir_distances->left_back;
        ir_readings.left_front = ir_distances->left_front;
        ir_readings.right_back = ir_distances->right_back;
        ir_readings.right_front = ir_distances->right_front;
        ir_readings.front_left = ir_distances->front_left;
        ir_readings.front_right = ir_distances->front_right;
    }

    void pose_callback(const geometry_msgs::Pose2D::ConstPtr & pose) {
        robot_pose.position.x = pose->x;
        robot_pose.position.y = pose->y;
        robot_pose.rotation = radians_to_degrees(pose->theta);
        
        // Make sure that rotation stays in the range [0, 360]
        if (robot_pose.rotation > 360){
            int toRange = floor(robot_pose.rotation/360);
            robot_pose.rotation = robot_pose.rotation - toRange*360; 
        }
        else if (robot_pose.rotation < 0){
            int toRange = ceil(robot_pose.rotation/360);
            robot_pose.rotation = robot_pose.rotation + (toRange+1)*360;    
        }
    }

    int get_current_heading(){
        ROS_INFO("REAL HEADING: %lf", robot_pose.rotation);
        if (robot_pose.rotation >= 315 || robot_pose.rotation < 45){
            ROS_INFO("HEADING EAST");
            return HEADING_EAST;
        }
        else if (robot_pose.rotation >= 45 && robot_pose.rotation < 135){
            ROS_INFO("HEADING NORTH");
            return HEADING_NORTH;
        }
        else if (robot_pose.rotation >= 135 && robot_pose.rotation < 225){
            ROS_INFO("HEADING WEST");
            return HEADING_WEST;
        }
        else{
            ROS_INFO("HEADING SOUTH");
            return HEADING_SOUTH;
        }
    }

    bool is_wall_north(int heading, bool isWallLeft, bool isWallForward, bool isWallRight){
        if ( (heading == HEADING_NORTH && isWallForward == true) || (heading == HEADING_WEST && isWallRight == true) || (heading == HEADING_EAST && isWallLeft == true) ){
            return true;
        }
        else{
            return false;
        }
    }

    bool is_wall_west(int heading, bool isWallLeft, bool isWallForward, bool isWallRight){
        if ( (heading == HEADING_WEST && isWallForward==true) || (heading == HEADING_SOUTH && isWallRight==true) || (heading == HEADING_NORTH && isWallLeft==true) ){
            return true;
        }
        else{
            return false;
        }
    }

    bool is_wall_south(int heading, bool isWallLeft, bool isWallForward, bool isWallRight){
        if ( (heading == HEADING_SOUTH && isWallForward==true) || (heading == HEADING_EAST && isWallRight==true) || (heading == HEADING_WEST && isWallLeft==true) ){
            return true;
        }
        else{
            return false;
        }
    }

    bool is_wall_east(int heading, bool isWallLeft, bool isWallForward, bool isWallRight){
        if ( (heading == HEADING_EAST && isWallForward==true) || (heading == HEADING_NORTH && isWallRight==true) || (heading == HEADING_SOUTH && isWallLeft==true) ){
            return true;
        }
        else{
            return false;
        }
    }

    bool should_render() {
        if(!render) {
            return false;
        }

        if(render_frame_skips == render_frames_to_skip) {
            render_frame_skips = 0;
            return true;
        }

        render_frame_skips++;

        return false;
    }

    void init_params() {
        add_param(PARAM_RENDER_NAME, render, PARAM_RENDER_DEFAULT);
        add_param(PARAM_SIDE_LENGTH_NAME, side_length, PARAM_SIDE_LENGTH_DEFAULT);
        add_param(PARAM_RESOLUTION_NAME, resolution, PARAM_RESOLUTION_DEFAULT);
        add_param(PARAM_ROBOT_WIDTH_NAME, robot_width, PARAM_ROBOT_WIDTH_DEFAULT);
        add_param(PARAM_ROBOT_LENGTH_NAME, robot_length, PARAM_ROBOT_LENGTH_DEFAULT);
    }
};

Mapper * mapper_ptr = NULL;

void mySigintHandler(int sig) {
    ROS_INFO("Quitting!");

    if(mapper_ptr != NULL) {
        mapper_ptr->save();
    }

    ros::shutdown();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, NODE_NAME);

    Mapper mapper;
    mapper_ptr = &mapper;
    ros::Rate loop_rate(HZ);

    // Override the default ros sigint handler.
    // This must be set after the first NodeHandle is created.
    signal(SIGINT, mySigintHandler);

    while(ros::ok()) {
        mapper.update();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
