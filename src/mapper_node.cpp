#include <ros/ros.h>
#include <s8_common_node/Node.h>
#include <s8_mapper/mapper_node.h>
#include <s8_ir_sensors/ir_sensors_node.h>
#include <s8_utils/math.h>
#include <s8_pose/pose_node.h>
#include <Map.h>
#include <unordered_set>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <s8_msgs/IRDistances.h>

#define HZ                          10
#define RENDER_HZ                   1

#define PARAM_SIDE_LENGTH_NAME      "side_length"
#define PARAM_SIDE_LENGTH_DEFAULT   1.0
#define PARAM_RESOLUTION_NAME       "resolution"
#define PARAM_RESOLUTION_DEFAULT    0.02
#define PARAM_RENDER_NAME           "render"
#define PARAM_RENDER_DEFAULT        true

#define TOPIC_IR_DISTANCES          s8::ir_sensors_node::TOPIC_IR_DISTANCES
#define TOPIC_POSE                  s8::pose_node::TOPIC_POSE_SIMPLE
#define TRESHOLD_VALUE              s8::ir_sensors_node::TRESHOLD_VALUE

using namespace s8::mapper_node;
using namespace s8::map;
using namespace s8::utils::math;
using s8::ir_sensors_node::is_valid_ir_value;
using s8::pose_node::FrontFacing;


struct Coordinate {
    double x;
    double y;

    Coordinate() : x(0.0), y(0.0) {}
    Coordinate(double x, double y) : x(x), y(y) {}

    bool operator== (Coordinate coordinate) {
        return is_zero(x - coordinate.x) && is_zero(y - coordinate.y);
    }
};

std::string to_string(Coordinate coordinate) {
    return "(" + std::to_string(coordinate.x) + ", " + std::to_string(coordinate.y) + ")";
}

class Mapper : public s8::Node {
    double side_length;
    double resolution;
    Map map;
    ros::Subscriber robot_position_subscriber;
    ros::Subscriber ir_sensors_subscriber;
    ros::Subscriber pose_subscriber;
    int robot_rotation;
    double robot_x;
    double robot_y;
    double prev_robot_x;
    double prev_robot_y;
    size_t prev_robot_i;
    size_t prev_robot_j;
    size_t robot_i;
    size_t robot_j;

    Coordinate left_back_position;
    Coordinate left_front_position;
    Coordinate right_back_position;
    Coordinate right_front_position;
    double left_back_reading;
    double left_front_reading;
    double right_front_reading;
    double right_back_reading;

    bool render;
    long map_state;
    long map_state_rendered;
    ros::Publisher rviz_markers_publisher;

    int render_frame_skips;
    const int render_frames_to_skip;

public:
    Mapper() : left_back_reading(TRESHOLD_VALUE), left_front_reading(TRESHOLD_VALUE), right_front_reading(TRESHOLD_VALUE), right_back_reading(TRESHOLD_VALUE), render_frame_skips(0), render_frames_to_skip((HZ / RENDER_HZ) - 1), robot_rotation(0), map_state(0), map_state_rendered(-1), robot_x(0.0), robot_y(0.0), prev_robot_x(0.0), prev_robot_y(0.0) {
        init_params();
        print_params();

        left_back_position = Coordinate(-0.045, -0.075);
        left_front_position = Coordinate(-0.045, 0.075);
        right_back_position = Coordinate(0.045, -0.075);
        right_front_position = Coordinate(0.045, 0.075);

        double side_cells_d = side_length / resolution;
        size_t side_cells = (size_t)side_cells_d;
        double loss = side_cells_d - side_cells;

        if(loss >= resolution) {
            ROS_WARN("Losing cells with current side_length and resolution. Cells lost on each side: %d", (int)(loss / resolution));
        }

        map = Map(side_cells, side_cells, CELL_UNKNOWN);
        ROS_INFO("map size: %ldx%ld", map.num_rows(), map.num_cols());

        prev_robot_i = robot_i = map.row_relative_origo(0);
        prev_robot_j = robot_j = map.col_relative_origo(0);

        if(render) {
            rviz_markers_publisher = nh.advertise<visualization_msgs::MarkerArray>(TOPIC_VISUALIZATION_MARKERS, 1, true);
        }

        robot_position_subscriber = nh.subscribe<geometry_msgs::Point>(TOPIC_ROBOT_POSITION, 1, &Mapper::robot_position_callback, this);
        ir_sensors_subscriber = nh.subscribe<s8_msgs::IRDistances>(TOPIC_IR_DISTANCES, 1, &Mapper::ir_distances_callback, this);
        pose_subscriber = nh.subscribe<geometry_msgs::Pose2D>(TOPIC_POSE, 1, &Mapper::pose_callback, this);
    }

    void update() {
        auto sensor_reading = [this](Coordinate sensor_position, double reading, int dir) {
            if(is_valid_ir_value(reading)) {
                Coordinate obstacle_relative_robot_position = Coordinate(sensor_position.x + dir * reading, sensor_position.y);
                Coordinate obstacle_world_position = robot_coord_system_to_world_coord_system(obstacle_relative_robot_position);
                MapCoordinate map_position = cartesian_to_grid(obstacle_world_position);

                auto cells = get_cells_touching_line(robot_coord_system_to_world_coord_system(sensor_position), obstacle_world_position);
                for(auto mc : cells) {
                    map[mc] = CELL_FREE;
                }

                map[map_position] = CELL_OBSTACLE;
            }
        };

        sensor_reading(left_back_position, left_back_reading, -1);
        sensor_reading(left_front_position, left_front_reading, -1);
        sensor_reading(right_back_position, right_back_reading, 1);
        sensor_reading(right_front_position, right_front_reading, 1);


        if(should_render()) {
            ROS_INFO("Rendering to rviz");
            renderToRviz();
        }
    }

    void renderToRviz() {
        visualization_msgs::MarkerArray markerArray;
        markerArray.markers = std::vector<visualization_msgs::Marker>((map.num_cells()));
        auto & markers = markerArray.markers;

        if(true || map_state != map_state_rendered) {

            for(size_t i = 0; i < map.num_rows(); i++) {
                for(size_t j = 0; j < map.num_cols(); j++) {
                    visualization_msgs::Marker & marker = markers[i * map.num_cols() + j];

                    marker.header.frame_id = "map";
                    marker.header.stamp = ros::Time();
                    marker.ns = "s8";
                    marker.id = i * map.num_cols() + j;
                    marker.type = visualization_msgs::Marker::CUBE;
                    marker.action = visualization_msgs::Marker::ADD;
                    marker.pose.position.x = i * resolution - side_length / 2;
                    marker.pose.position.y = j * resolution - side_length / 2;
                    marker.pose.position.z = 0;
                    marker.pose.orientation.x = 0;
                    marker.pose.orientation.y = 0;
                    marker.pose.orientation.z = 0;
                    marker.pose.orientation.w = 0;
                    marker.scale.x = resolution;
                    marker.scale.y = resolution;
                    marker.scale.z = resolution;

                    if(is_point_obstacle(i, j)) {
                        marker.color.a = 1.0;
                        marker.color.r = 1.0;
                        marker.color.g = 0.0;
                        marker.color.b = 0.0;
                    } else if(is_point_wall(i, j)) {

                    } else if(is_point_free(i, j)) {
                        marker.color.a = 0.0;
                        marker.color.r = 1.0;
                        marker.color.g = 1.0;
                        marker.color.b = 1.0;
                    } else if(is_point_object(i, j)) {

                    } else if(is_point_unknown(i, j)) {
                        marker.color.a = 0.5;
                        marker.color.r = 1.0;
                        marker.color.g = 1.0;
                        marker.color.b = 1.0;
                    } else {
                        ROS_WARN("Unknown cell type: %d at (%ld,%ld)", map[i][j], i, j);
                    }
                }
            }

            ROS_INFO("Sending recomputed map to be rendered");
        } else {
            ROS_INFO("Sending cached map to be rendered");
        }

        auto render_point = [&markers, this](MapCoordinate map_coordinate, double r, double g, double b) {
            visualization_msgs::Marker & marker = markers[map_coordinate.i * map.num_cols() + map_coordinate.j];
            marker.color.a = 1.0;
            marker.color.r = r;
            marker.color.g = g;
            marker.color.b = b;
        };

        auto render_sensor = [&markers, this, render_point](Coordinate sensor_position, double r, double g, double b) {
            Coordinate world_position = robot_coord_system_to_world_coord_system(sensor_position);
            ROS_INFO("world: x: %lf, y: %lf", world_position.x, world_position.y);
            MapCoordinate map_coordinate = cartesian_to_grid(world_position);
            render_point(map_coordinate, r, g, b);
        };

        auto render_sensor_real = [&markers, this, render_point](Coordinate sensor_position, double r, double g, double b) {
            //Hypotenuse of sensor relative origo of robot
            double h_s = std::sqrt(sensor_position.x * sensor_position.x + sensor_position.y * sensor_position.y);

            //Angle of sensor hypotenuse relative robot forward normal
            double theta_s = std::asin(sensor_position.y / h_s);

            //Angle of sensor relative world coordinate system
            double theta = degrees_to_radians(robot_rotation) + theta_s;

            double x = h_s * std::cos(theta);
            double y = h_s * std::sin(theta);

            ROS_INFO("sx: %lf sy: %lf, h_s: %lf, theta_s: %lf, theta: %lf, x: %lf, y: %lf", sensor_position.x, sensor_position.y, h_s, theta_s, theta, x, y);

            Coordinate position_world_coord_system = robot_coord_system_to_world_coord_system(Coordinate(x, y));
            ROS_INFO("world: x: %lf, y: %lf", position_world_coord_system.x, position_world_coord_system.y);

            render_point(cartesian_to_grid(position_world_coord_system), r, g, b);
        };

        auto render_robot = [this, render_point](Coordinate robot_position, double r, double g, double b) {
            MapCoordinate mc = cartesian_to_grid(robot_position);
            render_point(mc, r, g, b);
        };


        //render_sensors(left_front_position, 0, 1, 0);
        //render_sensors(left_front_position, right_front_position, 0, 1, 0);

        render_sensor(left_front_position, 1, 1, 0);
        render_sensor(left_back_position, 1, 0, 1);
        render_sensor(right_back_position, 0, 0, 1);
        render_sensor(right_front_position, 0, 1, 0);

        render_robot(Coordinate(robot_x, robot_y), 0, 0, 0);

        rviz_markers_publisher.publish(markerArray);

        map_state_rendered = map_state;
    }

    bool is_point_value(size_t row_index, size_t col_index, int value) {
        return (map[row_index][col_index] | value) == value;
    }

    bool is_point_obstacle(size_t row_index, size_t col_index) {
        return is_point_value(row_index, col_index, CELL_OBSTACLE);
    }

    bool is_point_wall(size_t row_index, size_t col_index) {
        return is_point_value(row_index, col_index, CELL_WALL);
    }

    bool is_point_free(size_t row_index, size_t col_index) {
        return is_point_value(row_index, col_index, CELL_FREE);
    }

    bool is_point_object(size_t row_index, size_t col_index) {
        return is_point_value(row_index, col_index, CELL_OBJECT);
    }

    bool is_point_unknown(size_t row_index, size_t col_index) {
        return is_point_value(row_index, col_index, CELL_UNKNOWN);
    }

    void set_point(size_t row_index, size_t col_index, int value) {
        map[row_index][col_index] = value;
        map_state++;
    }

    void set_obstacle(size_t row_index, size_t col_index) {
        set_point(row_index, col_index, CELL_OBSTACLE);
    }

    void set_wall(size_t row_index, size_t col_index) {
        set_point(row_index, col_index, CELL_WALL);
    }

    void set_free(size_t row_index, size_t col_index) {
        set_point(row_index, col_index, CELL_FREE);
    }

    void set_object(size_t row_index, size_t col_index) {
        set_point(row_index, col_index, CELL_OBJECT);
    }

private:
    void robot_position_callback(const geometry_msgs::Point::ConstPtr & point) {
        robot_x = point->x;
        robot_y = point->z;

        if(!is_zero(point->y)) {
            ROS_WARN("Y part of robot Point position is not used. Only X and Z!");
        }

        int movement_i = (robot_x - prev_robot_x) / resolution;
        int movement_j = (robot_y - prev_robot_y) / resolution;

        robot_i = prev_robot_i + movement_i;
        robot_j = prev_robot_j + movement_j;

        ROS_INFO("Robot movement: (%ld,%ld) -> (%ld,%ld)", prev_robot_i, prev_robot_j, robot_i, robot_j);

        set_free(robot_i, robot_j);

        prev_robot_i = robot_i;
        prev_robot_j = robot_j;
    }

    void ir_distances_callback(const s8_msgs::IRDistances::ConstPtr & ir_distances) {
        left_back_reading = ir_distances->left_back;
        left_front_reading = ir_distances->left_front;
        right_back_reading = ir_distances->right_back;
        right_front_reading = ir_distances->right_front;
    }

    void pose_callback(const geometry_msgs::Pose2D::ConstPtr & pose) {
        robot_x = pose->x;
        robot_y = pose->y;
        robot_rotation = radians_to_degrees(pose->theta);
        ROS_INFO("%lf %d", pose->theta, radians_to_degrees(pose->theta));
    }

    Coordinate get_sensor_cartesian_position(Coordinate sensor_relative_position) {
        ROS_INFO("robot_ration: %d", robot_rotation);

        const int NORTH = 90;
        const int SOUTH = 270;
        const int EAST = 0;
        const int WEST = 180;

        if(robot_rotation == EAST) {
            return Coordinate(robot_x + sensor_relative_position.x, robot_y + sensor_relative_position.y);
        } else if(robot_rotation == SOUTH) {
            return Coordinate(robot_x - sensor_relative_position.x, robot_y - sensor_relative_position.y);
        }
    }

    MapCoordinate cartesian_to_grid(Coordinate position) {
        int map_x = (position.x / resolution) + sign(position.x)*0.99;
        int map_y = (position.y / resolution) + sign(position.y)*0.99;
        int x = map.row_relative_origo(map_x);
        int y = map.col_relative_origo(map_y);

        ROS_INFO("resolution: %lf, x: %lf, y: %lf, map_x: %d map_y: %d fx: %d fy: %d", resolution, position.x, position.y, map_x, map_y, x, y);
        return MapCoordinate(x, y);
    }

    Coordinate robot_coord_system_to_world_coord_system(Coordinate coordinate) {
        double theta = degrees_to_radians(robot_rotation - 90);
        double x = coordinate.x;
        double y = coordinate.y;
        double xc = robot_x;
        double yc = robot_y;

        double xr = (x - xc) * std::cos(theta) - (y - yc) * std::sin(theta) + xc;
        double yr = (y - yc) * std::cos(theta) + (x - xc) * std::sin(theta) + yc;

        return Coordinate(xr, yr);
    }

    std::unordered_set<MapCoordinate, MapCoordinateHash> get_cells_touching_line(Coordinate p1, Coordinate p2) {
        double step_size = resolution / 2;

        std::unordered_set<MapCoordinate, MapCoordinateHash> result;

        if(!is_zero(p2.x - p1.x) && !is_zero(p2.y - p1.y)) {
            double m = (p2.y - p1.y) / (p2.x - p1.x);
            double b = p1.y - m * p1.x;
            auto line = [m, b](double x) {
                return x * m + b;
            };

            double start_x = p1.x < p2.x ? p1.x : p2.x;
            double end_x = p1.x < p2.x ? p2.x : p1.x;

            for(double x = start_x; x <= end_x; x += step_size) {
                double y = line(x);
                MapCoordinate mc = cartesian_to_grid(Coordinate(x, y));
                result.insert(mc);
            }
        } else {
            if(is_zero(p1.x - p2.x)) {
                //Same x point. Step through y
                double start_y = p1.y < p2.y ? p1.y : p2.y;
                double end_y = p1.y < p2.y ? p2.y : p1.y;
                for(double y = start_y; y <= end_y; y += step_size) {
                    MapCoordinate mc = cartesian_to_grid(Coordinate(p1.x, y));
                    result.insert(mc);
                }
            } else if(is_zero(p1.y - p2.y)) {
                //Same y point. Step through x
                double start_x = p1.x < p2.x ? p1.x : p2.x;
                double end_x = p1.x < p2.x ? p2.x : p1.x;
                for(double x = start_x; x <= end_x; x += step_size) {
                    MapCoordinate mc = cartesian_to_grid(Coordinate(x, p1.y));
                    result.insert(mc);
                }
            } else {
                ROS_FATAL("WTF");
            }
        }
        
        return result;
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
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, NODE_NAME);

    Mapper mapper;
    ros::Rate loop_rate(HZ);

    while(ros::ok()) {
        mapper.update();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
