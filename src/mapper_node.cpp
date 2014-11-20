#include <ros/ros.h>
#include <s8_common_node/Node.h>
#include <s8_mapper/mapper_node.h>
#include <s8_ir_sensors/ir_sensors_node.h>
#include <s8_utils/math.h>
#include <s8_pose/pose_node.h>
#include <s8_mapper/Map.h>
#include <unordered_set>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose2D.h>
#include <s8_msgs/IRDistances.h>

#define HZ                          10
#define RENDER_HZ                   10

#define PARAM_SIDE_LENGTH_NAME      "side_length"
#define PARAM_SIDE_LENGTH_DEFAULT   1.0
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
    double robot_width;
    double robot_length;

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
    ros::Publisher rviz_publisher;
    ros::Publisher rviz_markers_publisher;

    int render_frame_skips;
    const int render_frames_to_skip;

public:
    Mapper() : left_back_reading(TRESHOLD_VALUE), left_front_reading(TRESHOLD_VALUE), right_front_reading(TRESHOLD_VALUE), right_back_reading(TRESHOLD_VALUE), render_frame_skips(0), render_frames_to_skip((HZ / RENDER_HZ) - 1), robot_rotation(0), map_state(0), map_state_rendered(-1), robot_x(0.0), robot_y(0.0), prev_robot_x(0.0), prev_robot_y(0.0) {
        init_params();
        print_params();

        left_back_position = Coordinate(-0.0725, -0.065);
        left_front_position = Coordinate(-0.0725, 0.07);
        right_back_position = Coordinate(0.0825, -0.065);
        right_front_position = Coordinate(0.0775, 0.07);

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
            rviz_publisher = nh.advertise<nav_msgs::OccupancyGrid>(TOPIC_RENDER, 1, true);
            rviz_markers_publisher = nh.advertise<visualization_msgs::MarkerArray>(TOPIC_VISUALIZATION_MARKERS, 1, true);
        }

        robot_position_subscriber = nh.subscribe<geometry_msgs::Point>(TOPIC_ROBOT_POSITION, 1, &Mapper::robot_position_callback, this);
        ir_sensors_subscriber = nh.subscribe<s8_msgs::IRDistances>(TOPIC_IR_DISTANCES, 1, &Mapper::ir_distances_callback, this);
        pose_subscriber = nh.subscribe<geometry_msgs::Pose2D>(TOPIC_POSE, 1, &Mapper::pose_callback, this);
    }

    void update() {
        auto free_cell = [](int & cell) {
            if(cell == CELL_UNKNOWN) {
                cell = 50;
            } else {
                cell -= 5;
            }

            if(cell < 0) {
                cell = 0;
            }
        };

        auto obstacle_cell = [](int & cell) {
            if(cell == CELL_UNKNOWN) {
                cell = 50;
            } else {
                cell += 10;
            }

            if(cell > 100) {
                cell = 100;
            }
        };

        auto sensor_reading = [this, obstacle_cell, free_cell](Coordinate sensor_position, double reading, int dir) {
            if(is_valid_ir_value(reading) && reading <= 0.2) {
                Coordinate obstacle_relative_robot_position = Coordinate(sensor_position.x + dir * reading, sensor_position.y);
                Coordinate obstacle_world_position = robot_coord_system_to_world_coord_system(obstacle_relative_robot_position);
                MapCoordinate map_position = cartesian_to_grid(obstacle_world_position);

                auto cells = get_cells_touching_line(robot_coord_system_to_world_coord_system(sensor_position), obstacle_world_position);
                for(auto mc : cells) {
                    free_cell(map[mc]);
                }

                obstacle_cell(map[map_position]);
            }
        };

        sensor_reading(left_back_position, left_back_reading, -1);
        sensor_reading(left_front_position, left_front_reading, -1);
        sensor_reading(right_back_position, right_back_reading, 1);
        sensor_reading(right_front_position, right_front_reading, 1);

        //Mark the area where to robot body is as free.
        Coordinate left_front = robot_coord_system_to_world_coord_system(Coordinate(-robot_width / 2, robot_length / 2));
        Coordinate left_back = robot_coord_system_to_world_coord_system(Coordinate(-robot_width / 2, -robot_length / 2));
        Coordinate right_front = robot_coord_system_to_world_coord_system(Coordinate(robot_width / 2, robot_length / 2));
        Coordinate right_back = robot_coord_system_to_world_coord_system(Coordinate(robot_width / 2, -robot_length / 2));

        auto free_between = [this, free_cell](Coordinate coordinate1, Coordinate coordinate2) {
            auto cells = get_cells_touching_line(coordinate1, coordinate2);
            for(auto mc : cells) {
                free_cell(map[mc]);
            }
        };

        free_between(left_front, right_front);
        free_between(right_front, right_back);
        free_between(right_back, left_back);
        free_between(left_back, left_front);
        
        if(should_render()) {
            renderToRviz();
        }
    }

    void renderToRviz() {
        nav_msgs::OccupancyGrid grid_msg;
        grid_msg.header.frame_id = "map";
        grid_msg.header.stamp = ros::Time();
        grid_msg.info.resolution = resolution;
        grid_msg.info.width = map.num_cols();
        grid_msg.info.height = map.num_rows();
        grid_msg.info.origin.position.x = -(float)map.get_origo_col() * resolution;
        grid_msg.info.origin.position.y = -(float)map.get_origo_row() * resolution;
        grid_msg.data = std::vector<signed char>(map.num_cells());
        auto & grid = grid_msg.data;

        for(size_t i = 0; i < map.num_rows(); i++) {
            for(size_t j = 0; j < map.num_cols(); j++) {
                auto & cell = grid[j * map.num_rows() + i];
                int value = map[i][j];
                if(value < 0) {
                    cell = -1;
                } else if(value > 50) {
                    cell = 100;
                } else if(value < 50) {
                    cell = 0;
                } else {
                    cell = -1;
                }
                /*if(is_point_obstacle(i, j)) {
                    cell = 100;
                } else if(is_point_wall(i, j)) {
                    cell = 100;
                } else if(is_point_free(i, j)) {
                    cell = 0;
                } else if(is_point_object(i, j)) {
                    cell = 100;
                } else if(is_point_unknown(i, j)) {
                    cell = -1;
                } else {
                    ROS_WARN("Unknown cell type: %d at (%ld,%ld)", map[i][j], i, j);
                }*/
            }
        }

        rviz_publisher.publish(grid_msg);

        visualization_msgs::MarkerArray markerArray;
        markerArray.markers = std::vector<visualization_msgs::Marker>();
        auto & markers = markerArray.markers;

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

        auto render_sensor = [this, add_marker](Coordinate sensor_position, double r, double g, double b) {
            Coordinate world_position = robot_coord_system_to_world_coord_system(sensor_position);
            MapCoordinate map_coordinate = cartesian_to_grid(world_position);
            add_marker(map_coordinate, 1.0, r, g, b);
        };

        auto render_sensor_real = [this, add_marker](Coordinate sensor_position, double r, double g, double b) {
            //Hypotenuse of sensor relative origo of robot
            double h_s = std::sqrt(sensor_position.x * sensor_position.x + sensor_position.y * sensor_position.y);

            //Angle of sensor hypotenuse relative robot forward normal
            double theta_s = std::asin(sensor_position.y / h_s);

            //Angle of sensor relative world coordinate system
            double theta = degrees_to_radians(robot_rotation) + theta_s;

            double x = h_s * std::cos(theta);
            double y = h_s * std::sin(theta);

            Coordinate position_world_coord_system = robot_coord_system_to_world_coord_system(Coordinate(x, y));

            add_marker(cartesian_to_grid(position_world_coord_system), 1.0, r, g, b);
        };

        auto render_robot = [this, add_marker](Coordinate robot_position, double r, double g, double b) {
            MapCoordinate mc = cartesian_to_grid(robot_position);
            add_marker(mc, 1.0, r, g, b);
        };


        //render_sensors(left_front_position, 0, 1, 0);
        //render_sensors(left_front_position, right_front_position, 0, 1, 0);

        render_sensor(left_front_position, 1, 1, 0);
        render_sensor(left_back_position, 1, 0, 1);
        render_sensor(right_back_position, 0, 0, 1);
        render_sensor(right_front_position, 0, 1, 0);

        render_robot(Coordinate(robot_x, robot_y), 0, 0, 0);

        rviz_markers_publisher.publish(markerArray);
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
        prev_robot_x = robot_x;
        prev_robot_y = robot_y;
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
    }

    MapCoordinate cartesian_to_grid(Coordinate position) {
        int map_x = (position.x / resolution) + sign(position.x)*0.5;
        int map_y = (position.y / resolution) + sign(position.y)*0.5;
        int x = map.row_relative_origo(map_x);
        int y = map.col_relative_origo(map_y);

        return MapCoordinate(x, y);
    }

    Coordinate robot_coord_system_to_world_coord_system(Coordinate coordinate) {
        double theta = degrees_to_radians(robot_rotation - 90);
        double x = coordinate.x;
        double y = coordinate.y;
        double xc = prev_robot_x;
        double yc = prev_robot_y;

        double xr = (x - xc) * std::cos(theta) - (y - yc) * std::sin(theta) + robot_x;
        double yr = (y - yc) * std::cos(theta) + (x - xc) * std::sin(theta) + robot_y;

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

        result.insert(cartesian_to_grid(Coordinate(p1.x, p1.y)));
        result.insert(cartesian_to_grid(Coordinate(p2.x, p2.y)));
        
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
        add_param(PARAM_ROBOT_WIDTH_NAME, robot_width, PARAM_ROBOT_WIDTH_DEFAULT);
        add_param(PARAM_ROBOT_LENGTH_NAME, robot_length, PARAM_ROBOT_LENGTH_DEFAULT);
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
