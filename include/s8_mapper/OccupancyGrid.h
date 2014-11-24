#ifndef __OCCUPANCY_GRID_H
#define __OCCUPANCY_GRID_H

#include <s8_mapper/mapper_node.h>
#include <s8_mapper/Coordinate.h>
#include <s8_mapper/Map.h>
#include <unordered_set>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

const int CELL_UNKNOWN =                            -1;
const int CELL_FREE =                               1 << 2;
const int CELL_OBSTACLE =                           1 << 3;
const int CELL_WALL =                               1 << 4 & CELL_OBSTACLE;
const int CELL_OBJECT =                             1 << 5 & CELL_OBSTACLE;

using s8::map::MapCoordinate;
using s8::map::Map;
using s8::map::MapCoordinateHash;
using s8::Coordinate;
using s8::ir_sensors_node::is_valid_ir_value;
using namespace s8::mapper_node;

class OccupancyGrid {
    double side_length;
    double resolution;
    double robot_width;
    double robot_length;
    Map map;
    ros::Publisher * occupancy_grid_publisher;
    RobotPose robot_pose;
    RobotPose prev_robot_pose;
    IRPositions ir_robot_positions;


public:
    OccupancyGrid() : side_length(0), resolution(0), ir_robot_positions(), robot_width(0), robot_length(0), occupancy_grid_publisher(NULL) {}

    OccupancyGrid(double side_length, double resolution, IRPositions ir_robot_positions, double robot_width, double robot_length, ros::Publisher * occupancy_grid_publisher) : side_length(side_length), resolution(resolution), ir_robot_positions(ir_robot_positions), robot_width(robot_width), robot_length(robot_length), occupancy_grid_publisher(occupancy_grid_publisher) {
        double side_cells_d = side_length / resolution;
        size_t side_cells = (size_t)side_cells_d;
        double loss = side_cells_d - side_cells;
        map = Map(side_cells, side_cells, CELL_UNKNOWN);

        if(loss >= resolution) {
            ROS_WARN("Losing cells with current side_length and resolution. Cells lost on each side: %d", (int)(loss / resolution));
        }
    }

    void update(IRReadings ir_readings, IRPositions ir_world_positions, RobotPose new_robot_pose) {
        prev_robot_pose = robot_pose;
        robot_pose = new_robot_pose;

        ROS_INFO("Old pose: (%lf %lf %d) New Pose: (%lf %lf %d)", prev_robot_pose.position.x, prev_robot_pose.position.y, prev_robot_pose.rotation, robot_pose.position.x, robot_pose.position.y, robot_pose.rotation);

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
            ROS_INFO("%lf", reading);
            if(is_valid_ir_value(reading) && reading <= 0.2) {
                Coordinate obstacle_robot_position = Coordinate(sensor_position.x + dir * reading, sensor_position.y);
                Coordinate obstacle_world_position = robot_coord_system_to_world_coord_system(obstacle_robot_position);
                MapCoordinate map_position = cartesian_to_grid(obstacle_world_position);

                auto cells = get_cells_touching_line(robot_coord_system_to_world_coord_system(sensor_position), obstacle_world_position);
                for(auto mc : cells) {
                    free_cell(map[mc]);
                }

                obstacle_cell(map[map_position]);
            }
        };

        sensor_reading(ir_robot_positions.left_back, ir_readings.left_back, -1);
        sensor_reading(ir_robot_positions.left_front, ir_readings.left_front, -1);
        sensor_reading(ir_robot_positions.right_back, ir_readings.right_back, 1);
        sensor_reading(ir_robot_positions.right_front, ir_readings.right_front, 1);

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
    }

    void render() {
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

        occupancy_grid_publisher->publish(grid_msg);
    }

    Coordinate robot_coord_system_to_world_coord_system(Coordinate coordinate) {
        double theta = degrees_to_radians(robot_pose.rotation - 90);
        double x = coordinate.x;
        double y = coordinate.y;
        double xc = 0; //prev_robot_pose.position.x;
        double yc = 0; //prev_robot_pose.position.y;

        double xr = (x - xc) * std::cos(theta) - (y - yc) * std::sin(theta) + robot_pose.position.x;
        double yr = (y - yc) * std::cos(theta) + (x - xc) * std::sin(theta) + robot_pose.position.y;

        //ROS_INFO("%s -> %s   xc: %lf yc: %lf", to_string(coordinate).c_str(), to_string(Coordinate(xr, yr)).c_str(), xc, yc, robot_pose.position.x, robot_pose.position.y);

        return Coordinate(xr, yr);
    }

    MapCoordinate cartesian_to_grid(Coordinate position) {
        int map_x = (position.x / resolution) + sign(position.x)*0.5;
        int map_y = (position.y / resolution) + sign(position.y)*0.5;
        int x = map.row_relative_origo(map_x);
        int y = map.col_relative_origo(map_y);

        return MapCoordinate(x, y);
    }

    std::unordered_set<MapCoordinate, MapCoordinateHash> get_cells_touching_line(Coordinate p1, Coordinate p2) {
        double step_size = resolution / 2;

        std::unordered_set<MapCoordinate, MapCoordinateHash> result;

        if(!s8::utils::math::is_zero(p2.x - p1.x) && !s8::utils::math::is_zero(p2.y - p1.y)) {
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
            if(s8::utils::math::is_zero(p1.x - p2.x)) {
                //Same x point. Step through y
                double start_y = p1.y < p2.y ? p1.y : p2.y;
                double end_y = p1.y < p2.y ? p2.y : p1.y;
                for(double y = start_y; y <= end_y; y += step_size) {
                    MapCoordinate mc = cartesian_to_grid(Coordinate(p1.x, y));
                    result.insert(mc);
                }
            } else if(s8::utils::math::is_zero(p1.y - p2.y)) {
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
};

#endif
