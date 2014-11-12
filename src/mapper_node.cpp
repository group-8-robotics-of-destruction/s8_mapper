#include <ros/ros.h>
#include <s8_common_node/Node.h>
#include <s8_mapper/mapper_node.h>
#include <s8_ir_sensors/ir_sensors_node.h>
#include <Map.h>

#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <s8_msgs/IRDistances.h>

#define HZ                          10

#define PARAM_SIDE_LENGTH_NAME      "side_length"
#define PARAM_SIDE_LENGTH_DEFAULT   10.0
#define PARAM_RESOLUTION_NAME       "resolution"
#define PARAM_RESOLUTION_DEFAULT    0.05
#define PARAM_RENDER_NAME           "render"
#define PARAM_RENDER_DEFAULT        false

#define TOPIC_IR_DISTANCES          s8::ir_sensors_node::TOPIC_IR_DISTANCES

using namespace s8::mapper_node;
using s8::ir_sensors_node::is_valid_ir_value;

class Mapper : public s8::Node {
    double side_length;
    double resolution;
    s8::Map map;
    ros::Subscriber robot_position_subscriber;
    ros::Subscriber ir_sensors_subscriber;
    float prev_robot_x;
    float prev_robot_y;
    size_t prev_robot_i;
    size_t prev_robot_j;
    size_t robot_i;
    size_t robot_j;

    bool render;
    long map_state;
    long map_state_rendered;
    ros::Publisher rviz_markers_publisher;

public:
    Mapper() : map_state(0), map_state_rendered(-1), prev_robot_x(0.0f), prev_robot_y(0.0f) {
        init_params();
        print_params();

        double side_cells_d = side_length / resolution;
        size_t side_cells = (size_t)side_cells_d;
        double loss = side_cells_d - side_cells;

        if(loss >= resolution) {
            ROS_WARN("Losing cells with current side_length and resolution. Cells lost on each side: %d", (int)(loss / resolution));
        }

        map = s8::Map(side_cells, side_cells, CELL_UNKNOWN);
        ROS_INFO("map size: %ldx%ld", map.num_rows(), map.num_cols());

        prev_robot_i = robot_i = map.row_relative_origo(0);
        prev_robot_j = robot_j = map.col_relative_origo(0);

        if(render) {
            rviz_markers_publisher = nh.advertise<visualization_msgs::MarkerArray>(TOPIC_VISUALIZATION_MARKERS, 1, true);
        }

        robot_position_subscriber = nh.subscribe<geometry_msgs::Point>(TOPIC_ROBOT_POSITION, 1, &Mapper::robot_position_callback, this);
        ir_sensors_subscriber = nh.subscribe<s8_msgs::IRDistances>(TOPIC_IR_DISTANCES, 1, &Mapper::ir_distances_callback, this);


        //TODO: Remove me.
        for(int i = map.row_relative_origo(-4); i < map.row_relative_origo(4); i++) {
            for(int j = map.col_relative_origo(-3); j < map.col_relative_origo(8); j++) {
                set_free(i, j);
            }
        }

        set_wall(map.row_relative_origo(-4), map.col_relative_origo(-3));
        set_wall(map.row_relative_origo(-3), map.col_relative_origo(-3));
        set_wall(map.row_relative_origo(-2), map.col_relative_origo(-3));
        set_wall(map.row_relative_origo(-1), map.col_relative_origo(-3));
        set_wall(map.row_relative_origo(0), map.col_relative_origo(-3));
        set_wall(map.row_relative_origo(1), map.col_relative_origo(-3));
        set_wall(map.row_relative_origo(2), map.col_relative_origo(-3));
        set_wall(map.row_relative_origo(3), map.col_relative_origo(-3));
    }

    void update() {
        if(should_render()) {
            renderToRviz();
        }
    }

    void renderToRviz() {
        visualization_msgs::MarkerArray markerArray;
        markerArray.markers = std::vector<visualization_msgs::Marker>((map.num_cells()));
        auto & markers = markerArray.markers;

        if(map_state != map_state_rendered) {

            for(size_t i = 0; i < map.num_rows(); i++) {
                for(size_t j = 0; j < map.num_cols(); j++) {
                    visualization_msgs::Marker & marker = markers[i * map.num_rows() + j];

                    marker.header.frame_id = "map";
                    marker.header.stamp = ros::Time();
                    marker.ns = "s8";
                    marker.id = i * map.num_rows() + j;
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

                    if(robot_i == i && robot_j == j) {
                        marker.color.a = 1.0;
                        marker.color.r = 0.0;
                        marker.color.g = 0.0;
                        marker.color.b = 0.0;
                        continue;
                    }

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
        float x = point->x;
        float y = point->y;

        int movement_i = (x - prev_robot_x) / resolution;
        int movement_j = (y - prev_robot_y) / resolution;

        robot_i = prev_robot_i + movement_i;
        robot_j = prev_robot_j + movement_j;

        ROS_INFO("Robot movement: (%ld,%ld) -> (%ld,%ld)", prev_robot_i, prev_robot_j, robot_i, robot_j);

        set_free(robot_i, robot_j);

        prev_robot_i = robot_i;
        prev_robot_j = robot_j;
    }

    void ir_distances_callback(const s8_msgs::IRDistances::ConstPtr & ir_distances) {
        /*double left_back = ir_distances->left_back;
        double left_front = ir_distances->left_front;

        auto process_ir_value = [](double value, double offset) {
            if(is_valid_ir_value(value)) {

            }
        };*/
    }

    bool should_render() {
        if(!render) {
            return false;
        }

        if(map_state == map_state_rendered) {
            return false;
        }

        return true;
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
