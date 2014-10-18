#include <ros/ros.h>
#include <nav_msgs/GridCells.h>
#include <geometry_msgs/Point.h>

#include <s8_common_node/Node.h>
#include <Map.h>

#define NODE_NAME                   "s8_mapper_node"

#define HZ                          10
#define BUFFER_SIZE                 1000
#define RENDER_BUFFER_SIZE          10

#define PARAM_SIDE_LENGTH_NAME      "side_length"
#define PARAM_SIDE_LENGTH_DEFAULT   10.0
#define PARAM_RESOLUTION_NAME       "resolution"
#define PARAM_RESOLUTION_DEFAULT    0.05
#define PARAM_RENDER_NAME           "render"
#define PARAM_RENDER_DEFAULT        false

#define CELL_UNKNOWN                -1
#define CELL_FREE                   0
#define CELL_OBSTACLE               0 << 1
#define CELL_WALL                   0 << 2 & CELL_OBSTACLE
#define CELL_OBJECT                 0 << 3 & CELL_OBSTACLE

class Mapper : public s8::Node {
    double side_length;
    double resolution;
    s8::Map map;

    bool render;
    long map_state;
    long map_state_rendered;

    ros::Publisher rviz_gridcells_publisher;

public:
    Mapper() : map_state(0), map_state_rendered(-1) {
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

        if(render) {
            rviz_gridcells_publisher = nh.advertise<nav_msgs::GridCells>("map", 1, true);
        }
    }

    void update() {
        if(should_render()) {
            renderToRviz();
        }
    }

    void renderToRviz() {
        static nav_msgs::GridCells grid_cells;

        if(map_state != map_state_rendered) {
            grid_cells.header.frame_id = "map_frame";
            grid_cells.header.stamp = ros::Time::now();
            grid_cells.cell_width = map.num_cols();
            grid_cells.cell_height = map.num_rows();

            std::vector<geometry_msgs::Point> points(map.num_cells());

            for(size_t i = 0; i < map.num_rows(); i++) {
                for(size_t j = 0; j < map.num_cols(); j++) {
                    geometry_msgs::Point & point = points[i * map.num_rows() + j];

                    if(is_point_obstacle(i, j)) {
                        point.x = 255.0f;
                    } else if(is_point_wall(i, j)) {

                    } else if(is_point_free(i, j)) {
                        point.y = 255.0f;
                    } else if(is_point_object(i, j)) {

                    } else if(is_point_unknown(i, j)) {
                        point.z = 255.0f;
                    } else {
                        ROS_WARN("Unknown cell type: %d at (%ld,%ld)", map[i][j], i, j);
                    }
                }
            }

            grid_cells.cells = points;

            ROS_INFO("Sending recomputed map to be rendered");
        } else {
            ROS_INFO("Sending cached map to be rendered");
        }

        rviz_gridcells_publisher.publish(grid_cells);

        map_state_rendered = map_state;
    }

    bool is_point_value(size_t row_index, size_t col_index, int value) {
        return map[row_index][col_index] & value == value;
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
