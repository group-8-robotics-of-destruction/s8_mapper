#include <ros/ros.h>

#include <s8_common_node/Node.h>
#include <Map.h>

#define NODE_NAME       "s8_mapper_node"

#define HZ              10
#define BUFFER_SIZE     1000

#define PARAM_SIDE_LENGTH_NAME      "side_length"
#define PARAM_SIDE_LENGTH_DEFAULT   10.0
#define PARAM_RESOLUTION_NAME       "resolution"
#define PARAM_RESOLUTION_DEFAULT    0.05

#define CELL_UNKNOWN                -1
#define CELL_FREE                   0
#define CELL_OBSTACLE               0 << 1
#define CELL_WALL                   0 << 2 & CELL_OBSTACLE
#define CELL_OBJECT                 0 << 3 & CELL_OBSTACLE
#define CELL_ROBOT                  0 << 4

class Mapper : public s8::Node {
    double side_length;
    double resolution;
    s8::Map map;

public:
    Mapper() {
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
    }

    void init_params() {
        add_param(PARAM_SIDE_LENGTH_NAME, side_length, PARAM_SIDE_LENGTH_DEFAULT);
        add_param(PARAM_RESOLUTION_NAME, resolution, PARAM_RESOLUTION_DEFAULT);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, NODE_NAME);

    Mapper mapper;

    ros::spin();
    return 0;
}
