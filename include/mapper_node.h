#ifndef __MAPPER_NODE_H
#define __MAPPER_NODE_H

#include <string>

namespace s8 {
    namespace mapper_node {
        const std::string NODE_NAME =                       "s8_mapper_node";

        const std::string TOPIC_VISUALIZATION_MARKERS =     "/visualization_marker_array";
        const std::string TOPIC_ROBOT_POSITION =            "robot_position";

        const int CELL_UNKNOWN =                            1;
        const int CELL_FREE =                               1 << 2;
        const int CELL_OBSTACLE =                           1 << 3;
        const int CELL_WALL =                               1 << 4 & CELL_OBSTACLE;
        const int CELL_OBJECT =                             1 << 5 & CELL_OBSTACLE;
    }
}

#endif