#ifndef __MAPPER_NODE_H
#define __MAPPER_NODE_H

#include <s8_mapper/Coordinate.h>
#include <string>

namespace s8 {
    namespace mapper_node {
        const std::string NODE_NAME =                       "s8_mapper_node";

        const std::string TOPIC_VISUALIZATION_MARKERS =     "/visualization_marker_array";
        const std::string TOPIC_RENDER =                    "/s8/map/occupancy_grid";
        const std::string TOPIC_ROBOT_POSITION =            "robot_position";
        const std::string SERVICE_PLACE_NODE =              "/s8/map/PlaceNode";

        enum NavigateType {
            ToClosestUnexplored,
            ToClosestObject,
            ToRoot
        };

        struct IRPositions {
            Coordinate left_back;
            Coordinate left_front;
            Coordinate right_back;
            Coordinate right_front;
            Coordinate front_right;
            Coordinate front_left;
        };

        struct IRReadings {
            double left_back;
            double left_front;
            double right_back;
            double right_front;
            double front_right;
            double front_left;

            IRReadings() : left_back(-1), left_front(-1), right_back(-1), right_front(-1), front_left(-1), front_right(-1) {}
        };

        struct RobotPose {
            Coordinate position;
            double rotation;

            RobotPose() : position(Coordinate()), rotation(-90) {}
            RobotPose(double x, double y, int rotation) : position(Coordinate(x, y)), rotation(rotation) {}
        };
    }
}

#endif
