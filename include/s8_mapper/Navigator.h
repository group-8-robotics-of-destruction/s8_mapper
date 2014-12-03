#ifndef __NAVIGATOR_H
#define __NAVIGATOR_H

#include <s8_mapper/Topological.h>

enum GoToUnexploredResult {
    SUCCEEDED = 0,
    FAILED = 1
};

class Navigator {
    Topological *topological;
    std::function<void(GoToUnexploredResult)> go_to_unexplored_place_callback;
    bool going_to_unexplored_place;

public:
    Navigator() {
        topological = NULL;
    }

    Navigator(Topological * topological, std::function<void(GoToUnexploredResult)> go_to_unexplored_place_callback) : topological(topological), go_to_unexplored_place_callback(go_to_unexplored_place_callback), going_to_unexplored_place(false) {

    }

    void go_to_unexplored_place() {
        going_to_unexplored_place = true;

        auto has_no_connections_in_heading = [this](Topological::Node * node) {
            std::vector<double> headings = { TOPO_EAST, TOPO_NORTH, TOPO_WEST, TOPO_SOUTH };

            for(auto heading : headings) {
                if(topological->neighbors_in_heading(node, TOPO_EAST).size() == 0) {
                    return true;
                }
            }

            return false;
        };

        auto path = topological->dijkstra(topological->get_last(), has_no_connections_in_heading);

        ROS_INFO("PATH");

        for(auto node : path) {
            ROS_INFO("(%.2lf, %.2lf)", node->x, node->y);
        }

        go_to_unexplored_place_callback(GoToUnexploredResult::FAILED);

        exit(0);
    }

    void update() {
        if(going_to_unexplored_place) {

        }
    }
};

#endif
