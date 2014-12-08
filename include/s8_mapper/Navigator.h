#ifndef __NAVIGATOR_H
#define __NAVIGATOR_H

#include <actionlib/client/simple_action_client.h>
#include <s8_mapper/Topological.h>
#include <s8_turner/TurnAction.h>
#include <s8_motor_controller/StopAction.h>
#include <s8_turner/turner_node.h>

const double ACTION_STOP_TIMEOUT =                         30.0;
const double ACTION_TURN_TIMEOUT =                         30.0;

const int TURN_DEGREES_90 =                             90;

const std::string ACTION_STOP =                     s8::motor_controller_node::ACTION_STOP;
const std::string ACTION_TURN =                     s8::turner_node::ACTION_TURN;

enum GoToUnexploredResult {
    SUCCEEDED = 0,
    FAILED = 1
};

typedef s8::turner_node::Direction RotateDirection;

class Navigator {
    actionlib::SimpleActionClient<s8_turner::TurnAction> turn_action;
    actionlib::SimpleActionClient<s8_motor_controller::StopAction> stop_action;
    Topological *topological;
    std::function<void(GoToUnexploredResult)> go_to_unexplored_place_callback;
    bool going_to_unexplored_place;
    bool going_to_object_place;
    IRReadings ir_readings;
    std::vector<Topological::Node*> path;
    int node_index;
    bool navigating;

public:
    Navigator(Topological * topological, std::function<void(GoToUnexploredResult)> go_to_unexplored_place_callback) : going_to_object_place(false), navigating(false), topological(topological), go_to_unexplored_place_callback(go_to_unexplored_place_callback), going_to_unexplored_place(false), turn_action(ACTION_TURN, true), stop_action(ACTION_STOP, true) {
        ROS_INFO("Waiting for turn action server...");
        turn_action.waitForServer();
        ROS_INFO("Connected to turn action server!");

        ROS_INFO("Waiting for stop action server...");
        stop_action.waitForServer();
        ROS_INFO("Connected to stop action server!");
    }

    void go_to_unexplored_place() {
        going_to_unexplored_place = true;
        going_to_object_place = false;
        node_index = 0;

        auto is_unexplored_node = [this](Topological::Node * node) {
            if(!topological->is_free(node)) {
                return false;
            }

            std::vector<double> headings = { TOPO_EAST, TOPO_NORTH, TOPO_WEST, TOPO_SOUTH };

            for(double heading : headings) {
                if(topological->neighbors_in_heading(node, heading).size() == 0) {
                    return true;
                }
            }

            return false;
        };

        path = topological->dijkstra(topological->get_last(), is_unexplored_node);

        ROS_INFO("PATH");

        for(auto node : path) {
            ROS_INFO("(%.2lf, %.2lf)", node->x, node->y);
        }

        navigating = true;

        // exit(0);
    }

    void go_to_object_place() {
        going_to_object_place = true;
        going_to_unexplored_place = false;
        node_index = 0;
        ROS_INFO("BEFORE AUTO");

        auto is_object_node = [this](Topological::Node * node) {
            if(!topological->is_object_viewer(node)) {
                ROS_INFO("NOT OBJECT VIEWER");
                return false;
            }
            std::vector<double> headings = { TOPO_EAST, TOPO_NORTH, TOPO_WEST, TOPO_SOUTH };

            for(double heading : headings) {
                if(topological->neighbors_in_heading(node, heading).size() == 0) {
                    return true;
                }
            }

            return false;
        };

        ROS_INFO("returned object viewer node");

        path = topological->dijkstra(topological->get_last(), is_object_node);
        
        ROS_INFO("FOUND PATH");
        ROS_INFO("PATH %d", path.size());

        for(auto node : path) {
            ROS_INFO("(%.2lf, %.2lf)", node->x, node->y);
        }

        ROS_INFO("AT THE END");
        navigating = true;
    }

    void update(double robot_x, double robot_y, double robot_rotation, IRReadings ir_readings) {
        if(!navigating) {
            return;
        }

        ROS_INFO("robot_rotation: %d", (int)robot_rotation);

        ir_readings = ir_readings;
        robot_rotation = ((int)robot_rotation % 360);

        robot_rotation = s8::utils::math::degrees_to_radians(robot_rotation);

        if(robot_rotation > M_PI) {
            robot_rotation -= 2 * M_PI;
        }
    

        if(going_to_object_place) {
            double heading = 0;

            auto current = path[node_index];

            if(path.size() == node_index + 1) {
                //At target. Success.
                ROS_INFO("At target");

                std::vector<double> headings = { TOPO_EAST, TOPO_NORTH, TOPO_WEST, TOPO_SOUTH };

                for(double h : headings) {
                    ROS_INFO("ouch");
                    if(topological->neighbors_in_heading(current, h).size() == 0) {
                        heading = h;
                        break;
                    }
                }

                if(s8::utils::math::is_zero(heading)) {
                    ROS_INFO("Nothing to explore left.");
                    //exit(0);      x              
                }
            } else {
                auto next = path[node_index + 1];
                ROS_INFO("(%lf, %lf) -> (%lf %lf)", current->x, current->y, next->x, next->y);
                heading = topological->heading_between_nodes(current, next);;
            }

            
            double robot_heading = topological->angle_to_heading(robot_rotation);

            ROS_INFO("Heading: %lf, robot_heading: %lf", heading, robot_heading);
            if(!s8::utils::math::is_equal(heading, robot_heading)) {
                ROS_INFO("Wrong heading.");
                if(s8::utils::math::is_equal(robot_heading - heading, -M_PI / 2) || s8::utils::math::is_equal(robot_heading - heading, 3 * M_PI / 2)) {
                    turn(TURN_DEGREES_90);
                } else {
                    turn(-TURN_DEGREES_90);
                }
            } else {
                ROS_INFO("Right heading. Will succeed.");
                go_to_unexplored_place_callback(GoToUnexploredResult::SUCCEEDED);
                navigating = false;
            }
        }
    }

    void turn(int degrees) {
        stop();
        ROS_INFO("Turning %s", to_string(RotateDirection(sign(degrees))).c_str());

        s8_turner::TurnGoal goal;
        goal.degrees = degrees;
        turn_action.sendGoal(goal);

        bool finised_before_timeout = turn_action.waitForResult(ros::Duration(ACTION_TURN_TIMEOUT));
        ROS_INFO("Turn action state: %s", turn_action.getState().toString().c_str());
        if(finised_before_timeout) {
            actionlib::SimpleClientGoalState state = turn_action.getState();
            if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            } else if (state == actionlib::SimpleClientGoalState::ABORTED) {
                //Time out from turner node.
            } else {
                ROS_WARN("Turn action finished with unknown state %s", state.toString().c_str());
            }
        } else {
        }
    }

    void stop() {
        s8_motor_controller::StopGoal goal;
        goal.stop = true;
        stop_action.sendGoal(goal);

        bool finised_before_timeout = stop_action.waitForResult(ros::Duration(ACTION_STOP_TIMEOUT));

        if(finised_before_timeout) {
            actionlib::SimpleClientGoalState state = stop_action.getState();
            if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            } else {
                ROS_INFO("Stop action finished with unknown state: %s", state.toString().c_str());
            }
        } else {
        }
    }

};

#endif
