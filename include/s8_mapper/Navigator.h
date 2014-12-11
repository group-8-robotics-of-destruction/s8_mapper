#ifndef __NAVIGATOR_H
#define __NAVIGATOR_H

#include <actionlib/client/simple_action_client.h>
#include <s8_mapper/mapper_node.h>
#include <s8_mapper/Topological.h>
#include <s8_turner/TurnAction.h>
#include <s8_motor_controller/StopAction.h>
#include <s8_turner/turner_node.h>
#include <geometry_msgs/Twist.h>

const double ACTION_STOP_TIMEOUT =                         30.0;
const double ACTION_TURN_TIMEOUT =                         30.0;

const int TURN_DEGREES_90 =                             90;

const std::string ACTION_STOP =                     s8::motor_controller_node::ACTION_STOP;
const std::string ACTION_TURN =                     s8::turner_node::ACTION_TURN;

enum GoToUnexploredResult {
    SUCCEEDED = 1,
    AllExplored = 2,
    FAILED = 3,
    AtRoot = 4,
    AtObjectViewer = 5
};

typedef s8::turner_node::Direction RotateDirection;

class Navigator {
    ros::Publisher *twist_publisher;
    actionlib::SimpleActionClient<s8_turner::TurnAction> turn_action;
    actionlib::SimpleActionClient<s8_motor_controller::StopAction> stop_action;
    Topological *topological;
    std::function<void(GoToUnexploredResult)> go_to_callback;
    bool going_to_unexplored_place;
    bool going_to_object_place;
    bool going_to_root;
    IRReadings *ir_readings;
    std::vector<Topological::Node*> path;
    int node_index;
    bool navigating;
    double go_straight_velocity;
    s8::mapper_node::RobotPose *robot_pose;

public:
    Navigator(Topological * topological, std::function<void(GoToUnexploredResult)> go_to_callback, ros::Publisher *twist_publisher, s8::mapper_node::RobotPose *robot_pose) : going_to_object_place(false), navigating(false), topological(topological), go_to_callback(go_to_callback), going_to_unexplored_place(false), turn_action(ACTION_TURN, true), stop_action(ACTION_STOP, true), twist_publisher(twist_publisher), go_straight_velocity(0.15), robot_pose(robot_pose) {
        ROS_INFO("Waiting for turn action server...");
        turn_action.waitForServer();
        ROS_INFO("Connected to turn action server!");

        ROS_INFO("Waiting for stop action server...");
        stop_action.waitForServer();
        ROS_INFO("Connected to stop action server!");
    }

    int go_to_unexplored_place() {
        init_go_to();
        going_to_unexplored_place = true;

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

        if(path.size() == 0) {
            //Nothing to explore.
            //go_to_callback(GoToUnexploredResult::AllExplored);
            going_to_unexplored_place = false;
            navigating = false;
            return GoToUnexploredResult::AllExplored;
        }

        return 0;
    }

    int go_to_object_place() {
        init_go_to();
        going_to_object_place = true;
        ROS_INFO("GOING TO OBJECT PLACE");

        auto is_object_node = [this](Topological::Node * node) {
            return topological->is_object_viewer(node) && !topological->is_object_viewer_visited(node);
        };

        path = topological->dijkstra(topological->get_last(), is_object_node);

        if(path.size() == 0) {
            //go_to_callback(GoToUnexploredResult::SUCCEEDED); //TODO: Other?
            navigating = false;
            going_to_object_place = false;
            ROS_INFO("Visited object viewer node!");
            system("espeak 'At object viewer node'");
            topological->visit_object_viewer(topological->get_last());
            go_to_callback(GoToUnexploredResult::AtObjectViewer);
            return GoToUnexploredResult::AtObjectViewer;
        }

        return 0;
    }

    int go_to_root() {
        init_go_to();
        going_to_root = true;

        auto is_root_node = [this](Topological::Node * node) {
            return topological->is_root(node);
        };

        path = topological->dijkstra(topological->get_last(), is_root_node);

        if(path.size() == 0 || path.size() == 1) {
            ROS_INFO("Cannot find root!");
            //go_to_callback(GoToUnexploredResult::SUCCEEDED); //TODO: Other?
            going_to_root = false;
            navigating = false;
            return GoToUnexploredResult::AtRoot;
        }

        return 0;
    }

    void init_go_to() {
        node_index = 0;
        going_to_object_place = false;
        going_to_unexplored_place = false;
        going_to_root = false;
        navigating = true;
    }

    void update(IRReadings *ir_readings) {
        if(!navigating) {
            return;
        }

        ROS_INFO("robot_rotation: %d", (int)robot_pose->rotation);

        this->ir_readings = ir_readings;
        double robot_rotation = ((int)robot_pose->rotation % 360);

        robot_rotation = s8::utils::math::degrees_to_radians(robot_rotation);

        if(robot_rotation > M_PI) {
            robot_rotation -= 2 * M_PI;
        }

        if(going_to_unexplored_place || going_to_object_place || going_to_root) {
            double heading = 0;
            Topological::Node *next = NULL;

            auto current = path[node_index];

            if(path.size() == node_index + 1) {
                //At target. Success.
                ROS_INFO("At target");

                if(going_to_object_place) {
                    navigating = false;
                    ROS_INFO("Visited object viewer node!");
                    topological->visit_object_viewer(current);
                    system("espeak 'At object viewer node'");
                    return go_to_callback(GoToUnexploredResult::AtObjectViewer);
                } else if(going_to_root) {
                    ROS_INFO("Back at root!!");
                    navigating = false;
                    return go_to_callback(GoToUnexploredResult::AtRoot);
                }

                std::vector<double> headings = { TOPO_EAST, TOPO_NORTH, TOPO_WEST, TOPO_SOUTH };

                for(double h : headings) {
                    if(topological->neighbors_in_heading(current, h).size() == 0) {
                        heading = h;
                        break;
                    }
                }

                if(s8::utils::math::is_zero(heading)) {
                    ROS_INFO("Nothing to explore left.");
                    go_to_callback(GoToUnexploredResult::AllExplored);
                    navigating = false;
                    path.clear();
                    //exit(0);      x              
                }
            } else {
                next = path[node_index + 1];
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
                ROS_INFO("Right heading");

                //Check if node has walls to stop to.
                if(next != NULL && !node_has_wall_in_heading(next, heading)) {
                    ROS_INFO("In-between node. Will go straight for it.");

                    double distance = topological->euclidian_distance(current->x, next->x, current->y, next->y);

                    //Doesnt have walls. Need to do custom traverse.
                    go_straight([&distance, &current, &robot_pose, this](){
                        double traveled_distance = topological->euclidian_distance(current->x, robot_pose->position.x, current->y, robot_pose->position.y);
                        ROS_INFO("Progress: %lf", (std::abs(distance - traveled_distance) / distance));
                        if((distance - traveled_distance) < 0.06) {
                            return false;
                        }

                        return true;
                    });

                    ROS_INFO("Done going straight.");
                    topological->set_as_last_node(next);

                    if(going_to_unexplored_place) {
                        go_to_unexplored_place();
                    } else {
                        stop();
                        ROS_INFO("Keep going to object place");
                        go_to_object_place();
                    }
                } else {
                    ROS_INFO("Will succeed.");
                    go_to_callback(GoToUnexploredResult::SUCCEEDED);
                    navigating = false;                    
                    path.clear();
                }
            }
        }
    }

    bool node_has_wall_in_heading(Topological::Node *node, double heading) {
        auto nodes = topological->neighbors_in_heading(node, heading);
        for(auto n : nodes) {
            if(topological->is_wall(n)) {
                return true;
            }
        }

        return false;
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

    void go_straight(std::function<bool()> condition) {
        geometry_msgs::Twist twist;
        twist.linear.x = go_straight_velocity;
        int heading = get_current_heading();

        bool should_stop_go_straight = false; //TODO: Remove?

        ros::Rate loop_rate(25);
        while(condition() && ros::ok() && !should_stop_go_straight) {
            ros::spinOnce();
            int rotation = (int)robot_pose->rotation % 360;
            if (rotation < 0){
                rotation += 360;
            }

            int diff = heading - rotation;
            if (diff <= -45 )
                diff += 360;
            double alpha = 1.0/45.0;
            twist.angular.z = alpha*(double)diff;

            ROS_INFO("left_front: %lf", ir_readings->left_front);
            if(ir_readings->left_front > 0 && ir_readings->left_front < 0.07) {
                twist.angular.z = -0.3;
            } else if(ir_readings->right_front > 0 && ir_readings->right_front < 0.07) {
                twist.angular.z = 0.3;
            }

            twist_publisher->publish(twist);
            loop_rate.sleep();
        }
    }

    void align_to_object(Topological::Node *object) {
        geometry_msgs::Twist twist;
        twist.linear.x = go_straight_velocity;
        int heading = get_current_heading();

        bool should_stop_go_straight = false; //TODO: Remove?
        double diff = 10;
        ros::Rate loop_rate(25);
        while(std::abs(diff) > 0.09 && ros::ok() && !should_stop_go_straight) {
            ros::spinOnce();

            //ROS_INFO("ALIGNING");
            double desired_angle = topological->angle_between_nodes(robot_pose->position.x, robot_pose->position.y, object);
            if (desired_angle < 0){
                desired_angle += 2*M_PI;
            }
            int rotation = (int)robot_pose->rotation % 360;
            if (rotation < 0){
                rotation += 360;
            }
            double rotation_rad = s8::utils::math::degrees_to_radians(rotation);

            ROS_INFO("desired angle %lf, rotation %lf", desired_angle, robot_pose->rotation);
            diff = rotation_rad - desired_angle;

            if (diff > M_PI){
                diff -= 2*M_PI;
            } else if (diff < -M_PI) {
                diff += 2*M_PI;
            }

            if (diff > 0){
                twist.angular.z = -0.7;
            } else {
                twist.angular.z = 0.7;
            }
            twist.linear.x = 0.0;

            twist_publisher->publish(twist);
            loop_rate.sleep();
        }
    }

    int get_current_heading(){
        int rotation = robot_pose->rotation;

        if (rotation >= 315 || rotation < 45){
            return 0;
        }
        else if (rotation >= 45 && rotation < 135){
            return 90;
        }
        else if (rotation >= 135 && rotation < 225){
            return 180;
        }
        else{
            return 270;
        }
    }

    std::vector<Topological::Node *> get_path() {
        return path;
    }

};

#endif
