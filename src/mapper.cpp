#include <ros/ros.h>

#include <s8_common_node/Node.h>

#define NODE_NAME       "s8_mapper_node"

#define HZ              10
#define BUFFER_SIZE     1000

class Mapper : public s8::Node {

public:
    Mapper() {
        
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, NODE_NAME);

    Mapper mapper;

    ros::spin();
    return 0;
}
