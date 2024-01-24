#include <dynamic_map_generator/random_dynamic_map.hpp>
#include <ros/exceptions.h>
#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "random_dynamic_map");
    ros::NodeHandle nh("~");

    RandomDynamicMap map(nh);
    map.init();
    map.renderMap();

    ros::spin();

    return 0;
}
