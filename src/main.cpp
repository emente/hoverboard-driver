#include <ros/ros.h>
#include "hover.h"


int main(int argc, char **argv) {
    ros::init(argc, argv, "hoverboard_driver");

    Hover h;
    while (ros::ok()) {
        h.loop();
        ros::spinOnce();
    }

    return 0;
}
