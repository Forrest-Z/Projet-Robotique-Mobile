#include <ros/ros.h>
#include "joy_teleop/joy_teleop.h"

static ros::Publisher twist_pub;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "joy_teleop");
    ros::NodeHandle n;

    joy_teleop::JoyTeleop jt;

    ros::spin();
    return 0;
}
