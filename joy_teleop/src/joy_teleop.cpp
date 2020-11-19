#include "joy_teleop/joy_teleop.h"
#include <geometry_msgs/Twist.h>
#include <string>

namespace joy_teleop
{
    JoyTeleop::JoyTeleop()
    {
        ros::NodeHandle n("~");

        lin_scale_ = n.param("lin_scale", 2);
        ang_scale_ = n.param("ang_scale", 2);
        std::string twist_topic = n.param<std::string>("twist_topic", "/cmd_vel");
        std::string joy_topic = n.param<std::string>("joy_topic", "/joy");

        joy_sub_ = n.subscribe<sensor_msgs::Joy>(joy_topic, 10, &JoyTeleop::joy_callback_, this);
        twist_pub_ = n.advertise<geometry_msgs::Twist>(twist_topic, 10);
    }

    void JoyTeleop::joy_callback_(const sensor_msgs::Joy::ConstPtr &msg)
    {
        float ud_axis_l_stick = msg->axes[0];
        float lf_axis_l_stick = msg->axes[1];

        geometry_msgs::Twist twist_msg;
        twist_msg.angular.z = ang_scale_ * ud_axis_l_stick;
        twist_msg.linear.x = lin_scale_ * lf_axis_l_stick;

        twist_pub_.publish(twist_msg);
    }

} // namespace joy_teleop
