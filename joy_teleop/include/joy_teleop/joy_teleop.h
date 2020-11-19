#ifndef __JOY_TELEOP_H__
#define __JOY_TELEOP_H__

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>

namespace joy_teleop
{
    class JoyTeleop
    {
    public:
        JoyTeleop();

    private:
        void joy_callback_(const sensor_msgs::Joy::ConstPtr &msg);

        float lin_scale_;
        float ang_scale_;
        ros::Subscriber joy_sub_;
        ros::Publisher twist_pub_;
    };
} // namespace joy_teleop
#endif // __JOY_TELEOP_H__
