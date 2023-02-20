#include <ros/ros.h>
#include <cmath>
#include <cstdint>
#include <can_plugins/Frame.h>
#include <r1video/packing.hpp>
#include <r1video/logicool.hpp>
#include <r1video/joy_to_key_button.hpp>


/*
    canのIDの設定
*/
const std::uint32_t canid_l = 0x300;
const std::uint32_t canid_r = 0x301;

class R1video2wheels
{
private:
    ros::NodeHandle& nh;
    float wheel_vel_l{};
    float wheel_vel_r{};
    int wheel_duty_l = 0;
    int wheel_duty_r = 0;
    CRSLib::Logicool logicool;
    
    ros::Publisher can_tx_pub = nh.advertise<can_plugins::Frame>("/can_tx", 2);
    ros::Timer pub_tim;

public:
    R1video2wheels(ros::NodeHandle& nh):
        nh{nh},
        logicool{nh, "/joy"},
        pub_tim{nh.createTimer(ros::Duration(1.0/1000), &R1video2wheels::publish_wheel_duty, this)}
    {}

private:
    void publish_wheel_duty(const ros::TimerEvent&)
    {
        can_plugins::Frame frame{};

        wheel_vel_l = logicool.get_axis(CRSLib::LogicoolXInputKeyMap::Axes::l_stick_UD);
        wheel_duty_l = 100 * wheel_vel_l;
 
        frame.id = canid_l; //id設定
        frame.dlc = sizeof(wheel_duty_l);
        adhoc_canplugins_half::pack(frame.data.data(), wheel_duty_l);
    
        can_tx_pub.publish(frame);

        //ROS_INFO("[%d]is publised to [%d]", wheel_duty_l, canid_l);

        wheel_vel_r = logicool.get_axis(CRSLib::LogicoolXInputKeyMap::Axes::r_stick_UD);
        wheel_duty_r = 100 * wheel_vel_r;
 
        frame.id = canid_r;
        frame.dlc = sizeof(wheel_duty_r);
        adhoc_canplugins_half::pack(frame.data.data(), wheel_duty_r);

        can_tx_pub.publish(frame);

        //ROS_INFO("[%d]is publised to [%d]", wheel_duty_r, canid_r);
    }
    
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "r1video2wheels");
    ros::NodeHandle nh{};
    R1video2wheels r1video2wheels(nh);
    
    ros::spin();
    return 0;
}