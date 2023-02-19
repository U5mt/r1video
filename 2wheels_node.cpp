#include <ros/ros.h>
#include <std_msgs/Float32.h>

#include <canplugins/Frame.h>
#include "packing.hpp"
#include "logicool.hpp"
#include "joy_to_key_button.hpp"
#include "2wheels.hpp"


//using namespace R1video;


class R1video2wheels
{
    
    ros::Publisher can_tx_pub = nh.advertise<can_plugins::Frame>(can_tx::topic, 1);
    CanPublisher<CanTxTopics::emergency_stop_canpub_{can_tx_pub_};

    
    float wheel_duty[2] = [CRSLib::JoyToKeyButton::get_axis(CRSLib::LogicoolXInputKeyMap::Axes::l_stick_UD), CRSLib::JoyToKeyButton::get_axis(CRSLib::LogicoolXInputKeyMap::Axes::r_stick_UD)];
    
    //pack関数をこのへんに入れる

    void make_data()
        for (int i = 0; i < 1; i++)
        {   
        /*
        送るメッセージのタイプ
        can通信用
        can_pluginsのFrame.msgを参照
        floatの-100から100まで送りたいので100倍
        */
            can_plugins::Frame.data[i] = 100 * packed_wheel_duty[i];
        }
    

    
};



int main(int argc, char** argv)
{
    ros::init(argc, argv, "2wheels");

    ros::NodeHandle nh{};
    
    ros::spin();
    return 0;
}



