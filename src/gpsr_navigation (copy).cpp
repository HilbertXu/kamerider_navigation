/*
Date: 2018/12/20
Author: Xu Yucheng
Abstract: Code for GPSR 当从语音识别出来要去的地点之后导航到指定地点，然后开始寻找待抓取的物体
*/
#include <ros/ros.h>
#include <iostream>
#include <std_msgs/String.h>
#include <string.h>
//navigation中需要使用的位姿信息头文件
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <kamerider_speech/mission.h>
//move_base头文件
#include <move_base_msgs/MoveBaseGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
//actionlib头文件
#include<actionlib/client/simple_action_client.h>
#include<stdlib.h>
#include<cstdlib>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

bool ifNavigate = false;
bool ifArrived  = false;
std::string destination; 
std::string nav_flag;
std_msgs::String sound_flag;
std_msgs::String send_flag;
geometry_msgs::Twist vel;
geometry_msgs::Pose goal_pose;

//定义不同房间内的导航点
//定义不同房间位置的坐标点
geometry_msgs::Pose living_room;
geometry_msgs::Pose living_room;
geometry_msgs::Pose living_room;
geometry_msgs::Pose entrance;

geometry_msgs::Pose kitchen;
geometry_msgs::Pose bar_table;
geometry_msgs::Pose bed;
geometry_msgs::Pose start;
geometry_msgs::Pose exit;

ros::Publisher nav_pub;
ros::Subscriber speech_sub;
std::string NAV_PUBLISH_TOPIC;
std::string SPEECH_SUBSCRIBER_TOPIC;

void initPlace ()
//--------------------------------------
//该函数用于初始化所有导航点的坐标以及机器人朝向
//使用get_pose来记录预先设定好的导航点
//--------------------------------------
{
    //1
    corridor.position.x =  0;
    corridor.position.y = 0;
    corridor.position.z = 0;
    corridor.orientation.x = 0;
    corridor.orientation.y = 0;
    corridor.orientation.z = 0;
    corridor.orientation.w = 0;

    
    exit.position.x = 0.0654283;
    exit.position.y = 1.06126;
    exit.position.z = 0;
    exit.orientation.x = 0;
    exit.orientation.y = 0;
    exit.orientation.z = 0.859506;
    exit.orientation.w = 0.511125;
    //2
    living_room.position.x = 2.75546;
    living_room.position.y = -5.33159;
    living_room.position.z = 0;
    living_room.orientation.x = 0;
    living_room.orientation.y = 0;
    living_room.orientation.z = -0.718957;
    living_room.orientation.w = 0.695055;
    
    //3
    dining_room.position.x = 1.82056;
    dining_room.position.y = -3.54136;
    dining_room.position.z = 0;
    dining_room.orientation.x = 0;
    dining_room.orientation.y = 0;
    dining_room.orientation.z = -0.693987;
    dining_room.orientation.w = 0.719987;

    //4
    bedroom.position.x = 0.947594;
    bedroom.position.y = -3.44213;
    bedroom.position.z = 0;
    bedroom.orientation.x = 0;
    bedroom.orientation.y = 0;
    bedroom.orientation.z =  0.99419;
    bedroom.orientation.w =  0.107641;


    //5
    entrance.position.x = 0.540956;
    entrance.position.y = -0.548329;
    entrance.position.z = 0;
    entrance.orientation.x = 0;
    entrance.orientation.y = 0;
    entrance.orientation.z = -0.0611887;
    entrance.orientation.w = 0.998126;
    
    bar_table.position.x = 8.51342;
    bar_table.position.y = -6.43248;
    bar_table.position.z = 0;
    bar_table.orientation.x = 0;
    bar_table.orientation.y = 0;
    bar_table.orientation.z = 0.277456;
    bar_table.orientation.w = 0.960738;
    
    //6
    kitchen.position.x = 11.9463;
    kitchen.position.y = 2.81379;
    kitchen.position.z = 0;
    kitchen.orientation.x = 0;
    kitchen.orientation.y = 0;
    kitchen.orientation.z = -0.113013;
    kitchen.orientation.w = 0.993593;
  
    //7
    bed.position.x = 2.98617397856;
    bed.position.y = -1.49467780406;
    bed.position.z = 0;
    bed.orientation.x = 0;
    bed.orientation.y = 0;
    bed.orientation.z = 0.492294159753;
    bed.orientation.w = 0.870428894438;
    
    side_table.position.x = 2.98617397856;
    side_table.position.y = -1.49467780406;
    side_table.position.z = 0;
    side_table.orientation.x = 0;
    side_table.orientation.y = 0;
    side_table.orientation.z = 0.492294159753;
    side_table.orientation.w = 0.870428894438;   

    desk.position.x = 2.98617397856;
    desk.position.y = -1.49467780406;
    desk.position.z = 0;
    desk.orientation.x = 0;
    desk.orientation.y = 0;
    desk.orientation.z = 0.492294159753;
    desk.orientation.w = 0.870428894438;
  
    dining_table.position.x = 2.98617397856;
    dining_table.position.y = -1.49467780406;
    dining_table.position.z = 0;
    dining_table.orientation.x = 0;
    dining_table.orientation.y = 0;
    dining_table.orientation.z = 0.492294159753;
    dining_table.orientation.w = 0.870428894438;

    couch.position.x = 2.98617397856;
    couch.position.y = -1.49467780406;
    couch.position.z = 0;
    couch.orientation.x = 0;
    couch.orientation.y = 0;
    couch.orientation.z = 0.492294159753;
    couch.orientation.w = 0.870428894438;

    end_table.position.x = 2.98617397856;
    end_table.position.y = -1.49467780406;
    end_table.position.z = 0;
    end_table.orientation.x = 0;
    end_table.orientation.y = 0;
    end_table.orientation.z = 0.492294159753;
    end_table.orientation.w = 0.870428894438;

    bookcase.position.x = 2.98617397856;
    bookcase.position.y = -1.49467780406;
    bookcase.position.z = 0;
    bookcase.orientation.x = 0;
    bookcase.orientation.y = 0;
    bookcase.orientation.z = 0.492294159753;
    bookcase.orientation.w = 0.870428894438;

    cupboard.position.x = 2.98617397856;
    cupboard.position.y = -1.49467780406;
    cupboard.position.z = 0;
    cupboard.orientation.x = 0;
    cupboard.orientation.y = 0;
    cupboard.orientation.z = 0.492294159753;
    cupboard.orientation.w = 0.870428894438;

    storage_table.position.x = 2.98617397856;
    storage_table.position.y = -1.49467780406;
    storage_table.position.z = 0;
    storage_table.orientation.x = 0;
    storage_table.orientation.y = 0;
    storage_table.orientation.z = 0.492294159753;
    storage_table.orientation.w = 0.870428894438;

    sink.position.x = 2.98617397856;
    sink.position.y = -1.49467780406;
    sink.position.z = 0;
    sink.orientation.x = 0;
    sink.orientation.y = 0;
    sink.orientation.z = 0.492294159753;
    sink.orientation.w = 0.870428894438;

    counter.position.x = 2.98617397856;
    counter.position.y = -1.49467780406;
    counter.position.z = 0;
    counter.orientation.x = 0;
    counter.orientation.y = 0;
    counter.orientation.z = 0.492294159753;
    counter.orientation.w = 0.870428894438;
 
    dishwasher.position.x = 2.98617397856;
    dishwasher.position.y = -1.49467780406;
    dishwasher.position.z = 0;
    dishwasher.orientation.x = 0;
    dishwasher.orientation.y = 0;
    dishwasher.orientation.z = 0.492294159753;
    dishwasher.orientation.w = 0.870428894438;
}

void speechCallback (const kamerider_speech::mission& msg)
{
    return 0
}

int main (int argc, char** argv)
{
    ROS_INFO ("start GPSR navigation");
    ros::init (argc, argv, "gpsr_navigation");
    ros::NodeHandle nh;

    //初始化导航点
    initPlace ();
    nav_pub = nh.advertise<std_msgs::String> (NAV_PUBLISH_TOPIC, 1);
    speech_sub = nh.subscribe (SPEECH_SUBSCRIBER_TOPIC, 1, speechCallback);

    MoveBaseClient mc_("move_base_client", true);
    move_base_msgs::MoveBaseGoal nav_goal;

    while (ros::ok())
    {
        if (ifNavigate)
        {
            ROS_INFO ("Start Navigating to Target Position");
            nav_goal.target_pose.header.frame_id = "map";
            nav_goal.target_pose.header.stamp = ros::Time::now();
            nav_goal.target_pose.pose = geometry_msgs::Pose (goal_pose);

            while (!mc_.waitForServer (ros::Duration(5.0)))
            {
                ROS_INFO ("Waiting For the Server");
            }
            mc_.sendGoal (nav_goal);
            mc_.waitForResult (ros::Duration (40.0));
            send_flag.data = "in_position";

            if (mc_.getState () == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO ("Successfully reached %s", nav_flag);
                ifNavigate = false;
                nav_pub.publish (send_flag);
            }
        }
        ros::spinOnce();
    }
    return 0;
}
