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
geometry_msgs::Pose living_room_ob1;
geometry_msgs::Pose living_room_ob2;
geometry_msgs::Pose living_room_GCP;
geometry_msgs::Pose entrance;

geometry_msgs::Pose kitchen;
geometry_msgs::Pose bar_table;
geometry_msgs::Pose balcony;
geometry_msgs::Pose start;
geometry_msgs::Pose exitus;

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
    start.position.x =  0;
    start.position.y = 0;
    start.position.z = 0;
    start.orientation.x = 0;
    start.orientation.y = 0;
    start.orientation.z = 0;
    start.orientation.w = 0;

    
    exitus.position.x = 0.0654283;
    exitus.position.y = 1.06126;
    exitus.position.z = 0;
    exitus.orientation.x = 0;
    exitus.orientation.y = 0;
    exitus.orientation.z = 0.859506;
    exitus.orientation.w = 0.511125;
    //2
    living_room_ob1.position.x = 2.75546;
    living_room_ob1.position.y = -5.33159;
    living_room_ob1.position.z = 0;
    living_room_ob1.orientation.x = 0;
    living_room_ob1.orientation.y = 0;
    living_room_ob1.orientation.z = -0.718957;
    living_room_ob1.orientation.w = 0.695055;
    
    //3
    living_room_ob2.position.x = 1.82056;
    living_room_ob2.position.y = -3.54136;
    living_room_ob2.position.z = 0;
    living_room_ob2.orientation.x = 0;
    living_room_ob2.orientation.y = 0;
    living_room_ob2.orientation.z = -0.693987;
    living_room_ob2.orientation.w = 0.719987;

    //4
    living_room_GCP.position.x = 0.947594;
    living_room_GCP.position.y = -3.44213;
    living_room_GCP.position.z = 0;
    living_room_GCP.orientation.x = 0;
    living_room_GCP.orientation.y = 0;
    living_room_GCP.orientation.z =  0.99419;
    living_room_GCP.orientation.w =  0.107641;


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
    balcony.position.x = 2.98617397856;
    balcony.position.y = -1.49467780406;
    balcony.position.z = 0;
    balcony.orientation.x = 0;
    balcony.orientation.y = 0;
    balcony.orientation.z = 0.492294159753;
    balcony.orientation.w = 0.870428894438;
}

void speechCallback (const std_msgs::String::ConstPtr& msg)
{
    /*
    @TODO
    此处尝试修改语音节点中消息发布的类型
    修改为自定义消息，其中包含待抓取物体所在房间以及物体所在的家具
    */
    nav_flag = msg->data;
    if (nav_flag == "living_room")
    {
        goal_pose = living_room_ob1;
        ifNavigate = true;
    }
    else if (nav_flag == "kitchen")
    {
        goal_pose = kitchen;
        ifNavigate = true;
    }
    else if (nav_flag == "bar_table")
    {
        goal_pose = bar_table;
        ifNavigate = true;
    }
    else if (nav_flag == "entrance")
    {
        goal_pose = entrance;
        ifNavigate = true;
    }
    else if (nav_flag == "balcony")
    {
        goal_pose = balcony;
        ifNavigate = true;
    }
    else if (nav_flag == "start")
    {
        goal_pose = start;
        ifNavigate = true;
    }
    else if (nav_flag == "exit")
    {
        goal_pose = exitus;
        ifNavigate = true;
    }
    else
    {
        ROS_ERROR ("%s cannot reach QAQ", nav_flag);
    }
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
