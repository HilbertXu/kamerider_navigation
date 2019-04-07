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

class gpsr_navigation
{
private:
    bool start_navigating;
    std::string target_type;
    std::string target_name;
    // ROS params
    std::string sub_speech_control_topic_name;
    std::string pub_nav_result_topic_name;
    // ROS publisher & subscriber
    ros::Publisher nav_pub;
    ros::Subscriber speech_sub;
    struct location_pose
    {
        string location_name="default";
        float x=0;
        float y=0;
        float z=0;
        float ori_x=0;
        float ori_y=0;
        float ori_z=0;
        float ori_w=0;
    };
    // Goal pose
    geometry_msgs::Pose goal_pose;
    //定义不同房间的坐标点
    location_pose  dining_room;
    location_pose  entrance;
    location_pose  kitchen;
    location_pose  bedroom;
    //定义不同位置的坐标点
    location_pose  bar_table;
    location_pose  bed;
    location_pose  side_table;
    location_pose  desk;
    location_pose  dining_table;
    location_pose  end_table;
    location_pose  Exit;
    location_pose  bookcase;
    location_pose  cupboard;
    location_pose  storage_table;
    location_pose  sink;
    location_pose  counter;
    location_pose  dishwasher;
    location_pose  couch;
    //the size of list must be predefined!!!!!!!!!!!!!!!!!!!!!!
    int list_size=18;
    location_pose list[18];
//read txt and change the poses

    
    
    void speechCallback(const kamerider_speech::mission& msg)
    {
        target_type = msg.mission_type;
        target_name = msg.mission_name;
        for(int i=0;i<list_size;i++)
        {
            if(list[i].location_name == target_name)
            {
                goal_pose.position.x = list[i].x;
                goal_pose.position.y = list[i].y;
                goal_pose.position.z = list[i].z;
                goal_pose.orientation.x = list[i].ori_x;
                goal_pose.orientation.y = list[i].ori_y;
                goal_pose.orientation.z = list[i].ori_z;
                goal_pose.orientation.w = list[i].ori_w;

            }
        }

       
    }
public:
    int run(int argc, char** argv)
    {
        ROS_INFO ("--------INIT--------");
        ros::init (argc, argv, "gpsr_navigation");
        ros::NodeHandle nh;

        // Set params
        start_navigating = false;

        nh.param<std::string>("sub_speech_control_topic_name", sub_speech_control_topic_name, "/control_to_nav");
        nh.param<std::string>("pub_nav_result_topic_name",     pub_nav_result_topic_name,     "/nav_to_control");

        nav_pub    = nh.advertise<std_msgs::String>(pub_nav_result_topic_name, 1);
        speech_sub = nh.subscribe(sub_speech_control_topic_name, 1, &gpsr_navigation::speechCallback, this);

        MoveBaseClient mc_("move_base_client", true);
        move_base_msgs::MoveBaseGoal nav_goal;

        while(ros::ok())
        {
            if (start_navigating)
            {
                ROS_INFO ("START navigating to TARGET position %s", target_name.c_str());
                nav_goal.target_pose.header.frame_id = "map";
                nav_goal.target_pose.header.stamp = ros::Time::now();
                nav_goal.target_pose.pose = geometry_msgs::Pose (goal_pose);
                while (!mc_.waitForServer (ros::Duration(5.0)))
                {
                    ROS_INFO ("Waiting For the Server...");
                }
                mc_.sendGoal (nav_goal);
                mc_.waitForResult (ros::Duration (40.0));
                if (mc_.getState () == actionlib::SimpleClientGoalState::SUCCEEDED)
                {
                    ROS_INFO ("Successfully reached %s", target_name.c_str());
                    start_navigating = false;
                    std_msgs::String send_flag;
                    if (target_type == "room")
                    {
                        send_flag.data = "room_target_arrived";
                    }
                    if (target_type == "location")
                    {
                        send_flag.data = "loc_target_arrived";
                    }
                    nav_pub.publish (send_flag);
                }
            }
            ros::spinOnce();
        }
    }
};

int main(int argc, char** argv)
{
    gpsr_navigation nav;
    return nav.run(argc, argv);
}
