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
    // Goal pose
    geometry_msgs::Pose goal_pose;
    //定义不同房间的导航点
    geometry_msgs::Pose living_room;
    geometry_msgs::Pose corridor;
    geometry_msgs::Pose dining_room;
    geometry_msgs::Pose entrance;
    geometry_msgs::Pose kitchen;
    geometry_msgs::Pose bedroom;
    //定义不同房间位置的坐标点
    geometry_msgs::Pose bar_table;
    geometry_msgs::Pose bed;
    geometry_msgs::Pose side_table;
    geometry_msgs::Pose desk;
    geometry_msgs::Pose dining_table;
    geometry_msgs::Pose end_table;
    geometry_msgs::Pose Exit;
    geometry_msgs::Pose bookcase;
    geometry_msgs::Pose cupboard;
    geometry_msgs::Pose storage_table;
    geometry_msgs::Pose sink;
    geometry_msgs::Pose counter;
    geometry_msgs::Pose dishwasher;
    geometry_msgs::Pose couch;


    void init_pose()
    {
        //1
        corridor.position.x =  0;
        corridor.position.y = 0;
        corridor.position.z = 0;
        corridor.orientation.x = 0;
        corridor.orientation.y = 0;
        corridor.orientation.z = 0;
        corridor.orientation.w = 0;

        
        Exit.position.x = 0.0654283;
        Exit.position.y = 1.06126;
        Exit.position.z = 0;
        Exit.orientation.x = 0;
        Exit.orientation.y = 0;
        Exit.orientation.z = 0.859506;
        Exit.orientation.w = 0.511125;
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
    
    void speechCallback(const kamerider_speech::mission& msg)
    {
        target_type = msg.mission_type;
        target_name = msg.mission_name;
        
        if (target_name == "living_room")
        {
            goal_pose = living_room;
            start_navigating = true;
        }
        else if (target_name == "corridor")
        {
            goal_pose = corridor;
            start_navigating = true;
        }
        else if (target_name == "dining_room")
        {
            goal_pose = dining_room;
            start_navigating = true;
        }
        else if (target_name == "entrance")
        {
            goal_pose = entrance;
            start_navigating = true;
        }
        else if (target_name == "kitchen")
        {
            goal_pose = kitchen;
            start_navigating = true;
        }
        else if (target_name == "bedroom")
        {
            goal_pose = bedroom;
            start_navigating = true;
        }
        else if (target_name == "bar_table")
        {
            goal_pose = bar_table;
            start_navigating = true;
        }
        else if (target_name == "bed")
        {
            goal_pose = bed;
            start_navigating = true;
        }
        else if (target_name == "side_table")
        {
            goal_pose = side_table;
            start_navigating = true;
        }
        else if (target_name == "desk")
        {
            goal_pose = desk;
            start_navigating = true;
        }
        else if (target_name == "dining_table")
        {
            goal_pose = dining_table;
            start_navigating = true;
        }
        else if (target_name == "end_table")
        {
            goal_pose = end_table;
            start_navigating = true;
        }
        else if (target_name == "Exit")
        {
            goal_pose = Exit;
            start_navigating = true;
        }
        else if (target_name == "bookcase")
        {
            goal_pose = bookcase;
            start_navigating = true;
        }
        else if (target_name == "cupboard")
        {
            goal_pose = cupboard;
            start_navigating = true;
        }
        else if (target_name == "storage_table")
        {
            goal_pose = storage_table;
            start_navigating = true;
        }
        else if (target_name == "sink")
        {
            goal_pose = sink;
            start_navigating = true;
        }
        else if (target_name == "counter")
        {
            goal_pose = counter;
            start_navigating = true;
        }
        else if (target_name == "dishwasher")
        {
            goal_pose = dishwasher;
            start_navigating = true;
        }
        else if (target_name == "couch")
        {
            goal_pose = couch;
            start_navigating = true;
        }
        else
        {
            ROS_INFO("I dont know where to go QAQ");
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
