//标准头文件
#include<ros/ros.h>
#include<iostream>
#include<std_msgs/String.h>
#include<string.h>
//navigation中需要使用的位姿信息头文件
#include<geometry_msgs/Pose.h>
#include<geometry_msgs/Point.h>
#include<geometry_msgs/PoseWithCovariance.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include<geometry_msgs/Twist.h>
#include<geometry_msgs/Quaternion.h>
//move_base头文件
#include<move_base_msgs/MoveBaseGoal.h>
#include<move_base_msgs/MoveBaseAction.h>
//actionlib头文件
#include<actionlib/client/simple_action_client.h>
#include<stdlib.h>
#include<cstdlib>

using namespace std;
//定义的全局变量
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient; //简化类型书写为MoveBaseClient

bool go_to_pos1 = false;
bool go_to_pos2 = false;
geometry_msgs::Pose pos1;
geometry_msgs::Pose pos2;
geometry_msgs::Pose goal_pose;//目标位置

//订阅器与发布器
ros::Subscriber sp_sub;
ros::Subscriber img_sub;

ros::Publisher nav_pub;

void initpose()
{
    pos1.position.x = 2.6771;
    pos1.position.y = -0.55025;
    pos1.position.z = 0;
    pos1.orientation.x = 0;
    pos1.orientation.y = 0;
    pos1.orientation.z = 0.00716835;
    pos1.orientation.w = 0.999974;
    
    pos2.position.x = 0.935774;
    pos2.position.y = -2.69883;
    pos2.position.z = 0;
    pos2.orientation.x = 0;
    pos2.orientation.y = 0;
    pos2.orientation.z = -0.999653;
    pos2.orientation.w = 0.02633;
}

/*
@TODO
添加语音模块，语音控制导航任务
解决语音无法加载声音的问题
*/

void spCallback(const std_msgs::String::ConstPtr& msg)
{
	if(msg->data == "go_to_pos1")
	{
		goal_pose = pos1;
		go_to_pos1 = true;
	}

	if(msg->data == "go_to_pos2")
	{
		goal_pose = pos2;
		go_to_pos1 = false;
		go_to_pos2 = true;
	}
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "navigation_demo");
    ros::NodeHandle nh;

	sp_sub = nh.subscribe("/speech_pub", 1, spCallback);
	nav_pub = nh.advertise<std_msgs::String>("/nav_pub", 1);
    initpose();


    MoveBaseClient  mc_("move_base", true); //建立导航客户端
	move_base_msgs::MoveBaseGoal naviGoal; //导航目标点

	while(ros::ok())
	{
		if(go_to_pos1)
		{
            ROS_INFO("****************************************");
            //goal_pose = pos1;
			//naviGoal.target_pose.header.frame_id = "map"; 
			naviGoal.target_pose.header.frame_id = "map"; 
			naviGoal.target_pose.header.stamp = ros::Time::now();
			naviGoal.target_pose.pose = geometry_msgs::Pose(goal_pose);

			while(!mc_.waitForServer(ros::Duration(5.0)))
			{
				//等待服务初始化
				cout<<"Waiting for the server..."<<endl;
			}
			mc_.sendGoal(naviGoal);
			cout << "Goal sent!" << endl;
			mc_.waitForResult(ros::Duration(40.0));

			//导航反馈直至到达目标点      
			if(mc_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			{

				cout<<"Yes! The robot has moved to the goal_pos1"<<endl;
				std_msgs::String flag;
				flag.data = "arrive_at_pos1";
				nav_pub.publish(flag);
				go_to_pos1 = false;
				cout<<"I will go to the goal_pos2"<<endl;
			}
        }
        if(go_to_pos2)
        {
            ROS_INFO("****************************************");
            //goal_pose = pos2;
			//naviGoal.target_pose.header.frame_id = "map"; 
			naviGoal.target_pose.header.frame_id = "map"; 
			naviGoal.target_pose.header.stamp = ros::Time::now();
			naviGoal.target_pose.pose = geometry_msgs::Pose(goal_pose);

			while(!mc_.waitForServer(ros::Duration(5.0)))
			{
				//等待服务初始化
				cout<<"Waiting for the server..."<<endl;
			}
			mc_.sendGoal(naviGoal);
			mc_.waitForResult(ros::Duration(40.0));

			//导航反馈直至到达目标点      
			if(mc_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
			{

				cout<<"Yes! The robot has moved to the goal_pos2"<<endl;
				cout<<"Mission Complete!"<<endl;
				std_msgs::String flag;
				flag.data = "arrive_at_pos2";
				nav_pub.publish(flag);
			}
        }
        ros::spinOnce();
    }
}