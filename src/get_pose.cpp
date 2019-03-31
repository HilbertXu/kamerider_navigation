/************************************
Author: Chengguang Xu
Date: 3/15/2016
Abstract: This is a program to get the 
          pose of the waypoint by input
          "get".
*************************************/
/*
  改进：可以get任何多的点。
*/
#include<ros/ros.h>
#include<std_msgs/String.h>
#include<geometry_msgs/PoseWithCovariance.h>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include<iostream>
#include<fstream>
#include<string>
#include<cstdlib>
using namespace std;

//define a waypoint variable
vector<geometry_msgs::PoseWithCovariance> pose;
unsigned int num = 0;

void waypointscallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{   
    pose.push_back(msg->pose);
    num++;
    cout<<"Waypoint "<<num<<" recevied!"<<endl;
}


int main(int argc, char** argv)
{
    //initialize the node named GetPose
    ros::init(argc, argv, "GetPose");
    ros::NodeHandle nh_;
    //define the waypoint subscriber
    ros::Subscriber waypoint_sub = nh_.subscribe("amcl_pose", 1, waypointscallback);

    unsigned int count = 0;

    //define the flags
    const string pose_flag = "get";
    const string end_flag = "stop";
    string input;

    cout<<"This program is used to get the four waypoints of the robot."<<endl;
    cout<<"Please input 'get' to get the waypoint."<<endl;
    cout<<"Please input 'stop' to end the waypoint getting."<<endl;

    while(getline(cin, input))
    {   
        //check the illegal input
        if(input != pose_flag && input != end_flag)
        {
            cout<<"Illegal input! Please input again."<<endl;
            continue;
        }

        //get the pose or end
        if(input == pose_flag)
        {
            cout<<"Get the waypoint "<<++count<<"."<<endl;
            ros::spinOnce();
        }else if(input == end_flag)
              {
                  cout<<"Get pose ended!"<<endl;
                  break;       
              }
    }

    //load the data into the position file
    ofstream posefile("/home/nvidia/catkin_ws/src/kamerider_navigation/waypoints.txt",ios::out);  
    if(!posefile){  
        cerr<<"Cannot open waypoints.txt please check the file path"<<endl;  
        system("pause");
        exit(0);  
    }   

    for(int i = 0; i < 10; i++)
    { 
        posefile<<"The "<<(i+1)<<" waypoint :"<<endl;
        posefile<<"Position:"<<endl;
        posefile<<"  x = "<<pose[i].pose.position.x<<endl; 
        posefile<<"  y = "<<pose[i].pose.position.y<<endl;
        posefile<<"  z = "<<pose[i].pose.position.z<<endl;
        posefile<<"Orientation:"<<endl;
        posefile<<"  x = "<<pose[i].pose.orientation.x<<endl;
        posefile<<"  y = "<<pose[i].pose.orientation.y<<endl;
        posefile<<"  z = "<<pose[i].pose.orientation.z<<endl;
        posefile<<"  w = "<<pose[i].pose.orientation.w<<endl;
        posefile<<"--------------------------"<<endl;
    }
    
    posefile.close();  
    cout<<"Processing over! Please open the file and check!"<<endl; 
    system("pause");

    return 0;    
}
