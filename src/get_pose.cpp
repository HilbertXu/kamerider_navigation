/*
    Date: 2019/04/07
    Author: Xu Yucheng
    Abstract: get waypoints and write to .txt file
*/
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <cstdlib>
using namespace std;

//define a waypoint variable
vector<geometry_msgs::PoseWithCovariance> pose;
vector<std::string> pose_name;
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
    const string end_flag = "stop";
    string input;

    cout<<"This program is used to get the four waypoints of the robot."<<endl;
    cout<<"Please input '<waypoint_name>' to get the waypoint."<<endl;
    cout<<"Please input 'stop' to end the waypoint getting."<<endl;

    while(getline(cin, input))
    {   
        //get the pose or end
        if(input != end_flag)
        {
            cout<<"Get the waypoint: "<<input<<"."<<endl;
            pose_name.push_back(input);
            ros::spinOnce();
        }
        else if(input == end_flag)
        {
            cout<<"Get pose ended!"<<endl;
            break;       
        }
    }

    //load the data into the position file
    ofstream posefile("/home/nvidia/catkin_ws/src/navigation/waypoints.txt",ios::out);  
    if(!posefile){  
        cerr<<"Cannot open waypoints.txt please check the file path"<<endl;  
        system("pause");
        exit(0);  
    }   

    for(int i = 0; i < pose.size(); i++)
    { 
        stringstream ssteam;
        string line;
        sstream << pose_name[i] << pose[i].pose.position.x << "," 
                << pose[i].pose.position.y << "," << pose[i].pose.position.z << ","
                << pose[i].pose.orientation.x << ","
                << pose[i].pose.orientation.y << ","
                << pose[i].pose.orientation.z << ","
                << pose[i].pose.orientation.w;
        sstream >> line;
        posefile << line << endl;
    }
    
    posefile.close();  
    cout<<"Processing over! Please open the file and check!"<<endl; 
    system("pause");

    return 0;    
}
