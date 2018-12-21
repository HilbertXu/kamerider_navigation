/*
test code for turn robot with action
*/
#include <ros/ros.h>
#include <string.h>
#include <cmath>
#include <actionlib/server/simple_action_client.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <navigation/turn_robotAction.h>

#define PI 3.1415926

typedef actionlib::SimpleActionClient<navigation::turn_robotAction> Client;

void doneCallback (const actionlib::SimpleActionClient& state,
                   const navigation::turn_robotResultConstPtr& result)
{
    ROS_INFO ("I have turned %s degree, waiting for your further command", result->final_angle);
}

void activeCallback ()
{
    ROS_INFO ("Goal actived");
}

void feedbackCallback (const navigation::turn_robotFeedbackConstPtr& feedback)
{
    ROS_INFO ("current angle is %s", feedback->current_angle);
}

int main (int argc, char** argv)
{
    ROS_INFO ("Turn robot Client online");

    Client client("turn_robot_client", true);
    ROS_INFO ("Waiting for action server to start");
    client.waitForServer();
    ROS_INFO ("Action server actived, start sending goal");

    navigation::turn_robot goal;
    goal.goal_angle = PI/2;
    client.sendGoal (goal, &doneCallback, &activeCallback, &feedbackCallback);
    ros::spin();

    return 0;
}