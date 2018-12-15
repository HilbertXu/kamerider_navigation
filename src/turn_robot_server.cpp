#include <ros/ros.h>
#include <string.h>
#include <cmath>
#include <actionlib/server/simple_action_server.h>
#include <std_msgs/string.h>
#include <geometry_msgs/Twist.h>
#include <navigation/turn_robotAction.h>

#define PI 3.1415926

class turn_robot_action
{
protected:

    ros::NodeHandle nh_;
    actionlib::SimpleActionServer<navigation::turn_robotAction> as_;
    //必须先声明NodeHandle 否则会有奇奇怪怪的错误
    //by ROS WIKI

    //action name
    std::string action_name_;

    //Twsit 类型的消息
    geometry_msgs::Twist vel;

    //发布消息控制机器人转动
    ros::Publisher twist_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    //声明用来发布 feedback/result 的消息
    navigation::turn_robotActionFeedback feedback_;
    navigation::turn_robotActionResult result_;

public:
    
    //构造函数
    turn_robot_action(std::string name) :
        as_(ng_, name, boost::bind(&turn_robot_action::executeCallback, this, _1), false),
        action_name_(name)
        {
            as_.start();
        }
    //析构函数
    ~turn_robot_action(void) {}

    //执行回调函数，在收到来自客户端的请求的时候执行操作
    //将会由客户端传来一个目标转动角度
    void executeCallback (const navigation::turn_robotActionGoalConstPtr& goal)
    {
        ROS_INFO("%s: Executing, setting goal twist angle %i", action_name_.c_str(), goal->goal_angle);
        double rate = 50;
        ros::Rate loopRate (rate);

        //开始执行收到的action
        //根据收到的转动目标角度不同使用不同的转动角速度
        if (fabs(goal->goal_angle) < PI/12)
        {
            float angular_speed = PI / 18;
            float turn_duration = goal->goal_angle / angular_speed;
            int ticks = int (turn_duration * rate);
            
            if (goal->goal_angle > 0)
            {
                vel.angular.z = angular_speed;
                for (int i=0; i<ticks; i++)
                {
                    twist_pub_.publish (vel);
                    /*
                    @TODO
                    理解什么是抢占注册机制
                    添加feedback，内容为每次发布twist消息后转动的角度
                    */
                }
            }
        }

    }

}

