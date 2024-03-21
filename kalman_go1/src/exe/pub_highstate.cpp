#include <ros/ros.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"
#include <chrono>
#include <pthread.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>

void pub_highstate();
using namespace UNITREE_LEGGED_SDK;


//ros::Publisher pub_high = nh.advertise<sensor_msgs::Imu>("high_state", 1);

class Custom
{
public:
    UDP low_udp;
    UDP high_udp;

    HighCmd high_cmd = {0};
    HighState high_state = {0};

    LowCmd low_cmd = {0};
    LowState low_state = {0};

public:
    Custom()
        : 
        // low_udp(LOWLEVEL),
        low_udp(LOWLEVEL, 8091, "192.168.123.10", 8007),
        high_udp(8090, "192.168.123.161", 8082, sizeof(HighCmd), sizeof(HighState))
    {
        

        high_udp.InitCmdData(high_cmd);
        low_udp.InitCmdData(low_cmd);

    }

    void highUdpSend()
    {
        // printf("high udp send is running\n");

        high_udp.SetSend(high_cmd);
        high_udp.Send();
    }

    void lowUdpSend()
    {

        low_udp.SetSend(low_cmd);
        low_udp.Send();
    }

    void lowUdpRecv()
    {

        low_udp.Recv();
        low_udp.GetRecv(low_state);
    }

    void highUdpRecv()
    {
        //printf("high udp recv is running\n");

        high_udp.Recv();
        high_udp.GetRecv(high_state);
        // printf("pub is running...........和imu linearAccZ = %f \n" , high_state.imu.accelerometer[2]);

    }
};

Custom custom;
ros::Publisher pub_high;
ros::Publisher pub_state;
ros::Publisher pub_imu;

void pub_highstate()
{
        unitree_legged_msgs::HighState high_state_ros;

         high_state_ros = state2rosMsg(custom.high_state);
         high_state_ros.stamp = ros::Time::now();
         pub_high.publish(high_state_ros);

        //printf("pub is running...........和imu linearAccZ = %f \n" , custom.high_state.imu.accelerometer[2]);

        nav_msgs::Odometry ori_state;
        ori_state.header.stamp = ros::Time::now();
        ori_state.header.frame_id = "sdk_odom";
        ori_state.child_frame_id = "sdk_base_link_3d";
        ori_state.pose.pose.orientation.x = high_state_ros.imu.quaternion[1];
        ori_state.pose.pose.orientation.y = high_state_ros.imu.quaternion[2];
        ori_state.pose.pose.orientation.z = high_state_ros.imu.quaternion[3];
        ori_state.pose.pose.orientation.w = high_state_ros.imu.quaternion[0];

        ori_state.pose.pose.position.x = high_state_ros.position[0];
        ori_state.pose.pose.position.y = high_state_ros.position[1];
        ori_state.pose.pose.position.z = high_state_ros.position[2];

        ori_state.twist.twist.linear.x = high_state_ros.velocity[0];
        ori_state.twist.twist.linear.y = high_state_ros.velocity[1];
        ori_state.twist.twist.linear.z = high_state_ros.velocity[2];

        ori_state.twist.twist.angular.x = high_state_ros.imu.gyroscope[0];
        ori_state.twist.twist.angular.y = high_state_ros.imu.gyroscope[1];
        ori_state.twist.twist.angular.z = high_state_ros.imu.gyroscope[2];
        pub_state.publish(ori_state);

         sensor_msgs::Imu  imu_msgs;
         imu_msgs.header.frame_id =  "imu_link";
         imu_msgs.header.stamp = ros::Time::now();
         imu_msgs.angular_velocity.x = high_state_ros.imu.gyroscope[0];
         imu_msgs.angular_velocity.y = high_state_ros.imu.gyroscope[1];         
        imu_msgs.angular_velocity.z = high_state_ros.imu.gyroscope[2];

        imu_msgs.linear_acceleration.x = high_state_ros.imu.accelerometer[0];
        imu_msgs.linear_acceleration.y = high_state_ros.imu.accelerometer[1];
       imu_msgs.linear_acceleration.z = high_state_ros.imu.accelerometer[2];

         imu_msgs.orientation.w = high_state_ros.imu.quaternion[0];
        imu_msgs.orientation.x = high_state_ros.imu.quaternion[1];
        imu_msgs.orientation.y = high_state_ros.imu.quaternion[2];
        imu_msgs.orientation.z = high_state_ros.imu.quaternion[3];
         pub_imu.publish(imu_msgs);
        //  printf("pub is running...........和imu linearAccZ = %f" , high_state_ros.imu.accelerometer[2]);
    }




int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "pub_highstate1");
    ros::NodeHandle nh;
    pub_high = nh.advertise<unitree_legged_msgs::HighState>("high_state", 100000);
    pub_state = nh.advertise<nav_msgs::Odometry>("sdk_state",10000);
    pub_imu = nh.advertise<sensor_msgs::Imu>("imu_raw",10000);
    //sub_cmd_vel = nh.subscribe("cmd_vel", 1, cmdVelCallback);

    LoopFunc loop_udpSend("high_udp_send", 0.002, 3, boost::bind(&Custom::highUdpSend, &custom));
    LoopFunc loop_udpRecv("high_udp_recv", 0.002, 3, boost::bind(&Custom::highUdpRecv, &custom));
    LoopFunc loop_pubstate("pub_highstate" , 0.002, 3, pub_highstate);

    loop_udpSend.start();
    
    loop_udpRecv.start();
    loop_pubstate.start();
    ros::spin();

    return 0;
}
