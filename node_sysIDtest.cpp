#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/Imu.h> // linear_acceleration, angular_velocity, orientation (quaternion)
#include <mavros_msgs/ActuatorControl.h> // de, da, dr, dt (not directly, pwm)
#include <geometry_msgs/PoseStamped.h> // position and orientation
#include <geometry_msgs/TwistStamped.h> // linear and angular body velocity
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>
#include <mavros_msgs/Altitude.h>
#include <iostream>
#include <fstream>
#include "rxtx.hpp"
using namespace std;

bool record_data;

sensor_msgs::Imu imu_data;
void imu_cb(const sensor_msgs::Imu::ConstPtr& msg){
    imu_data = *msg;
}

std_msgs::Float64 float64_data;
void float64_cb(const std_msgs::Float64::ConstPtr& msg){
    float64_data = *msg;
}

sensor_msgs::NavSatFix NavSat_data;
void NavSat_cb(const sensor_msgs::NavSatFix::ConstPtr& msg){
    NavSat_data = *msg;
}

mavros_msgs::ActuatorControl actuator_data;
void actuator_cb(const mavros_msgs::ActuatorControl::ConstPtr& msg){
    actuator_data = *msg;
}

geometry_msgs::PoseStamped pose_data;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    pose_data = *msg;
}

geometry_msgs::PoseWithCovarianceStamped pose_wcv_data;
void pose_wcv_cb(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){
    pose_wcv_data = *msg;
}

nav_msgs::Odometry odom_data;
void odom_cb(const nav_msgs::Odometry::ConstPtr& msg){
    odom_data = *msg;
}

geometry_msgs::TwistStamped twist_data;
void twist_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    twist_data = *msg;
}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

mavros_msgs::Altitude alt_data;
void alt_data_cb(const mavros_msgs::Altitude::ConstPtr& msg){
    alt_data = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber alt_data_sub = nh.subscribe<mavros_msgs::Altitude>
           ("mavros/altitude", 25, alt_data_cb);
    ros::Subscriber imu_data_sub = nh.subscribe<sensor_msgs::Imu>
           ("mavros/imu/data", 25, imu_cb);
    ros::Subscriber actuator_data_sub = nh.subscribe<mavros_msgs::ActuatorControl>
           ("mavros/target_actuator_control", 25, actuator_cb);
    ros::Subscriber pose_data_sub = nh.subscribe<geometry_msgs::PoseStamped>
           ("mavros/local_position/pose", 25, pose_cb);
    ros::Subscriber pose_wcv_data_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>
           ("mavros/global_position/local", 25, pose_wcv_cb);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 25, state_cb);
    ros::Subscriber odom_data_sub = nh.subscribe<nav_msgs::Odometry>
           ("mavros/local_position/odom", 25, odom_cb);
    ros::Subscriber float64_data_sub = nh.subscribe<std_msgs::Float64>
           ("mavros/global_position/rel_alt", 25, float64_cb);

    ros::Subscriber NavSat_data_sub = nh.subscribe<sensor_msgs::NavSatFix>
           ("mavros/global_position/global", 25, NavSat_cb);
    ros::Subscriber twist_data_sub = nh.subscribe<geometry_msgs::TwistStamped>
           ("mavros/global_position/gp_vel", 25, twist_cb);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");

    // reading arduino data from serial
    double rd;
    double wt;
    char charbuf;
    int buf[5] = {0,0,0,0,0};
    char *portname;
  
    if( argc > 1 ){
      portname = (char *) argv[1];
    }
    else{
    portname = (char *)"/dev/ttyACM1";
    }
  
    fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
    printf ("error %d opening %s: %s \r\n", errno, portname, strerror (errno));
    return -1;
    }
  
    set_interface_attribs (fd, B57600, 0);  // set speed to 115,200 bps, 8n1 (no parity)
    set_blocking (fd, 1);                   // set no blocking
  
  

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    // create a file
    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );
    char buffer [80];
    strftime (buffer,80,"%F(%r).csv",now);
    ofstream myfile;
    myfile.open (buffer);

    // file header
    myfile <<"Time"<<","<<"ax"<<","<<"ay"<<","<<"az"<<","<<"dE"<<","<<"dA"<<","<<"dT"<<","<<"dR"
    <<","<<"V_true"<<","<<"V_indicated"<<","<<"V1"<<","<<"V2"<<","<<"V3"<<","<<"V4"<<","<<"V5"<<","<<"u"<<","<<"v"<<","<<"w"<<
    ","<<"u"<<","<<"v"<<","<<"w"<<","<<"p"<<","<<"q"<<","<<"r"<<","<<"p"<<","<<"q"<<","<<"r"<<","<<
    "p"<<","<<"q"<<","<<"r"<<","<<"q0"<<","<<"q1"<<","<<"q2"<<","<<"q3"<<","<<"q0"<<","<<"q1"<<","<<"q2"<<","<<"q3"<<","<<
    "q0"<<","<<"q1"<<","<<"q2"<<","<<"q3"<<","<<"q0"<<","<<"q1"<<","<<"q2"<<","<<"q3"<<","<<
    "lat"<<","<<"long"<<","<<"alt"<<","<<"alt"<<","<<"x"<<","<<"y"<<","<<"z"<<","<<
    "x"<<","<<"y"<<","<<"z"<<","<<"x"<<","<<"y"<<","<<"z"<<","<<"alt_amsl"<<","<<"alt_rel"<<","<<"alt_local"<<","<<"\n";

    /*mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;*/
    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
      
            /*if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }*/

      //if (current_state.armed){

       rd = read(fd, &buf[0], sizeof(charbuf));  // data from arduino
       rd = read(fd, &buf[1], sizeof(charbuf));  // data from arduino
       rd = read(fd, &buf[2], sizeof(charbuf));  // data from arduino
       rd = read(fd, &buf[3], sizeof(charbuf));  // data from arduino
       rd = read(fd, &buf[4], sizeof(charbuf));  // data from arduino
                    
      
         myfile <<ros::Time::now()<<","<<imu_data.linear_acceleration.x<<","<<imu_data.linear_acceleration.y<<","
        <<imu_data.linear_acceleration.z<<","<<actuator_data.controls[1]<<","<<actuator_data.controls[0]<<","<<
        actuator_data.controls[3]<<","<<actuator_data.controls[2]<<","<<actuator_data.controls[5]<<","<<actuator_data.controls[7]<<","<<
        buf[0]<<","<<buf[1]<<","<<buf[2]<<","<<buf[3]<<","<<buf[4]<<","<<odom_data.twist.twist.linear.x<<","<<
        odom_data.twist.twist.linear.y<<","<<odom_data.twist.twist.linear.z<<","<<twist_data.twist.linear.x<<","
        <<twist_data.twist.linear.y<<","<<twist_data.twist.linear.z<<","<<imu_data.angular_velocity.x<<
        ","<<imu_data.angular_velocity.y<<","<<imu_data.angular_velocity.z<<","<<odom_data.twist.twist.angular.x<<","<<
        odom_data.twist.twist.angular.y<<","<<odom_data.twist.twist.angular.z<<","<<twist_data.twist.angular.x<<","
        <<twist_data.twist.angular.y<<","<<twist_data.twist.angular.z<<","<<pose_data.pose.orientation.w<<","<<
        pose_data.pose.orientation.x<<","<<pose_data.pose.orientation.y<<","<<pose_data.pose.orientation.z<<","<<
        pose_wcv_data.pose.pose.orientation.w<<","<<pose_wcv_data.pose.pose.orientation.x<<","<<pose_wcv_data.pose.pose.orientation.y<<","
        <<pose_wcv_data.pose.pose.orientation.z<<","<<imu_data.orientation.w<<","<<imu_data.orientation.x<<","<<imu_data.orientation.y
        <<","<<imu_data.orientation.z<<","<<odom_data.pose.pose.orientation.w<<","<<odom_data.pose.pose.orientation.x<<","<<
        odom_data.pose.pose.orientation.y<<","<<odom_data.pose.pose.orientation.z<<","<<NavSat_data.latitude<<","<<NavSat_data.longitude
        <<","<<NavSat_data.altitude<<","<<float64_data.data<<","<<pose_data.pose.position.x<<","<<
        pose_data.pose.position.y<<","<<pose_data.pose.position.z<<","<<odom_data.pose.pose.position.x<<","<<
        odom_data.pose.pose.position.y<<","<<odom_data.pose.pose.position.z<<","<<pose_wcv_data.pose.pose.position.x<<","<<
        pose_wcv_data.pose.pose.position.y<<","<<pose_wcv_data.pose.pose.position.z<<","<<alt_data.amsl<<","<<
        alt_data.relative<<","<<alt_data.local<<","<<"\n";
        
        ROS_INFO("saving data");
      //}



        ros::spinOnce();
        rate.sleep();
    }

    myfile.close();
    close(fd);
    return 0;
}

