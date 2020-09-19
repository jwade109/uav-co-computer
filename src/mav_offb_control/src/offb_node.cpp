#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/ActuatorControl.h>
#include <mavros_msgs/ManualControl.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr msg){
    current_state = *msg;
}

mavros_msgs::RCIn rc_input;
void rcin_cb(const mavros_msgs::RCIn::ConstPtr msg){
    rc_input = *msg;
}

mavros_msgs::ManualControl manual_input;
void manual_cb(const mavros_msgs::ManualControl::ConstPtr msg){
    manual_input = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::spin();

    return 0;

    // subscribers
    ros::Subscriber rc_in_sub = nh.subscribe<mavros_msgs::RCIn>("mavros/rc/in", 100, rcin_cb);
    ros::Subscriber manual_in_sub = nh.subscribe<mavros_msgs::ManualControl>("mavros/manual_control/control", 100, manual_cb);
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 100, state_cb);
    // service clients
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    // publishers
    ros::Publisher actuator_pub = nh.advertise<mavros_msgs::ActuatorControl>("mavros/actuator_control", 100);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(50);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    //send a few setpoints before starting
    for(size_t i = 0; i < 100 && ros::ok(); ++i){
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    while(ros::ok())
    {
        if(current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0)))
        {
            if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else
        {
            if(!current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if(arming_client.call(arm_cmd) && arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        mavros_msgs::ActuatorControl act_con;
        if(rc_input.channels[4] < 1500)
        {
            act_con.controls[0] = manual_input.y;
            act_con.controls[1] = -manual_input.x;
            act_con.controls[2] = manual_input.r;
            act_con.controls[3] = manual_input.z;
        }
        else
        {
            act_con.controls[0] = 1.0;
            act_con.controls[1] = 1.0;
            act_con.controls[2] = 1.0;
            act_con.controls[3] = 1.0;
        }

        actuator_pub.publish(act_con);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
