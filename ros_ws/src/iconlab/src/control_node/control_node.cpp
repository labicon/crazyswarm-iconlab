#include "ros/ros.h"

#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <sstream>
#include <fstream>
#include <chrono>

#include <termios.h>
#include <stdio.h>
#include <iostream>

/* crazyflie_driver */
#include "crazyflie_driver/AddCrazyflie.h"
#include "crazyflie_driver/LogBlock.h"
#include "crazyflie_driver/GenericLogData.h"
#include "crazyflie_driver/UpdateParams.h"
#include "crazyflie_driver/UploadTrajectory.h"
#include "crazyflie_driver/NotifySetpointsStop.h"
#undef major
#undef minor
#include "crazyflie_driver/Hover.h"
#include "crazyflie_driver/Takeoff.h"
#include "crazyflie_driver/Land.h"
#include "crazyflie_driver/GoTo.h"
#include "crazyflie_driver/StartTrajectory.h"
#include "crazyflie_driver/SetGroupMask.h"
#include "crazyflie_driver/FullState.h"
#include "crazyflie_driver/Position.h"
#include "crazyflie_driver/VelocityWorld.h"


void poseCallback(const tf2_msgs::TFMessage::ConstPtr& msg) {

    /* Collect pose and timestamp information */
    std::string cf_id = msg->transforms[0].child_frame_id;
    int secs = msg->transforms[0].header.stamp.sec;
    int nsecs = msg->transforms[0].header.stamp.nsec;
    float x = msg->transforms[0].transform.translation.x;
    float y = msg->transforms[0].transform.translation.y;
    float z = msg->transforms[0].transform.translation.z;
    float qx = msg->transforms[0].transform.rotation.x;
    float qy = msg->transforms[0].transform.rotation.y;
    float qz = msg->transforms[0].transform.rotation.z;
    float qw = msg->transforms[0].transform.rotation.w;
    tf2::Quaternion q(qx, qy, qz, qw);

    /* Quaternion to roll,pitch,yaw conversion */
    double roll, pitch, yaw;
    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    /* Calculate timestamp in nanoseconds*/
    unsigned long timeStamp = (secs * 1e9) + nsecs;
    double deltaT = 0.0;


    if (cf_id == "cf1") {
        CFState_cf1.x = x;
        CFState_cf1.y = y;
        CFState_cf1.z = z;
        CFState_cf1.roll = roll;
        CFState_cf1.pitch = pitch;
        CFState_cf1.yaw = yaw;
    } else if (cf_id == "cf2") {
        CFState_cf2.x = x;
        CFState_cf2.y = y;
        CFState_cf2.z = z;
        CFState_cf2.roll = roll;
        CFState_cf2.pitch = pitch;
        CFState_cf2.yaw = yaw;
    } else if (cf_id == "cf3") {
        CFState_cf3.x = x;
        CFState_cf3.y = y;
        CFState_cf3.z = z;
        CFState_cf3.roll = roll;
        CFState_cf3.pitch = pitch;
        CFState_cf3.yaw = yaw;
    } else if (cf_id == "cf4") {
        CFState_cf4.x = x;
        CFState_cf4.y = y;
        CFState_cf4.z = z;
        CFState_cf4.roll = roll;
        CFState_cf4.pitch = pitch;
        CFState_cf4.yaw = yaw;
    }

}


/* Modification of getch() to be non-blocking */
int getch() {
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering
  newt.c_cc[VMIN] = 0; newt.c_cc[VTIME] = 0;
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings
  int c = getchar();                         // read character (non-blocking)
  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "control_node");

    ros::NodeHandle n;

    ros::Subscriber state_subscriber = n.subscribe("tf", 1000, poseCallback);

    /* Service clients for simple high-level commands */
    ros::ServiceClient takeoffClient = n.serviceClient<crazyflie_driver::Takeoff>("/takeoff");
    ros::ServiceClient landClient = n.serviceClient<crazyflie_driver::Land>("/land");
    ros::ServiceClient GoToClient_cf1 = n.serviceClient<crazyflie_driver::GoTo>("/cf1/go_to");
    ros::ServiceClient GoToClient_cf2 = n.serviceClient<crazyflie_driver::GoTo>("/cf2/go_to");
    ros::ServiceClient GoToClient_cf3 = n.serviceClient<crazyflie_driver::GoTo>("/cf3/go_to");
    ros::ServiceClient GoToClient_cf4 = n.serviceClient<crazyflie_driver::GoTo>("/cf4/go_to");

    /* Wait for <Enter> key press to begin mission */
    std::cout << "\t########## PRESS <Enter> TO BEGIN MISSION ##########" << std::endl;
    while (true) {
        int c = getch();
            if (c == '\n') {
                std::cout << "\t########## BEGINNING MISSION ##########" << std::endl;
                break;
            }
            ros::spinOnce();
    }

    /* Construct GoTo service once to be used in loop */
    crazyflie_driver::GoTo srvGoTo_cf1;
    srvGoTo_cf1.request.groupMask = 0;
    crazyflie_driver::GoTo srvGoTo_cf2;
    srvGoTo_cf2.request.groupMask = 0;
    crazyflie_driver::GoTo srvGoTo_cf3;
    srvGoTo_cf3.request.groupMask = 0;
    crazyflie_driver::GoTo srvGoTo_cf4;
    srvGoTo_cf4.request.groupMask = 0;

    /* Send takeoff command through /takeoff service */
    crazyflie_driver::Takeoff srvTakeoff;
    srvTakeoff.request.groupMask = 0;
    srvTakeoff.request.height = 1.0;
    srvTakeoff.request.duration = ros::Duration(3.0);
    takeoffClient.call(srvTakeoff);
    ros::Duration(3.0).sleep();

    
    while (ros::ok()) {

        int c = getch(); // Get the character that was typed

	// If the "Enter" key was pressed, break and land immediately
        if (c == '\n') {
            std::cout << "########## LANDING ##########" << std::endl;
            break;
        }

	// If the "o" key was pressed, bring CFs back to their original
	// locations and land
        if (c == 'o') {
            std::cout << "########## RETURN TO ORIGIN ##########" << std::endl;
            srvGoTo_cf1.request.goal.x = -0.2;
            srvGoTo_cf1.request.goal.y = 1.1;
            srvGoTo_cf1.request.goal.z = 1.5;
            srvGoTo_cf1.request.yaw = 0.0;
            srvGoTo_cf1.request.duration = ros::Duration(3.0);
            GoToClient_cf1.call(srvGoTo_cf1);
            srvGoTo_cf2.request.goal.x = 0.55;
            srvGoTo_cf2.request.goal.y = 0.3;
            srvGoTo_cf2.request.goal.z = 0.5;
            srvGoTo_cf2.request.yaw = 0.0;
            srvGoTo_cf2.request.duration = ros::Duration(3.0);
            GoToClient_cf2.call(srvGoTo_cf2);
            ros::Duration(3.0).sleep();
            break;
        }	    
    }


    crazyflie_driver::Land srvLand;
    srvLand.request.duration = ros::Duration(3.0);
    landClient.call(srvLand);

    ros::spin();

    return 0;
}
