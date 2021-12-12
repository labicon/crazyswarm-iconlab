#include "ros/ros.h"

#include <tf/transform_listener.h>
// #include <tf_conversions/tf_eigen.h>
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

/* Includes form iLQR main.cpp */
#include <unsupported/Eigen/AdolcForward>
#include <Eigen/Dense>
#include <adolc/adolc.h>
#include <adolc/adouble.h>
#include <vector>
#include <time.h>
#include <math.h>
#include <fstream>
#include <thread>
#include <future>
#include <algorithm>
#include <iterator>
#include "cost.h"
#include "dynamics.h"
#include "iLQR.h"
#include "utils.h"
#include <unistd.h>

/* simple moving average filter */
#define USE_SMA_FILTER 0
#define SMA_WINDOW_LEN 10

/* first order exponential low-pass filter */
#define USE_EXP_FILTER 1
#define EXP_FILT_ALPHA 0.9f


// static stateVector prevState_cf1;
// static stateVector stateWindowBuf_cf1[SMA_WINDOW_LEN];

// static stateVector prevState_cf2;
// static stateVector stateWindowBuf_cf2[SMA_WINDOW_LEN];

static stateVector CFState_cf1;
static stateVector CFState_cf2;
static stateVector CFState_cf3;
static stateVector CFState_cf4;

static pose currentPose;
// static pose prevPose_cf1;

// static pose currentPose_cf2;
// static pose prevPose_cf2;

std::ofstream droneDatalogFile;
std::ofstream optimizerDatalogFile;

bool first = true;

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

    std::vector<stateVector> states{CFState_cf1, CFState_cf2, CFState_cf3, CFState_cf4};

    logDroneStates(droneDatalogFile, states);

    // std::printf("\n");
    // std::printf("##### CF1:\n");
    // std::printf("X: %f,\t Y: %f,\t Z: %f\t\n ROLL: %f,\t PITCH: %f,\t YAW %f\n",
    //             CFState_cf1.x, CFState_cf1.y, CFState_cf1.z, CFState_cf1.roll, CFState_cf1.pitch, CFState_cf1.yaw);
    // std::printf("X_d: %f\t, Y_d: %f\t, Z_d: %f\t\nROLL_d: %f\t, PITCH_d: %f\t, YAW_d %f\n",
    //             CFState_cf1.x_d, CFState_cf1.y_d, CFState_cf1.z_d, CFState_cf1.roll_d, CFState_cf1.pitch_d, CFState_cf1.yaw_d);

    // std::printf("\n##### CF2:\n");
    // std::printf("X: %f,\t Y: %f,\t Z: %f\t\n ROLL: %f,\t PITCH: %f,\t YAW %f\n",
    //             CFState_cf2.x, CFState_cf2.y, CFState_cf2.z, CFState_cf2.roll, CFState_cf2.pitch, CFState_cf2.yaw);
    // std::printf("X_d: %f\t, Y_d: %f\t, Z_d: %f\t\nROLL_d: %f\t, PITCH_d: %f\t, YAW_d %f\n",
    //             CFState_cf2.x_d, CFState_cf2.y_d, CFState_cf2.z_d, CFState_cf2.roll_d, CFState_cf2.pitch_d, CFState_cf2.yaw_d);
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

    srand((unsigned int) time(0));

    init_datalog(droneDatalogFile,
                 "/home/ayberk/Documents/ACTIONLAB/crazyswarm/ros_ws/src/iLQR/datalog/",
                 "droneStates_2x");

    init_datalog(optimizerDatalogFile,
                 "/home/ayberk/Documents/ACTIONLAB/crazyswarm/ros_ws/src/iLQR/datalog/",
                 "optimizer_2x");

    ros::init(argc, argv, "iLQR_Node");

    ros::NodeHandle n;

    ros::Subscriber state_subscriber = n.subscribe("tf", 1000, poseCallback);

    /* Publisher to cmd_position topic (currently doesn't work) */
    // ros::Publisher pub = n.advertise<crazyflie_driver::Position>("cf1/cmd_position",1);
    // crazyflie_driver::Position position_cmd;

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
    srvGoTo_cf1.request.groupMask = 0;  // signal all CFs (I think?)
    crazyflie_driver::GoTo srvGoTo_cf2;
    srvGoTo_cf2.request.groupMask = 0;
    crazyflie_driver::GoTo srvGoTo_cf3;
    srvGoTo_cf3.request.groupMask = 0;
    crazyflie_driver::GoTo srvGoTo_cf4;
    srvGoTo_cf4.request.groupMask = 0;

    /* Send takeoff command through /Takeoff service */
    crazyflie_driver::Takeoff srvTakeoff;
    srvTakeoff.request.groupMask = 0;
    srvTakeoff.request.height = 1.0;
    srvTakeoff.request.duration = ros::Duration(3.0);
    takeoffClient.call(srvTakeoff);
    ros::Duration(3.0).sleep();







    // srvGoTo_cf1.request.goal.x = 0.0;
    // srvGoTo_cf1.request.goal.y = 1.25;
    // srvGoTo_cf1.request.goal.z = 1.5;
    // srvGoTo_cf1.request.yaw = 0.0;
    // srvGoTo_cf1.request.duration = ros::Duration(2.0);
    // GoToClient_cf1.call(srvGoTo_cf1);
    // srvGoTo_cf2.request.goal.x = 1.25;
    // srvGoTo_cf2.request.goal.y = 0.0;
    // srvGoTo_cf2.request.goal.z = 1.5;
    // srvGoTo_cf2.request.yaw = 0.0;
    // srvGoTo_cf2.request.duration = ros::Duration(2.0);
    // GoToClient_cf2.call(srvGoTo_cf2);
    // // ros::Duration(2.0).sleep();
    // srvGoTo_cf3.request.goal.x = 0.0;
    // srvGoTo_cf3.request.goal.y = 1.0;
    // srvGoTo_cf3.request.goal.z = 1.0;
    // srvGoTo_cf3.request.yaw = 0.0;
    // srvGoTo_cf3.request.duration = ros::Duration(2.0);
    // GoToClient_cf3.call(srvGoTo_cf3);
    // srvGoTo_cf4.request.goal.x = 1.5;
    // srvGoTo_cf4.request.goal.y = 0.0;
    // srvGoTo_cf4.request.goal.z = 1.0;
    // srvGoTo_cf4.request.yaw = 0.0;
    // srvGoTo_cf4.request.duration = ros::Duration(2.0);
    // GoToClient_cf4.call(srvGoTo_cf4);
    // ros::Duration(2.0).sleep();


//     /* iLQR solver parameters */
//     constexpr size_t horizon=5;
//     constexpr int total_steps=300;
//     unsigned int tag1(1),tag2(2),tag3(3),tag4(4),tag5(5),
//                  tag6(6),tag7(7),tag8(8),tag9(9);
//
//     double time_step=0.2;
//
//
//     /*First Order 2 Drone Simulation*/
//
// 	cost<12,12> drone2_running_cost(Drone_First_Order_Cost::running_cost2,tag7);
// 	cost<12,12> drone2_terminal_cost(Drone_First_Order_Cost::terminal_cost2,tag8);
// 	dynamics<12,12> drone2_dynamics(Drone_First_Order_Dynamics::dynamics_2,tag9,time_step);
// 	iLQR<12,12,horizon>::input_trajectory u_init_drone2;
// 	iLQR<12,12,horizon> drone2_solver(drone2_running_cost,drone2_terminal_cost,drone2_dynamics);
// 	Eigen::Matrix<double,12,1> u_drone2=Eigen::Matrix<double,12,1>::Zero();
// 	Eigen::Matrix<double,12,1> x0_drone2=Eigen::Matrix<double,12,1>::Zero();
// 	Eigen::Matrix<double,12,1> x_goal_drone2;
// 	iLQR<12,12,horizon>::state_input_trajectory soln_drone2;
// 	iLQR<12,12,total_steps>::state_input_trajectory soln_drone2_rhc;
//
// 	//	u_drone2=u_drone2*sqrt(Drone_Dynamics::mass*Drone_Dynamics::g/(4*Drone_Dynamics::C_T));
// 	std::fill(std::begin(u_init_drone2), std::end(u_init_drone2), u_drone2);
//     for (auto &item : u_init_drone2)
//         item = Eigen::Matrix<double,12,1>::Random()*5;
//
// //	iLQR<12*2,4*2,horizon>::state_input_trajectory soln_drone2;
// //	iLQR<12*2,4*2,total_steps>::state_input_trajectory soln_drone2_rhc;
//
// 	clock_t t_start, t_end;
//
//
// 	double seconds;
// 	drone2_solver.set_MPC(u_init_drone2);
// 	int execution_steps=1;
// 	soln_drone2_rhc.first[0]=x0_drone2;
// 	// std::string state_path="/Users/talhakavuncu/Desktop/research/cpp_code/states.txt";
// 	// std::string input_path="/Users/talhakavuncu/Desktop/research/cpp_code/inputs.txt";
// 	t_start = clock();
//
//     x_goal_drone2 << 1.25, 0.0, 2.5, 0,0,0,
//                      0.0, 1.25,  2.5, 0,0,0;

    while (ros::ok())
//     // while (true)
	{
//
//         ros::spinOnce();
//
//
// 		x0_drone2 << CFState_cf1.x, CFState_cf1.y, CFState_cf1.z,
//                      CFState_cf1.roll, CFState_cf1.pitch, CFState_cf1.yaw,
//                      CFState_cf2.x, CFState_cf2.y, CFState_cf2.z,
//                      CFState_cf2.roll, CFState_cf2.pitch, CFState_cf2.yaw;
//
//
// 		soln_drone2=drone2_solver.run_MPC(x0_drone2,
// 				x_goal_drone2, 50, 2, execution_steps);
//
//         /* Optimizer output logging */
//         std::string data;
//         for (int i = 0; i < horizon; ++i) {
//             for (int q = 0; q < 12; ++q) {
//                 data += std::to_string(soln_drone2.first[i][q]) + ",";
//             }
//
//         }
//         const auto p1 = std::chrono::system_clock::now();
//     	data += std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(p1.time_since_epoch()).count()) + "\n";
//     	optimizerDatalogFile << data;
//
//         int idx_to_use = 3;
//
//         std::printf("\n#### SENDING WAYPOINT CF1  ");
//         std::printf("X: %f,\t Y: %f,\t Z: %f\n", soln_drone2.first[idx_to_use][0], soln_drone2.first[idx_to_use][1], soln_drone2.first[idx_to_use][2]);
//         std::printf("\n#### SENDING WAYPOINT CF2   ");
//         std::printf("X: %f,\t Y: %f,\t Z: %f\n", soln_drone2.first[idx_to_use][6], soln_drone2.first[idx_to_use][7], soln_drone2.first[idx_to_use][8]);
//
//         srvGoTo_cf1.request.goal.x = soln_drone2.first[idx_to_use][0];
//         srvGoTo_cf1.request.goal.y = soln_drone2.first[idx_to_use][1];
//         srvGoTo_cf1.request.goal.z = soln_drone2.first[idx_to_use][2];
//         srvGoTo_cf1.request.yaw = 0.0;
//         srvGoTo_cf1.request.duration = ros::Duration(0.8);
//         GoToClient_cf1.call(srvGoTo_cf1);
//
//         srvGoTo_cf2.request.goal.x = soln_drone2.first[idx_to_use][6];
//         srvGoTo_cf2.request.goal.y = soln_drone2.first[idx_to_use][7];
//         srvGoTo_cf2.request.goal.z = soln_drone2.first[idx_to_use][8];
//         srvGoTo_cf2.request.yaw = 0.0;
//         srvGoTo_cf2.request.duration = ros::Duration(0.8);
//         GoToClient_cf2.call(srvGoTo_cf2);
//
        int c = getch();   // call your non-blocking input function
        if (c == '\n') {
            std::cout << "########## LANDING ##########" << std::endl;
            break;
        }
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
//
//
	}
// 	t_end = clock();
// ////
// //////
// ////////	soln_drone=drone_solver.solve_open_loop(X0_drone, X0_drone, 200, u_init_drone, horizon);
// ////////	write_file<12,4,horizon>(state_path,input_path, soln_drone);
// 	// write_file<12,12,total_steps>(state_path,input_path, soln_drone2_rhc);
// ////
// 	std::cout<<"finished"<<std::endl;
// 	seconds = 1/((double)(t_end - t_start) / CLOCKS_PER_SEC/total_steps);
// 	printf("Time: %f s\n", seconds);


    /*First Order 2 Drone Simulation Ends*/




    crazyflie_driver::Land srvLand;
    srvLand.request.duration = ros::Duration(3.0);
    landClient.call(srvLand);

    ros::spin();

    return 0;
}

/*############################################################################*/
/*############################################################################*/
/*############################################################################*/



// /*First Order Drone Simulation */
//
//
//
// 	cost<6,6> drone_running_cost(Drone_First_Order_Cost::running_cost,tag7);
// 	cost<6,6> drone_terminal_cost(Drone_First_Order_Cost::terminal_cost,tag8);
// 	dynamics<6,6> drone_dynamics(Drone_First_Order_Dynamics::dynamics,tag9,time_step);
// 	iLQR<6,6,horizon>::input_trajectory u_init_drone;
// 	iLQR<6,6,horizon> drone_solver(drone_running_cost,drone_terminal_cost,drone_dynamics);
// 	Eigen::Matrix<double,6,1> u_drone=Eigen::Matrix<double,6,1>::Zero();
// 	Eigen::Matrix<double,6,1> x0_drone=Eigen::Matrix<double,6,1>::Zero();
// 	Eigen::Matrix<double,6,1> x_goal_drone;
// 	iLQR<6,6,horizon>::state_input_trajectory soln_drone;
// 	iLQR<6,6,total_steps>::state_input_trajectory soln_drone_rhc;
//
// 	//	u_drone2=u_drone2*sqrt(Drone_Dynamics::mass*Drone_Dynamics::g/(4*Drone_Dynamics::C_T));
// 		std::fill(std::begin(u_init_drone), std::end(u_init_drone), u_drone);
// 	//	iLQR<12*2,4*2,horizon>::state_input_trajectory soln_drone2;
// 	//	iLQR<12*2,4*2,total_steps>::state_input_trajectory soln_drone2_rhc;
//
//
//
//
//
// 		clock_t t_start, t_end;
// 	//
// 	////
// 		x0_drone<<0,0,0.5, 0,0,0;
// 	//	x0_integrator+=0.1*Eigen::Matrix<double,3,1>::Random();
// 		double seconds;
// 		drone_solver.set_MPC(u_init_drone);
// 		int execution_steps=1;
// 		// soln_drone_rhc.first[0]=x0_drone;
// 		// std::string state_path="/Users/talhakavuncu/Desktop/research/cpp_code/states.txt";
// 		// std::string input_path="/Users/talhakavuncu/Desktop/research/cpp_code/inputs.txt";
// 		t_start = clock();
// 		// for(int i=0;i<total_steps;++i)
//
//         x_goal_drone<<1,1,1, 0,0,0;
//
//         while (ros::ok())
// 		{
//             ros::spinOnce();
//
// 	//		if (i>total_steps/3)
// 	//			x_goal_integrator<<0,0,0;
// 			// std::cout<<"iteration "<<i<<std::endl;
// 			// double scale=0.01;
// 			// auto term1=soln_drone_rhc.first[i]+scale*Eigen::Matrix<double,6,1>::Random();
// 			// auto term2=soln_drone_rhc.first[i]+scale*Eigen::Matrix<double,6,1>::Random();
// //			auto term3=soln_drone_rhc.first[i]+scale*Eigen::Matrix<double,6,1>::Random();
// //			auto term4=soln_drone_rhc.first[i]+scale*Eigen::Matrix<double,6,1>::Random();
// //			auto term5=soln_drone_rhc.first[i]+scale*Eigen::Matrix<double,6,1>::Random();
// //			auto term6=soln_drone_rhc.first[i]+scale*Eigen::Matrix<double,6,1>::Random();
// //			auto term7=soln_drone_rhc.first[i]+scale*Eigen::Matrix<double,6,1>::Random();
// //			auto term8=soln_drone_rhc.first[i]+scale*Eigen::Matrix<double,6,1>::Random();
// //			auto term9=soln_drone_rhc.first[i]+scale*Eigen::Matrix<double,6,1>::Random();
// //			auto term10=soln_drone_rhc.first[i]+scale*Eigen::Matrix<double,6,1>::Random();
// //			auto term=(term1+term2+term3+term4+term5+term6+term7+term8+term9+term10)/10;
//
//             x0_drone << CFState_cf1.x, CFState_cf1.y, CFState_cf1.z,
//                         CFState_cf1.roll, CFState_cf1.pitch,  CFState_cf1.yaw;
//
// 			soln_drone=drone_solver.run_MPC(x0_drone,
// 					x_goal_drone, 5, execution_steps);
//
//             int idx_to_use = 1;
//
//             std::printf("\n#### SENDING WAYPOINT   ");
//             std::printf("X: %f,\t Y: %f,\t Z: %f\n", soln_drone.first[idx_to_use][0], soln_drone.first[idx_to_use][1], soln_drone.first[idx_to_use][2]);
//
//             srvGoTo_cf1.request.goal.x = soln_drone.first[idx_to_use][0];
//             srvGoTo_cf1.request.goal.y = soln_drone.first[idx_to_use][1];
//             srvGoTo_cf1.request.goal.z = soln_drone.first[idx_to_use][2];
//             srvGoTo_cf1.request.yaw = 0.0;
//             srvGoTo_cf1.request.duration = ros::Duration(1.0);
//             // GoToClient_cf1.call(srvGoTo_cf1);
//
//             int c = getch();   // call your non-blocking input function
//             if (c == '\n') {
//                 std::cout << "########## LANDING ##########" << std::endl;
//                 break;
//             }
//             if (c == 'o') {
//                 std::cout << "########## RETURN TO ORIGIN ##########" << std::endl;
//                 srvGoTo_cf1.request.goal.x = 0.5;
//                 srvGoTo_cf1.request.goal.y = 0.5;
//                 srvGoTo_cf1.request.goal.z = 0.5;
//                 srvGoTo_cf1.request.yaw = 0.0;
//                 srvGoTo_cf1.request.duration = ros::Duration(3.0);
//                 GoToClient_cf1.call(srvGoTo_cf1);
//                 srvGoTo_cf2.request.goal.x = -0.5;
//                 srvGoTo_cf2.request.goal.y = -0.5;
//                 srvGoTo_cf2.request.goal.z = 0.5;
//                 srvGoTo_cf2.request.yaw = 0.0;
//                 srvGoTo_cf2.request.duration = ros::Duration(3.0);
//                 GoToClient_cf2.call(srvGoTo_cf2);
//                 ros::Duration(3.0).sleep();
//                 break;
//             }
// 	////
// 	////////		unsigned int microsecond = 1000000;
// 	////////		usleep(2 * microsecond);
// 	////		write_file<12,4,horizon>(state_path,input_path, soln_drone);
// 			// soln_drone_rhc.first[i+1]=soln_drone.first[1];
// 			// soln_drone_rhc.second[i]=soln_drone.second[0];
// 		}
// 		t_end = clock();
// 	////
// 	//////
// 	////////	soln_drone=drone_solver.solve_open_loop(X0_drone, X0_drone, 200, u_init_drone, horizon);
// 	////////	write_file<12,4,horizon>(state_path,input_path, soln_drone);
// 		// write_file<6,6,total_steps>(state_path,input_path, soln_drone_rhc);
// 	////
// 		std::cout<<"finished"<<std::endl;
// 		seconds = 1/((double)(t_end - t_start) / CLOCKS_PER_SEC/total_steps);
// 		printf("Time: %f s\n", seconds);
//
//
//
//
// /*First Order Drone Simulation Ends*/


// /* This part is single drone simulation*/
//
//     cost<12,4> running_cost_dr(Drone_Cost::running_cost,tag1);
//     cost<12,4> terminal_cost_dr(Drone_Cost::terminal_cost,tag2);
//     dynamics<12,4> dynamics_dr(Drone_Dynamics::dynamics,tag3,time_step);
//
//     iLQR<12,4,horizon> drone_solver(running_cost_dr,terminal_cost_dr,dynamics_dr);
//     Eigen::Matrix<double,4,1> u_drone=Eigen::Matrix<double,4,1>::Ones();
//     Eigen::Matrix<double,12,1> X0_drone=Eigen::Matrix<double,12,1>::Zero();
// ////	X0_drone<<0,-50,10, M_PI/20,0,0, 0,0,0, 0,0,0;
//     Eigen::Matrix<double,12,1> X_goal_drone;
//
//     u_drone=u_drone*sqrt(Drone_Dynamics::mass*Drone_Dynamics::g/(4*Drone_Dynamics::C_T));
// //
//     iLQR<12,4,horizon>::input_trajectory u_init_drone;
//     std::fill(std::begin(u_init_drone), std::end(u_init_drone), u_drone);
//     iLQR<12,4,horizon>::state_input_trajectory soln_drone;
//     iLQR<12,4,total_steps>::state_input_trajectory soln_drone_rhc;
//
//     clock_t t_start, t_end;
//
// //
//     double seconds;
//     drone_solver.set_MPC(u_init_drone);
//     int execution_steps=1;
//     soln_drone_rhc.first[0]=X0_drone;
//     // std::string state_path="/Users/talhakavuncu/Desktop/research/cpp_code/states.txt";
//     // std::string input_path="/Users/talhakavuncu/Desktop/research/cpp_code/inputs.txt";
//     t_start = clock();
//     // for(int i=0;i<total_steps;++i)
//
//
//     X_goal_drone<<-1.0,1.0,1.0, 0,0,0, 0,0,0, 0,0,0;
//
//
//     while (ros::ok())
//     {
//         // if (i>total_steps/3)
//         //     X_goal_drone<<0,0,0, 0,0,0, 0,0,0, 0,0,0;
//         // std::cout<<"iteration "<<i<<std::endl;
//
//         ros::spinOnce();
// ////
//         X0_drone << CFState_cf1.x, CFState_cf1.y, CFState_cf1.z,
//                     CFState_cf1.roll, CFState_cf1.pitch,  CFState_cf1.yaw,
//                     CFState_cf1.x_d, CFState_cf1.y_d, CFState_cf1.z_d,
//                     CFState_cf1.roll_d, CFState_cf1.pitch_d, CFState_cf1.yaw_d;
//
//         soln_drone=drone_solver.run_MPC(X0_drone, X_goal_drone, 20, execution_steps);
//
//         // ros::Duration(0.5).sleep();
// //
// //////		unsigned int microsecond = 1000000;
// //////		usleep(2 * microsecond);
// //		write_file<12,4,horizon>(state_path,input_path, soln_drone);
//         // soln_drone_rhc.first[i+1]=soln_drone.first[1];
//         // soln_drone_rhc.second[i]=soln_drone.second[0];
//
//         // std::printf("\n\n");
//         // std::cout << X0_drone << std::endl;
//         // std::printf("X: %f,\t Y: %f,\t Z: %f\n", soln_drone.first[4][0], soln_drone.first[4][1], soln_drone.first[4][2]);
//
//         std::printf("\n#### SENDING WAYPOINT   ");
//         std::printf("X: %f,\t Y: %f,\t Z: %f\n", soln_drone.first[4][0], soln_drone.first[4][1], soln_drone.first[4][2]);
//
//         srvGoTo_cf1.request.goal.x = soln_drone.first[4][0];
//         srvGoTo_cf1.request.goal.y = soln_drone.first[4][1];
//         srvGoTo_cf1.request.goal.z = soln_drone.first[4][2];
//         srvGoTo_cf1.request.yaw = 0.0;
//         srvGoTo_cf1.request.duration = ros::Duration(3.0);
//         GoToClient_cf1.call(srvGoTo_cf1);
//
//         int c = getch();   // call your non-blocking input function
//         if (c == '\n') {
//             std::cout << "########## LANDING ##########" << std::endl;
//             break;
//         }
//
//         // ros::spinOnce();
//
//     }
//     t_end = clock();
//
//
// ////
// ////
// //////	soln_drone=drone_solver.solve_open_loop(X0_drone, X0_drone, 200, u_init_drone, horizon);
// //////	write_file<12,4,horizon>(state_path,input_path, soln_drone);
//     // write_file<12,4,total_steps>(state_path,input_path, soln_drone_rhc);
// //
//     std::cout<<"finished"<<std::endl;
//     seconds = 1/((double)(t_end - t_start) / CLOCKS_PER_SEC/total_steps);
//     printf("Time: %f s\n", seconds);
//
//
// /* End of single drone simulation*/


// cost<3,3> integrator_running_cost(Single_Integrator_Cost::running_cost,tag7);
// cost<3,3> integrator_terminal_cost(Single_Integrator_Cost::terminal_cost,tag8);
// dynamics<3,3> integrator_dynamics(Single_Integrator_3D::dynamics,tag9,time_step);
// iLQR<3,3,horizon>::input_trajectory u_init_integrator;
// iLQR<3,3,horizon> integrator_solver(integrator_running_cost,integrator_terminal_cost,integrator_dynamics);
// Eigen::Matrix<double,3,1> u_integrator=Eigen::Matrix<double,3,1>::Zero();
// Eigen::Matrix<double,3,1> x0_integrator=Eigen::Matrix<double,3,1>::Zero();
// Eigen::Matrix<double,3,1> x_goal_integrator;
// iLQR<3,3,horizon>::state_input_trajectory soln_integrator;
// iLQR<3,3,total_steps>::state_input_trajectory soln_integrator_rhc;
//
// //	u_drone2=u_drone2*sqrt(Drone_Dynamics::mass*Drone_Dynamics::g/(4*Drone_Dynamics::C_T));
// //	std::fill(std::begin(u_init_drone2), std::end(u_init_drone2), u_drone2);
// //	iLQR<12*2,4*2,horizon>::state_input_trajectory soln_drone2;
// //	iLQR<12*2,4*2,total_steps>::state_input_trajectory soln_drone2_rhc;
//
//
// clock_t t_start, t_end;
//
// double seconds;
// integrator_solver.set_MPC(u_init_integrator);
// int execution_steps=1;
// soln_integrator_rhc.first[0]=x0_integrator;
// std::string state_path="/home/ayberk/Documents/ACTIONLAB/crazyswarm/ros_ws/src/iLQR/datalog/states.txt";
// std::string input_path="/home/ayberk/Documents/ACTIONLAB/crazyswarm/ros_ws/src/iLQR/datalog/inputs.txt";
// t_start = clock();
//
// /* Goal position of trajectory */
// x_goal_integrator << -1.0,1.0,1.0;
//
// while (ros::ok())
// {
//
//     /* Spin once for ROS callback to work */
//     ros::spinOnce();
//
//     /* populate state vector with current state */
//     x0_integrator << CFState_cf1.x, CFState_cf1.y, CFState_cf1.z;
//
//     /* run iLQR algorithm */
// 	soln_integrator = integrator_solver.run_MPC(x0_integrator,
// 			                                    x_goal_integrator,
//                                                 20,
//                                                 execution_steps);
//
//     /* Write solution to file for debugging */
//     write_file<3,3,horizon>(state_path, input_path, soln_integrator);
// 	// soln_integrator_rhc.first[i+1]=soln_integrator.first[1];
// 	// soln_integrator_rhc.second[i]=soln_integrator.second[0];
//
//     // std::printf("\nX0_integrator: \n");
//     // std::cout << x0_integrator << std::endl;
//
//     std::printf("\n#### SENDING WAYPOINT");
//     std::printf("X: %f,\t Y: %f,\t Z: %f\n", soln_integrator.first[1][0], soln_integrator.first[1][1], soln_integrator.first[1][2]);
//
//     /* Populate GoTo command with waypoint and call service */
//     srvGoTo_cf1.request.goal.x = soln_integrator.first[1][0];
//     srvGoTo_cf1.request.goal.y = soln_integrator.first[1][1];
//     srvGoTo_cf1.request.goal.z = soln_integrator.first[1][2];
//     srvGoTo_cf1.request.yaw = 0.0;
//     srvGoTo_cf1.request.duration = ros::Duration(1.0);
//     GoToClient_cf1.call(srvGoTo_cf1);
//
//     // srvGoTo_cf2.request.goal.x = soln_integrator.first[4][0];
//     // srvGoTo_cf2.request.goal.y = soln_integrator.first[4][1];
//     // srvGoTo_cf2.request.goal.z = soln_integrator.first[4][2];
//     // srvGoTo_cf2.request.yaw = 0.0;
//     // srvGoTo_cf2.request.duration = ros::Duration(3.0);
//     // GoToClient_cf2.call(srvGoTo_cf2);
//
//
//     /* Check if <Enter> key way pressed */
//     int c = getch();
//     if (c == '\n') {
//         std::cout << "\t##### LANDING #####" << std::endl;
//         break;
//     }
//
//
// }
// t_end = clock();
//
//
// // soln_drone=drone_solver.solve_open_loop(X0_drone, X0_drone, 200, u_init_drone, horizon);
// // write_file<12,4,horizon>(state_path,input_path, soln_drone);
// // write_file<3,3,total_steps>(state_path,input_path, soln_integrator_rhc);
//
//
// std::cout<<"finished"<<std::endl;
// seconds = 1/((double)(t_end - t_start) / CLOCKS_PER_SEC/total_steps);
// printf("Time: %f s\n", seconds);




// /* This part is single drone simulation*/
//
//     cost<12,4> running_cost_dr(Drone_Cost::running_cost,tag1);
//     cost<12,4> terminal_cost_dr(Drone_Cost::terminal_cost,tag2);
//     dynamics<12,4> dynamics_dr(Drone_Dynamics::dynamics,tag3,time_step);
//
//     iLQR<12,4,horizon> drone_solver(running_cost_dr,terminal_cost_dr,dynamics_dr);
//     Eigen::Matrix<double,4,1> u_drone=Eigen::Matrix<double,4,1>::Ones();
//     Eigen::Matrix<double,12,1> X0_drone=Eigen::Matrix<double,12,1>::Zero();
// ////	X0_drone<<0,-50,10, M_PI/20,0,0, 0,0,0, 0,0,0;
//     Eigen::Matrix<double,12,1> X_goal_drone;
//
//     u_drone=u_drone*sqrt(Drone_Dynamics::mass*Drone_Dynamics::g/(4*Drone_Dynamics::C_T));
// //
//     iLQR<12,4,horizon>::input_trajectory u_init_drone;
//     std::fill(std::begin(u_init_drone), std::end(u_init_drone), u_drone);
//     iLQR<12,4,horizon>::state_input_trajectory soln_drone;
//     iLQR<12,4,total_steps>::state_input_trajectory soln_drone_rhc;
//
//     std::cout << "DEBUG 1" << std::endl;
//
//     clock_t t_start, t_end;
//
// //
//     double seconds;
//     drone_solver.set_MPC(u_init_drone);
//     int execution_steps=1;
//     soln_drone_rhc.first[0]=X0_drone;
//     // std::string state_path="/Users/talhakavuncu/Desktop/research/cpp_code/states.txt";
//     // std::string input_path="/Users/talhakavuncu/Desktop/research/cpp_code/inputs.txt";
//     t_start = clock();
//     // for(int i=0;i<total_steps;++i)
//
//
//     X_goal_drone<<1.0,1.0,1.5, 0,0,0, 0,0,0, 0,0,0;
//
//
//     while (ros::ok())
//     {
//         // if (i>total_steps/3)
//         //     X_goal_drone<<0,0,0, 0,0,0, 0,0,0, 0,0,0;
//         // std::cout<<"iteration "<<i<<std::endl;
//
//         ros::spinOnce();
// ////
//         X0_drone << CFState_cf1.x, CFState_cf1.y, CFState_cf1.z,
//                     CFState_cf1.roll, CFState_cf1.pitch,  CFState_cf1.yaw,
//                     CFState_cf1.x_d, CFState_cf1.y_d, CFState_cf1.z_d,
//                     CFState_cf1.roll_d, CFState_cf1.pitch_d, CFState_cf1.yaw_d;
//
//         soln_drone=drone_solver.run_MPC(X0_drone, X_goal_drone, 100, execution_steps);
//
//         // ros::Duration(0.5).sleep();
// //
// //////		unsigned int microsecond = 1000000;
// //////		usleep(2 * microsecond);
// //		write_file<12,4,horizon>(state_path,input_path, soln_drone);
//         // soln_drone_rhc.first[i+1]=soln_drone.first[1];
//         // soln_drone_rhc.second[i]=soln_drone.second[0];
//
//         std::printf("\n\n");
//         std::cout << X0_drone << std::endl;
//         std::printf("X: %f,\t Y: %f,\t Z: %f\n", soln_drone.first[39][0], soln_drone.first[39][1], soln_drone.first[39][2]);
//
//         srvGoTo_cf2.request.goal.x = soln_drone.first[39][0];
//         srvGoTo_cf2.request.goal.y = soln_drone.first[39][1];
//         srvGoTo_cf2.request.goal.z = soln_drone.first[39][2];
//         srvGoTo_cf2.request.yaw = 0.0;
//         srvGoTo_cf2.request.duration = ros::Duration(5.0);
//         GoToClient_cf1.call(srvGoTo_cf2);
//
//         int c = getch();   // call your non-blocking input function
//         if (c == '\n') {
//             std::cout << "LANDING" << std::endl;
//             break;
//         }
//
//         // ros::spinOnce();
//
//     }
//     t_end = clock();
//
//
// ////
// ////
// //////	soln_drone=drone_solver.solve_open_loop(X0_drone, X0_drone, 200, u_init_drone, horizon);
// //////	write_file<12,4,horizon>(state_path,input_path, soln_drone);
//     // write_file<12,4,total_steps>(state_path,input_path, soln_drone_rhc);
// //
//     std::cout<<"finished"<<std::endl;
//     seconds = 1/((double)(t_end - t_start) / CLOCKS_PER_SEC/total_steps);
//     printf("Time: %f s\n", seconds);
//
//
// /* End of single drone simulation*/


/*############################################################################*/
/*############################################################################*/
/*############################################################################*/



// /* Simple moving average filter */
// #if (USE_SMA_FILTER == 1)
//
//     /* Propogate state measurements */
//     for (int idx = 1; idx < SMA_WINDOW_LEN; ++idx) {
//         stateWindowBuf_cf1[idx] = stateWindowBuf_cf1[idx-1];
//         stateWindowBuf_cf2[idx] = stateWindowBuf_cf2[idx-1];
//     }
//
//     /* Append current state to head of buffer */
//     stateWindowBuf_cf1[0] = CFState_cf1;
//     stateWindowBuf_cf2[0] = CFState_cf2;
//
//     // /* Set all state variables to zero */
//     // memset(&CFState_cf1, 0x00, sizeof(stateVector));
//     // memset(&CFState_cf2, 0x00, sizeof(stateVector));
//
//     static stateVector tempState_cf1, tempState_cf2;
//
//     /* Accumulate state variables */
//     for (int idx = 0; idx < SMA_WINDOW_LEN; ++idx) {
//         tempState_cf1.x += stateWindowBuf_cf1[idx].x;
//     	tempState_cf1.y += stateWindowBuf_cf1[idx].y;
//     	tempState_cf1.z += stateWindowBuf_cf1[idx].z;
//         tempState_cf1.roll += stateWindowBuf_cf1[idx].roll;
//     	tempState_cf1.pitch += stateWindowBuf_cf1[idx].pitch;
//     	tempState_cf1.yaw += stateWindowBuf_cf1[idx].yaw;
//         tempState_cf1.x_d += stateWindowBuf_cf1[idx].x_d;
//     	tempState_cf1.y_d += stateWindowBuf_cf1[idx].y_d;
//     	tempState_cf1.z_d += stateWindowBuf_cf1[idx].z_d;
//     	tempState_cf1.roll_d += stateWindowBuf_cf1[idx].roll_d;
//     	tempState_cf1.pitch_d += stateWindowBuf_cf1[idx].pitch_d;
//     	tempState_cf1.yaw_d += stateWindowBuf_cf1[idx].yaw_d;
//
//         tempState_cf2.x += stateWindowBuf_cf2[idx].x;
//     	tempState_cf2.y += stateWindowBuf_cf2[idx].y;
//     	tempState_cf2.z += stateWindowBuf_cf2[idx].z;
//         tempState_cf2.roll += stateWindowBuf_cf2[idx].roll;
//     	tempState_cf2.pitch += stateWindowBuf_cf2[idx].pitch;
//     	tempState_cf2.yaw += stateWindowBuf_cf2[idx].yaw;
//         tempState_cf2.x_d += stateWindowBuf_cf2[idx].x_d;
//     	tempState_cf2.y_d += stateWindowBuf_cf2[idx].y_d;
//     	tempState_cf2.z_d += stateWindowBuf_cf2[idx].z_d;
//     	tempState_cf2.roll_d += stateWindowBuf_cf2[idx].roll_d;
//     	tempState_cf2.pitch_d += stateWindowBuf_cf2[idx].pitch_d;
//     	tempState_cf2.yaw_d += stateWindowBuf_cf2[idx].yaw_d;
//     }
//
//     CFState_cf1.x = tempState_cf1.x / ((float) SMA_WINDOW_LEN);
//     CFState_cf1.y = tempState_cf1.y / ((float) SMA_WINDOW_LEN);
//     CFState_cf1.z = tempState_cf1.z / ((float) SMA_WINDOW_LEN);
//     CFState_cf1.x = tempState_cf1.roll / ((float) SMA_WINDOW_LEN);
//     CFState_cf1.y = tempState_cf1.pitch / ((float) SMA_WINDOW_LEN);
//     CFState_cf1.z = tempState_cf1.yaw / ((float) SMA_WINDOW_LEN);
//     CFState_cf1.x_d = tempState_cf1.x_d / ((float) SMA_WINDOW_LEN);
//     CFState_cf1.y_d = tempState_cf1.y_d / ((float) SMA_WINDOW_LEN);
//     CFState_cf1.z_d = tempState_cf1.z_d / ((float) SMA_WINDOW_LEN);
//     CFState_cf1.roll_d = tempState_cf1.roll_d / ((float) SMA_WINDOW_LEN);
//     CFState_cf1.pitch_d = tempState_cf1.pitch_d / ((float) SMA_WINDOW_LEN);
//     CFState_cf1.yaw_d = tempState_cf1.yaw_d / ((float) SMA_WINDOW_LEN);
//
//     // CFState_cf2.x = CFState_cf2.x / ((float) SMA_WINDOW_LEN);
//     // CFState_cf2.y = CFState_cf2.y / ((float) SMA_WINDOW_LEN);
//     // CFState_cf2.z = CFState_cf2.z / ((float) SMA_WINDOW_LEN);
//     // CFState_cf2.x = CFState_cf2.roll / ((float) SMA_WINDOW_LEN);
//     // CFState_cf2.y = CFState_cf2.pitch / ((float) SMA_WINDOW_LEN);
//     // CFState_cf2.z = CFState_cf2.yaw / ((float) SMA_WINDOW_LEN);
//     // CFState_cf2.x_d = CFState_cf2.x_d / ((float) SMA_WINDOW_LEN);
//     // CFState_cf2.y_d = CFState_cf2.y_d / ((float) SMA_WINDOW_LEN);
//     // CFState_cf2.z_d = CFState_cf2.z_d / ((float) SMA_WINDOW_LEN);
//     // CFState_cf2.roll_d = CFState_cf2.roll_d / ((float) SMA_WINDOW_LEN);
//     // CFState_cf2.pitch_d = CFState_cf2.pitch_d / ((float) SMA_WINDOW_LEN);
//     // CFState_cf2.yaw_d = CFState_cf2.yaw_d / ((float) SMA_WINDOW_LEN);
//
// #endif


// /* First order exponential low-pass filter */
// #if (USE_EXP_FILTER == 1)

    // float alpha = 0.9;
    //
    // CFState_cf1.x = (prevState_cf1.x * alpha) + (1.0 - alpha) * CFState_cf1.x;
    // CFState_cf1.y = (prevState_cf1.y * alpha) + (1.0 - alpha) * CFState_cf1.y;
    // CFState_cf1.z = (prevState_cf1.z * alpha) + (1.0 - alpha) * CFState_cf1.z;
    // CFState_cf1.roll = (prevState_cf1.roll * alpha) + (1.0 - alpha) * CFState_cf1.roll;
    // CFState_cf1.pitch = (prevState_cf1.pitch * alpha) + (1.0 - alpha) * CFState_cf1.pitch;
    // CFState_cf1.yaw = (prevState_cf1.yaw * alpha) + (1.0 - alpha) * CFState_cf1.yaw;
    // CFState_cf1.x_d = (prevState_cf1.x_d * alpha) + (1.0 - alpha) * CFState_cf1.x_d;
    // CFState_cf1.y_d = (prevState_cf1.y_d * alpha) + (1.0 - alpha) * CFState_cf1.y_d;
    // CFState_cf1.z_d = (prevState_cf1.z_d * alpha) + (1.0 - alpha) * CFState_cf1.z_d;
    // CFState_cf1.roll_d = (prevState_cf1.roll_d * alpha) + (1.0 - alpha) * CFState_cf1.roll_d;
    // CFState_cf1.pitch_d = (prevState_cf1.pitch_d * alpha) + (1.0 - alpha) * CFState_cf1.pitch_d;
    // CFState_cf1.yaw_d = (prevState_cf1.yaw_d * alpha) + (1.0 - alpha) * CFState_cf1.yaw_d;

    // CFState_cf2.x = (prevState_cf2.x * EXP_FILT_ALPHA) + (1.0 - EXP_FILT_ALPHA) * CFState_cf2.x;
    // CFState_cf2.y = (prevState_cf2.y * EXP_FILT_ALPHA) + (1.0 - EXP_FILT_ALPHA) * CFState_cf2.y;
    // CFState_cf2.z = (prevState_cf2.z * EXP_FILT_ALPHA) + (1.0 - EXP_FILT_ALPHA) * CFState_cf2.z;
    // CFState_cf2.roll = (prevState_cf2.roll * EXP_FILT_ALPHA) + (1.0 - EXP_FILT_ALPHA) * CFState_cf2.roll;
    // CFState_cf2.pitch = (prevState_cf2.pitch * EXP_FILT_ALPHA) + (1.0 - EXP_FILT_ALPHA) * CFState_cf2.pitch;
    // CFState_cf2.yaw = (prevState_cf2.yaw * EXP_FILT_ALPHA) + (1.0 - EXP_FILT_ALPHA) * CFState_cf2.yaw;
    // CFState_cf2.x_d = (prevState_cf2.x_d * EXP_FILT_ALPHA) + (1.0 - EXP_FILT_ALPHA) * CFState_cf2.x_d;
    // CFState_cf2.y_d = (prevState_cf2.y_d * EXP_FILT_ALPHA) + (1.0 - EXP_FILT_ALPHA) * CFState_cf2.y_d;
    // CFState_cf2.z_d = (prevState_cf2.z_d * EXP_FILT_ALPHA) + (1.0 - EXP_FILT_ALPHA) * CFState_cf2.z_d;
    // CFState_cf2.roll_d = (prevState_cf2.roll_d * EXP_FILT_ALPHA) + (1.0 - EXP_FILT_ALPHA) * CFState_cf2.roll_d;
    // CFState_cf2.pitch_d = (prevState_cf2.pitch_d * EXP_FILT_ALPHA) + (1.0 - EXP_FILT_ALPHA) * CFState_cf2.pitch_d;
    // CFState_cf2.yaw_d = (prevState_cf2.yaw_d * EXP_FILT_ALPHA) + (1.0 - EXP_FILT_ALPHA) * CFState_cf2.yaw_d;

    // prevState_cf1 = CFState_cf1;
    // prevState_cf2 = CFState_cf2;

// #endif




// /* Draw square trajectory */
//
// srvGoTo_cf2.request.goal.x = 0.0;
// srvGoTo_cf2.request.goal.y = 0.0;
// srvGoTo_cf2.request.goal.z = 2.0;
// srvGoTo_cf2.request.yaw = 0.0;
// srvGoTo_cf2.request.duration = ros::Duration(3.0);
// GoToClient_cf1.call(srvGoTo_cf2);
//
// ros::Duration(1.0).sleep();
//
// srvGoTo_cf2.request.goal.x = 0.0;
// srvGoTo_cf2.request.goal.y = 1.0;
// srvGoTo_cf2.request.goal.z = 2.0;
// srvGoTo_cf2.request.yaw = 0.0;
// srvGoTo_cf2.request.duration = ros::Duration(3.0);
// GoToClient_cf1.call(srvGoTo_cf2);
//
// ros::Duration(1.0).sleep();
//
// srvGoTo_cf2.request.goal.x = 1.0;
// srvGoTo_cf2.request.goal.y = 1.0;
// srvGoTo_cf2.request.goal.z = 2.0;
// srvGoTo_cf2.request.yaw = 0.0;
// srvGoTo_cf2.request.duration = ros::Duration(3.0);
// GoToClient_cf1.call(srvGoTo_cf2);
//
// ros::Duration(1.0).sleep();
//
// srvGoTo_cf2.request.goal.x = 1.0;
// srvGoTo_cf2.request.goal.y = 0.0;
// srvGoTo_cf2.request.goal.z = 2.0;
// srvGoTo_cf2.request.yaw = 0.0;
// srvGoTo_cf2.request.duration = ros::Duration(3.0);
// GoToClient_cf1.call(srvGoTo_cf2);
//
// ros::Duration(1.0).sleep();
//
// srvGoTo_cf2.request.goal.x = 0.0;
// srvGoTo_cf2.request.goal.y = 0.0;
// srvGoTo_cf2.request.goal.z = 2.0;
// srvGoTo_cf2.request.yaw = 0.0;
// srvGoTo_cf2.request.duration = ros::Duration(3.0);
// GoToClient_cf1.call(srvGoTo_cf2);
//
// ros::Duration(1.0).sleep();





// /* Control loop */
// while (ros::ok()) {
//
//     int c = getch();   // call your non-blocking input function
//     if (c == '\n') {
//         std::cout << "LANDING" << std::endl;
//         break;
//     }
//
//     ros::spinOnce();
//
// 	// position_cmd.x = 0;
// 	// position_cmd.y = 0;
// 	// position_cmd.z = 2.0;
// 	// position_cmd.yaw = 0.0;
// 	// pub.publish(position_cmd);
//
//
// }
