//============================================================================
// Name        : test2.cpp
// Author      : 
// Version     :
// Copyright   : Your copyright notice
// Description : Hello World in C++, Ansi-style
//============================================================================

#include <iostream>
#include <Eigen/Dense>
//#include <unsupported/Eigen/AutoDiff>
#include <unsupported/Eigen/AdolcForward>
//#include <unsupported/Eigen/AdolcSupport>

//#include <chrono>
//#include <iomanip>
#include <adolc.h>
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
#include<unistd.h>
#include "DataStreamClient.h"



int main()
{



//
//	Eigen::VectorXd X0(4,1),inputs(2,1);
//	X0<<0,0,0,0;
//	inputs<<0,0;
//
//	constexpr size_t state_size=4;
//	constexpr size_t input_size=2;
	constexpr size_t horizon=5;
	constexpr int total_steps=300;
	srand((unsigned int) time(0));
////	int horizon=1000;
	unsigned int tag1(1),tag2(2),tag3(3),tag4(4),tag5(5),tag6(6)
	,tag7(7),tag8(8),tag9(9);

//	double time_step=0.2;
	double time_step=0.1;

//

//	/* This part is unicycle simulation*/
//
//	cost<4,2> running_cost_uni(Unicycle_Cost::running_cost,tag1);
//	cost<4,2> terminal_cost_uni(Unicycle_Cost::terminal_cost,tag2);
//	dynamics<4,2> dynamics_uni(Unicycle_Dynamics::dynamics,tag3,time_step);
//
//	iLQR<4,2,horizon> uni_solver(running_cost_uni,terminal_cost_uni,dynamics_uni);
//	Eigen::Matrix<double,2,1> u_uni=Eigen::Matrix<double,2,1>::Zero();
//	Eigen::Matrix<double,4,1> x0_uni=Eigen::Matrix<double,4,1>::Zero();
//	x0_uni<<-5,-5,0,0;
//////	X0_drone<<0,-50,10, M_PI/20,0,0, 0,0,0, 0,0,0;
//	Eigen::Matrix<double,4,1> x_goal_uni;
//
//
////
//	iLQR<4,2,horizon>::input_trajectory u_init_uni;
//	std::fill(std::begin(u_init_uni), std::end(u_init_uni), u_uni);
//	iLQR<4,2,horizon>::state_input_trajectory soln_uni;
//	iLQR<4,2,total_steps>::state_input_trajectory soln_uni_rhc;
//
//
//
//	clock_t t_start, t_end;
//
////
//	double seconds;
//	uni_solver.set_MPC(u_init_uni);
//	int execution_steps=1;
//	soln_uni_rhc.first[0]=x0_uni;
//	std::string state_path="/Users/talhakavuncu/Desktop/research/cpp_code/states.txt";
//	std::string input_path="/Users/talhakavuncu/Desktop/research/cpp_code/inputs.txt";
//
//	x_goal_uni<<5,5,0,0;
//	t_start = clock();
//	for(int i=0;i<total_steps;++i)
//	{
////		X_goal_drone<<10,0,0, 0,0,0, 0,0,0, 0,0,0;
////		if (i>total_steps/3)
////			X_goal_drone<<0,0,0, 0,0,0, 0,0,0, 0,0,0;
//		std::cout<<"iteration "<<i<<std::endl;
//////
//		soln_uni=uni_solver.run_MPC(soln_uni_rhc.first[i], x_goal_uni, 100, execution_steps);
////
////////		unsigned int microsecond = 1000000;
////////		usleep(2 * microsecond);
////		write_file<12,4,horizon>(state_path,input_path, soln_drone);
////		std::cout<<soln_uni.first[39]<<std::endl;
//		soln_uni_rhc.first[i+1]=soln_uni.first[1];
//		soln_uni_rhc.second[i]=soln_uni.second[0];
//	}
//	t_end = clock();
//////
//////
////////	soln_drone=drone_solver.solve_open_loop(X0_drone, X0_drone, 200, u_init_drone, horizon);
////////	write_file<12,4,horizon>(state_path,input_path, soln_drone);
//	write_file<4,2,total_steps>(state_path,input_path, soln_uni_rhc);
////
//	std::cout<<"finished"<<std::endl;
//	seconds = 1/((double)(t_end - t_start) / CLOCKS_PER_SEC/total_steps);
//	printf("Time: %f s\n", seconds);
//
//
//	/* End of unicycle simulation*/

/* This part is 2-unicycle simulation*/

//	cost<8,4> running_cost_uni2(Unicycle_Cost::running_cost2,tag1);
//	cost<8,4> terminal_cost_uni2(Unicycle_Cost::terminal_cost2,tag2);
//	dynamics<8,4> dynamics_uni2(Unicycle_Dynamics::dynamics_2,tag3,time_step);
//
//	iLQR<8,4,horizon> uni2_solver(running_cost_uni2,terminal_cost_uni2,dynamics_uni2);
//	Eigen::Matrix<double,4,1> u_uni2=Eigen::Matrix<double,4,1>::Random()*0.1;
//	Eigen::Matrix<double,8,1> x0_uni2=Eigen::Matrix<double,8,1>::Zero();
//	x0_uni2<<0,0.1,0,0, 10,0,M_PI,0;
//
//	Eigen::Matrix<double,8,1> x_goal_uni2;
//
//
////
//	iLQR<8,4,horizon>::input_trajectory u_init_uni2;
//	for(auto &item : u_init_uni2)
//		item=Eigen::Matrix<double,4,1>::Random()*0.1;
//	std::fill(std::begin(u_init_uni2), std::end(u_init_uni2), u_uni2);
//	iLQR<8,4,horizon>::state_input_trajectory soln_uni2;
//	iLQR<8,4,total_steps>::state_input_trajectory soln_uni2_rhc;
//
//
//
//	clock_t t_start, t_end;
//
////
//	double seconds;
//	uni2_solver.set_MPC(u_init_uni2);
//	int execution_steps=1;
//	soln_uni2_rhc.first[0]=x0_uni2;
//	std::string state_path="/Users/talhakavuncu/Desktop/research/cpp_code/states.txt";
//	std::string input_path="/Users/talhakavuncu/Desktop/research/cpp_code/inputs.txt";
//
//	x_goal_uni2<<10,0,0,0, 0,0,M_PI,0;
//	t_start = clock();
//	for(int i=0;i<total_steps;++i)
//	{
////		X_goal_drone<<10,0,0, 0,0,0, 0,0,0, 0,0,0;
////		if (i>total_steps/3)
////			X_goal_drone<<0,0,0, 0,0,0, 0,0,0, 0,0,0;
//		std::cout<<"iteration "<<i<<std::endl;
//////
//		soln_uni2=uni2_solver.run_MPC(soln_uni2_rhc.first[i], x_goal_uni2, 5, execution_steps);
////
////////		unsigned int microsecond = 1000000;
////////		usleep(2 * microsecond);
////		write_file<12,4,horizon>(state_path,input_path, soln_drone);
////		std::cout<<soln_uni.first[39]<<std::endl;
//		soln_uni2_rhc.first[i+1]=soln_uni2.first[1];
//		soln_uni2_rhc.second[i]=soln_uni2.second[0];
//	}
//	t_end = clock();
////
////
//////	soln_drone=drone_solver.solve_open_loop(X0_drone, X0_drone, 200, u_init_drone, horizon);
//////	write_file<12,4,horizon>(state_path,input_path, soln_drone);
//	write_file<8,4,total_steps>(state_path,input_path, soln_uni2_rhc);
////
//	std::cout<<"finished"<<std::endl;
//	seconds = 1/((double)(t_end - t_start) / CLOCKS_PER_SEC/total_steps);
//	printf("Time: %f s\n", seconds);


/* End of 2-unicycle simulation*/








/* This part is single drone simulation*/

//	cost<12,4> running_cost_dr(Drone_Cost::running_cost,tag1);
//	cost<12,4> terminal_cost_dr(Drone_Cost::terminal_cost,tag2);
//	dynamics<12,4> dynamics_dr(Drone_Dynamics::dynamics,tag3,time_step);
//
//	iLQR<12,4,horizon> drone_solver(running_cost_dr,terminal_cost_dr,dynamics_dr);
//	Eigen::Matrix<double,4,1> u_drone=Eigen::Matrix<double,4,1>::Ones();
//	Eigen::Matrix<double,12,1> X0_drone=Eigen::Matrix<double,12,1>::Zero();
//////	X0_drone<<0,-50,10, M_PI/20,0,0, 0,0,0, 0,0,0;
//	Eigen::Matrix<double,12,1> X_goal_drone;
//
//	u_drone=u_drone*sqrt(Drone_Dynamics::mass*Drone_Dynamics::g/(4*Drone_Dynamics::C_T));
////
//	iLQR<12,4,horizon>::input_trajectory u_init_drone;
//	std::fill(std::begin(u_init_drone), std::end(u_init_drone), u_drone);
//	iLQR<12,4,horizon>::state_input_trajectory soln_drone;
//	iLQR<12,4,total_steps>::state_input_trajectory soln_drone_rhc;
//
//
//
//	clock_t t_start, t_end;
//
////
//	double seconds;
//	drone_solver.set_MPC(u_init_drone);
//	int execution_steps=1;
//	soln_drone_rhc.first[0]=X0_drone;
//	std::string state_path="/Users/talhakavuncu/Desktop/research/cpp_code/states.txt";
//	std::string input_path="/Users/talhakavuncu/Desktop/research/cpp_code/inputs.txt";
//	X_goal_drone<<1,1,1.5, 0,0,0, 0,0,0, 0,0,0;
//	t_start = clock();
//	for(int i=0;i<total_steps;++i)
//	{
////		X_goal_drone<<10,0,0, 0,0,0, 0,0,0, 0,0,0;
////		if (i>total_steps/3)
////			X_goal_drone<<0,0,0, 0,0,0, 0,0,0, 0,0,0;
//		std::cout<<"iteration "<<i<<std::endl;
//////
//		soln_drone=drone_solver.run_MPC(soln_drone_rhc.first[i], X_goal_drone, 100, execution_steps);
////
////////		unsigned int microsecond = 1000000;
////////		usleep(2 * microsecond);
////		write_file<12,4,horizon>(state_path,input_path, soln_drone);
//		std::cout<<soln_drone.first[39]<<std::endl;
//		soln_drone_rhc.first[i+1]=soln_drone.first[1];
//		soln_drone_rhc.second[i]=soln_drone.second[0];
//	}
//	t_end = clock();
//////
//////
////////	soln_drone=drone_solver.solve_open_loop(X0_drone, X0_drone, 200, u_init_drone, horizon);
////////	write_file<12,4,horizon>(state_path,input_path, soln_drone);
//	write_file<12,4,total_steps>(state_path,input_path, soln_drone_rhc);
////
//	std::cout<<"finished"<<std::endl;
//	seconds = 1/((double)(t_end - t_start) / CLOCKS_PER_SEC/total_steps);
//	printf("Time: %f s\n", seconds);


/* End of single drone simulation*/










/* This part is 2-drone simulation*/


//	cost<12*2,4*2> running_cost_dr2(Drone_Cost::running_cost2,tag4);
//	cost<12*2,4*2> terminal_cost_dr2(Drone_Cost::terminal_cost2,tag5);
//	dynamics<12*2,4*2> dynamics_dr2(Drone_Dynamics::dynamics_2,tag6,time_step);
//	iLQR<12*2,4*2,horizon>::input_trajectory u_init_drone2;
//
//	iLQR<12*2,4*2,horizon> drone2_solver(running_cost_dr2,terminal_cost_dr2,dynamics_dr2);
//	Eigen::Matrix<double,4*2,1> u_drone2=Eigen::Matrix<double,4*2,1>::Ones();
//	Eigen::Matrix<double,12*2,1> X0_drone2=Eigen::Matrix<double,12*2,1>::Zero();
////	X0_drone2<<-2.5,-2.5,-2.5, 0,0,0, 0,0,0, 0,0,0,
////			2.5,-2.5,-2.5, 0,0,0, 0,0,0, 0,0,0;
//	X0_drone2<<-1,0,2, 0,0,0, 0,0,0, 0,0,0,
//			1,0,2, 0,0,0, 0,0,0, 0,0,0;
//	Eigen::Matrix<double,12*2,1> X_goal_drone2;
//	u_drone2=u_drone2*sqrt(Drone_Dynamics::mass*Drone_Dynamics::g/(4*Drone_Dynamics::C_T));
//	std::fill(std::begin(u_init_drone2), std::end(u_init_drone2), u_drone2);
//	iLQR<12*2,4*2,horizon>::state_input_trajectory soln_drone2;
//	iLQR<12*2,4*2,total_steps>::state_input_trajectory soln_drone2_rhc;
//
//	clock_t t_start, t_end;
//
////
//	double seconds;
//	drone2_solver.set_MPC(u_init_drone2);
//	int execution_steps=1;
//	soln_drone2_rhc.first[0]=X0_drone2;
//	std::string state_path="/Users/talhakavuncu/Desktop/research/cpp_code/states.txt";
//	std::string input_path="/Users/talhakavuncu/Desktop/research/cpp_code/inputs.txt";
////	X_goal_drone2<<2.5,2.5,2.5, 0,0,0, 0,0,0, 0,0,0,
////			-2.5,2.5,2.5, 0,0,0, 0,0,0, 0,0,0;
//	X_goal_drone2<<1,0,2, 0,0,0, 0,0,0, 0,0,0,
//			-1,0,2, 0,0,0, 0,0,0, 0,0,0;
//	t_start = clock();
//	for(int i=0;i<total_steps;++i)
//	{
//
////		if (i>total_steps/3)
////			X_goal_drone<<0,0,0, 0,0,0, 0,0,0, 0,0,0;
//		std::cout<<"iteration "<<i<<std::endl;
//////
//		soln_drone2=drone2_solver.run_MPC(soln_drone2_rhc.first[i], X_goal_drone2, 20, execution_steps);
////
////////		unsigned int microsecond = 1000000;
////////		usleep(2 * microsecond);
////		write_file<12,4,horizon>(state_path,input_path, soln_drone);
//		soln_drone2_rhc.first[i+1]=soln_drone2.first[1];
//		soln_drone2_rhc.second[i]=soln_drone2.second[0];
//	}
//	t_end = clock();
//////
//////
////////	soln_drone=drone_solver.solve_open_loop(X0_drone, X0_drone, 200, u_init_drone, horizon);
////////	write_file<12,4,horizon>(state_path,input_path, soln_drone);
//	write_file<12*2,4*2,total_steps>(state_path,input_path, soln_drone2_rhc);
////
//	std::cout<<"finished"<<std::endl;
//	seconds = 1/((double)(t_end - t_start) / CLOCKS_PER_SEC/total_steps);
//	printf("Time: %f s\n", seconds);
//
/* End of 2-drone simulation*/


//
//

//
////	diff = difftime( stop, start );
////	std::cout << "Time: " << diff << " seconds";
////	seconds = (double)(t_end - t_start) / CLOCKS_PER_SEC;
////	printf("Time: %f s\n", seconds);
//		drone_solver.set_MPC(u_init_drone);
////		t_start = clock();
//		std::string state_write_path="/Users/talhakavuncu/Desktop/cflib_stream.txt";
//		std::string state_read_path="/Users/talhakavuncu/Desktop/stream.txt";
//		std::ifstream state_read_file(state_read_path);
//		std::ofstream state_write_file(state_write_path);
//		Eigen::IOFormat CommaInitFmt(Eigen::FullPrecision, Eigen::DontAlignCols, ",", ", ", "", "", "", "");
//		double dt,x,y,z,roll,pitch,yaw;
//		state_read_file>>dt>>x>>y >> z>>roll >> pitch>>yaw;
//		X0_drone<<x,y,z,roll,pitch,yaw,x/dt,y/dt,z/dt,roll/dt,pitch/dt,yaw/dt;
//		X_goal_drone<<0,0,1, 0,0,0, 0,0,0, 0,0,0;
//		while(true)
//		{
////			X_goal_drone<<10,0,0, 0,0,0, 0,0,0, 0,0,0;
////			if (i>total_steps/3)
////				X_goal_drone<<0,0,0, 0,0,0, 0,0,0, 0,0,0;
////			std::cout<<"iteration "<<i<<std::endl;
//	//
//
//			soln_drone=drone_solver.run_MPC(X0_drone, X_goal_drone, 50, 1);
//			state_write_file.open(state_write_path,std::ios::out | std::ios::trunc);
//			state_write_file<<soln_drone.first[1].transpose().format(CommaInitFmt);
////			i.transpose().format(CommaInitFmt)
//			state_write_file.close();
//			state_read_file>>dt>>x>>y >> z>>roll >> pitch>>yaw;
//			X0_drone<<x,y,z,roll,pitch,yaw,x/dt,y/dt,z/dt,roll/dt,pitch/dt,yaw/dt;
//	////		unsigned int microsecond = 1000000;
//	////		usleep(2 * microsecond);
//	////		write_file<12,4,horizon>(state_path,input_path, soln_drone);
////			soln_drone_rhc.first[i+1]=soln_drone.first[1];
//
////			soln_drone_rhc.second[i]=soln_drone.second[0];
//		}
////		t_end = clock();


/* This part is integrator simulation */

//
//cost<3,3> integrator_running_cost(Single_Integrator_Cost::running_cost,tag7);
//cost<3,3> integrator_terminal_cost(Single_Integrator_Cost::terminal_cost,tag8);
//dynamics<3,3> integrator_dynamics(Single_Integrator_3D::dynamics,tag9,time_step);
//iLQR<3,3,horizon>::input_trajectory u_init_integrator;
//iLQR<3,3,horizon> integrator_solver(integrator_running_cost,integrator_terminal_cost,integrator_dynamics);
//Eigen::Matrix<double,3,1> u_integrator=Eigen::Matrix<double,3,1>::Zero();
//Eigen::Matrix<double,3,1> x0_integrator=Eigen::Matrix<double,3,1>::Zero();
//Eigen::Matrix<double,3,1> x_goal_integrator;
//iLQR<3,3,horizon>::state_input_trajectory soln_integrator;
//iLQR<3,3,total_steps>::state_input_trajectory soln_integrator_rhc;
//
////	u_drone2=u_drone2*sqrt(Drone_Dynamics::mass*Drone_Dynamics::g/(4*Drone_Dynamics::C_T));
//	std::fill(std::begin(u_init_integrator), std::end(u_init_integrator), u_integrator);
////	iLQR<12*2,4*2,horizon>::state_input_trajectory soln_drone2;
////	iLQR<12*2,4*2,total_steps>::state_input_trajectory soln_drone2_rhc;
//
//
//
//
//
//	clock_t t_start, t_end;
////
//////
//	x0_integrator<<0,0,0.5;
////	x0_integrator+=0.1*Eigen::Matrix<double,3,1>::Random();
//	double seconds;
//	integrator_solver.set_MPC(u_init_integrator);
//	int execution_steps=1;
//	soln_integrator_rhc.first[0]=x0_integrator;
//	std::string state_path="/Users/talhakavuncu/Desktop/research/cpp_code/states.txt";
//	std::string input_path="/Users/talhakavuncu/Desktop/research/cpp_code/inputs.txt";
//	t_start = clock();
//	for(int i=0;i<total_steps;++i)
//	{
//		x_goal_integrator<<2,2,2;
////		if (i>total_steps/3)
////			x_goal_integrator<<0,0,0;
//		std::cout<<"iteration "<<i<<std::endl;
////////
////		auto term1=soln_integrator_rhc.first[i]+0.2*Eigen::Matrix<double,3,1>::Random();
////		auto term2=soln_integrator_rhc.first[i]+0.02*Eigen::Matrix<double,3,1>::Random();
////		auto term3=soln_integrator_rhc.first[i]+0.02*Eigen::Matrix<double,3,1>::Random();
////		auto term4=soln_integrator_rhc.first[i]+0.02*Eigen::Matrix<double,3,1>::Random();
////		auto term5=soln_integrator_rhc.first[i]+0.02*Eigen::Matrix<double,3,1>::Random();
////		auto term6=soln_integrator_rhc.first[i]+0.02*Eigen::Matrix<double,3,1>::Random();
////		auto term7=soln_integrator_rhc.first[i]+0.02*Eigen::Matrix<double,3,1>::Random();
////		auto term8=soln_integrator_rhc.first[i]+0.02*Eigen::Matrix<double,3,1>::Random();
////		auto term9=soln_integrator_rhc.first[i]+0.02*Eigen::Matrix<double,3,1>::Random();
////		auto term10=soln_integrator_rhc.first[i]+0.02*Eigen::Matrix<double,3,1>::Random();
////		auto term=(term1+term2+term3+term4+term5+term6+term7+term8+term9+term10)/10;
//		soln_integrator=integrator_solver.run_MPC(soln_integrator_rhc.first[i],
//				x_goal_integrator, 100, execution_steps);
//////
//////////		unsigned int microsecond = 1000000;
//////////		usleep(2 * microsecond);
//////		write_file<12,4,horizon>(state_path,input_path, soln_drone);
//		soln_integrator_rhc.first[i+1]=soln_integrator.first[1];
//		soln_integrator_rhc.second[i]=soln_integrator.second[0];
//	}
//	t_end = clock();
//////
////////
//////////	soln_drone=drone_solver.solve_open_loop(X0_drone, X0_drone, 200, u_init_drone, horizon);
//////////	write_file<12,4,horizon>(state_path,input_path, soln_drone);
//	write_file<3,3,total_steps>(state_path,input_path, soln_integrator_rhc);
//////
//	std::cout<<"finished"<<std::endl;
//	seconds = 1/((double)(t_end - t_start) / CLOCKS_PER_SEC/total_steps);
//	printf("Time: %f s\n", seconds);


/*single integrator simulation ends */


/*Single integrator 2 agent simulation */

//	cost<6,6> integrator2_running_cost(Single_Integrator_Cost::running_cost2,tag7);
//	cost<6,6> integrator2_terminal_cost(Single_Integrator_Cost::terminal_cost2,tag8);
//	dynamics<6,6> integrator2_dynamics(Single_Integrator_3D::dynamics2,tag9,time_step);
//	iLQR<6,6,horizon>::input_trajectory u_init_integrator2;
//	iLQR<6,6,horizon> integrator2_solver(integrator2_running_cost,integrator2_terminal_cost,integrator2_dynamics);
//	Eigen::Matrix<double,6,1> u_integrator2=Eigen::Matrix<double,6,1>::Zero();
//	Eigen::Matrix<double,6,1> x0_integrator2=Eigen::Matrix<double,6,1>::Zero();
//	x0_integrator2<<-1,0,2.001, 1,0,2;
////	std::cout<<x0_integrator2<<std::endl;
//	Eigen::Matrix<double,6,1> x_goal_integrator2;
//	x_goal_integrator2<<1,0,2, -1,0,2;
//	iLQR<6,6,horizon>::state_input_trajectory soln_integrator2;
//	iLQR<6,6,total_steps>::state_input_trajectory soln_integrator2_rhc;
//
//	//	u_drone2=u_drone2*sqrt(Drone_Dynamics::mass*Drone_Dynamics::g/(4*Drone_Dynamics::C_T));
//	//	std::fill(std::begin(u_init_drone2), std::end(u_init_drone2), u_drone2);
//	//	iLQR<12*2,4*2,horizon>::state_input_trajectory soln_drone2;
//	//	iLQR<12*2,4*2,total_steps>::state_input_trajectory soln_drone2_rhc;
//
//
//
//
//
//		clock_t t_start, t_end;
//	//
//	////
//		double seconds;
//		integrator2_solver.set_MPC(u_init_integrator2);
//		int execution_steps=1;
//		soln_integrator2_rhc.first[0]=x0_integrator2;
//		std::string state_path="/Users/talhakavuncu/Desktop/research/cpp_code/states.txt";
//		std::string input_path="/Users/talhakavuncu/Desktop/research/cpp_code/inputs.txt";
//		t_start = clock();
//		for(int i=0;i<total_steps;++i)
//		{
////			x_goal_integrator2<<2,2,2;
//	//		if (i>total_steps/3)
//	//			x_goal_integrator<<0,0,0;
//			std::cout<<"iteration "<<i<<std::endl;
//	//////
//			soln_integrator2=integrator2_solver.run_MPC(soln_integrator2_rhc.first[i],
//					x_goal_integrator2, 5, execution_steps);
//	////
//	////////		unsigned int microsecond = 1000000;
//	////////		usleep(2 * microsecond);
//	////		write_file<12,4,horizon>(state_path,input_path, soln_drone);
//			soln_integrator2_rhc.first[i+1]=soln_integrator2.first[1];
//			soln_integrator2_rhc.second[i]=soln_integrator2.second[0];
//		}
//		t_end = clock();
//	////
//	//////
//	////////	soln_drone=drone_solver.solve_open_loop(X0_drone, X0_drone, 200, u_init_drone, horizon);
//	////////	write_file<12,4,horizon>(state_path,input_path, soln_drone);
//		write_file<6,6,total_steps>(state_path,input_path, soln_integrator2_rhc);
//	////
//		std::cout<<"finished"<<std::endl;
//		seconds = 1/((double)(t_end - t_start) / CLOCKS_PER_SEC/total_steps);
//		printf("Time: %f s\n", seconds);

/*Single integrator 2 agent simulation ends*/



/*Double Integrator Simulation*/

//
//
//	cost<6,3> integrator_running_cost(Double_Integrator_Cost::running_cost,tag7);
//	cost<6,3> integrator_terminal_cost(Double_Integrator_Cost::terminal_cost,tag8);
//	dynamics<6,3> integrator_dynamics(Double_Integrator_3D::dynamics,tag9,time_step);
//	iLQR<6,3,horizon>::input_trajectory u_init_integrator;
//	iLQR<6,3,horizon> integrator_solver(integrator_running_cost,integrator_terminal_cost,integrator_dynamics);
//	Eigen::Matrix<double,3,1> u_integrator=Eigen::Matrix<double,3,1>::Zero();
//	Eigen::Matrix<double,6,1> x0_integrator=Eigen::Matrix<double,6,1>::Zero();
//	Eigen::Matrix<double,6,1> x_goal_integrator;
//	iLQR<6,3,horizon>::state_input_trajectory soln_integrator;
//	iLQR<6,3,total_steps>::state_input_trajectory soln_integrator_rhc;
//
//	//	u_drone2=u_drone2*sqrt(Drone_Dynamics::mass*Drone_Dynamics::g/(4*Drone_Dynamics::C_T));
//		std::fill(std::begin(u_init_integrator), std::end(u_init_integrator), u_integrator);
//	//	iLQR<12*2,4*2,horizon>::state_input_trajectory soln_drone2;
//	//	iLQR<12*2,4*2,total_steps>::state_input_trajectory soln_drone2_rhc;
//
//
//
//
//
//		clock_t t_start, t_end;
//	//
//	////
//		x0_integrator<<0,0,0.5, 0,0,0;
//	//	x0_integrator+=0.1*Eigen::Matrix<double,3,1>::Random();
//		double seconds;
//		integrator_solver.set_MPC(u_init_integrator);
//		int execution_steps=1;
//		soln_integrator_rhc.first[0]=x0_integrator;
//		std::string state_path="/Users/talhakavuncu/Desktop/research/cpp_code/states.txt";
//		std::string input_path="/Users/talhakavuncu/Desktop/research/cpp_code/inputs.txt";
//		t_start = clock();
//		for(int i=0;i<total_steps;++i)
//		{
//			x_goal_integrator<<2,2,2, 0,0,0;
//	//		if (i>total_steps/3)
//	//			x_goal_integrator<<0,0,0;
//			std::cout<<"iteration "<<i<<std::endl;
//	//////
//	//		auto term1=soln_integrator_rhc.first[i]+0.2*Eigen::Matrix<double,3,1>::Random();
//	//		auto term2=soln_integrator_rhc.first[i]+0.02*Eigen::Matrix<double,3,1>::Random();
//	//		auto term3=soln_integrator_rhc.first[i]+0.02*Eigen::Matrix<double,3,1>::Random();
//	//		auto term4=soln_integrator_rhc.first[i]+0.02*Eigen::Matrix<double,3,1>::Random();
//	//		auto term5=soln_integrator_rhc.first[i]+0.02*Eigen::Matrix<double,3,1>::Random();
//	//		auto term6=soln_integrator_rhc.first[i]+0.02*Eigen::Matrix<double,3,1>::Random();
//	//		auto term7=soln_integrator_rhc.first[i]+0.02*Eigen::Matrix<double,3,1>::Random();
//	//		auto term8=soln_integrator_rhc.first[i]+0.02*Eigen::Matrix<double,3,1>::Random();
//	//		auto term9=soln_integrator_rhc.first[i]+0.02*Eigen::Matrix<double,3,1>::Random();
//	//		auto term10=soln_integrator_rhc.first[i]+0.02*Eigen::Matrix<double,3,1>::Random();
//	//		auto term=(term1+term2+term3+term4+term5+term6+term7+term8+term9+term10)/10;
//			soln_integrator=integrator_solver.run_MPC(soln_integrator_rhc.first[i],
//					x_goal_integrator, 5, execution_steps);
//	////
//	////////		unsigned int microsecond = 1000000;
//	////////		usleep(2 * microsecond);
//	////		write_file<12,4,horizon>(state_path,input_path, soln_drone);
//			soln_integrator_rhc.first[i+1]=soln_integrator.first[1];
//			soln_integrator_rhc.second[i]=soln_integrator.second[0];
//		}
//		t_end = clock();
//	////
//	//////
//	////////	soln_drone=drone_solver.solve_open_loop(X0_drone, X0_drone, 200, u_init_drone, horizon);
//	////////	write_file<12,4,horizon>(state_path,input_path, soln_drone);
//		write_file<6,3,total_steps>(state_path,input_path, soln_integrator_rhc);
//	////
//		std::cout<<"finished"<<std::endl;
//		seconds = 1/((double)(t_end - t_start) / CLOCKS_PER_SEC/total_steps);
//		printf("Time: %f s\n", seconds);




/*Double Integrator Simulation Ends*/


/*First Order Drone Simulation */



	cost<6,6> drone_running_cost(Drone_First_Order_Cost::running_cost,tag7);
	cost<6,6> drone_terminal_cost(Drone_First_Order_Cost::terminal_cost,tag8);
	dynamics<6,6> drone_dynamics(Drone_First_Order_Dynamics::dynamics,tag9,time_step);
	iLQR<6,6,horizon>::input_trajectory u_init_drone;
	iLQR<6,6,horizon> drone_solver(drone_running_cost,drone_terminal_cost,drone_dynamics);
	Eigen::Matrix<double,6,1> u_drone=Eigen::Matrix<double,6,1>::Zero();
	Eigen::Matrix<double,6,1> x0_drone=Eigen::Matrix<double,6,1>::Zero();
	Eigen::Matrix<double,6,1> x_goal_drone;
	iLQR<6,6,horizon>::state_input_trajectory soln_drone;
	iLQR<6,6,total_steps>::state_input_trajectory soln_drone_rhc;

	//	u_drone2=u_drone2*sqrt(Drone_Dynamics::mass*Drone_Dynamics::g/(4*Drone_Dynamics::C_T));
		std::fill(std::begin(u_init_drone), std::end(u_init_drone), u_drone);
	//	iLQR<12*2,4*2,horizon>::state_input_trajectory soln_drone2;
	//	iLQR<12*2,4*2,total_steps>::state_input_trajectory soln_drone2_rhc;





		clock_t t_start, t_end;
	//
	////
		x0_drone<<0,0,0.5, 0,0,0;
	//	x0_integrator+=0.1*Eigen::Matrix<double,3,1>::Random();
		double seconds;
		drone_solver.set_MPC(u_init_drone);
		int execution_steps=1;
		soln_drone_rhc.first[0]=x0_drone;
		std::string state_path="/Users/talhakavuncu/Desktop/research/cpp_code/states.txt";
		std::string input_path="/Users/talhakavuncu/Desktop/research/cpp_code/inputs.txt";
		t_start = clock();
		for(int i=0;i<total_steps;++i)
		{
			x_goal_drone<<2,2,2, 0,0,0;
	//		if (i>total_steps/3)
	//			x_goal_integrator<<0,0,0;
			std::cout<<"iteration "<<i<<std::endl;
			double scale=0.01;
			auto term1=soln_drone_rhc.first[i]+scale*Eigen::Matrix<double,6,1>::Random();
//			auto term2=soln_drone_rhc.first[i]+scale*Eigen::Matrix<double,6,1>::Random();
//			auto term3=soln_drone_rhc.first[i]+scale*Eigen::Matrix<double,6,1>::Random();
//			auto term4=soln_drone_rhc.first[i]+scale*Eigen::Matrix<double,6,1>::Random();
//			auto term5=soln_drone_rhc.first[i]+scale*Eigen::Matrix<double,6,1>::Random();
//			auto term6=soln_drone_rhc.first[i]+scale*Eigen::Matrix<double,6,1>::Random();
//			auto term7=soln_drone_rhc.first[i]+scale*Eigen::Matrix<double,6,1>::Random();
//			auto term8=soln_drone_rhc.first[i]+scale*Eigen::Matrix<double,6,1>::Random();
//			auto term9=soln_drone_rhc.first[i]+scale*Eigen::Matrix<double,6,1>::Random();
//			auto term10=soln_drone_rhc.first[i]+scale*Eigen::Matrix<double,6,1>::Random();
//			auto term=(term1+term2+term3+term4+term5+term6+term7+term8+term9+term10)/10;
			soln_drone=drone_solver.run_MPC(term1,
					x_goal_drone, 5, execution_steps);
	////
	////////		unsigned int microsecond = 1000000;
	////////		usleep(2 * microsecond);
	////		write_file<12,4,horizon>(state_path,input_path, soln_drone);
			soln_drone_rhc.first[i+1]=soln_drone.first[1];
			soln_drone_rhc.second[i]=soln_drone.second[0];
		}
		t_end = clock();
	////
	//////
	////////	soln_drone=drone_solver.solve_open_loop(X0_drone, X0_drone, 200, u_init_drone, horizon);
	////////	write_file<12,4,horizon>(state_path,input_path, soln_drone);
		write_file<6,6,total_steps>(state_path,input_path, soln_drone_rhc);
	////
		std::cout<<"finished"<<std::endl;
		seconds = 1/((double)(t_end - t_start) / CLOCKS_PER_SEC/total_steps);
		printf("Time: %f s\n", seconds);




/*First Order Drone Simulation Ends*/

/*First Order 2 Drone Simulation*/

//	cost<12,12> drone2_running_cost(Drone_First_Order_Cost::running_cost2,tag7);
//	cost<12,12> drone2_terminal_cost(Drone_First_Order_Cost::terminal_cost2,tag8);
//	dynamics<12,12> drone2_dynamics(Drone_First_Order_Dynamics::dynamics_2,tag9,time_step);
//	iLQR<12,12,horizon>::input_trajectory u_init_drone2;
//	iLQR<12,12,horizon> drone2_solver(drone2_running_cost,drone2_terminal_cost,drone2_dynamics);
//	Eigen::Matrix<double,12,1> u_drone2=Eigen::Matrix<double,12,1>::Zero();
//	Eigen::Matrix<double,12,1> x0_drone2=Eigen::Matrix<double,12,1>::Zero();
//	Eigen::Matrix<double,12,1> x_goal_drone2;
//	iLQR<12,12,horizon>::state_input_trajectory soln_drone2;
//	iLQR<12,12,total_steps>::state_input_trajectory soln_drone2_rhc;
//
//	//	u_drone2=u_drone2*sqrt(Drone_Dynamics::mass*Drone_Dynamics::g/(4*Drone_Dynamics::C_T));
//		std::fill(std::begin(u_init_drone2), std::end(u_init_drone2), u_drone2);
//	//	iLQR<12*2,4*2,horizon>::state_input_trajectory soln_drone2;
//	//	iLQR<12*2,4*2,total_steps>::state_input_trajectory soln_drone2_rhc;
//
//
//
//
//
//		clock_t t_start, t_end;
//	//
//	////
////		x0_drone2<<0,0,0.5, 0,0,1;
//		x0_drone2<<-3,0,2.1, 0,0,0, 3,0,2, 0,0,0;
//		x_goal_drone2<<3,0,2, 0,0,0, -3,0,2, 0,0,0;
//
//	//	x0_integrator+=0.1*Eigen::Matrix<double,3,1>::Random();
//		double seconds;
//		drone2_solver.set_MPC(u_init_drone2);
//		int execution_steps=1;
//		soln_drone2_rhc.first[0]=x0_drone2;
//		std::string state_path="/Users/talhakavuncu/Desktop/research/cpp_code/states.txt";
//		std::string input_path="/Users/talhakavuncu/Desktop/research/cpp_code/inputs.txt";
//		t_start = clock();
//		for(int i=0;i<total_steps;++i)
//		{
////			x_goal_drone2<<2,2,2, 0,0,0;
//	//		if (i>total_steps/3)
//	//			x_goal_integrator<<0,0,0;
//			std::cout<<"iteration "<<i<<std::endl;
//	//////
//	//		auto term1=soln_integrator_rhc.first[i]+0.2*Eigen::Matrix<double,3,1>::Random();
//	//		auto term2=soln_integrator_rhc.first[i]+0.02*Eigen::Matrix<double,3,1>::Random();
//	//		auto term3=soln_integrator_rhc.first[i]+0.02*Eigen::Matrix<double,3,1>::Random();
//	//		auto term4=soln_integrator_rhc.first[i]+0.02*Eigen::Matrix<double,3,1>::Random();
//	//		auto term5=soln_integrator_rhc.first[i]+0.02*Eigen::Matrix<double,3,1>::Random();
//	//		auto term6=soln_integrator_rhc.first[i]+0.02*Eigen::Matrix<double,3,1>::Random();
//	//		auto term7=soln_integrator_rhc.first[i]+0.02*Eigen::Matrix<double,3,1>::Random();
//	//		auto term8=soln_integrator_rhc.first[i]+0.02*Eigen::Matrix<double,3,1>::Random();
//	//		auto term9=soln_integrator_rhc.first[i]+0.02*Eigen::Matrix<double,3,1>::Random();
//	//		auto term10=soln_integrator_rhc.first[i]+0.02*Eigen::Matrix<double,3,1>::Random();
//	//		auto term=(term1+term2+term3+term4+term5+term6+term7+term8+term9+term10)/10;
//			soln_drone2=drone2_solver.run_MPC(soln_drone2_rhc.first[i],
//					x_goal_drone2, 5, execution_steps);
//	////
//	////////		unsigned int microsecond = 1000000;
//	////////		usleep(2 * microsecond);
//	////		write_file<12,4,horizon>(state_path,input_path, soln_drone);
//			soln_drone2_rhc.first[i+1]=soln_drone2.first[1];
//			soln_drone2_rhc.second[i]=soln_drone2.second[0];
//		}
//		t_end = clock();
//	////
//	//////
//	////////	soln_drone=drone_solver.solve_open_loop(X0_drone, X0_drone, 200, u_init_drone, horizon);
//	////////	write_file<12,4,horizon>(state_path,input_path, soln_drone);
//		write_file<12,12,total_steps>(state_path,input_path, soln_drone2_rhc);
//	////
//		std::cout<<"finished"<<std::endl;
//		seconds = 1/((double)(t_end - t_start) / CLOCKS_PER_SEC/total_steps);
//		printf("Time: %f s\n", seconds);







/*First Order 2 Drone Simulation Ends*/
};
