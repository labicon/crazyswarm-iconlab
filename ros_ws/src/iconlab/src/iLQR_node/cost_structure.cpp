/*
 * cost_structure.cpp
 *
 *  Created on: Jan 20, 2021
 *      Author: talhakavuncu
 */
#include <iostream>
#include <unsupported/Eigen/AdolcForward>
#include <adolc/adolc.h>
#include <Eigen/Dense>
#include "cost_structure.h"
#include <math.h>
namespace Unicycle_Cost

{
	adouble running_cost(const state_type_unicycle & x,const input_type_unicycle & u,const state_type_unicycle & x_goal)
	{


		adouble output,input_cost,state_cost;
		state_type_unicycle diff=x-x_goal;
	//	Eigen::Matrix<adouble,4,4> Q;
	//	Eigen::Matrix<adouble,2,2> R;
	//	Q<<1,0,0,0,
	//		0,1,0,0,
	//		0,0,1,0,
	//		0,0,0,1;

	//		Q<<0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
	//		R<<2,2,2,2;
	//	R<<1,0,0,1;
	//
	//	intermediate=inputs.transpose()*R*inputs;//+inputs[0]*states[2];//inputs.dot(inputs)*states.dot(states);
	//	output=diff.transpose()*Q*diff + intermediate;
		input_cost=u.dot(u);
		state_cost=diff.dot(diff);
		output=input_cost+state_cost;
		return output;
	};




	adouble terminal_cost(const state_type_unicycle & x,const input_type_unicycle & u,const state_type_unicycle & x_goal)
	{

		adouble output;
		state_type_unicycle diff=x-x_goal;

//		Eigen::Matrix<adouble,4,4> Q;
//		Q<<1,0,0,0,
//			0,1,0,0,
//			0,0,1,0,
//			0,0,0,1;
//		Eigen::Matrix<adouble,2,2> R;
//		R<<2,2,2,2;
//		R<<1,1,1,1;
//		intermediate=inputs.transpose()*R*inputs;//+inputs[0]*states[2];//inputs.dot(inputs)*states.dot(states);

		output=100*diff.dot(diff);
		return output;
	};



	adouble running_cost2(const state_type_unicycle2 & x,const input_type_unicycle2 & u,const state_type_unicycle2 & x_goal)
	{
		adouble output,input_cost,state_cost,ca_total;
		int row_size=1;
		int col_size=2;

		adouble d_prox=1.2;
//		adouble c_ij_costs[1][2]={{0}};
		std::vector<std::vector<adouble>> c_ij_costs(row_size,std::vector<adouble>(col_size));
		for(int i=0;i<row_size;++i)
			for(int j=i+1;j<col_size;++j)
				c_ij_cost(x.segment(i*4,2),x.segment(j*4,2),c_ij_costs[i][j],d_prox);

		calculate_total_c_ij_cost(c_ij_costs,ca_total);


		state_type_unicycle2 diff=x-x_goal;
		input_cost=u.dot(u);
		state_cost=diff.dot(diff);
		output=input_cost+state_cost+ca_total;
		return output;
	};
	adouble terminal_cost2(const state_type_unicycle2 & x,const input_type_unicycle2 & u,const state_type_unicycle2 & x_goal)
	{
		adouble output;
		state_type_unicycle2 diff=x-x_goal;
		output=25*diff.dot(diff);
		return output;
	};

	adouble running_cost3(const state_type_unicycle3 & x,const input_type_unicycle3 & u,const state_type_unicycle3 & x_goal)
	{
		adouble output,input_cost,state_cost,ca_total;
		int row_size=2;
		int col_size=3;

		adouble d_prox=0.6;
//		adouble c_ij_costs[1][2]={{0}};
		std::vector<std::vector<adouble>> c_ij_costs(row_size,std::vector<adouble>(col_size));
		for(int i=0;i<row_size;++i)
			for(int j=i+1;j<col_size;++j)
				c_ij_cost(x.segment(i*4,2),x.segment(j*4,2),c_ij_costs[i][j],d_prox);

		calculate_total_c_ij_cost(c_ij_costs,ca_total);
		state_type_unicycle3 diff=x-x_goal;
		input_cost=u.dot(u);
		state_cost=diff.dot(diff);
		output=input_cost+state_cost;
		return output;
	};
	adouble terminal_cost3(const state_type_unicycle3 & x,const input_type_unicycle3 & u,const state_type_unicycle3 & x_goal)
	{
		adouble output;
		state_type_unicycle3 diff=x-x_goal;
		output=diff.dot(diff);
		return output;
	};

	adouble running_cost4(const state_type_unicycle4 & x,const input_type_unicycle4 & u,const state_type_unicycle4 & x_goal)
	{
		adouble output,input_cost,state_cost,ca_total;
		int row_size=3;
		int col_size=4;

		adouble d_prox=0.6;
//		adouble c_ij_costs[1][2]={{0}};
		std::vector<std::vector<adouble>> c_ij_costs(row_size,std::vector<adouble>(col_size));
		for(int i=0;i<row_size;++i)
			for(int j=i+1;j<col_size;++j)
				c_ij_cost(x.segment(i*4,2),x.segment(j*4,2),c_ij_costs[i][j],d_prox);

		calculate_total_c_ij_cost(c_ij_costs,ca_total);
		state_type_unicycle4 diff=x-x_goal;
		input_cost=u.dot(u);
		state_cost=diff.dot(diff);
		output=input_cost+state_cost;
		return output;
	};
	adouble terminal_cost4(const state_type_unicycle4 & x,const input_type_unicycle4 & u,const state_type_unicycle4 & x_goal)
	{
		adouble output;
		state_type_unicycle4 diff=x-x_goal;
		output=diff.dot(diff);
		return output;
	};

	adouble running_cost6(const state_type_unicycle6 & x,const input_type_unicycle6 & u,const state_type_unicycle6 & x_goal)
	{
		adouble output,input_cost,state_cost,ca_total;
		int row_size=5;
		int col_size=6;

		adouble d_prox=0.6;
//		adouble c_ij_costs[1][2]={{0}};
		std::vector<std::vector<adouble>> c_ij_costs(row_size,std::vector<adouble>(col_size));
		for(int i=0;i<row_size;++i)
			for(int j=i+1;j<col_size;++j)
				c_ij_cost(x.segment(i*4,2),x.segment(j*4,2),c_ij_costs[i][j],d_prox);

		calculate_total_c_ij_cost(c_ij_costs,ca_total);
		state_type_unicycle6 diff=x-x_goal;
		input_cost=u.dot(u);
		state_cost=diff.dot(diff);
		output=input_cost+state_cost;
		return output;
	};
	adouble terminal_cost6(const state_type_unicycle6 & x,const input_type_unicycle6 & u,const state_type_unicycle6 & x_goal)
	{
		adouble output;
		state_type_unicycle6 diff=x-x_goal;
		output=diff.dot(diff);
		return output;
	};



	void c_ij_cost(const Eigen::Matrix<adouble,2,1> & x_i,const Eigen::Matrix<adouble,2,1> & x_j,
			adouble & cost,const adouble &d_prox)
	{
		Eigen::Matrix<adouble,2,1> diff;
		diff=x_i-x_j;
		adouble distance,distance_diff;
		distance=pow(diff.dot(diff),0.5);
		distance_diff=distance-d_prox;
		condassign(cost,distance_diff,adouble(0),1e3*pow(distance_diff,2));

	};

	void calculate_total_c_ij_cost(const std::vector<std::vector<adouble>> &c_ij_costs,adouble & total_cost)
	{
		for(int i=0;i<c_ij_costs.size();++i)
			for(int j=i+1;j<c_ij_costs[0].size();++j)
				total_cost+=c_ij_costs[i][j];
	};
};


//adouble terminal_cost_drone(const state_type_tensor & states,const input_type_tensor & inputs);
namespace Drone_Cost
{


	adouble running_cost(const state_type_drone & states,const input_type_drone & inputs,const state_type_drone & X_goal)
	{

		adouble output,input_cost,collision_cost,d_prox,distance,distance_diff;
		d_prox=3;
		Eigen::Matrix<adouble,3,1> avoid_pos,avoid_diff;
		avoid_pos<<5,-1,0;
//		avoid_diff<<
		state_type_drone diff;
		for (int i=0;i<3;++i)
			avoid_diff[i]=states[i]-avoid_pos[i];


		distance=pow(avoid_diff.dot(avoid_diff),0.5);
		distance_diff=distance-d_prox;
//		adouble zero=0;
//		adouble value=1e6*pow(distance_diff,2);
		condassign(collision_cost,distance_diff,adouble(0),1e3*pow(distance_diff,2));
//		for(int i=0;i<3;++i)
//			xyz_diff[i]=diff[i];

//		X_goal<<0,0,0, M_PI/2,0,0, 0,30,0, 0,0,0;
//		Eigen::Matrix<adouble,12,12> Q;
//		Q<<	1,0,0,0,0,0,0,0,0,0,0,0,
//			0,1,0,0,0,0,0,0,0,0,0,0,
//			0,0,1,0,0,0,0,0,0,0,0,0,
//			0,0,0,1/9,0,0,0,0,0,0,0,0,
//			0,0,0,0,1,0,0,0,0,0,0,0,
//			0,0,0,0,0,1,0,0,0,0,0,0,
//			0,0,0,0,0,0,1,0,0,0,0,0,
//			0,0,0,0,0,0,0,1/900,0,0,0,0,
//			0,0,0,0,0,0,0,0,1,0,0,0,
//			0,0,0,0,0,0,0,0,0,1,0,0,
//			0,0,0,0,0,0,0,0,0,0,1,0,
//			0,0,0,0,0,0,0,0,0,0,0,1;

//		diff=states-X_goal;
		input_cost=10*1/(mass*g/(4*C_T))*inputs.dot(inputs);
//		output=0.000001*diff.transpose()*Q*diff;//intermediate;
		diff=states-X_goal;
		output=100*diff.dot(diff)+input_cost+collision_cost;
//		output=1;
//		output=100*(1*diff[0]*diff[0]+1*diff[1]*diff[1]+1*diff[2]*diff[2]+100*diff[3]*diff[3]+100*diff[7]*diff[7]);
		return output;

	};

	adouble terminal_cost(const state_type_drone & states,const input_type_drone & inputs,const state_type_drone & X_goal)
	{
//		state_type_drone X_goal,diff;
		state_type_drone diff;
//		for (int i=0;i<12;++i)
//			diff[i]=states[i]-X_goal[i];
		diff=states-X_goal;
//		X_goal<<5,5,5, 0,0,0 ,0,0,0, 0,0,0;
//		X_goal<<0,0,0, M_PI/2,0,0, 0,30,0, 0,0,0;
//		diff=states-X_goal;
//		Eigen::Matrix<adouble,12,12> Q;
//		Q<<	1,0,0,0,0,0,0,0,0,0,0,0,
//			0,1,0,0,0,0,0,0,0,0,0,0,
//			0,0,1,0,0,0,0,0,0,0,0,0,
//			0,0,0,1/9,0,0,0,0,0,0,0,0,
//			0,0,0,0,1,0,0,0,0,0,0,0,
//			0,0,0,0,0,1,0,0,0,0,0,0,
//			0,0,0,0,0,0,1,0,0,0,0,0,
//			0,0,0,0,0,0,0,1/900,0,0,0,0,
//			0,0,0,0,0,0,0,0,1,0,0,0,
//			0,0,0,0,0,0,0,0,0,1,0,0,
//			0,0,0,0,0,0,0,0,0,0,1,0,
//			0,0,0,0,0,0,0,0,0,0,0,1;
		adouble output;
//		output=100*diff.transpose()*Q*diff;
		output=100*diff.dot(diff);
//		output=100*(1*diff[0]*diff[0]+1*diff[1]*diff[1]+1*diff[2]*diff[2]+100*diff[3]*diff[3]+100*diff[7]*diff[7]);
		return output;
	};

	adouble running_cost2(const state_type_drone2 & states,const input_type_drone2 & inputs,const state_type_drone2 & X_goal)
	{

		adouble output,input_cost,state_cost;

//		adouble d_prox_collision,collision_distance,distance_diff,collision_cost;
//		d_prox_collision=3;
//		Eigen::Matrix<adouble,3,1> avoid_pos,avoid_diff;
//		avoid_pos<<5,-1,0;
//		for (int i=0;i<3;++i)
//			avoid_diff[i]=states[i]-avoid_pos[i];
//
//		collision_distance=pow(avoid_diff.dot(avoid_diff),0.5);
//		distance_diff=collision_distance-d_prox_collision;
//		condassign(collision_cost,distance_diff,adouble(0),1e3*pow(distance_diff,2));

		adouble ca_distance_diff,ca_cost,ca_distance,ca_dprox;
		ca_dprox=0.5;
		Eigen::Matrix<adouble,3,1> pos_diff;
		for (int i=0;i<3;++i)
			pos_diff[i]=states[i]-states[i+12];

		ca_distance=pow(pos_diff.dot(pos_diff),0.5);
		ca_distance_diff=ca_distance-ca_dprox;
		condassign(ca_cost,ca_distance_diff,adouble(0),1e3*pow(ca_distance_diff,2));

		state_type_drone2 goal_diff;
		goal_diff=states-X_goal;
		input_cost=10*1/(mass*g/(4*C_T))*inputs.dot(inputs);
		state_cost=100*goal_diff.dot(goal_diff);
		output=state_cost+input_cost+ca_cost;
		return output;

	};

	adouble terminal_cost2(const state_type_drone2 & states,const input_type_drone2 & inputs,const state_type_drone2 & X_goal)
	{
		state_type_drone2 diff;
		diff=states-X_goal;
		adouble output;
		output=100*diff.dot(diff);
		return output;
	};

};

namespace Single_Integrator_Cost{

	adouble running_cost(const state_tensor & x, const input_tensor & u,const state_tensor & x_goal)
	{
		adouble output,input_cost,state_cost,collision_cost,d_prox,distance,distance_diff;
		state_tensor diff,avoid_pose,avoid_diff;
		d_prox=0.6;
		avoid_pose<<-0.5,0.5,0.5;
		avoid_diff=x-avoid_pose;

		distance=pow(avoid_diff.dot(avoid_diff),0.5);
		distance_diff=distance-d_prox;

		condassign(collision_cost,distance_diff,adouble(0),1e5*pow(distance_diff,2));

		diff=x-x_goal;
		state_cost=1e2*diff.dot(diff);
		input_cost=1e1*u.dot(u);

		output=state_cost+input_cost+collision_cost;

		return output;

	}
	adouble terminal_cost(const state_tensor & x, const input_tensor & u,const state_tensor & x_goal)
	{
		state_tensor diff=x-x_goal;
		adouble state_cost=1e2*diff.dot(diff);

		return state_cost;

	}

	adouble running_cost2(const state_tensor2 & x, const input_tensor2 & u,const state_tensor2 & x_goal)
	{
		adouble output,input_cost,state_cost;

//		adouble d_prox_collision,collision_distance,distance_diff,collision_cost;
//		d_prox_collision=3;
//		Eigen::Matrix<adouble,3,1> avoid_pos,avoid_diff;
//		avoid_pos<<5,-1,0;
//		for (int i=0;i<3;++i)
//			avoid_diff[i]=states[i]-avoid_pos[i];
//
//		collision_distance=pow(avoid_diff.dot(avoid_diff),0.5);
//		distance_diff=collision_distance-d_prox_collision;
//		condassign(collision_cost,distance_diff,adouble(0),1e3*pow(distance_diff,2));

		adouble ca_distance_diff,ca_cost,ca_distance,ca_dprox;
		ca_dprox=0.5;
		Eigen::Matrix<adouble,3,1> pos_diff;
		for (int i=0;i<3;++i)
			pos_diff[i]=x[i]-x[i+3];

		ca_distance=pow(pos_diff.dot(pos_diff),0.5);
		ca_distance_diff=ca_distance-ca_dprox;
		condassign(ca_cost,ca_distance_diff,adouble(0),1e5*pow(ca_distance_diff,2));

		state_tensor2 goal_diff;
		goal_diff=x-x_goal;
//		input_cost=0.12*1e1*u.dot(u);
		input_cost=0.5*1e1*u.dot(u);
		state_cost=1e-20*goal_diff.dot(goal_diff);
		output=state_cost+input_cost+ca_cost;
		return output;


	}
	adouble terminal_cost2(const state_tensor2 & x, const input_tensor2 & u,const state_tensor2 & x_goal)
	{
		state_tensor2 diff=x-x_goal;
		adouble state_cost=100*diff.dot(diff);

		return state_cost;
	}

}




namespace Double_Integrator_Cost{

	adouble running_cost(const state_tensor & x, const input_tensor & u,const state_tensor & x_goal)
	{
		adouble output,input_cost,state_cost,collision_cost,d_prox,distance,distance_diff;
		state_tensor diff;
		Eigen::Matrix<adouble,3,1> avoid_pos,avoid_diff;
		d_prox=1;
		avoid_pos<<1,1,1;
		avoid_diff=x.segment(0,3)-avoid_pos;
		distance=pow(avoid_diff.dot(avoid_diff),0.5);
		distance_diff=distance-d_prox;

		condassign(collision_cost,distance_diff,adouble(0),1e3*pow(distance_diff,2));

		diff=x-x_goal;
		state_cost=1e2*diff.dot(diff);
		input_cost=1*1e-2*u.dot(u);

		output=state_cost+input_cost+collision_cost;

		return output;

	}
	adouble terminal_cost(const state_tensor & x, const input_tensor & u,const state_tensor & x_goal)
	{
		state_tensor diff=x-x_goal;
		adouble state_cost=1e2*diff.dot(diff);

		return state_cost;

	}

//	adouble running_cost2(const state_tensor2 & x, const input_tensor2 & u,const state_tensor2 & x_goal)
//	{
//		adouble output,input_cost,state_cost;
//
////		adouble d_prox_collision,collision_distance,distance_diff,collision_cost;
////		d_prox_collision=3;
////		Eigen::Matrix<adouble,3,1> avoid_pos,avoid_diff;
////		avoid_pos<<5,-1,0;
////		for (int i=0;i<3;++i)
////			avoid_diff[i]=states[i]-avoid_pos[i];
////
////		collision_distance=pow(avoid_diff.dot(avoid_diff),0.5);
////		distance_diff=collision_distance-d_prox_collision;
////		condassign(collision_cost,distance_diff,adouble(0),1e3*pow(distance_diff,2));
//
//		adouble ca_distance_diff,ca_cost,ca_distance,ca_dprox;
//		ca_dprox=0.5;
//		Eigen::Matrix<adouble,3,1> pos_diff;
//		for (int i=0;i<3;++i)
//			pos_diff[i]=x[i]-x[i+3];
//
//		ca_distance=pow(pos_diff.dot(pos_diff),0.5);
//		ca_distance_diff=ca_distance-ca_dprox;
//		condassign(ca_cost,ca_distance_diff,adouble(0),1e5*pow(ca_distance_diff,2));
//
//		state_tensor2 goal_diff;
//		goal_diff=x-x_goal;
////		input_cost=0.12*1e1*u.dot(u);
//		input_cost=0.5*1e1*u.dot(u);
//		state_cost=1e-20*goal_diff.dot(goal_diff);
//		output=state_cost+input_cost+ca_cost;
//		return output;
//
//
//	}
//	adouble terminal_cost2(const state_tensor2 & x, const input_tensor2 & u,const state_tensor2 & x_goal)
//	{
//		state_tensor2 diff=x-x_goal;
//		adouble state_cost=100*diff.dot(diff);
//
//		return state_cost;
//	}

}

namespace Drone_First_Order_Cost
{

	adouble running_cost(const state_type_drone & states,const input_type_drone & inputs,const state_type_drone & X_goal)
	{
		adouble output,input_cost,collision_cost,d_prox,distance,distance_diff;
		d_prox=1.0;
		Eigen::Matrix<adouble,3,1> avoid_pos,avoid_diff;
		avoid_pos<<0.5,0.5,0.5;
//		avoid_diff<<
		state_type_drone diff;
		for (int i=0;i<3;++i)
			avoid_diff[i]=states[i]-avoid_pos[i];


		distance=pow(avoid_diff.dot(avoid_diff),0.5);
		distance_diff=distance-d_prox;
//		adouble zero=0;
//		adouble value=1e6*pow(distance_diff,2);
		condassign(collision_cost,distance_diff,adouble(0),1e6*pow(distance_diff,2));
//		for(int i=0;i<3;++i)
//			xyz_diff[i]=diff[i];

//		X_goal<<0,0,0, M_PI/2,0,0, 0,30,0, 0,0,0;
//		Eigen::Matrix<adouble,12,12> Q;
//		Q<<	1,0,0,0,0,0,0,0,0,0,0,0,
//			0,1,0,0,0,0,0,0,0,0,0,0,
//			0,0,1,0,0,0,0,0,0,0,0,0,
//			0,0,0,1/9,0,0,0,0,0,0,0,0,
//			0,0,0,0,1,0,0,0,0,0,0,0,
//			0,0,0,0,0,1,0,0,0,0,0,0,
//			0,0,0,0,0,0,1,0,0,0,0,0,
//			0,0,0,0,0,0,0,1/900,0,0,0,0,
//			0,0,0,0,0,0,0,0,1,0,0,0,
//			0,0,0,0,0,0,0,0,0,1,0,0,
//			0,0,0,0,0,0,0,0,0,0,1,0,
//			0,0,0,0,0,0,0,0,0,0,0,1;

//		diff=states-X_goal;
		input_cost=3e1*inputs.dot(inputs);
//		output=0.000001*diff.transpose()*Q*diff;//intermediate;
		diff=states-X_goal;
		output=100*diff.dot(diff)+input_cost+collision_cost;
//		output=1;
//		output=100*(1*diff[0]*diff[0]+1*diff[1]*diff[1]+1*diff[2]*diff[2]+100*diff[3]*diff[3]+100*diff[7]*diff[7]);
		return output;
	}
	adouble terminal_cost(const state_type_drone & states,const input_type_drone & inputs,const state_type_drone & X_goal)
	{
		adouble output;
		state_type_drone diff;
		diff=states-X_goal;
//		output=100*diff.transpose()*Q*diff;
		output=100*diff.dot(diff);
//		output=100*(1*diff[0]*diff[0]+1*diff[1]*diff[1]+1*diff[2]*diff[2]+100*diff[3]*diff[3]+100*diff[7]*diff[7]);
		return output;
	}

	adouble running_cost2(const state_type_drone2 & states,const input_type_drone2 & inputs,const state_type_drone2 & X_goal)
	{
		adouble output,input_cost,state_cost;

//		adouble d_prox_collision,collision_distance,distance_diff,collision_cost;
//		d_prox_collision=3;
//		Eigen::Matrix<adouble,3,1> avoid_pos,avoid_diff;
//		avoid_pos<<5,-1,0;
//		for (int i=0;i<3;++i)
//			avoid_diff[i]=states[i]-avoid_pos[i];
//
//		collision_distance=pow(avoid_diff.dot(avoid_diff),0.5);
//		distance_diff=collision_distance-d_prox_collision;
//		condassign(collision_cost,distance_diff,adouble(0),1e3*pow(distance_diff,2));

		adouble ca_distance_diff,ca_cost,ca_distance,ca_dprox;
		ca_dprox=0.30;
		Eigen::Matrix<adouble,3,1> pos_diff;
		for (int i=0;i<3;++i)
			pos_diff[i]=states[i]-states[i+6];

		ca_distance=pow(pos_diff.dot(pos_diff),0.5);
		ca_distance_diff=ca_distance-ca_dprox;
		condassign(ca_cost,ca_distance_diff,adouble(0),1e6*pow(ca_distance_diff,2));

		state_type_drone2 goal_diff;
		goal_diff=states-X_goal;
		input_cost=0.4e2*inputs.dot(inputs);
		state_cost=0.5*goal_diff.dot(goal_diff);
		output=state_cost+input_cost+ca_cost;
		return output;
	}
	adouble terminal_cost2(const state_type_drone2 & states,const input_type_drone2 & inputs,const state_type_drone2 & X_goal)
	{
		state_type_drone2 diff;
		diff=states-X_goal;
		adouble output;
		output=200*diff.dot(diff);
		return output;
	}
}
