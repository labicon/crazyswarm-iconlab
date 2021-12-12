/*
 * cost_structure.h
 *
 *  Created on: Jan 21, 2021
 *      Author: talhakavuncu
 */

#ifndef COST_STRUCTURE_H_
#define COST_STRUCTURE_H_

//#include <iostream>
#include <unsupported/Eigen/AdolcForward>
#include <adolc/adolc.h>
#include <Eigen/Dense>
//#include "cost.h"
//typedef Eigen::Matrix<adouble,Eigen::Dynamic,1> vec_type;
//template<size_t state_size,size_t input_size>


namespace Unicycle_Cost{
	typedef Eigen::Matrix<adouble,4,1> state_type_unicycle;
	typedef Eigen::Matrix<adouble,2,1> input_type_unicycle;
	adouble running_cost(const state_type_unicycle & x,const input_type_unicycle & u,const state_type_unicycle & x_goal);
	adouble terminal_cost(const state_type_unicycle & x,const input_type_unicycle & u,const state_type_unicycle & x_goal);

	typedef Eigen::Matrix<adouble,4*2,1> state_type_unicycle2;
	typedef Eigen::Matrix<adouble,2*2,1> input_type_unicycle2;
	adouble running_cost2(const state_type_unicycle2 & x,const input_type_unicycle2 & u,const state_type_unicycle2 & x_goal);
	adouble terminal_cost2(const state_type_unicycle2 & x,const input_type_unicycle2 & u,const state_type_unicycle2 & x_goal);

	typedef Eigen::Matrix<adouble,4*3,1> state_type_unicycle3;
	typedef Eigen::Matrix<adouble,2*3,1> input_type_unicycle3;
	adouble running_cost3(const state_type_unicycle3 & x,const input_type_unicycle3 & u,const state_type_unicycle3 & x_goal);
	adouble terminal_cost3(const state_type_unicycle3 & x,const input_type_unicycle3 & u,const state_type_unicycle3 & x_goal);

	typedef Eigen::Matrix<adouble,4*4,1> state_type_unicycle4;
	typedef Eigen::Matrix<adouble,2*4,1> input_type_unicycle4;
	adouble running_cost4(const state_type_unicycle4 & x,const input_type_unicycle4 & u,const state_type_unicycle4 & x_goal);
	adouble terminal_cost4(const state_type_unicycle4 & x,const input_type_unicycle4 & u,const state_type_unicycle4 & x_goal);

	typedef Eigen::Matrix<adouble,4*6,1> state_type_unicycle6;
	typedef Eigen::Matrix<adouble,2*6,1> input_type_unicycle6;
	adouble running_cost6(const state_type_unicycle6 & x,const input_type_unicycle6 & u,const state_type_unicycle6 & x_goal);
	adouble terminal_cost6(const state_type_unicycle6 & x,const input_type_unicycle6 & u,const state_type_unicycle6 & x_goal);

	void c_ij_cost(const Eigen::Matrix<adouble,2,1> & x_i,const Eigen::Matrix<adouble,2,1> & x_j,
			adouble & cost,const adouble &d_prox);
	void calculate_total_c_ij_cost(const std::vector<std::vector<adouble>> &c_ij_costs,adouble & total_cost);
}



namespace Drone_Cost{
//	adouble C_T,C_D,g,d,I_xx,I_yy,I_zz,mass;
	const double C_T=3.1582*1e-10;
	const double C_D=7.9379*1e-12;
	const double g=9.80665;
	const double d=39.73*1e-3;
	const double I_xx=1.395*1e-5;
	const double I_yy=1.436*1e-5;
	const double I_zz=2.173*1e-5;
	const double mass=0.033;
	typedef Eigen::Matrix<adouble,12,1> state_type_drone;
//	typedef Eigen::Matrix<double,12,1> goal_type_drone;
	typedef Eigen::Matrix<adouble,4,1> input_type_drone;
	typedef Eigen::Matrix<adouble,12*2,1> state_type_drone2;
//	typedef Eigen::Matrix<double,12*2,1> goal_type_drone2;
	typedef Eigen::Matrix<adouble,4*2,1> input_type_drone2;
	adouble running_cost(const state_type_drone & states,const input_type_drone & inputs,const state_type_drone & X_goal);
	adouble terminal_cost(const state_type_drone & states,const input_type_drone & inputs,const state_type_drone & X_goal);

	adouble running_cost2(const state_type_drone2 & states,const input_type_drone2 & inputs,const state_type_drone2 & X_goal);
	adouble terminal_cost2(const state_type_drone2 & states,const input_type_drone2 & inputs,const state_type_drone2 & X_goal);

};

namespace Single_Integrator_Cost
{
	typedef Eigen::Matrix<adouble,3,1> state_tensor;
	typedef Eigen::Matrix<adouble,3,1> input_tensor;
	adouble running_cost(const state_tensor & x, const input_tensor & u,const state_tensor & x_goal);
	adouble terminal_cost(const state_tensor & x, const input_tensor & u,const state_tensor & x_goal);

	typedef Eigen::Matrix<adouble,6,1> state_tensor2;
	typedef Eigen::Matrix<adouble,6,1> input_tensor2;
	adouble running_cost2(const state_tensor2 & x, const input_tensor2 & u,const state_tensor2 & x_goal);
	adouble terminal_cost2(const state_tensor2 & x, const input_tensor2 & u,const state_tensor2 & x_goal);
}

namespace Double_Integrator_Cost
{
	typedef Eigen::Matrix<adouble,6,1> state_tensor;
	typedef Eigen::Matrix<adouble,3,1> input_tensor;
	adouble running_cost(const state_tensor & x, const input_tensor & u,const state_tensor & x_goal);
	adouble terminal_cost(const state_tensor & x, const input_tensor & u,const state_tensor & x_goal);

	typedef Eigen::Matrix<adouble,12,1> state_tensor2;
	typedef Eigen::Matrix<adouble,6,1> input_tensor2;
	adouble running_cost2(const state_tensor2 & x, const input_tensor2 & u,const state_tensor2 & x_goal);
	adouble terminal_cost2(const state_tensor2 & x, const input_tensor2 & u,const state_tensor2 & x_goal);
}
namespace Drone_First_Order_Cost
{
	typedef Eigen::Matrix<adouble,6,1> state_type_drone;
	//	typedef Eigen::Matrix<double,12,1> goal_type_drone;
	typedef Eigen::Matrix<adouble,6,1> input_type_drone;
	typedef Eigen::Matrix<adouble,6*2,1> state_type_drone2;
	//	typedef Eigen::Matrix<double,12*2,1> goal_type_drone2;
	typedef Eigen::Matrix<adouble,6*2,1> input_type_drone2;
	adouble running_cost(const state_type_drone & states,const input_type_drone & inputs,const state_type_drone & X_goal);
	adouble terminal_cost(const state_type_drone & states,const input_type_drone & inputs,const state_type_drone & X_goal);

	adouble running_cost2(const state_type_drone2 & states,const input_type_drone2 & inputs,const state_type_drone2 & X_goal);
	adouble terminal_cost2(const state_type_drone2 & states,const input_type_drone2 & inputs,const state_type_drone2 & X_goal);
}



#endif /* COST_STRUCTURE_H_ */
