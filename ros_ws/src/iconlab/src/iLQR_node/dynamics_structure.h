/*
 * dynamics_structure.h
 *
 *  Created on: Jan 21, 2021
 *      Author: talhakavuncu
 */

#ifndef DYNAMICS_STRUCTURE_H_
#define DYNAMICS_STRUCTURE_H_


//#include <iostream>
#include <unsupported/Eigen/AdolcForward>
//#include <adolc.h>
#include <Eigen/Dense>
#include "dynamics.h"


namespace Unicycle_Dynamics
{
	typedef Eigen::Matrix<adouble,4,1> unicycle_state_tensor;
	typedef Eigen::Matrix<adouble,2,1> unicycle_input_tensor;

	unicycle_state_tensor dynamics(const unicycle_state_tensor & x,const unicycle_input_tensor & u);

//unicycle_state_type dynamics_unicycle2(const unicycle_state_type & x,const unicycle_input_type & u);
	typedef Eigen::Matrix<adouble,4*2,1> unicycle2_state_tensor;
	typedef Eigen::Matrix<adouble,2*2,1> unicycle2_input_tensor;
	unicycle2_state_tensor dynamics_2(const unicycle2_state_tensor & x,const unicycle2_input_tensor & u);

	typedef Eigen::Matrix<adouble,4*3,1> unicycle3_state_tensor;
	typedef Eigen::Matrix<adouble,2*3,1> unicycle3_input_tensor;
	unicycle3_state_tensor dynamics_3(const unicycle3_state_tensor & x,const unicycle3_input_tensor & u);

	typedef Eigen::Matrix<adouble,4*4,1> unicycle4_state_tensor;
	typedef Eigen::Matrix<adouble,2*4,1> unicycle4_input_tensor;
	unicycle4_state_tensor dynamics_4(const unicycle4_state_tensor & x,const unicycle4_input_tensor & u);

	typedef Eigen::Matrix<adouble,4*6,1> unicycle6_state_tensor;
	typedef Eigen::Matrix<adouble,2*6,1> unicycle6_input_tensor;
	unicycle6_state_tensor dynamics_6(const unicycle6_state_tensor & x,const unicycle6_input_tensor & u);


}





namespace Drone_Dynamics
{
	typedef Eigen::Matrix<adouble,12,1> drone_state_tensor;
	typedef Eigen::Matrix<adouble,4,1> drone_input_tensor;

	typedef Eigen::Matrix<adouble,12*2,1> drone2_state_tensor;
	typedef Eigen::Matrix<adouble,4*2,1> drone2_input_tensor;

	const double C_T=3.1582*1e-10;
	const double C_D=7.9379*1e-12;
	const double g=9.80665;
	const double d=39.73*1e-3;
	const double I_xx=1.395*1e-5;
	const double I_yy=1.436*1e-5;
	const double I_zz=2.173*1e-5;
	const double mass=0.033;
	drone_state_tensor dynamics(const drone_state_tensor & x,const drone_input_tensor & u);

	drone2_state_tensor dynamics_2(const drone2_state_tensor & x,const drone2_input_tensor & u);

}


namespace Single_Integrator_3D
{

	typedef Eigen::Matrix<adouble,3,1> state_tensor;
	typedef Eigen::Matrix<adouble,3,1> input_tensor;

	state_tensor dynamics(const state_tensor & x, const input_tensor & u);

	typedef Eigen::Matrix<adouble,6,1> state_tensor2;
	typedef Eigen::Matrix<adouble,6,1> input_tensor2;

	state_tensor2 dynamics2(const state_tensor2 & x, const input_tensor2 & u);


}
namespace Double_Integrator_3D
{
	const double mb=1;
	typedef Eigen::Matrix<adouble,6,1> state_tensor;
	typedef Eigen::Matrix<adouble,3,1> input_tensor;

	state_tensor dynamics(const state_tensor & x, const input_tensor & u);

	typedef Eigen::Matrix<adouble,12,1> state_tensor2;
	typedef Eigen::Matrix<adouble,6,1> input_tensor2;

	state_tensor2 dynamics2(const state_tensor2 & x, const input_tensor2 & u);


}
namespace Drone_First_Order_Dynamics
{
	typedef Eigen::Matrix<adouble,6,1> drone_state_tensor;
	typedef Eigen::Matrix<adouble,6,1> drone_input_tensor;

	typedef Eigen::Matrix<adouble,6*2,1> drone2_state_tensor;
	typedef Eigen::Matrix<adouble,6*2,1> drone2_input_tensor;

//	const double C_T=3.1582*1e-10;
//	const double C_D=7.9379*1e-12;
//	const double g=9.80665;
//	const double d=39.73*1e-3;
//	const double I_xx=1.395*1e-5;
//	const double I_yy=1.436*1e-5;
//	const double I_zz=2.173*1e-5;
//	const double mass=0.033;
	drone_state_tensor dynamics(const drone_state_tensor & X,const drone_input_tensor & input);

	drone2_state_tensor dynamics_2(const drone2_state_tensor & x,const drone2_input_tensor & u);

}


#endif /* DYNAMICS_STRUCTURE_H_ */
