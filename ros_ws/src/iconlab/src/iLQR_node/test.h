/*
 * test3.cpp
 *
 *  Created on: Jan 18, 2021
 *      Author: talhakavuncu
 */

#include <unsupported/Eigen/AdolcForward>
#include <adolc/adolc.h>
#include <Eigen/Dense>

//void cost()
//{
//	trace_on(1);
//	int state_dim=4;
//	int input_dim=2;
//	adouble output,intermediate;
//	double output_d;
//	Eigen::Matrix<adouble,4,1> states;
//	Eigen::Matrix<adouble,2,1> inputs;
//	Eigen::Matrix<double,4,1> states_d;
//	Eigen::Matrix<double,2,1> inputs_d;
//	Eigen::Matrix<adouble,4,4> Q;
//	Eigen::Matrix<adouble,2,2> R;
//	Q<<1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1;
//	R<<2,2,2,2;
////	std::cout<<Q;
//	for(int i=0;i<state_dim;++i)
//	{
//		states[i]<<=states_d[i];
//	}
//	for(int i=0;i<input_dim;++i)
//	{
//		inputs[i]<<=inputs_d[i];
//	}
//
//	intermediate=inputs.transpose()*R*inputs;
//	output=states.transpose()*Q*states + intermediate;
//
//	output>>=output_d;
//	trace_off();
//}
