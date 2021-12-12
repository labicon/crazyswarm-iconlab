// This file is part of Eigen, a lightweight C++ template library
// for linear algebra.
//
// Copyright (C) 2009 Gael Guennebaud <gael.guennebaud@inria.fr>
//
// This Source Code Form is subject to the terms of the Mozilla
// Public License v. 2.0. If a copy of the MPL was not distributed
// with this file, You can obtain one at https://urldefense.com/v3/__http://mozilla.org/MPL/2.0/__;!!DZ3fjg!v5H_MtY41GsBht4EP2ryGNiJdIFmlmHnuOaL04RB_wTTVM4Cx7iADjAmTBDYHKPPtEIG$ .
#ifndef COST_H_
#define COST_H_
#include <unsupported/Eigen/AdolcForward>
#include <adolc/adolc.h>
#include <Eigen/Dense>
#include "cost_structure.h"

//typedef Eigen::Matrix<adouble,Eigen::Dynamic,Eigen::Dynamic> mat_type;

//typedef Eigen::Matrix<double,Eigen::Dynamic,1> vec_type_d;
//typedef Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> input_type;
//adouble running_cost(const vec_type & states,const vec_type & inputs);
//adouble terminal_cost(const vec_type & states,const vec_type & inputs);
template<size_t state_size,size_t input_size>
class cost
{
public:
	typedef Eigen::Matrix<adouble,state_size,1> state_type_tensor;
	typedef Eigen::Matrix<adouble,input_size,1> input_type_tensor;
	typedef adouble (*cost_type)(const state_type_tensor & states,
			const input_type_tensor & inputs,const state_type_tensor & X_goal);
	typedef Eigen::Matrix<double,state_size,state_size> mat_type1;
	typedef Eigen::Matrix<double,input_size,input_size> mat_type2;
	typedef Eigen::Matrix<double,input_size,state_size> mat_type3;
	typedef Eigen::Matrix<double,state_size,1> state_type;
	typedef Eigen::Matrix<double,input_size,1> input_type;
	typedef Eigen::Matrix<double,state_size+input_size+1,state_size+input_size+1> tensor_type;


//	cost(const mat_type &Q,const mat_type &R);
	cost(cost_type cost_fun,unsigned int tag);
//	cost(){};
//	double eval(const double * const state,const double * const input);
	double eval(const state_type &states,const input_type &inputs);
	state_type _x(const state_type &states,const input_type &inputs);
	input_type _u(const state_type &states,const input_type &inputs);
	mat_type1 _xx(const state_type &states,const input_type &inputs);
	mat_type2 _uu(const state_type &states,const input_type &inputs);
	mat_type3 _ux(const state_type &states,const input_type &inputs);
	tensor_type evaluate_tensor(const state_type &states,const input_type &inputs,state_type &X_goal);
	unsigned int get_tag();
	int get_state_size();
	int get_input_size();
private:
	unsigned int tag;
	adouble (*cost_fun)(const state_type_tensor & states,const input_type_tensor & inputs,const state_type_tensor & X_goal);
//	int state_size;
//	int input_size;
//	Eigen::MatrixXd values(state_size+input_size+1,state_size+input_size+1);
	tensor_type values;
	state_type _x_val;
	input_type _u_val;
	mat_type1 _xx_val;
	mat_type2 _uu_val;
	mat_type3 _ux_val;
	int num_in_vars=state_size+input_size;
	int tensor_dim=(num_in_vars+2)*(num_in_vars+1)/2;
	double **S=myalloc(num_in_vars,num_in_vars);
	double **tensor_=myalloc(1,tensor_dim);

};


template<size_t state_size,size_t input_size>
cost<state_size,input_size>::cost(cost_type cost_fun,unsigned int tag)
{
	this->cost_fun=cost_fun;
	cost::tag=tag;
	for(int i=0;i<num_in_vars;++i)
	{
		for(int j=0;j<num_in_vars;++j)
		{
			if(i==j)
			{
				S[i][j]=1;
			}
			else
			{
				S[i][j]=0;
			}
		}
	}

	adouble output;
	double output_d=0;
	state_type_tensor states,goal_states;
	input_type_tensor inputs;
	state_type states_d,goal_states_d;
	input_type inputs_d;

	trace_on(tag);
//	Eigen::Matrix<double,state_size,1> goal_state;
//	locint indices[state_size];


	for(int i=0;i<state_size;++i)
	{
//		std::cout<<i<<std::endl;
		states[i]<<=states_d[i];
		goal_states[i]=mkparam(goal_states_d[i]);
	}
	for(int i=0;i<input_size;++i)
	{
//		std::cout<<i<<std::endl;
		inputs[i]<<=inputs_d[i];
	}

	output=this->cost_fun(states,inputs,goal_states);

	output>>=output_d;
	trace_off();

};
template<size_t state_size,size_t input_size>
typename cost<state_size,input_size>::tensor_type
cost<state_size,input_size>::evaluate_tensor(const state_type &states,const input_type &inputs,state_type &X_goal)
{
	Eigen::Matrix<double,state_size+input_size,1> concat(state_size + input_size);
	concat<<states,inputs;
	set_param_vec(tag, state_size, &X_goal[0]);
//	set_param_vec(tag, numparam, paramvec)
	tensor_eval(cost::tag, 1, num_in_vars, 2, num_in_vars, &concat[0], tensor_, S);

	for (int j=0;j<=state_size+input_size;++j)
	{

		for(int k=j;k*(k+1)/2+j<tensor_dim;++k)
		{
//			std::cout<<"j "<<j<<" k "<<k<<std::endl;
			values(j,k)=tensor_[0][k*(k+1)/2+j];//(0,k*(k+1)/2+j);
			values(k,j)=tensor_[0][k*(k+1)/2+j];//(0,k*(k+1)/2+j);
		}
	}
	return values;

}
template<size_t state_size,size_t input_size>
double cost<state_size,input_size>::eval(const state_type &states,const input_type &inputs)
{
	return values(0,0);
};

template<size_t state_size,size_t input_size>
typename cost<state_size,input_size>::mat_type1
cost<state_size,input_size>::_xx(const state_type &states,const input_type &inputs)
{

//	for(int i=0;i<state_size;++i)
//	{
//		for(int j=0;j<state_size;++j)
//		{
//			_xx_val(i,j)=values(1+i,1+j);
//		}
//	}

	return values.block(1, 1, state_size, state_size);
//	return _xx_val;
//	return values.block<state_size,state_size>(1,1)
}
template<size_t state_size,size_t input_size>
typename cost<state_size,input_size>::mat_type2
cost<state_size,input_size>::_uu(const state_type &states,const input_type &inputs)
{
	return values.block(state_size+1, state_size+1, input_size, input_size);
}
template<size_t state_size,size_t input_size>
typename cost<state_size,input_size>::mat_type3
cost<state_size,input_size>::_ux(const state_type &states,const input_type &inputs)
{
	return values.block(state_size+1, 1, input_size, state_size);
}
template<size_t state_size,size_t input_size>
typename cost<state_size,input_size>::state_type
cost<state_size,input_size>::_x(const state_type &states,const input_type &inputs)
{
	return values.block(1, 0, state_size, 1);
}
template<size_t state_size,size_t input_size>
typename cost<state_size,input_size>::input_type
cost<state_size,input_size>::_u(const state_type &states,const input_type &inputs)
{
	return values.block(1+state_size, 0, input_size, 1);
}
//Eigen::MatrixXd cost::_uu(const Eigen::VectorXd &states,const Eigen::VectorXd &inputs)
//{
//	return values.block(state_size+1, state_size+1, input_size, input_size);
//}
//
//Eigen::MatrixXd cost::_ux(const Eigen::VectorXd &states,const Eigen::VectorXd &inputs)
//{
//	return values.block(state_size+1, 1, input_size, state_size);
//}
//Eigen::MatrixXd cost::_x(const Eigen::VectorXd &states,const Eigen::VectorXd &inputs)
//{
//	return values.block(1, 0, state_size, 1);
//}
//Eigen::MatrixXd cost::_u(const Eigen::VectorXd &states,const Eigen::VectorXd &inputs)
//{
//	return values.block(1+state_size, 0, input_size, 1);
//}
//int cost::get_state_size()
//{
//	return this->state_size;
//}
//int cost::get_input_size()
//{
//	return this->input_size;
//}


#endif
