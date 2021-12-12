/*
 * dynamics.h
 *
 *  Created on: Jan 20, 2021
 *      Author: talhakavuncu
 */



#ifndef DYNAMICS_H_
#define DYNAMICS_H_

#include <unsupported/Eigen/AdolcForward>
#include <adolc/adolc.h>
#include <Eigen/Dense>
#include "dynamics_structure.h"
//template <typename Derived>
//typedef Eigen::Matrix<adouble,Eigen::Dynamic,Eigen::Dynamic> mat_type;
//typedef Eigen::Matrix<adouble,Eigen::Dynamic,1> vec_type;
////typedef Eigen::Matrix<double,Eigen::Dynamic,1> vec_type_d;
//typedef Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> input_type;

template<size_t state_size,size_t input_size>
class dynamics
{
public:

	typedef Eigen::Matrix<adouble,state_size,1> state_type_tensor;
	typedef Eigen::Matrix<adouble,input_size,1> input_type_tensor;
	typedef state_type_tensor (*dynamics_type)(const state_type_tensor & states,const input_type_tensor & inputs);
	typedef Eigen::Matrix<double,state_size,state_size> mat_type1;
	typedef Eigen::Matrix<double,input_size,input_size> mat_type2;
	typedef Eigen::Matrix<double,input_size,state_size> mat_type3;
	typedef Eigen::Matrix<double,state_size,input_size> mat_type4;
	typedef Eigen::Matrix<double,state_size,1> state_type;
	typedef Eigen::Matrix<double,input_size,1> input_type;
	typedef Eigen::Matrix<double,state_size,state_size+input_size+1> tensor_type;

//	typedef state_type_tensor (*dynamics_type)(const state_type_tensor &,const input_type_tensor &);
	dynamics(dynamics_type dynamics_fun,unsigned int tag,double time_step);
//	dynamics(const dynamics &);
//	dynamics(){};
	state_type eval(const state_type &states,const input_type &inputs);
	mat_type1 _x(const state_type &states,const input_type &inputs);
	mat_type4 _u(const state_type &states,const input_type &inputs);
	tensor_type evaluate_tensor(const state_type &states,const input_type &inputs);
	unsigned int get_tag();
private:
	unsigned int tag;
	double time_step;
	state_type_tensor (*dynamics_fun)(const state_type_tensor&, const input_type_tensor&);
	state_type_tensor integrate(dynamics_type, const state_type_tensor&, const input_type_tensor&);
	tensor_type values;
	mat_type1 _x_val;
	mat_type4 _u_val;
	int num_in_vars=state_size+input_size;
	int tensor_dim=num_in_vars+1;
	double **S=myalloc(num_in_vars,num_in_vars);
	double **tensor_=myalloc(state_size,tensor_dim);

};
template<size_t state_size,size_t input_size>
dynamics<state_size,input_size>::dynamics(dynamics_type dynamics_fun,unsigned int tag,double time_step)
{
	this->dynamics_fun=dynamics_fun;
	dynamics::tag=tag;
	this->time_step=time_step;

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

	trace_on(tag);

	state_type_tensor states,output;
	input_type_tensor inputs;

	state_type states_d,output_d;
	input_type inputs_d;

	for(int i=0;i<state_size;++i)
	{
//		std::cout<<i<<std::endl;
		states[i]<<=states_d[i];

	}
	for(int i=0;i<input_size;++i)
	{
//		std::cout<<i<<std::endl;
		inputs[i]<<=inputs_d[i];
	}

	output=integrate(dynamics_fun,states,inputs);

	for(int i=0;i<state_size;++i)
	{
		output[i]>>=output_d[i];
	}

	trace_off();
};

template<size_t state_size,size_t input_size>
typename dynamics<state_size,input_size>::state_type_tensor
dynamics<state_size,input_size>::integrate(dynamics_type, const state_type_tensor& states, const input_type_tensor& inputs)
{

	adouble h=time_step;

	state_type_tensor k1,k2,k3,k4;

	k1=dynamics_fun(states,inputs);

	k2=dynamics_fun(states+k1*h/2,inputs);
	k3=dynamics_fun(states+k2*h/2,inputs);
	k4=dynamics_fun(states+k3*h,inputs);
//	return k4;
//	intermediate=1/6*h*(k1+2*k2+2*k3+k4);

	return (k1+2*k2+2*k3+k4)*h*1/6+states;
//	return k1*h+states;
};

template<size_t state_size,size_t input_size>
typename dynamics<state_size,input_size>::tensor_type
dynamics<state_size,input_size>::evaluate_tensor(const state_type &states,const input_type &inputs)
{
//	int input_size=dynamics::input_size;
//	int state_size=dynamics::state_size;
	Eigen::Matrix<double,state_size+input_size,1> concat(state_size + input_size);
	concat<<states,inputs;


	tensor_eval(dynamics::tag, state_size, num_in_vars, 1, num_in_vars, &concat[0], tensor_, S);
//	Eigen::Matrix<double,m,28> tensor=
//			Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic>::Zero(m, 28);
//	tensor=Eigen::Map<Eigen::Matrix<double,m,28,Eigen::RowMajor> > (*tensor_);
//	Eigen::MatrixXd tensor(state_size,tensor_dim);
//	for (int i=0;i<state_size;++i)
//	{
//		std::cout<<tensor_[i][1]<<std::endl;
//	}

//	values=Eigen::Map<Eigen::MatrixXd, 0, Eigen::InnerStride<> >(*tensor_, state_size
//			, tensor_dim, Eigen::InnerStride<>(1));

	for (int i=0;i<state_size;++i)
	{
//		std::cout<<tensor_[i][0]<<std::endl;
		for (int j=0;j<state_size+input_size+1;++j)
		{
			values(i,j)=tensor_[i][j];
//			std::cout<<i<<j<<std::endl;
		}
	}



	return values;

};


template<size_t state_size,size_t input_size>
typename dynamics<state_size,input_size>::state_type
dynamics<state_size,input_size>::eval(const state_type &states,const input_type &inputs)
{
	return values.block(0,0,state_size,1);
};

template<size_t state_size,size_t input_size>
typename dynamics<state_size,input_size>::mat_type1
dynamics<state_size,input_size>::_x(const state_type &states,const input_type &inputs)
{
	return values.block(0, 1, state_size, state_size);
};

template<size_t state_size,size_t input_size>
typename dynamics<state_size,input_size>::mat_type4
dynamics<state_size,input_size>::_u(const state_type &states,const input_type &inputs)
{
	return values.block(0, state_size+1, state_size, input_size);
}






#endif /* DYNAMICS_H_ */
