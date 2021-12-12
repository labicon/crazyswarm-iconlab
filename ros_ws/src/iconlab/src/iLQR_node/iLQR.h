/*
 * iLQR.h
 *
 *  Created on: Jan 21, 2021
 *      Author: talhakavuncu
 */

#ifndef ILQR_H_
#define ILQR_H_
#include "cost.h"
#include "dynamics.h"
#include <array>
#include <vector>
#include <algorithm>
#include <math.h>
#include <Eigen/Dense>
#include <numeric>
#include <cstdlib>
#include <unsupported/Eigen/AdolcForward>
#include <adolc/adolc.h>
#include <omp.h>
//typedef Eigen::Matrix<Eigen::VectorXd,Eigen::Dynamic,2> trajectory_type;
//typedef Eigen::MatrixXd input_trajectory;
//typedef Eigen::MatrixXd state_trajectory;
//std::vector<int> a;

//typedef template<size_t state_size,size_t input_size> temp;
template<size_t st_sz,size_t ac_sz,size_t hrzn>
class iLQR
{
//	typedef std::vector<std::vector<std::vector<vec_type> > > soln;
//	typedef std::vector<std::array<vec_type,2> > trajectory;


//	typedef vec_type[2];
public:

	typedef Eigen::Matrix<double,st_sz,1> state_type;
	typedef Eigen::Matrix<double,ac_sz,1> input_type;
	typedef std::array<input_type,hrzn> input_trajectory;
	typedef std::array<state_type,hrzn+1> state_trajectory;
	typedef std::pair<state_trajectory,input_trajectory> state_input_trajectory;

//	typedef std::array<Eigen::Matrix<double,ac_sz,1>,1> input_tr_MPC;
//	typedef std::array<Eigen::Matrix<double,st_sz,1>,2> state_tr_MPC;
//	typedef std::pair<state_tr_MPC,input_tr_MPC> state_input_tr_MPC;
	iLQR(cost<st_sz,ac_sz> running_cost,cost<st_sz,ac_sz> terminal_cost,
			dynamics<st_sz,ac_sz> sys_dynamics)
	:running_cost(running_cost),terminal_cost(terminal_cost),sys_dynamics(sys_dynamics)
	{
		this->horizon=hrzn;
		this->state_size=st_sz;
		this->action_size=ac_sz;
		std::cout<<"this is constructor"<<std::endl;
		mu=1.0;
		mu_min=1e-6;
		mu_max=1e300;
		delta_0=2.0;
		delta=delta_0;
		tol=1e-8;
		iteration_count=0;

//		std::cout<<L[0]<<std::endl;
		alphas={0,1,2,3,4,5,6,7,8,9};
		std::for_each(alphas.begin(), alphas.end(), [](double &a) { a=pow(1.1,-a*a);});
//		for(double i :alphas)std::cout<<i<<" ";
//		std::cout<<alphas[4];
//		for(double i: )
	};

	state_input_trajectory solve_open_loop(const state_type& X0,state_type& Xgoal,int n_iterations,
			const input_trajectory& u_init,int horizon);
	void set_MPC(const input_trajectory& u_init)
	{
		us_MPC=u_init;
	}
	state_input_trajectory run_MPC(const state_type& X0,
			 state_type& Xgoal,int n_iterations, int subsequent_iterations, int execution_steps);

//	void solve_MPC(vec_type init,vec_type goal,int horizon);
private:
	cost<st_sz,ac_sz> running_cost;
	cost<st_sz,ac_sz> terminal_cost;
	dynamics<st_sz,ac_sz> sys_dynamics;
	size_t state_size;
	size_t action_size;
	size_t horizon;
	double mu;
	double mu_min;
	double mu_max;
	double delta_0;
	double delta;
	double tol;
	int iteration_count;
	std::vector<double> alphas;
//	alphas.
//	std::vector<int> vect1 { 10, 20, 30 };
//	std::vector<int> v = {3, 1, 9, 4};
//	std::for_each(alphas.begin(), alphas.end(), [](int) { /* do something here*/ });
//	const Eigen::VectorXd& X0;
//	const Eigen::VectorXd& Xgoal;
//	std::vector<Eigen::VectorXd> ks;
	std::array<Eigen::Matrix<double,ac_sz,1>,hrzn> ks;
	std::array<Eigen::Matrix<double,ac_sz,st_sz>,hrzn> Ks;
//	std::vector<Eigen::MatrixXd> Ks;
	state_trajectory xs,xs_new;
	input_trajectory us,us_new,us_MPC;
	state_input_trajectory new_tr;
	state_trajectory L_x;
	input_trajectory L_u;
	std::array<double,hrzn+1> L;
	std::array<Eigen::Matrix<double,st_sz,st_sz>,hrzn+1> L_xx;
	std::array<Eigen::Matrix<double,ac_sz,st_sz>,hrzn> L_ux;
	std::array<Eigen::Matrix<double,ac_sz,ac_sz>,hrzn> L_uu;
	std::array<Eigen::Matrix<double,st_sz,st_sz>,hrzn> F_x;
	std::array<Eigen::Matrix<double,st_sz,ac_sz>,hrzn> F_u;
	Eigen::Matrix<double,st_sz,1> Q_x;
	Eigen::Matrix<double,st_sz,1> V_x;
	Eigen::Matrix<double,ac_sz,1> Q_u;
	Eigen::Matrix<double,st_sz,st_sz> Q_xx;
	Eigen::Matrix<double,st_sz,st_sz> V_xx;
	Eigen::Matrix<double,ac_sz,st_sz> Q_ux;
	Eigen::Matrix<double,ac_sz,ac_sz> Q_uu;



//	bool is_converged();
	void forward_rollout(const state_type& X0,const input_trajectory&, state_type& X_goal);
	void backward_pass();
	void calculate_total_cost();
//	state_input_trajectory control(const state_trajectory& xs,const input_trajectory& us,double alpha);
	double trajectory_cost( state_type& X_goal);
	void expand_Qfun(int i,Eigen::Matrix<double,st_sz,1>& V_x,const Eigen::Matrix<double,st_sz,st_sz>& V_xx);
//	void trajectory_cost();
	void control(double);

};



template<size_t st_sz,size_t ac_sz,size_t hrzn>
typename iLQR<st_sz,ac_sz,hrzn>::state_input_trajectory iLQR<st_sz,ac_sz,hrzn>::solve_open_loop
(const state_type& X0,state_type& X_goal,int n_iterations,
		const input_trajectory& u_init,int horizon)
{
//	while(is_converged()==false)
//	{
////		forward_rollout(X0,u_init);
////		backward_pass();
//	}
//	std::cout<<"this is solve_open_loop"<<std::endl;
	bool changed=true;
	bool converged=false;
	bool accepted;
	us=u_init;
//	std::cout<<us[19]<<std::endl;
	double J_opt,J_new;

	for(int iteration=0;iteration<n_iterations;++iteration)
	{
//		std::cout<<"entered loop"<<std::endl;
//		std::cout<<Q_x.transpose()<<std::endl;
		accepted=false;

		if (changed)
		{
//			std::cout<<"changed"<<std::endl;
			forward_rollout(X0,us,X_goal);

			J_opt = std::accumulate(L.begin(), L.end(), 0.0);
//			L.resize(horizon);
//			for(int i=0;i<L.size();++i)
//			{std::cout<<L[i]<<std::endl;};
//			if(isnan(J_opt))
//			std::cout<<std::accumulate(L.begin(), L.end(), 0.0)<<std::endl;
			changed=false;
		}

		backward_pass();

//		try
//		{
//			this->backward_pass();
//		}
//		catch()
//		{
//
//		}
//		backward_pass(F_x,);
		for(auto i=0;i<alphas.size();++i) //?
		{

//			std::cout<<alphas.size()<<std::endl;
				control(alphas[i]);
//				std::cout<<"asdfsa"<<std::endl;
				J_new=trajectory_cost(X_goal);
//				std::cout<<J_new<<std::endl;

				if(J_new<J_opt)
				{
					if(abs(J_opt-J_new)/J_opt<tol)
					{
						converged=true;
					}
//					std::cout<<"asdfsa"<<std::endl;
					J_opt=J_new;
					xs=xs_new;
					us=us_new;
					changed=true;
//					double foo=std::min(1.0,delta);
					delta=std::min(1.0,delta)/delta_0;
					mu*=delta;
					if(mu<=mu_min)
					{
						mu=0.0;
					}
					accepted=true;

					break;
				}


		}
		if(not accepted)
		{

			delta=std::max(1.0,delta)*delta_0;
			mu=std::max(mu_min,mu*delta);
			if(mu_max!=0 and mu>=mu_max)
			{
				std::cout<<"exceeded max regularization term"<<std::endl;
			}
		}
		std::cout<<iteration<<std::endl;
		//implement on_iteration;
		if(converged)
		{

			break;
		}


	}


	return std::make_pair(xs, us);

}
template<size_t st_sz,size_t ac_sz,size_t hrzn>
void iLQR<st_sz,ac_sz,hrzn>::forward_rollout(const state_type& X0,const input_trajectory& us, state_type& X_goal)
{
//	Eigen::VectorXd x,u;
	state_type x;
	input_type u;
//	state_trajectory x
	xs[0]=X0;
//	std::cout<<"xs0 "<<xs[0]<<std::endl;
	for(int i=0;i<horizon;++i)
	{
		x=xs[i];

		u=us[i];
//		std::cout<<u<<std::endl;
//		std::cout<<sys_dynamics.evaluate_tensor(x,u)<<std::endl;
		sys_dynamics.evaluate_tensor(x,u);
		running_cost.evaluate_tensor(x, u,X_goal);
//		std::cout<<sys_dynamics.eval(x, u)<<std::endl;
		xs[i+1]=sys_dynamics.eval(x, u);
//		std::cout<<xs[i+1].transpose()<<std::endl;
		F_x[i]=sys_dynamics._x(x, u);
//		std::cout<<F_x[i]<<std::endl;
		F_u[i]=sys_dynamics._u(x, u);
//		std::cout<<F_u[i]<<std::endl;
		L[i]=running_cost.eval(x, u);
//		std::cout<<L[i]<<std::endl;
		L_x[i]=running_cost._x(x, u);
//		std::cout<<L_x[i]<<std::endl;
		L_u[i]=running_cost._u(x, u);
//		std::cout<<L_u[i]<<std::endl;
		L_xx[i]=running_cost._xx(x, u);
//		std::cout<<L_xx[i]<<std::endl;
		L_ux[i]=running_cost._ux(x, u);
//		std::cout<<L_ux[i]<<std::endl;
		L_uu[i]=running_cost._uu(x, u);
//		std::cout<<L_uu[i]<<std::endl;
	}
	x=xs.back();
//	std::cout<<xs.back()<<std::endl;
	terminal_cost.evaluate_tensor(x,u,X_goal);
	L.back()=terminal_cost.eval(x, u);
//	std::cout<<L.back()<<std::endl;
	L_x.back()=terminal_cost._x(x, u);
//	std::cout<<L_x.back()<<std::endl;
	L_xx.back()=terminal_cost._xx(x, u);
//	std::cout<<L_xx.back()<<std::endl;
};

template<size_t st_sz,size_t ac_sz,size_t hrzn>
void iLQR<st_sz,ac_sz,hrzn>::backward_pass()
{
//	Eigen::MatrixXd V_x,V_xx;
//	Eigen::Matrix<double,st_sz,1> V_x;
//	Eigen::Matrix<double,st_sz,st_sz> V_xx;
	V_x=L_x.back();
	V_xx=L_xx.back();
//	std::cout<<V_x<<std::endl;
//	std::cout<<V_xx<<std::endl;
	for(int i=horizon-1;i>-1;--i)
	{
		expand_Qfun(i,V_x,V_xx);
		ks[i]=-Q_uu.colPivHouseholderQr().solve(Q_u);
		Ks[i]=-Q_uu.colPivHouseholderQr().solve(Q_ux);
//		ks[i]=-Q_uu.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Q_u);
//		Ks[i]=-Q_uu.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Q_ux);
//		ks[i]=-(Q_uu.transpose() * Q_uu).ldlt().solve(Q_uu.transpose() * Q_u);
//		Ks[i]=-(Q_uu.transpose() * Q_uu).ldlt().solve(Q_uu.transpose() * Q_ux);
//		std::cout<<"ks: "<<ks[i]<<std::endl;
//		std::cout<<"Ks: "<<Ks[i]<<std::endl;
//		if (i<1)
//		{
////			std::cout<<ks[i]<<std::endl;
////			std::cout<<Ks[i]<<std::endl;
//			std::cout<<Q_uu<<std::endl;
//			std::cout<<Q_u<<std::endl;
//		}

		V_x=Q_x+Ks[i].transpose()*Q_uu*ks[i];
		V_x+=Ks[i].transpose()*Q_u+Q_ux.transpose()*ks[i];

		V_xx=Q_xx+Ks[i].transpose()*Q_uu*Ks[i];
		V_xx+=Ks[i].transpose()*Q_ux+Q_ux.transpose()*Ks[i];
		V_xx=0.5*(V_xx+V_xx.transpose());

	}
}
template<size_t st_sz,size_t ac_sz,size_t hrzn>
void iLQR<st_sz,ac_sz,hrzn>::expand_Qfun(int i,Eigen::Matrix<double,st_sz,1>& V_x,const Eigen::Matrix<double,st_sz,st_sz>& V_xx)
{

	Q_x=L_x[i]+F_x[i].transpose()*V_x;
	Q_u=L_u[i]+F_u[i].transpose()*V_x;
	Q_xx=L_xx[i]+F_x[i].transpose()*V_xx*F_x[i];
//	Eigen::MatrixXd reg=Eigen::MatrixXd::Identity(state_size, state_size);
	Eigen::Matrix<double,st_sz,st_sz> reg=Eigen::MatrixXd::Identity(state_size, state_size);
//	Eigen::Array33f a1 = Eigen::Array33f::Zero();
	reg=mu*reg;
	Q_ux=L_ux[i]+F_u[i].transpose()*(V_xx+reg)*F_x[i];
	Q_uu=L_uu[i]+F_u[i].transpose()*(V_xx+reg)*F_u[i];

}
template<size_t st_sz,size_t ac_sz,size_t hrzn>
void iLQR<st_sz,ac_sz,hrzn>::control(double alpha)
{

	xs_new[0]=xs[0];
//	Eigen::Matrix<adouble,st_sz,1> temp_st;
//	Eigen::Matrix<adouble,Eigen::Dynamic,1> temp_res;
//	Eigen::Matrix<adouble,ac_sz,1> temp_ac;
//	std::cout<<xs[0]<<std::endl;
	for(int i=0;i<horizon;++i)
	{
		us_new[i]=us[i]+alpha*ks[i]+Ks[i]*(xs_new[i]-xs[i]);
//		std::cout<<us_new[i].transpose()<<std::endl;
//		for(int j=0;i<st_sz;++j)
//		{
//			temp_st[j]=xs_new[i][j];
//			std::cout<<xs_new[i][j]<<std::endl;
//		}
//
//		for(int k=0;k<ac_sz;++k)
//			temp_ac[k]=us_new[i][k];
//
//		temp_res=sys_dynamics.dynamics_fun(temp_st,temp_ac);
//		for(int j=0;j<st_sz;++j)
//		{
//			std::cout<<temp_res[j]<<std::endl;
//			xs_new[i+1][j]=(temp_res[j]).getValue();
//		}

		sys_dynamics.evaluate_tensor(xs_new[i], us_new[i]);
		xs_new[i+1]=sys_dynamics.eval(xs_new[i], us_new[i]);

	}
}
template<size_t st_sz,size_t ac_sz,size_t hrzn>
double iLQR<st_sz,ac_sz,hrzn>::trajectory_cost(state_type& X_goal)
{
	double cost=0;

	for(int i=0;i<horizon;++i)
	{
		running_cost.evaluate_tensor(xs_new[i], us_new[i],X_goal);
		cost+=running_cost.eval(xs_new[i], us_new[i]);
////		std::cout<<running_cost.eval(xs[i], us[i])<<std::endl;
//		if(isnan(cost))
//			std::cout<<xs_new[i].transpose()<<" "<<us_new[i].transpose()<<std::endl;
	}
	terminal_cost.evaluate_tensor(xs_new.back(), us_new.back(),X_goal);
//	std::cout<<cost<<std::endl;
	return cost+terminal_cost.eval(xs_new.back(), us_new.back());

};

template<size_t st_sz,size_t ac_sz,size_t hrzn>
typename iLQR<st_sz,ac_sz,hrzn>::state_input_trajectory iLQR<st_sz,ac_sz,hrzn>::run_MPC(const state_type& X0,
		state_type& X_goal,int n_iterations, int subsequent_iterations, int execution_steps)
{
	state_input_trajectory soln;
//	state_input_tr_MPC soln_MPC;
//	std::cout<<"X0: "<<X0.transpose()<<std::endl;

//	std::cout<<"X0: "<<X0.transpose()<<std::endl;
	if (iteration_count == 0)
		soln=solve_open_loop(X0, X_goal, n_iterations, us_MPC, horizon);
	else
		soln=solve_open_loop(X0, X_goal, subsequent_iterations, us_MPC, horizon);

	++iteration_count;
	us_MPC=soln.second;
//	soln_MPC.first[0]=soln.first[0];
//	soln_MPC.first[1]=soln.first[1];
//	soln_MPC.second[0]=soln.second[0];
//	for(auto i:us_MPC)
//		std::cout<<"us: "<<i.transpose()<<" ";
//	std::cout<<std::endl;
	std::rotate(us_MPC.begin(), us_MPC.begin()+execution_steps, us_MPC.end());
//	for(auto i:us_MPC)
//		std::cout<<"us: "<<i.transpose()<<" ";
//	std::cout<<std::endl;
	for (int i=0;i<execution_steps;++i)
		us_MPC.at(hrzn-1-i)=5*Eigen::Matrix<double,ac_sz,1>::Random();

	return soln;



};



#endif /* ILQR_H_ */
