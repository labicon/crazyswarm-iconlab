/*
 * dynamics_structure.cpp
 *
 *  Created on: Jan 20, 2021
 *      Author: talhakavuncu
 */



#include <iostream>
#include <unsupported/Eigen/AdolcForward>
#include <adolc/adolc.h>
#include <Eigen/Dense>
#include "dynamics_structure.h"
namespace Unicycle_Dynamics
{
	unicycle_state_tensor dynamics(const unicycle_state_tensor & x,const unicycle_input_tensor & u)
	{
		unicycle_state_tensor x_dot;

	//	std::cout<<"no issues"<<std::endl;
	//	std::cout<<x.rows()<<std::endl;
		adouble px,py,theta,v,omega,a;
	//	px=x[0];
	//	py=x[1];
	//
	//	theta=x[2];
	//
	//	v=x[3];
	//
	//	omega=u[0];
	//	a=u[1];
	//
	//	x_dot[0]=v*cos(theta);
	//	x_dot[1]=v*sin(theta);
	//	x_dot[2]=omega;
	//	x_dot[3]=a;

	//	px=x[0];
	//	py=x[1];
	//
	//	theta=x[2];
	//
	//	v=x[3];
	//
	//	omega=u[0];
	//	a=u[1];

		x_dot[0]=x[3]*cos(x[2]);
		x_dot[1]=x[3]*sin(x[2]);
		x_dot[2]=u[0];
		x_dot[3]=u[1];

		return x_dot;

	};
	unicycle2_state_tensor dynamics_2(const unicycle2_state_tensor & x,const unicycle2_input_tensor & u)
	{
		unicycle2_state_tensor x_dot;

		for(int i=0;i<2;++i)
			x_dot.segment(4*i,4)=dynamics(x.segment(4*i,4),u.segment(2*i,2));

		return x_dot;
	};
	unicycle4_state_tensor dynamics_4(const unicycle4_state_tensor & x,const unicycle4_input_tensor & u)
	{
		unicycle4_state_tensor x_dot;

		for(int i=0;i<4;++i)
			x_dot.segment(4*i,4)=dynamics(x.segment(4*i,4),u.segment(2*i,2));

		return x_dot;
	};

	unicycle3_state_tensor dynamics_3(const unicycle3_state_tensor & x,const unicycle3_input_tensor & u)
	{
		unicycle3_state_tensor x_dot;
		for(int i=0;i<3;++i)
			x_dot.segment(4*i,4)=dynamics(x.segment(4*i,4),u.segment(2*i,2));
//	//	std::cout<<"no issues"<<std::endl;
//	//	std::cout<<x.rows()<<std::endl;
//		adouble px1,py1,theta1,v1,omega1,a1;
//		adouble px2,py2,theta2,v2,omega2,a2;
//		adouble px3,py3,theta3,v3,omega3,a3;
//		px1=x[0];
//		py1=x[1];
//		theta1=x[2];
//		v1=x[3];
//		omega1=u[0];
//		a1=u[1];
//
//		px2=x[4];
//		py2=x[5];
//		theta2=x[6];
//		v2=x[7];
//		omega2=u[2];
//		a2=u[3];
//
//
//		px3=x[8];
//		py3=x[9];
//		theta3=x[10];
//		v3=x[11];
//		omega3=u[4];
//		a3=u[5];
//
//		x_dot[0]=v1*cos(theta1);
//		x_dot[1]=v1*sin(theta1);
//		x_dot[2]=omega1;
//		x_dot[3]=a1;
//
//		x_dot[4]=v2*cos(theta2);
//		x_dot[5]=v2*sin(theta2);
//		x_dot[6]=omega2;
//		x_dot[7]=a2;
//
//		x_dot[8]=v3*cos(theta3);
//		x_dot[9]=v3*sin(theta3);
//		x_dot[10]=omega3;
//		x_dot[11]=a3;

		return x_dot;

	};

	unicycle6_state_tensor dynamics_6(const unicycle6_state_tensor & x,const unicycle6_input_tensor & u)
	{
		unicycle6_state_tensor x_dot;
		for(int i=0;i<6;++i)
			x_dot.segment(4*i,4)=dynamics(x.segment(4*i,4),u.segment(2*i,2));

		return x_dot;
	}
}

namespace Drone_Dynamics{

drone_state_tensor dynamics(const drone_state_tensor & X,const drone_input_tensor & u)
{
	drone_state_tensor X_dot;
	adouble x,y,z,phi,theta,psi,x_d,y_d,z_d,phi_d,theta_d,psi_d,w1,w2,w3,w4,thrust;
	x=X[0];
	y=X[1];
	z=X[2];
	phi=X[3];
	theta=X[4];
	psi=X[5];
	x_d=X[6];
	y_d=X[7];
	z_d=X[8];
	phi_d=X[9];
	theta_d=X[10];
	psi_d=X[11];

	w1=u[0];
	w2=u[1];
	w3=u[2];
	w4=u[3];

	Eigen::Matrix<adouble,3,3> Rsb,mat,mat_inv,I,I_inv;

	Rsb<<cos(theta)*cos(psi), cos(theta)*sin(psi), -sin(theta),
		-cos(phi)*sin(psi)+sin(theta)*cos(psi)*sin(phi), cos(psi)*cos(phi)+sin(theta)*sin(psi)*sin(phi), sin(phi)*cos(theta),
		sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi), -sin(phi)*cos(psi)+cos(phi)*sin(theta)*sin(psi), cos(theta)*cos(phi);

	mat<<1,0,-sin(theta),
		 0,cos(phi),sin(phi)*cos(theta),
		 0,-sin(phi),cos(theta)*cos(phi);

	mat_inv<<1,sin(phi)*tan(theta),cos(phi)*tan(theta),
			0,cos(phi),-sin(phi),
			0,sin(phi)/cos(theta),cos(phi)/cos(theta);


	I<<I_xx,0,0,
		0,I_yy,0,
		0,0,I_zz;

	I_inv<<1/I_xx,0,0,
		0,1/I_yy,0,
		0,0,1/I_zz;





	thrust=C_T*(w1*w1 + w2*w2 + w3*w3 + w4*w4);

	Eigen::Matrix<adouble,3,1> angle_dots,Omega_in,v_dots,gravity_term,thrust_term,pos_dots,Omega_b,L_b,Moments,angle_ddots,temp;
	Eigen::Matrix<adouble,3,3> skew1,skew2;
	pos_dots<<x_d,y_d,z_d;
	gravity_term<<0,0,g;
	thrust_term<<0,0,thrust/mass;
	angle_dots<<phi_d,theta_d,psi_d;
	Omega_in=Rsb*(mat*angle_dots);
//	temp=Omega_in.cross(pos_dots);
	skew1<<0,-Omega_in[2],Omega_in[1],
			Omega_in[2],0,-Omega_in[0],
			-Omega_in[1],Omega_in[0],0;
	v_dots=Rsb*thrust_term-gravity_term-skew1*pos_dots;//Omega_in.cross(pos_dots);

	X_dot[0]=x_d;
	X_dot[1]=y_d;
	X_dot[2]=z_d;
	X_dot[3]=phi_d;
	X_dot[4]=theta_d;
	X_dot[5]=psi_d;
	X_dot[6]=v_dots[0];
	X_dot[7]=v_dots[1];
	X_dot[8]=v_dots[2];

	Omega_b=mat*angle_dots;

	L_b=I*Omega_b;

	Moments<<d*C_T/sqrt(2)*(-w1*w1-w2*w2+w3*w3+w4*w4),
			d*C_T/sqrt(2)*(-w1*w1+w2*w2+w3*w3-w4*w4),
			C_D*(-w1*w1+w2*w2-w3*w3+w4*w4);

	skew2<<0,-Omega_b[2],Omega_b[1],
			Omega_b[2],0,-Omega_b[0],
			-Omega_b[1],Omega_b[0],0;

	angle_ddots=mat_inv*(I_inv*(Moments-skew2*L_b));//mat_inv*(I_inv*(Moments-Omega_b.cross(L_b))); //Euler's eqn.

	X_dot[9]=angle_ddots[0];
	X_dot[10]=angle_ddots[1];
	X_dot[11]=angle_ddots[2];

	return X_dot;

};

drone2_state_tensor dynamics_2(const drone2_state_tensor & X,const drone2_input_tensor & u)
{
	drone2_state_tensor X_dot;
	drone_state_tensor X_dot1,X_dot2;
	X_dot1=dynamics(X.segment(0, 12),u.segment(0,4));
	X_dot2=dynamics(X.segment(12, 12),u.segment(4,4));
	X_dot<<X_dot1,X_dot2;
	return X_dot;

//	drone2_state_tensor X_dot;
//	adouble x1,y1,z1,phi1,theta1,psi1,x_d1,y_d1,z_d1,phi_d1,theta_d1,psi_d1,w11,w21,w31,w41,thrust1;
//	adouble x2,y2,z2,phi2,theta2,psi2,x_d2,y_d2,z_d2,phi_d2,theta_d2,psi_d2,w12,w22,w32,w42,thrust2;
//	x1=X[0];
//	y1=X[1];
//	z1=X[2];
//	phi1=X[3];
//	theta1=X[4];
//	psi1=X[5];
//	x_d1=X[6];
//	y_d1=X[7];
//	z_d1=X[8];
//	phi_d1=X[9];
//	theta_d1=X[10];
//	psi_d1=X[11];
//
//	w11=u[0];
//	w21=u[1];
//	w31=u[2];
//	w41=u[3];
//
//	x2=X[12];
//	y2=X[13];
//	z2=X[14];
//	phi2=X[15];
//	theta2=X[16];
//	psi2=X[17];
//	x_d2=X[18];
//	y_d2=X[19];
//	z_d2=X[20];
//	phi_d2=X[21];
//	theta_d2=X[22];
//	psi_d2=X[23];
//
//	w12=u[4];
//	w22=u[5];
//	w32=u[6];
//	w42=u[7];
//
//	Eigen::Matrix<adouble,3,3> Rsb1,mat1,mat_inv1,I,I_inv,
//	Rsb2,mat2,mat_inv2;
//
//	Rsb1<<cos(theta1)*cos(psi1), cos(theta1)*sin(psi1), -sin(theta1),
//		-cos(phi1)*sin(psi1)+sin(theta1)*cos(psi1)*sin(phi1), cos(psi1)*cos(phi1)+sin(theta1)*sin(psi1)*sin(phi1), sin(phi1)*cos(theta1),
//		sin(phi1)*sin(psi1)+cos(phi1)*sin(theta1)*cos(psi1), -sin(phi1)*cos(psi1)+cos(phi1)*sin(theta1)*sin(psi1), cos(theta1)*cos(phi1);
//
//	mat1<<1,0,-sin(theta1),
//		 0,cos(phi1),sin(phi1)*cos(theta1),
//		 0,-sin(phi1),cos(theta1)*cos(phi1);
//
//	mat_inv1<<1,sin(phi1)*tan(theta1),cos(phi1)*tan(theta1),
//			0,cos(phi1),-sin(phi1),
//			0,sin(phi1)/cos(theta1),cos(phi1)/cos(theta1);
//
//
//
//	Rsb2<<cos(theta2)*cos(psi2), cos(theta2)*sin(psi2), -sin(theta2),
//		-cos(phi2)*sin(psi2)+sin(theta2)*cos(psi2)*sin(phi2), cos(psi2)*cos(phi2)+sin(theta2)*sin(psi2)*sin(phi2), sin(phi2)*cos(theta2),
//		sin(phi2)*sin(psi2)+cos(phi2)*sin(theta2)*cos(psi2), -sin(phi2)*cos(psi2)+cos(phi2)*sin(theta2)*sin(psi2), cos(theta2)*cos(phi2);
//
//	mat2<<1,0,-sin(theta2),
//		 0,cos(phi2),sin(phi2)*cos(theta2),
//		 0,-sin(phi2),cos(theta2)*cos(phi2);
//
//	mat_inv2<<1,sin(phi2)*tan(theta2),cos(phi2)*tan(theta2),
//			0,cos(phi2),-sin(phi2),
//			0,sin(phi2)/cos(theta2),cos(phi2)/cos(theta2);
//
//
//	I<<I_xx,0,0,
//		0,I_yy,0,
//		0,0,I_zz;
//
//	I_inv<<1/I_xx,0,0,
//		0,1/I_yy,0,
//		0,0,1/I_zz;
//
//
//
//
//
//	thrust1=C_T*(w11*w11 + w21*w21 + w31*w31 + w41*w41);
//	thrust2=C_T*(w12*w12 + w22*w22 + w32*w32 + w42*w42);
//
//	Eigen::Matrix<adouble,3,1> angle_dots1,Omega_in1,v_dots1,gravity_term1,thrust_term1,pos_dots1,Omega_b1,L_b1,Moments1,angle_ddots1,temp1;
//	Eigen::Matrix<adouble,3,1> angle_dots2,Omega_in2,v_dots2,gravity_term2,thrust_term2,pos_dots2,Omega_b2,L_b2,Moments2,angle_ddots2,temp2;
//	Eigen::Matrix<adouble,3,3> skew11,skew21,skew12,skew22;
//	pos_dots1<<x_d1,y_d1,z_d1;
//	gravity_term1<<0,0,g;
//	thrust_term1<<0,0,thrust1/mass;
//	angle_dots1<<phi_d1,theta_d1,psi_d1;
//	Omega_in1=Rsb1*(mat1*angle_dots1);
////	temp=Omega_in.cross(pos_dots);
//	skew11<<0,-Omega_in1[2],Omega_in1[1],
//			Omega_in1[2],0,-Omega_in1[0],
//			-Omega_in1[1],Omega_in1[0],0;
//	v_dots1=Rsb1*thrust_term1-gravity_term1-skew11*pos_dots1;//Omega_in.cross(pos_dots);
//
//	pos_dots2<<x_d2,y_d2,z_d2;
//	gravity_term2<<0,0,g;
//	thrust_term2<<0,0,thrust2/mass;
//	angle_dots2<<phi_d2,theta_d2,psi_d2;
//	Omega_in2=Rsb2*(mat2*angle_dots2);
////	temp=Omega_in.cross(pos_dots);
//	skew12<<0,-Omega_in2[2],Omega_in2[1],
//			Omega_in2[2],0,-Omega_in2[0],
//			-Omega_in2[1],Omega_in2[0],0;
//	v_dots2=Rsb2*thrust_term2-gravity_term2-skew12*pos_dots2;//Omega_in.cross(pos_dots);
//
//
//
//
//
//	X_dot[0]=x_d1;
//	X_dot[1]=y_d1;
//	X_dot[2]=z_d1;
//	X_dot[3]=phi_d1;
//	X_dot[4]=theta_d1;
//	X_dot[5]=psi_d1;
//	X_dot[6]=v_dots1[0];
//	X_dot[7]=v_dots1[1];
//	X_dot[8]=v_dots1[2];
//
//	X_dot[12]=x_d2;
//	X_dot[13]=y_d2;
//	X_dot[14]=z_d2;
//	X_dot[15]=phi_d2;
//	X_dot[16]=theta_d2;
//	X_dot[17]=psi_d2;
//	X_dot[18]=v_dots2[0];
//	X_dot[19]=v_dots2[1];
//	X_dot[20]=v_dots2[2];
//
//	Omega_b1=mat1*angle_dots1;
//	Omega_b2=mat2*angle_dots2;
//
//	L_b1=I*Omega_b1;
//	L_b2=I*Omega_b2;
//
//	Moments1<<d*C_T/sqrt(2)*(-w11*w11-w21*w21+w31*w31+w41*w41),
//			d*C_T/sqrt(2)*(-w11*w11+w21*w21+w31*w31-w41*w41),
//			C_D*(-w11*w11+w21*w21-w31*w31+w41*w41);
//
//	Moments2<<d*C_T/sqrt(2)*(-w12*w12-w22*w22+w32*w32+w42*w42),
//			d*C_T/sqrt(2)*(-w12*w12+w22*w22+w32*w32-w42*w42),
//			C_D*(-w12*w12+w22*w22-w32*w32+w42*w42);
//
//	skew21<<0,-Omega_b1[2],Omega_b1[1],
//			Omega_b1[2],0,-Omega_b1[0],
//			-Omega_b1[1],Omega_b1[0],0;
//
//	skew22<<0,-Omega_b2[2],Omega_b2[1],
//			Omega_b2[2],0,-Omega_b2[0],
//			-Omega_b2[1],Omega_b2[0],0;
//
//
//	angle_ddots1=mat_inv1*(I_inv*(Moments1-skew21*L_b1));//mat_inv*(I_inv*(Moments-Omega_b.cross(L_b))); //Euler's eqn.
//
//
//	angle_ddots2=mat_inv2*(I_inv*(Moments2-skew22*L_b2));
//
//
//	X_dot[9]=angle_ddots1[0];
//	X_dot[10]=angle_ddots1[1];
//	X_dot[11]=angle_ddots1[2];
//
//	X_dot[21]=angle_ddots2[0];
//	X_dot[22]=angle_ddots2[1];
//	X_dot[23]=angle_ddots2[2];
//
//	return X_dot;





}

}

namespace Single_Integrator_3D
{
	state_tensor dynamics(const state_tensor & x, const input_tensor & u)
	{
		return u;
	};

	state_tensor2 dynamics2(const state_tensor2 & x, const input_tensor2 & u)
	{
		return u;
	}
}
namespace Double_Integrator_3D
{
	state_tensor dynamics(const state_tensor & x, const input_tensor & u)
	{
		state_tensor x_dot;
		x_dot[0]=x[3];
		x_dot[1]=x[4];
		x_dot[2]=x[5];
		x_dot[3]=u[0]/mb;
		x_dot[4]=u[1]/mb;
		x_dot[5]=u[2]/mb;

		return x_dot;
	};
//
//	state_tensor2 dynamics2(const state_tensor2 & x, const input_tensor2 & u)
//	{
//		return u;
//	}
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
	drone_state_tensor dynamics(const drone_state_tensor & X,const drone_input_tensor & input)
	{
		adouble x,y,z,phi,theta,psi,u,v,w,p,q,r,x_d,y_d,z_d,phi_d,theta_d,psi_d;
		drone_state_tensor x_dot;
		u=input[0];
		v=input[1];
		w=input[2];
		p=input[3];
		q=input[4];
		r=input[5];
		Eigen::Matrix<adouble,3,3> Rsb,mat,mat_inv;
		Eigen::Matrix<adouble,3,1> v_b,w_b,v_i,euler_d;

		v_b<<u,v,w;
		w_b<<p,q,r;

		Rsb<<cos(theta)*cos(psi), cos(theta)*sin(psi), -sin(theta),
				-cos(phi)*sin(psi)+sin(theta)*cos(psi)*sin(phi), cos(psi)*cos(phi)+sin(theta)*sin(psi)*sin(phi), sin(phi)*cos(theta),
				sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi), -sin(phi)*cos(psi)+cos(phi)*sin(theta)*sin(psi), cos(theta)*cos(phi);

//		mat<<1,0,-sin(theta),
//			 0,cos(phi),sin(phi)*cos(theta),
//			 0,-sin(phi),cos(theta)*cos(phi);

		mat_inv<<1,sin(phi)*tan(theta),cos(phi)*tan(theta),
				0,cos(phi),-sin(phi),
				0,sin(phi)/cos(theta),cos(phi)/cos(theta);

		v_i=Rsb*v_b;
		euler_d=mat_inv*w_b;

		x_d=v_i[0];
		y_d=v_i[1];
		z_d=v_i[2];

		phi_d=euler_d[0];
		theta_d=euler_d[1];
		psi_d=euler_d[2];

//		x_d=w*(sin(phi)*sin(psi)+cos(phi)*cos(psi)*sin(theta))-
//				v*(cos(phi)*sin(psi)-cos(psi)*sin(phi)*sin(theta))+
//				u*(cos(psi)*cos(theta));
//
//		y_d=v*(cos(phi)*cos(psi)+sin(phi)*sin(psi)*sin(theta))-
//				w*(cos(psi)*sin(phi)-cos(phi)*sin(psi)*sin(theta))+
//				u*(cos(theta)*sin(psi));
//		z_d=w*(cos(phi)*cos(theta))-u*(sin(theta))+v*(cos(theta)*sin(phi));
//
//		phi_d=p+r*(cos(phi)*tan(theta))+q*(sin(phi)*tan(theta));
//		theta_d=q*cos(phi)-r*sin(phi);
//		psi_d=r*cos(phi)/cos(theta)+q*sin(phi)/cos(theta);
//
		x_dot<<x_d,y_d,z_d,phi_d,theta_d,psi_d;

		return x_dot;
	}

	drone2_state_tensor dynamics_2(const drone2_state_tensor & x,const drone2_input_tensor & u)
	{
		drone2_state_tensor X_dot;
		drone_state_tensor X_dot1,X_dot2;
		X_dot1=dynamics(x.segment(0, 6),u.segment(0,6));
		X_dot2=dynamics(x.segment(6, 6),u.segment(6,6));
		X_dot<<X_dot1,X_dot2;
		return X_dot;
	}

}
