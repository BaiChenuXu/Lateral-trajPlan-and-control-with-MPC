#pragma once
#include"inputPara\Parameters.hpp"
#include<Eigen/Dense>

//This class define the vehicle dynamic model.
//In detail:
//1) base on linear and 2 freedom degrees bicycle model:
//first - get the discret state space equation matrixs A and B (x_k+1 = A*x_k + B*u);
//second - get the discret state space equation matrixs A_baseErr and B_baseErr,
//where the state are the control errors (err_k+1 = A_baseErr*err_k + B_baseErr*u);
//the B matrix is equa to the B_baseErr.
//2) When the control input u(front wheel steer angle) is stationary, the vehicle
//yawrate (radius of motion cycle) and the front wheel steer angle has a propotion
//relationship. According to this proporion, get vehicle pose under the control input u.
//3) according to the state space equation from 1), update the vehicle state x_k.
//4) according to the relationship described in 2), convert the objective yawrate
//(that is from the trajectory kappar) to the objective front wheel steer angle
//which is used by the feedforward control.

class VehDyn
{
public:
	VehDyn();

	void vehStateEqua();

	void vehPoseCalc(float steerAng);

	void vehStateCalc(float steerAng);

	float objSteerAngCalc(float yawRate);

	~VehDyn();

	const vehPara m_vehPara_obj_vd;
	vehState m_vehState_obj_vd;
	const ctlSysPara m_ctlSysPara_obj_vd;

    Eigen::Matrix4f m_A_matrix_baseErr;
	Eigen::Matrix4f m_A_matrix;
	Eigen::Vector4f m_B_matrix;

};