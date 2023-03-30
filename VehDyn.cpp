#include"include\VehDyn.hpp"
#include<Eigen/Dense>
#include<math.h>
#include<iostream>

VehDyn::VehDyn()
{
	m_A_matrix_baseErr = Eigen::MatrixXf::Zero(4, 4);
	m_A_matrix = Eigen::MatrixXf::Zero(4, 4);
	m_B_matrix = Eigen::MatrixXf::Zero(4, 1);
}

//Regarding to the discret state space equation matrixs A and B (x_k+1 = A*x_k + B*u)
//the references are:
//1) book: "Vehicle Dynamics and Control by Rajesh Rajamani", chapter 2, page 20;
//2) book(in chinese): "汽车理论 (余志生)",chapter 5, page 146;  
//Regarding to the discret state space equation matrixs A_baseErr and B_baseErr 
//(err_k+1 = A_baseErr*err_k + B_baseErr*u);
//the amazing reference is:
//video(in chinese): https://www.bilibili.com/video/BV1GD4y1o7Vf/?spm_id_from=333.999.0.0
void VehDyn::vehStateEqua()
{
	float l_matrixElement_a22(0.0f);
	float l_matrixElement_a23(0.0f);
	float l_matrixElement_a24(0.0f);
	float l_matrixElement_a42(0.0f);
	float l_matrixElement_a43(0.0f);
	float l_matrixElement_a44(0.0f);
    float l_matrixElement_a24_1(0.0f);
	float l_matrixElement_b21(0.0f);
	float l_matrixElement_b41(0.0f);

	Eigen::MatrixXf l_matrixEye = Eigen::MatrixXf::Identity(4, 4);
	Eigen::MatrixXf l_matrixTemp = Eigen::MatrixXf::Zero(4, 4);

	l_matrixElement_a22 = (m_vehPara_obj_vd.m_stiffFWhl + m_vehPara_obj_vd.m_stiffRWhl)
		/ m_vehPara_obj_vd.m_curbWeight / m_vehState_obj_vd.m_vehSpdX;
	l_matrixElement_a23 = -(m_vehPara_obj_vd.m_stiffFWhl + m_vehPara_obj_vd.m_stiffRWhl)
		/ m_vehPara_obj_vd.m_curbWeight;
	l_matrixElement_a24 = (m_vehPara_obj_vd.m_stiffFWhl* m_vehPara_obj_vd.m_CG2fAxis
		- m_vehPara_obj_vd.m_stiffRWhl* m_vehPara_obj_vd.m_CG2rAxis)/ m_vehPara_obj_vd.m_curbWeight
		/ m_vehState_obj_vd.m_vehSpdX;
	l_matrixElement_a42 = (m_vehPara_obj_vd.m_stiffFWhl * m_vehPara_obj_vd.m_CG2fAxis
		- m_vehPara_obj_vd.m_stiffRWhl * m_vehPara_obj_vd.m_CG2rAxis)/ m_vehPara_obj_vd.m_inertiaZaxis
		/ m_vehState_obj_vd.m_vehSpdX;
	l_matrixElement_a43 = -(m_vehPara_obj_vd.m_stiffFWhl * m_vehPara_obj_vd.m_CG2fAxis
		- m_vehPara_obj_vd.m_stiffRWhl * m_vehPara_obj_vd.m_CG2rAxis) / m_vehPara_obj_vd.m_inertiaZaxis;
	l_matrixElement_a44 = (m_vehPara_obj_vd.m_stiffFWhl * m_vehPara_obj_vd.m_CG2fAxis * m_vehPara_obj_vd.m_CG2fAxis
		+ m_vehPara_obj_vd.m_stiffRWhl * m_vehPara_obj_vd.m_CG2rAxis * m_vehPara_obj_vd.m_CG2rAxis)
		/ m_vehPara_obj_vd.m_inertiaZaxis / m_vehState_obj_vd.m_vehSpdX;
	l_matrixElement_a24_1 = l_matrixElement_a24 - m_vehState_obj_vd.m_vehSpdX;

	l_matrixElement_b21 = -m_vehPara_obj_vd.m_stiffFWhl / m_vehPara_obj_vd.m_curbWeight;
	l_matrixElement_b41 = -m_vehPara_obj_vd.m_CG2fAxis * m_vehPara_obj_vd.m_stiffFWhl
		/ m_vehPara_obj_vd.m_inertiaZaxis;

	m_A_matrix_baseErr(0, 1) = m_A_matrix(0, 1) = 1.0f;
	m_A_matrix_baseErr(1, 1) = m_A_matrix(1, 1) = l_matrixElement_a22;
	m_A_matrix_baseErr(1, 2) = l_matrixElement_a23;
	m_A_matrix_baseErr(1, 3) = l_matrixElement_a24;
    m_A_matrix(1, 3) = l_matrixElement_a24_1;
	m_A_matrix_baseErr(2, 3) = m_A_matrix(2, 3) = 1.0f;
	m_A_matrix_baseErr(3, 1) = m_A_matrix(3, 1) = l_matrixElement_a42;
	m_A_matrix_baseErr(3, 2) = l_matrixElement_a43;
	m_A_matrix_baseErr(3, 3) = m_A_matrix(3, 3) = l_matrixElement_a44;

	m_B_matrix(1, 0) = l_matrixElement_b21;
	m_B_matrix(3, 0) = l_matrixElement_b41;

	m_A_matrix_baseErr = (l_matrixEye - m_A_matrix_baseErr * m_ctlSysPara_obj_vd.m_tDurPerStep / 2).inverse()
		* (l_matrixEye + m_A_matrix_baseErr * m_ctlSysPara_obj_vd.m_tDurPerStep / 2);

	m_A_matrix = (l_matrixEye - m_A_matrix * m_ctlSysPara_obj_vd.m_tDurPerStep / 2).inverse()
		* (l_matrixEye + m_A_matrix * m_ctlSysPara_obj_vd.m_tDurPerStep / 2);

	m_B_matrix = m_B_matrix * m_ctlSysPara_obj_vd.m_tDurPerStep;

}

//according to the state space equatino to update the vehicle state
void VehDyn::vehStateCalc(float steerAng)
{
	Eigen::Vector4f l_vehState_new;
	Eigen::Vector4f l_vehState_old;

	vehStateEqua();

	l_vehState_old << m_vehState_obj_vd.m_vehY_vCoor.back(), m_vehState_obj_vd.m_vehDotY_vCoor.back(),
		m_vehState_obj_vd.m_vehAng_vCoor.back(), m_vehState_obj_vd.m_vehAngSpd.back();
	l_vehState_new = m_A_matrix * l_vehState_old + m_B_matrix * steerAng;

	m_vehState_obj_vd.m_vehY_vCoor.push_back(l_vehState_new(0));
	m_vehState_obj_vd.m_vehDotY_vCoor.push_back(l_vehState_new(1));
	m_vehState_obj_vd.m_vehAng_vCoor.push_back(l_vehState_new(2));
	m_vehState_obj_vd.m_vehAngSpd.push_back(l_vehState_new(3));

}

//According to relationship of the stationary u and vehicle motion cycle radius, calculate the vehicle pose.
//The amazing reference about relationship of the stationary u and vehicle motion cycle radius is:
//book: "Single Track Models", you can get it from same package which contain this code
//But, model of the book calculate motion of center of gravity, our goal is that let
//the center of rear shaft to track the trajectory, so the wirter do some modifications.
void VehDyn::vehPoseCalc(float steerAng)
{
	float l_yawRate(0.0f);
	float l_arcLen(0.0f);
	float l_tempVariable_1(0.0f);
	float l_tempVariable_2(0.0f);
	float l_tempVariable_3(0.0f);
	float l_tempVariable_4(0.0f);
	float l_tempVariable_5(0.0f);
    float l_steerRadius(0.0f);
	float l_arcAngle(0.0f);
	float l_chordLen(0.0f);
	float l_Xdelta(0.0f);
	float l_Ydelta(0.0f);
	float l_XcoorTrans(0.0f);
	float l_YcoorTrans(0.0f);
	float l_angleTemp(0.0f);
	float l_vehPoseX_new(0.0f);
	float l_vehPoseY_new(0.0f);
	float l_vehPoseAng_new(0.0f);
	
	l_yawRate = m_vehState_obj_vd.m_vehAngSpd.back();

	if (fabs(l_yawRate) > 0.001f)
	{
		//during 1 time step, the arc length of the vehicle moton 
		l_arcLen = m_vehState_obj_vd.m_vehSpdX * m_ctlSysPara_obj_vd.m_tDurPerStep;
		
		l_tempVariable_1 = fabs(m_vehState_obj_vd.m_vehSpdX / l_yawRate);
		l_tempVariable_2 = l_tempVariable_1 / (m_vehPara_obj_vd.m_CG2fAxis * m_vehPara_obj_vd.m_curbWeight
			/ m_vehPara_obj_vd.m_whlBase / m_vehPara_obj_vd.m_stiffRWhl * m_vehState_obj_vd.m_vehSpdX 
			* m_vehState_obj_vd.m_vehSpdX);
		l_tempVariable_3 = m_vehPara_obj_vd.m_CG2rAxis / l_tempVariable_1 - l_tempVariable_2;
		l_tempVariable_4 = m_vehPara_obj_vd.m_CG2rAxis - sin(l_tempVariable_3) * l_tempVariable_1;
		l_tempVariable_5 = cos(l_tempVariable_3) * l_tempVariable_1;

		//the steer radius at the center of the rear shaft 
		l_steerRadius = sqrt(l_tempVariable_5 * l_tempVariable_5 + l_tempVariable_4 * l_tempVariable_4);
		
		//the corresponding angle of the arc length
		l_arcAngle = l_arcLen / l_steerRadius;

		//the corresponding chord length of the arc length
		l_chordLen = 2 * sin(l_arcAngle / 2) * l_steerRadius;

		//x position change at current vehicle coordinate
		l_Xdelta = l_chordLen * cos(l_arcAngle / 2);

		//y position change at current vehicle coordinate
		if (steerAng >= 0)
		{
			l_Ydelta = l_chordLen * sin(l_arcAngle / 2);
		}
		else
		{
			l_Ydelta = -l_chordLen * sin(l_arcAngle / 2);
		}

		//theta change 
		l_vehPoseAng_new = m_vehState_obj_vd.m_vehPoseAng.back() + l_yawRate * m_ctlSysPara_obj_vd.m_tDurPerStep;
	}
	else
	{
		l_Xdelta = m_vehState_obj_vd.m_vehSpdX * m_ctlSysPara_obj_vd.m_tDurPerStep;
		l_Ydelta = 0.0f;
		l_vehPoseAng_new = m_vehState_obj_vd.m_vehPoseAng.back();
	}

	
	//We want kow the vehicle position at the freeze vehicle coordinate, after vehicle position 
	//change Xdelta and Ydelta. But the Xdelta and Ydelta is in the current vehicle coordinate.
	//So, firstly we convert the origin point of the freeze vehicle coordinate to the coordinate
	//of current vehicle coordinate. At the same coordinate,the Xdelta/Ydelta and the origin point
	//of the freeze vehicle coordinate can establish relationship.
	//Secondly, we convert the Xdelta/Ydelta to the freeze vehicle coordinate.

    //first coordinate change using the direction cosine matrix
	l_XcoorTrans = -cos(m_vehState_obj_vd.m_vehPoseAng.back()) * (m_vehState_obj_vd.m_vehPoseX.back() - 0)
		- sin(m_vehState_obj_vd.m_vehPoseAng.back())*(m_vehState_obj_vd.m_vehPoseY.back() - 0);
	l_YcoorTrans = -cos(m_vehState_obj_vd.m_vehPoseAng.back()) * (m_vehState_obj_vd.m_vehPoseY.back() - 0)
		+ sin(m_vehState_obj_vd.m_vehPoseAng.back()) * (m_vehState_obj_vd.m_vehPoseX.back() - 0);
	l_angleTemp = 0 - m_vehState_obj_vd.m_vehPoseAng.back();

	//second coordinate change using the direction cosine matrix
	l_vehPoseX_new = -cos(l_angleTemp) * (l_XcoorTrans - l_Xdelta) - sin(l_angleTemp) * (l_YcoorTrans - l_Ydelta);
	l_vehPoseY_new = -cos(l_angleTemp) * (l_YcoorTrans - l_Ydelta) + sin(l_angleTemp) * (l_XcoorTrans - l_Xdelta);

	m_vehState_obj_vd.m_vehPoseX.push_back(l_vehPoseX_new);
	m_vehState_obj_vd.m_vehPoseY.push_back(l_vehPoseY_new);
	m_vehState_obj_vd.m_vehPoseAng.push_back(l_vehPoseAng_new);
}

//convert the objective yawrate to the objective front wheel steer angle
float VehDyn::objSteerAngCalc(float yawRate)
{
	float l_EGpara(0.0f);
	float l_whlAng(0.0f);

	l_EGpara = m_vehPara_obj_vd.m_curbWeight / m_vehPara_obj_vd.m_whlBase / m_vehPara_obj_vd.m_stiffRWhl
		/ m_vehPara_obj_vd.m_stiffFWhl * (m_vehPara_obj_vd.m_stiffRWhl * m_vehPara_obj_vd.m_CG2rAxis
			- m_vehPara_obj_vd.m_stiffFWhl * m_vehPara_obj_vd.m_CG2fAxis);

	l_whlAng = yawRate * (m_vehPara_obj_vd.m_whlBase + l_EGpara * m_vehState_obj_vd.m_vehSpdX
		* m_vehState_obj_vd.m_vehSpdX) / m_vehState_obj_vd.m_vehSpdX;

	return l_whlAng;
}

//note: meticulous reader may find that the function "vehStateCalc(float steerAng)", 
//"vehPoseCalc(float steerAng)" and "objSteerAngCalc(float yawRate)" are all related to
//convert to the control u to the state or opposite, but the specification of them 
//are not exactly same.
//Of course we can change the code and let them become same; but, due to the mathmatic
//model we established could not entire correctly simulate the vehicle behavior in real world,
//so the vehicle behavior we predicted always have difference compare to the behavior in real world.
//That one of reasons we must use the feedback control, the proper feedback controller
//can eliminate the difference.
//At this demo, the difference is simulated by the difference between vehicle dynamic model 
//that is delivered to the MPC controller and the model that determine the vehicle pose
//in the simulation environment. 

VehDyn::~VehDyn()
{

}