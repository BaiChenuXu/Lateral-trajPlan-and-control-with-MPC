#pragma once
#include<vector>
#include"HighOrderCurve.hpp"
#include<Eigen/Dense>

using namespace Eigen;
using namespace std;

//This hpp file define the input parameters that the model 
//and the algorithem request.

//Define the lane center parameters.The lane center is
//described by the 3 order polynomia, and that is subordinate 
//to the HighOrderCurve.
struct laneCenterPara: curveEquaCoeffi
{
	laneCenterPara()
	{
		m_curveEquaD0 = 8.0f; //constant term of 3 order polynomia
		m_curveEquaD1 = 0.023f; //first order term of 3 order polynomia
		m_curveEquaD2 = 0.002f; //second order term of 3 order polynomia
		m_curveEquaD3 = 0.0001f; //third order term of 3 order polynomia
		m_curveEquaD4 = 0.0f;
		m_curveEquaD5 = 0.0f;
	}
};

//Define the vehicle parameters. 
struct vehPara
{
	vehPara()
		: m_curbWeight(1225.0f),
		m_CG2fAxis(1.125f),
		m_CG2rAxis(1.375f),
		m_whlBase(2.5f),
		m_stiffFWhl(-104459.0f),
		m_stiffRWhl(-101592.0f),
		m_inertiaZaxis(200.0f),
		m_vehWidth(1.6f),
		m_RS2front(3.0f),
		m_RS2rear(0.5f),
		m_fw_steerLb(-0.7f),
		m_fw_steerUb(0.7f)
	{
	}
	float m_curbWeight; //curb weight, unit:kg
	float m_CG2fAxis; //distance between center of gravity to front axis, unit:m
	float m_CG2rAxis; //distance between center of gravity to rear axis, unit:m
	float m_whlBase; //wheel base, unit:m
	float m_stiffFWhl; //Tire cornering stiffness of front wheels(2 wheels), unit:n/rad
	float m_stiffRWhl; //Tire cornering stiffness of rear wheels(2 wheels), unit:n/rad
	float m_inertiaZaxis; //rotational inertial of vehicle Z axis, unit:kg.m^2

	float m_vehWidth; //vehicle width, unit:m
	float m_RS2front; //distance between rear axis to front edge, unit:m
	float m_RS2rear; //distance between rear axis to rear edge, unit:mm

	float m_fw_steerLb; //front wheel max left rotation angle unit:rad
	float m_fw_steerUb; //front wheel max right rotation angle unit:rad

};

//The definition of the control system
struct ctlSysPara
{
	ctlSysPara()
		: m_tDurPerStep(0.01f),
		m_actuatorErr(0.0f),
		m_ctlDelayStep(0)
	{
	}
	float m_tDurPerStep; //the time step of control system, unit:s
	float m_actuatorErr; //the percentage of control system error, unit:faction
	int m_ctlDelayStep; //the time steps of control system delay
};

//Use vector to define vehicle state Array. Vehicle state update with the
//time step increasing. This demo only focus on the vehicle lateral planing
//and control, so the vehicle longtitude speed is constant.
struct vehState
{
	vehState()
	{
		m_vehPoseX.push_back(0.0f);
		m_vehPoseY.push_back(0.0f);
		m_vehPoseAng.push_back(0.0f);
		m_vehY_vCoor.push_back(0.0f);
		m_vehDotY_vCoor.push_back(0.05f);
		m_vehAng_vCoor.push_back(0.0f);
		m_vehAngSpd.push_back(0.06f);
		m_vehSpdX = 33.0f;
	}
	vector<float> m_vehPoseX; //vehicle x position array in freeze veh coordinate, unit:m
	vector<float> m_vehPoseY; //vehicle y position array in freeze veh coordinate, unit:m
	vector<float> m_vehPoseAng; //vehicle theta array in freeze veh coordinate, unit:rad
	vector<float> m_vehAng_vCoor; //vehicle theta array in updating veh cootdinate unit:rad

	                              //the theta is the included angle between the tangent 
	                              //line of the curve and the x axis

	vector<float> m_vehAngSpd; //vehicle angle speed array, unit:rad/s
	vector<float> m_vehY_vCoor; //vehicle y position array in updating veh cootdinate, unit:m
	vector<float> m_vehDotY_vCoor; ///vehicle y speed array in updating veh cootdinate, unit:m/s
	float m_vehSpdX; // vehicle longtitude speed in updating veh cootdinate, unit:m/s
};

//Define the perception range of lane line and the resolution.
//In fact this is only used to get the discret lane center points, according 
//to the 3 order polynomia of lane center parameters (that is defined by 
//the lane center parameters).
//We do not request the perception range less than 0 or very close to 0, the single
//vision system conld not satify this request.
struct roiPara
{
	roiPara()
		: m_roiBegin(-10.0f),
		m_roiEnd(50.0f),
		m_roiResolution(1.0f)
	{
	}
	float m_roiBegin; //the begin of the perception range, unit:m
	float m_roiEnd; //the begin of the perception range, unit:m
	float m_roiResolution; //the resolusion to calculate discret lane center points, unit:m
};

//Define optimization paramters.
struct optPara
{
	optPara()
		: m_sEndOpt(30.0f),
		m_tPre(0.01f),
		m_mpcPreStep(15)
	{
        m_Qmatrix = Eigen::MatrixXf::Zero(4, 4);
		m_Qmatrix(0, 0) = 100;
		m_Qmatrix(1, 1) = 10;
		m_Qmatrix(2, 2) = 5;
		m_Qmatrix(3, 3) = 0.0f;
		m_Fmatrix = Eigen::MatrixXf::Zero(4, 4);
		m_Fmatrix(0, 0) = 100;
		m_Fmatrix(1, 1) = 10;
		m_Fmatrix(2, 2) = 5;
		m_Fmatrix(3, 3) = 0.0f;
		m_Rmatrix = Eigen::MatrixXf::Zero(1, 1);
		m_Rmatrix(0, 0) = 1;
	}
	//The distance of backing to lane center, that is key parameter to calculate
	//the planning trajectory. usually, getting it need to run a optimization
	//algorithem. We give it directly in this demo. unit:m
	float m_sEndOpt; 

	//Usually, we do not control the vehicle according to the current vehicle state.
	//We do the control according to the state after a certain time.
	//The following is the definition of the certain time. unit:s
	float m_tPre; 

	Eigen::Matrix4f m_Qmatrix; //Q matrix of MPC controller
	Eigen::MatrixXf m_Rmatrix; //R matrix of MPC controller
	Eigen::Matrix4f m_Fmatrix; //F matrix of MPC controller

    //The horizon of MPC controller, in this demo,the prediction horizon 
	//is euqual to the control horizon.
	int m_mpcPreStep; 
};