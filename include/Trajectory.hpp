#pragma once
#include"inputPara\Parameters.hpp"
#include"include\HighOrderCurve.hpp"
#include<vector>

//this class define the planning trajectory

//the optTrajPara inherit from curveEquaCoeffi
struct optTrajPara : curveEquaCoeffi
{

};

class Trajectory
{
public:
	Trajectory();

	//by the instantiation (let the instance become 
	//the member of current defining class), we import
	//the paramters those are defined in the "Paramters.hpp"
	//and use the member funcion that is defined in other class

	const laneCenterPara m_laneCenterPara_obj_traj;

	HighOrderCurve m_highOrderCurve_obj_traj;

	//note: in the constructor "Trajectory()", use the 
    //"m_laneCenterPara_obj_traj" as the parameter to 
    //initialize the "m_highOrderCurve_obj_traj"
    //so the "m_laneCenterPara_obj_traj" must be defined before
    //the  "m_highOrderCurve_obj_traj"

	const optPara m_optPara_obj;

	vehState m_vehState_obj_traj;

	optTrajPara m_optTrajPara_obj;

	vector<float> m_trajectoryX_vec; //m
	vector<float> m_trajectoryY_vec; //m
	vector<float> m_trajectoryTheta_vec; //rad
	vector<float> m_trajectoryKappa_vec; // 1/m

	const roiPara m_roiPara_obj_traj;

	void discrectTrajectoryCalc();

	~Trajectory();
};

