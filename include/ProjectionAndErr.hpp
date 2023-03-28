#pragma once
#include"inputPara\Parameters.hpp"
#include"Trajectory.hpp"

struct ctlErr
{
	float m_ed;
	float m_dot_ed;
	float m_ephi;
	float m_dot_ephi;
};

//This class calculate the projection of current vehicle pose
//to the trajectory, and get the control error.
//The control error is consist of lateral deviation, the derivative
//of the lateral deviation, the angle difference between the vehicle 
//direction and the tangent direction of the trajectory
//and its derivative
class ProjectionAndErr
{
public:
	ProjectionAndErr();

	const optPara m_optPara_obj_pe;

	void projectionAndErrCalc(vehState vehState, Trajectory traj);

	int minElementPos(vector<float> Arr);

	ctlErr m_ctlErr;

	float m_kapparP;

	~ProjectionAndErr();
};

;
