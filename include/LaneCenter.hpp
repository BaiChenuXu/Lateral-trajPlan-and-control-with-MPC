#pragma once
#include"inputPara\Parameters.hpp"
#include"HighOrderCurve.hpp"
#include<vector>

//this class define the lane center

class LaneCenter
{
public:
	LaneCenter();

	//by the instantiation (let the instance become 
	//the member of current defining class), we import
	//the paramters those are defined in the "Paramters.hpp"
	//and use the member funcion that is defined in other class

	const laneCenterPara m_laneCenterPara_obj_lc;

	HighOrderCurve m_highOrderCurve_obj_lc;

	//note: in the constructor "LaneCenter()", use the 
	//"m_laneCenterPara_obj_lc" as the parameter to 
	//initialize the "m_highOrderCurve_obj_lc"
	//so the "m_laneCenterPara_obj_lc" must be defined before
	//the  "m_highOrderCurve_obj_lc"

	const roiPara m_roiPara_obj;

	vector<float> m_laneCenterX_vec; //unit:m
	vector<float> m_laneCenterY_vec; //unit:m
	vector<float> m_laneCenterTheta_vec; //unit:rad
	vector<float> m_laneCenterKappa_vec; //unit:1/m

	void discrectLaneCenterCalc();

	~LaneCenter();
};