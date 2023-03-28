#include"include\LaneCenter.hpp"
#include<vector>
#include<iostream>

using namespace std;

LaneCenter::LaneCenter(): m_highOrderCurve_obj_lc(m_laneCenterPara_obj_lc)
{
	this->m_laneCenterX_vec.push_back(0.0f);
	this->m_laneCenterY_vec.push_back(0.0f);
	this->m_laneCenterTheta_vec.push_back(0.0f);
	this->m_laneCenterKappa_vec.push_back(0.0f);
}

void LaneCenter::discrectLaneCenterCalc()
{
	for (float xCoor = m_roiPara_obj.m_roiBegin;
		xCoor < m_roiPara_obj.m_roiEnd + m_roiPara_obj.m_roiResolution;
		xCoor = xCoor + m_roiPara_obj.m_roiResolution)
	{
		if (xCoor == m_roiPara_obj.m_roiBegin)
		{
			m_laneCenterX_vec[0] = m_highOrderCurve_obj_lc.curvePropertyCalc(xCoor).m_curveX;
			m_laneCenterY_vec[0] = m_highOrderCurve_obj_lc.curvePropertyCalc(xCoor).m_curveY;
			m_laneCenterTheta_vec[0] = m_highOrderCurve_obj_lc.curvePropertyCalc(xCoor).m_curveTheta;
			m_laneCenterKappa_vec[0] = m_highOrderCurve_obj_lc.curvePropertyCalc(xCoor).m_curveKappa;
		}
		else
		{
			m_laneCenterX_vec.push_back(m_highOrderCurve_obj_lc.curvePropertyCalc(xCoor).m_curveX);
			m_laneCenterY_vec.push_back(m_highOrderCurve_obj_lc.curvePropertyCalc(xCoor).m_curveY);
			m_laneCenterTheta_vec.push_back(m_highOrderCurve_obj_lc.curvePropertyCalc(xCoor).m_curveTheta);
			m_laneCenterKappa_vec.push_back(m_highOrderCurve_obj_lc.curvePropertyCalc(xCoor).m_curveKappa);
		}
	}
}

LaneCenter::~LaneCenter()
{

}