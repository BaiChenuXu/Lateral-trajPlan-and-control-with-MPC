#include"include\Trajectory.hpp"
#include<iostream>

using namespace std;

Trajectory::Trajectory(): m_highOrderCurve_obj_traj(m_laneCenterPara_obj_traj)
{
	this->m_trajectoryX_vec.push_back(0.0f);
	this->m_trajectoryY_vec.push_back(0.0f);
	this->m_trajectoryTheta_vec.push_back(0.0f);
	this->m_trajectoryKappa_vec.push_back(0.0f);
	//cout << "调用traj有参构造函数" << endl;
}

void Trajectory::discrectTrajectoryCalc()
{
	float l_y(0.0f);
	float l_dydx(0.0f);
	float l_ddydx(0.0f);
	float l_d0(0.0f);
	float l_d1(0.0f);
	float l_d2(0.0f);
	float l_sEnd(m_optPara_obj.m_sEndOpt);
	float l_sEnd_p2(l_sEnd * l_sEnd);
	float l_sEnd_p3(l_sEnd_p2 * l_sEnd);
	float l_sEnd_p4(l_sEnd_p3 * l_sEnd);
	float l_sEnd_p5(l_sEnd_p4 * l_sEnd);

	l_y = m_highOrderCurve_obj_traj.curvePropertyCalc(l_sEnd).m_curveY;
	l_dydx = m_highOrderCurve_obj_traj.curvePropertyCalc(l_sEnd).m_curveDydx;
	l_ddydx = m_highOrderCurve_obj_traj.curvePropertyCalc(l_sEnd).m_curveDdydx;

	//This code block calculate the 5 order polynomia coefficients.
	//Because, in the whole run stage (this demo), the planning trajectory exist in the 
	//freeze vehicle coordinate (at the moment of planning), and at the start point of
	//planning, the vehicle state (y position, heading angle, yawrate) which are determimed
	//by the planning curve is equa to vehicle real state, so we can get the "d0",
	//"d1", and the "d2".
	//And, at the end point (which is determimed by the "l_sEnd"), the vehicle state 
	//(y position, heading angle, yawrate) which are determimed by the planning curve 
	//is equa to vehicle real state which are determimed by the lane center curve,
	//so we can get equation set (matrix format): 
	//Ax = B, A = [s_end^3  , s_end^4   , s_end^5   ; B = [y - d0 - d1*s_end - d2/2*s_end^2,
	//             3*s_end^2, 4*s_end^3 , 5*s_end^4 ;      dydx - (d1 + 2*d2/2*s_end)      ,
	//             6*s_end  , 12*s_end^2, 20*s_end^3]      ddydx - 2*d2/2                  ]
	//solve it，we can get the expression of "d3","d4","d5" as following.
	{
		l_d0 = m_optTrajPara_obj.m_curveEquaD0 = 0;
		l_d1 = m_optTrajPara_obj.m_curveEquaD1 = 0;
		l_d2 = m_optTrajPara_obj.m_curveEquaD2 = m_vehState_obj_traj.m_vehAngSpd[0] / m_vehState_obj_traj.m_vehSpdX;

		m_optTrajPara_obj.m_curveEquaD3 = -(20 * l_d0 - 20 * l_y + 10 * l_d2 * l_sEnd_p2
			- 8 * l_sEnd * l_d1 + 8 * l_sEnd * l_dydx + l_sEnd_p2 * l_d2 - l_sEnd_p2 * l_ddydx
			+ 20 * l_d1 * l_sEnd - 8 * l_sEnd_p2 * l_d2) / (2 * l_sEnd_p3);
		m_optTrajPara_obj.m_curveEquaD4 = (30 * l_d0 - 30 * l_y + 15 * l_d2 * l_sEnd_p2
			- 14 * l_sEnd * l_d1 + 14 * l_sEnd * l_dydx + 2 * l_sEnd_p2 * l_d2 - 2 * l_sEnd_p2 * l_ddydx
			+ 30 * l_d1 * l_sEnd - 14 * l_sEnd_p2 * l_d2) / (2 * l_sEnd_p4);
		m_optTrajPara_obj.m_curveEquaD5 = -(12 * l_d0 - 12 * l_y + 6 * l_d2 * l_sEnd_p2
			- 6 * l_sEnd * l_d1 + 6 * l_sEnd * l_dydx + l_sEnd_p2 * l_d2 - l_sEnd_p2 * l_ddydx
			+ 12 * l_d1 * l_sEnd - 6 * l_sEnd_p2 * l_d2) / (2 * l_sEnd_p5);

		m_highOrderCurve_obj_traj.m_curveEquaCoeffi.m_curveEquaD0 = m_optTrajPara_obj.m_curveEquaD0;
		m_highOrderCurve_obj_traj.m_curveEquaCoeffi.m_curveEquaD1 = m_optTrajPara_obj.m_curveEquaD1;

		//note: actually the d2 that is calculated at above is the kappar, the kappar is twice of 
		//real d2 of the 5 order polynomia curve
		m_highOrderCurve_obj_traj.m_curveEquaCoeffi.m_curveEquaD2 = m_optTrajPara_obj.m_curveEquaD2 / 2;
		m_highOrderCurve_obj_traj.m_curveEquaCoeffi.m_curveEquaD3 = m_optTrajPara_obj.m_curveEquaD3;
		m_highOrderCurve_obj_traj.m_curveEquaCoeffi.m_curveEquaD4 = m_optTrajPara_obj.m_curveEquaD4;
		m_highOrderCurve_obj_traj.m_curveEquaCoeffi.m_curveEquaD5 = m_optTrajPara_obj.m_curveEquaD5;
	}

	for (float xCoor = m_roiPara_obj_traj.m_roiBegin;
		xCoor < l_sEnd + m_roiPara_obj_traj.m_roiResolution;
		xCoor = xCoor + m_roiPara_obj_traj.m_roiResolution)
	{
		if (xCoor == m_roiPara_obj_traj.m_roiBegin)
		{
			m_trajectoryX_vec[0] = m_highOrderCurve_obj_traj.curvePropertyCalc(xCoor).m_curveX;
			m_trajectoryY_vec[0] = m_highOrderCurve_obj_traj.curvePropertyCalc(xCoor).m_curveY;
			m_trajectoryTheta_vec[0] = m_highOrderCurve_obj_traj.curvePropertyCalc(xCoor).m_curveTheta;
			m_trajectoryKappa_vec[0] = m_highOrderCurve_obj_traj.curvePropertyCalc(xCoor).m_curveKappa;
		}
		else
		{
			m_trajectoryX_vec.push_back(m_highOrderCurve_obj_traj.curvePropertyCalc(xCoor).m_curveX);
			m_trajectoryY_vec.push_back(m_highOrderCurve_obj_traj.curvePropertyCalc(xCoor).m_curveY);
			m_trajectoryTheta_vec.push_back(m_highOrderCurve_obj_traj.curvePropertyCalc(xCoor).m_curveTheta);
			m_trajectoryKappa_vec.push_back(m_highOrderCurve_obj_traj.curvePropertyCalc(xCoor).m_curveKappa);
		}
	}

}

Trajectory::~Trajectory()
{

}