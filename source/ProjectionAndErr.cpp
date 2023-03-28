#include"include\ProjectionAndErr.hpp"
#include<math.h>
#include<vector>
#include<Eigen/Dense>

using namespace std;
using namespace Eigen;

ProjectionAndErr::ProjectionAndErr()
{
    this->m_ctlErr.m_ed = 0;
	this->m_ctlErr.m_dot_ed = 0.0f;
	this->m_ctlErr.m_ephi = 0.0f;
	this->m_ctlErr.m_dot_ephi = 0.0f;
	this->m_kapparP = 0.0f;
}

//calculate the projection of current vehicle pose to the trajectory
//the amazing reference is:
//video(in chinese): https://www.bilibili.com/video/BV1t54y1m7dg/?spm_id_from=333.999.0.0
//the writer of this demo do some modifications
void ProjectionAndErr::projectionAndErrCalc(vehState vehState, Trajectory traj)
{
	float l_Xpre(0.0f);
	float l_Ypre(0.0f);
	float l_phiPre(0.0f);
    int l_lenVector(0);
    vector<float> distArr;
	Eigen::Vector2f xx_temp;
	Eigen::Vector2f nn_temp;
	Eigen::Vector2f vehPos_temp;
	Eigen::Vector2f Pneg_temp;
	Eigen::Vector2f Ppos_temp;
	Eigen::Vector2f Pm_temp;

	int l_indexMin(0);
	Eigen::Vector2f l_Ppoint;
	float l_theta_1(0.0f);
	float l_theta_2(0.0f);
	Eigen::Vector2f p_temp;
	float l_arclen(0.0f);
	float l_distProj(0.0f);
	float l_distResi(0.0f);
	float l_dkappaP(0.0f);
	float l_thetaP(0.0f);
	int l_distResiSign(0);
	float l_dots(0.0f);

	//predict the veh pose after certain time
	//In fact, if the steer anlge is very small, the following is right exactly.
	//If the steer anlge is big, there is less accuracy, and using the cycle arc
	//to replace straght line is better choice    
	l_Xpre = vehState.m_vehPoseX.back() + vehState.m_vehSpdX * m_optPara_obj_pe.m_tPre
		* cos(vehState.m_vehPoseAng.back()) - vehState.m_vehDotY_vCoor.back()
		* m_optPara_obj_pe.m_tPre * sin(vehState.m_vehPoseAng.back());
	l_Ypre = vehState.m_vehPoseY.back() + vehState.m_vehDotY_vCoor.back() * m_optPara_obj_pe.m_tPre
		* cos(vehState.m_vehPoseAng.back()) + vehState.m_vehSpdX * m_optPara_obj_pe.m_tPre 
		* sin(vehState.m_vehPoseAng.back());
	l_phiPre = vehState.m_vehPoseAng.back() + vehState.m_vehAngSpd.back() * m_optPara_obj_pe.m_tPre;

	l_lenVector = traj.m_trajectoryX_vec.size();

	//get the match point index in the trajectory array
	for (int i = 0; i < l_lenVector; i++)
	{
		float temp = pow((traj.m_trajectoryX_vec[i] - l_Xpre), 2)
			+ pow((traj.m_trajectoryY_vec[i] - l_Ypre), 2);
		temp = sqrt(temp);
		distArr.push_back(temp);
	}
	l_indexMin = minElementPos(distArr);

	xx_temp(0, 0) = l_Xpre - traj.m_trajectoryX_vec[l_indexMin];
	xx_temp(1, 0) = l_Ypre - traj.m_trajectoryY_vec[l_indexMin];

	nn_temp(0, 0) = -sin(traj.m_trajectoryTheta_vec[l_indexMin]);
	nn_temp(1, 0) = cos(traj.m_trajectoryTheta_vec[l_indexMin]);
	
	//get distance between the vehicle position and the projection point 
	l_distProj = xx_temp.dot(nn_temp);

	vehPos_temp << l_Xpre, l_Ypre;

	l_Ppoint = vehPos_temp - l_distProj * nn_temp;

	Pneg_temp(0, 0) = traj.m_trajectoryX_vec[l_indexMin] - traj.m_trajectoryX_vec[l_indexMin - 1];
	Pneg_temp(1, 0) = traj.m_trajectoryY_vec[l_indexMin] - traj.m_trajectoryY_vec[l_indexMin - 1];
	Ppos_temp(0, 0) = traj.m_trajectoryX_vec[l_indexMin] - traj.m_trajectoryX_vec[l_indexMin + 1];
	Ppos_temp(1, 0) = traj.m_trajectoryY_vec[l_indexMin] - traj.m_trajectoryY_vec[l_indexMin + 1];
	Pm_temp << traj.m_trajectoryX_vec[l_indexMin], traj.m_trajectoryY_vec[l_indexMin];
	Pm_temp = Pm_temp - l_Ppoint;

	l_theta_1 = acos(Pneg_temp.dot(Pm_temp) / Pneg_temp.norm() / Pm_temp.norm());
	l_theta_2 = acos(Ppos_temp.dot(Pm_temp) / Ppos_temp.norm() / Pm_temp.norm());

	//compare the theta_1 and theta_2 to detect that the projection point is in front of the match point
	//or the projection point is behind of the match point
	if (l_theta_1 >= l_theta_2 || Pm_temp.norm() == 0)
	{
		//get the kappar of the projection point
		m_kapparP = (traj.m_trajectoryKappa_vec[l_indexMin] + traj.m_trajectoryKappa_vec[l_indexMin + 1]) / 2;
		float kapparAbs_temp = fabs(m_kapparP);
		p_temp << traj.m_trajectoryX_vec[l_indexMin], traj.m_trajectoryY_vec[l_indexMin];

		l_distResi = 2 * asin((p_temp - l_Ppoint).norm() * kapparAbs_temp / 2) / kapparAbs_temp;

		p_temp(0, 0) = traj.m_trajectoryX_vec[l_indexMin + 1] - traj.m_trajectoryX_vec[l_indexMin];
		p_temp(1, 0) = traj.m_trajectoryY_vec[l_indexMin + 1] - traj.m_trajectoryY_vec[l_indexMin];

		l_arclen = 2 * asin(p_temp.norm() * kapparAbs_temp / 2) / kapparAbs_temp;
		l_dkappaP = (traj.m_trajectoryKappa_vec[l_indexMin + 1] - traj.m_trajectoryKappa_vec[l_indexMin]) / l_arclen;

		//get the theta of the projection point
		l_thetaP = traj.m_trajectoryTheta_vec[l_indexMin] + l_arclen * m_kapparP;
		l_distResiSign = 1;
	}
	else
	{
		m_kapparP = (traj.m_trajectoryKappa_vec[l_indexMin] + traj.m_trajectoryKappa_vec[l_indexMin - 1]) / 2;
		float kapparAbs_temp = fabs(m_kapparP);
		p_temp << traj.m_trajectoryX_vec[l_indexMin], traj.m_trajectoryY_vec[l_indexMin];

		l_distResi = 2 * asin((p_temp - l_Ppoint).norm() * kapparAbs_temp / 2) / kapparAbs_temp;

		p_temp(0, 0) = traj.m_trajectoryX_vec[l_indexMin] - traj.m_trajectoryX_vec[l_indexMin - 1];
		p_temp(1, 0) = traj.m_trajectoryY_vec[l_indexMin] - traj.m_trajectoryY_vec[l_indexMin - 1];

		l_arclen = 2 * asin(p_temp.norm() * kapparAbs_temp / 2) / kapparAbs_temp;
		l_dkappaP = (traj.m_trajectoryKappa_vec[l_indexMin] - traj.m_trajectoryKappa_vec[l_indexMin - 1]) / l_arclen;

		l_thetaP = traj.m_trajectoryTheta_vec[l_indexMin] - l_arclen * m_kapparP;
		l_distResiSign = -1;

	}

	if (l_Ppoint[2, 1] > 0)
	{
		m_ctlErr.m_ed = l_distProj;
	}
	else
	{
		m_ctlErr.m_ed = -l_distProj;
	}
	m_ctlErr.m_dot_ed = vehState.m_vehSpdX * sin(l_phiPre - l_thetaP);
	l_dots = vehState.m_vehSpdX * cos(l_phiPre - l_thetaP) / (1 - m_kapparP * m_ctlErr.m_ed);
	m_ctlErr.m_ephi = l_phiPre - l_thetaP;

	m_ctlErr.m_dot_ephi = vehState.m_vehAngSpd.back() - m_kapparP * vehState.m_vehSpdX;
	//m_ctlErr.m_dot_ephi = 0;

}

//get min element index of vector
int ProjectionAndErr::minElementPos(vector<float> Arr)
{ 
	int l_lenVector(0);
	int l_minIndex(0);

	l_lenVector = Arr.size();
	float temp = 1000000;
	for (int i = 0; i < l_lenVector; i++)
	{
		if (Arr[i] < temp)
		{
			temp = Arr[i];
			l_minIndex = i;
		}
		else
		{
		}
	}
	return l_minIndex;
}

ProjectionAndErr::~ProjectionAndErr()
{


}