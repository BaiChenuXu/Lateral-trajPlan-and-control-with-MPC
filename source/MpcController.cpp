#include"include\MpcController.hpp"
#include<OsqpEigen/OsqpEigen.h>
#include<Eigen/Dense>
#include<osqp.h>

using namespace Eigen;
using namespace std;

MpcController::MpcController()
{
	this->m_FFCtlValue = 0.0f;
	this->m_ctlFinalValue = 0.0f;
}

//get the hessian matrix,E matrix and the other matrix that 
//the MPC algorithem request
//the amazing reference is:
//video(in chinese): https://www.bilibili.com/video/BV1Y44y1b7ke/?spm_id_from=333.999.0.0
void MpcController::matrixGet()
{
	m_VehDyn_obj_mpc.vehStateEqua();

	Eigen::Index l_Adimension;
	Eigen::Index l_Udimension;

	l_Adimension = m_VehDyn_obj_mpc.m_A_matrix_baseErr.rows();
	l_Udimension = m_VehDyn_obj_mpc.m_B_matrix.cols();

	m_Mmatrix = Eigen::MatrixXf::Zero((m_optPara_obj_mpc.m_mpcPreStep + 1)
		* l_Adimension, l_Adimension);
	m_Cmatrix = Eigen::MatrixXf::Zero((m_optPara_obj_mpc.m_mpcPreStep + 1)
		* l_Adimension, m_optPara_obj_mpc.m_mpcPreStep * l_Udimension);

	m_QbarMatrix = Eigen::MatrixXf::Zero((m_optPara_obj_mpc.m_mpcPreStep + 1)
		* l_Adimension, (m_optPara_obj_mpc.m_mpcPreStep + 1) * l_Adimension);
	m_RbarMatrix = Eigen::MatrixXf::Zero((m_optPara_obj_mpc.m_mpcPreStep)
		* l_Udimension, (m_optPara_obj_mpc.m_mpcPreStep) * l_Udimension);

	Eigen::MatrixXf l_matrixEye = Eigen::MatrixXf::Identity(l_Adimension, l_Adimension);

	m_Mmatrix.block(0, 0, l_Adimension, l_Adimension) = l_matrixEye;

	Eigen::MatrixXf l_matrixTemp_1 = Eigen::MatrixXf::Identity(l_Adimension, l_Adimension);
	for (int i = 0; i < m_optPara_obj_mpc.m_mpcPreStep; i++)
	{
		l_matrixTemp_1 = m_VehDyn_obj_mpc.m_A_matrix_baseErr * l_matrixTemp_1;
		m_Mmatrix.block(i * l_Adimension + l_Adimension, 0, l_Adimension, l_Adimension) = l_matrixTemp_1;
	}

	for (int j = 0; j < m_optPara_obj_mpc.m_mpcPreStep; j++)
	{
		Eigen::MatrixXf l_matrixTemp_2 = m_VehDyn_obj_mpc.m_B_matrix;
		for (int i = 0; i < m_optPara_obj_mpc.m_mpcPreStep; i++)
		{
			if (j <= i)
			{
				m_Cmatrix.block(i * l_Adimension + l_Adimension, j * l_Udimension,
					l_Adimension, l_Udimension) = l_matrixTemp_2;
				l_matrixTemp_2 = m_VehDyn_obj_mpc.m_A_matrix_baseErr * l_matrixTemp_2;
			}
		}
	}

	for (int i = 0; i < m_optPara_obj_mpc.m_mpcPreStep + 1; i++)
	{
		if (i < m_optPara_obj_mpc.m_mpcPreStep)
		{
			m_QbarMatrix.block(i * l_Adimension, i * l_Adimension, l_Adimension, l_Adimension)
				= m_optPara_obj_mpc.m_Qmatrix;
		}
		else
		{
			m_QbarMatrix.block(i * l_Adimension, i * l_Adimension, l_Adimension, l_Adimension)
				= m_optPara_obj_mpc.m_Fmatrix;
		}
	}

	for (int i = 0; i < m_optPara_obj_mpc.m_mpcPreStep; i++)
	{
		m_RbarMatrix.block(i * l_Udimension, i * l_Udimension, l_Udimension, l_Udimension)
			= m_optPara_obj_mpc.m_Rmatrix;
	}

	m_Gmatrix = m_Mmatrix.transpose() * m_QbarMatrix * m_Mmatrix;
	m_Ematrix = m_Cmatrix.transpose() * m_QbarMatrix * m_Mmatrix;
	m_Hmatrix = m_Cmatrix.transpose() * m_QbarMatrix * m_Cmatrix + m_RbarMatrix;
}

//using the quadratic programming library OsqpEigen
//to get optimization control array, and obtain the final control
//value. (combine the feedforward control value)
void MpcController::optCtlCalc(ctlErr ctlErr,float kapparRef, vehState vehState)
{
	int l_numOfCons = m_optPara_obj_mpc.m_mpcPreStep;
	int l_numOfVar = static_cast<int>(m_Hmatrix.rows());
	
	Eigen::Index l_Adimension;
	l_Adimension = m_VehDyn_obj_mpc.m_A_matrix_baseErr.rows();

	VectorXf err = Eigen::VectorXf::Zero(l_Adimension, 1);
	err << ctlErr.m_ed, ctlErr.m_dot_ed, ctlErr.m_ephi, ctlErr.m_dot_ephi;

	Eigen::SparseMatrix<float> hessian;
	Eigen::SparseMatrix<float> linearMatrix;
	VectorXf gradient_temp;
	VectorXd gradient;
	VectorXd lowerBound = Eigen::VectorXd::Zero(l_numOfCons, 1);
	VectorXd upperBound = Eigen::VectorXd::Zero(l_numOfCons, 1);;

	OsqpEigen::Solver FB_optCtl;

	FB_optCtl.data()->setNumberOfVariables(l_numOfVar);
	FB_optCtl.data()->setNumberOfConstraints(l_numOfCons);

	hessian.resize(l_numOfVar, l_numOfVar);
	gradient.resize(l_numOfVar);
	linearMatrix.resize(l_numOfCons, l_numOfVar);
	lowerBound.resize(l_numOfCons);
	upperBound.resize(l_numOfCons);

	gradient_temp = m_Ematrix * err;

	//the OsqpEigen input data type is double, so convert the data type
	for (int i = 0; i < gradient.size(); i++)
	{
		gradient(i) = static_cast<double>(gradient_temp(i));
	}

	//the OsqpEigen input matrix is sparse matrix, so convert the dense 
	//matrix to the sparse matrix
	hessian = m_Hmatrix.sparseView();

	//OsqpEigen set all the constrains (the equality constrain, the inequality constrain,
	//and the independent variable constrain) by the lineMatrix and the lowerBound and upperBound.
	//In this demo, we only use the independent variable to limit the change range of
	//front wheel steer angle.
	Eigen::MatrixXf l_MatrixTemp = Eigen::MatrixXf::Identity(l_numOfCons,l_numOfCons);
	linearMatrix = l_MatrixTemp.sparseView();

	for (int i = 0; i < l_numOfCons; i++)
	{
		lowerBound(i) = static_cast<double>(m_vehPara_obj_mpc.m_fw_steerLb);
	}

	for (int i = 0; i < l_numOfCons; i++)
	{
		upperBound(i) = static_cast<double>(m_vehPara_obj_mpc.m_fw_steerUb);
	}

	FB_optCtl.settings()->setVerbosity(true);
	FB_optCtl.settings()->setWarmStart(true);
	
	FB_optCtl.data()->setHessianMatrix(hessian);
	FB_optCtl.data()->setGradient(gradient);
	FB_optCtl.data()->setLinearConstraintsMatrix(linearMatrix);
	FB_optCtl.data()->setLowerBound(lowerBound);
	FB_optCtl.data()->setUpperBound(upperBound);
	
	FB_optCtl.initSolver();
	FB_optCtl.solve();

	m_optFbCtlArr = FB_optCtl.getSolution().transpose();

	float l_objYawRate(0.0f);

	l_objYawRate = kapparRef * vehState.m_vehSpdX;
	m_FFCtlValue = m_VehDyn_obj_mpc.objSteerAngCalc(l_objYawRate);

	m_ctlFinalValue = static_cast<float>(m_optFbCtlArr(0)) + m_FFCtlValue;
}

MpcController::~MpcController()
{

}