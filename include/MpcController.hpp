#pragma once
#include"VehDyn.hpp"
#include"inputPara\Parameters.hpp"
#include<Eigen/Dense>
#include"ProjectionAndErr.hpp"

//this class calculate the optimization control u 
//there are 2 functions:
//First one: according to the input from "Parameters.hpp"
//and the matrix A and B of vehicle dynamic model, get the
//hessian matrix,E matrix and the other matrix that 
//the MPC algorithem request.
//Second one: using the quadratic programming library OsqpEigen
//to get optimization control array, and obtain the final control
//value. (combine the feedforward control value)
class MpcController
{
public:

	VehDyn m_VehDyn_obj_mpc;

	optPara m_optPara_obj_mpc;

	vehPara m_vehPara_obj_mpc;

	float m_FFCtlValue;
	float m_ctlFinalValue;

	Eigen::VectorXd m_optFbCtlArr;
	Eigen::MatrixXf m_Mmatrix;
	Eigen::MatrixXf m_Cmatrix;
	Eigen::MatrixXf m_QbarMatrix;
	Eigen::MatrixXf m_RbarMatrix;
	Eigen::MatrixXf m_Gmatrix;
	Eigen::MatrixXf m_Ematrix;
	Eigen::MatrixXf m_Hmatrix;

    MpcController();

	void matrixGet();

	void optCtlCalc(ctlErr ctlErr, float kapparRef, vehState vehState);

	~MpcController();

};