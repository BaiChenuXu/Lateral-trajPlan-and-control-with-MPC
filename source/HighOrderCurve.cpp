#include"include\HighOrdercurve.hpp"
#include<cmath>
#include<iostream>

using namespace std;

HighOrderCurve::HighOrderCurve()
{
	m_curveEquaCoeffi.m_curveEquaD0 = 0.0f;
	m_curveEquaCoeffi.m_curveEquaD1 = 0.0f;
	m_curveEquaCoeffi.m_curveEquaD2 = 0.0f;
	m_curveEquaCoeffi.m_curveEquaD3 = 0.0f;
	m_curveEquaCoeffi.m_curveEquaD4 = 0.0f;
	m_curveEquaCoeffi.m_curveEquaD5 = 0.0f;
}

HighOrderCurve::HighOrderCurve(curveEquaCoeffi curveEquaCoeffi)
{
	this->m_curveEquaCoeffi.m_curveEquaD0 = curveEquaCoeffi.m_curveEquaD0;
	this->m_curveEquaCoeffi.m_curveEquaD1 = curveEquaCoeffi.m_curveEquaD1;
	this->m_curveEquaCoeffi.m_curveEquaD2 = curveEquaCoeffi.m_curveEquaD2;
	this->m_curveEquaCoeffi.m_curveEquaD3 = curveEquaCoeffi.m_curveEquaD3;
	this->m_curveEquaCoeffi.m_curveEquaD4 = curveEquaCoeffi.m_curveEquaD4;
	this->m_curveEquaCoeffi.m_curveEquaD5 = curveEquaCoeffi.m_curveEquaD5;

	//cout << "m_curveEquaD0 = " << m_curveEquaCoeffi.m_curveEquaD0 << endl;
	//cout << "调用父类有参构造函数" << endl;
}

curveProperty HighOrderCurve::curvePropertyCalc(float xCoor)
{
	curveProperty cp;

	float xCoor_p2 = xCoor * xCoor;
	float xCoor_p3 = xCoor * xCoor_p2;
	float xCoor_p4 = xCoor * xCoor_p3;
	float xCoor_p5 = xCoor * xCoor_p4;
	float temp;

	//cout << "in curvePropertyCalc m_curveEquaD0 = " << m_curveEquaCoeffi.m_curveEquaD0 << endl;

	cp.m_curveX = xCoor;

	cp.m_curveY = m_curveEquaCoeffi.m_curveEquaD0 +
		m_curveEquaCoeffi.m_curveEquaD1 * xCoor +
		m_curveEquaCoeffi.m_curveEquaD2 * xCoor_p2 +
		m_curveEquaCoeffi.m_curveEquaD3 * xCoor_p3 +
		m_curveEquaCoeffi.m_curveEquaD4 * xCoor_p4 +
		m_curveEquaCoeffi.m_curveEquaD5 * xCoor_p5;

	cp.m_curveDydx = m_curveEquaCoeffi.m_curveEquaD1 +
		2 * m_curveEquaCoeffi.m_curveEquaD2 * xCoor +
		3 * m_curveEquaCoeffi.m_curveEquaD3 * xCoor_p2 +
		4 * m_curveEquaCoeffi.m_curveEquaD4 * xCoor_p3 +
		5 * m_curveEquaCoeffi.m_curveEquaD5 * xCoor_p4;

	cp.m_curveDdydx = 2 * m_curveEquaCoeffi.m_curveEquaD2 +
		6 * m_curveEquaCoeffi.m_curveEquaD3 * xCoor +
		12 * m_curveEquaCoeffi.m_curveEquaD4 * xCoor_p2 +
		20 * m_curveEquaCoeffi.m_curveEquaD5 * xCoor_p3;

	//according to the relationship between the first order derivative of the curve and
	//the included angle between the tangent line of the curve and the x axis
	cp.m_curveTheta = atan2(static_cast<double>(cp.m_curveDydx), 1.0);

	//get the kappar calculation equation easily on the internet, that is |y''|/(1 + y'^2)^1.5
	//we delete the abs() calculation, because we need the kappar have the sign
	//when the curve bend to the left of the vehicle, the kappar is positive
	//on the contrary, the kappar is negtive
	cp.m_curveKappa = cp.m_curveDdydx / pow((1 + cp.m_curveDydx * cp.m_curveDydx), 1.5f);

	return cp;
}

HighOrderCurve::~HighOrderCurve()
{

}