#pragma once

//This hpp file define:
//1) the HighOrderCurve class
//2) the curve property
//3) the curve equation coefficients

struct curveProperty
{
	float m_curveX; //unit:m
	float m_curveY; //unit:m
	float m_curveDydx; //the first order derivative of the curve   
	float m_curveDdydx; //the second order derivative of the curve
	float m_curveTheta; //unit:rad
	float m_curveKappa; //unit:1/m
};

//In this demo, the highest order of the curve is 5
//so there are 6 equation coefficients
struct curveEquaCoeffi
{
	float m_curveEquaD0;
	float m_curveEquaD1;
	float m_curveEquaD2;
	float m_curveEquaD3;
	float m_curveEquaD4;
	float m_curveEquaD5;
};

class HighOrderCurve
{
public:

	HighOrderCurve(); 

	curveEquaCoeffi m_curveEquaCoeffi; 

	HighOrderCurve(curveEquaCoeffi curveEquaCoeffi); 

	curveProperty curvePropertyCalc(float xCoor);

	~HighOrderCurve();

};

