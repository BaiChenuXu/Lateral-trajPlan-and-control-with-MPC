#include"include\VehDyn.hpp"
#include"include\LaneCenter.hpp"
#include"inputPara\Parameters.hpp"
#include"include\Trajectory.hpp"
#include<iostream>
#include<vector>
#include"MatlabEngine.hpp"
#include"MatlabDataArray.hpp"
#include"include\ProjectionAndErr.hpp"
#include"include\MpcController.hpp"

using namespace std;

int main()
{
	//Set simualtion time step number. Note that if the number
	//become bigger, there is risk the distance of vehicle motion
	//exceed the trajectory max distance.
	int simStep = 90;
	
	//instantiation all classes
	VehDyn vd;
	
	LaneCenter lc;

    Trajectory traj;

	ProjectionAndErr pe;

	MpcController mc;

	//get the discret lane center
	lc.discrectLaneCenterCalc();

    size_t vecSize = lc.m_laneCenterX_vec.size();
    cout << vecSize << endl;

	//get the discret trajectory
	traj.discrectTrajectoryCalc();

	size_t vecSize_1 = traj.m_trajectoryY_vec.size();
	cout << vecSize_1 << endl;

	//get the matrixs that MPC algorithem request
	mc.matrixGet();
	cout << mc.m_Hmatrix << endl;

	//do the simulation process
	for (int i = 0; i < simStep; i++)
	{
		//calculate the projection and control errors
		pe.projectionAndErrCalc(vd.m_vehState_obj_vd, traj);

		//calculate the optimization control value
		mc.optCtlCalc(pe.m_ctlErr, pe.m_kapparP, vd.m_vehState_obj_vd);

		//according to the control value to update the vehicle state
		vd.vehStateCalc(mc.m_ctlFinalValue);

		//calculate the vehicle pose in the freeze vehicle coordinate
		vd.vehPoseCalc(mc.m_ctlFinalValue);
	}

	size_t vecSize_2 = vd.m_vehState_obj_vd.m_vehPoseX.size();
	cout << vecSize_2 << endl;

	using namespace matlab::engine;

	//the following code use the matlab to do the simulation animation

	//import the matlab engine pointer and creat the instance
	std::unique_ptr<MATLABEngine> matlabPtr = startMATLAB();
	matlab::data::ArrayFactory factory;

	 //std::vector<matlab::data::Array> data_1({
		// factory.createArray({ 1,vecSize }, lc.m_laneCenterX_vec.begin(),lc.m_laneCenterX_vec.end()),
		// factory.createArray({ 1,vecSize }, lc.m_laneCenterY_vec.begin(),lc.m_laneCenterY_vec.end()),
		// factory.createCharArray("--r"),
		// factory.createArray({ 1,vecSize_1 }, traj.m_trajectoryX_vec.begin(),traj.m_trajectoryX_vec.end()),
		// factory.createArray({ 1,vecSize_1 }, traj.m_trajectoryY_vec.begin(),traj.m_trajectoryY_vec.end()),
		// factory.createCharArray("--b"),
		//  factory.createArray({ 1,vecSize_2 }, vd.m_vehState_obj_vd.m_vehPoseX.begin(),vd.m_vehState_obj_vd.m_vehPoseX.end()),
		// factory.createArray({ 1,vecSize_2 }, vd.m_vehState_obj_vd.m_vehPoseY.begin(),vd.m_vehState_obj_vd.m_vehPoseY.end()),
		// factory.createCharArray("k"),
		// });

	 //matlabPtr->feval(u"plot", data_1);

	 //creat the vector<matlab::data::Array> data_2
	 //that will deliver the data to matlab function
	 std::vector<matlab::data::Array> data_2({
	 factory.createArray({ 1,vecSize }, lc.m_laneCenterX_vec.begin(),lc.m_laneCenterX_vec.end()),
	 factory.createArray({ 1,vecSize }, lc.m_laneCenterY_vec.begin(),lc.m_laneCenterY_vec.end()),
	 factory.createArray({ 1,vecSize_1 }, traj.m_trajectoryX_vec.begin(),traj.m_trajectoryX_vec.end()),
	 factory.createArray({ 1,vecSize_1 }, traj.m_trajectoryY_vec.begin(),traj.m_trajectoryY_vec.end()),
	 factory.createArray({ 1,vecSize_2 }, vd.m_vehState_obj_vd.m_vehPoseX.begin(),vd.m_vehState_obj_vd.m_vehPoseX.end()),
	 factory.createArray({ 1,vecSize_2 }, vd.m_vehState_obj_vd.m_vehPoseY.begin(),vd.m_vehState_obj_vd.m_vehPoseY.end()),
	 factory.createArray({ 1,vecSize_2 }, vd.m_vehState_obj_vd.m_vehPoseAng.begin(),vd.m_vehState_obj_vd.m_vehPoseAng.end()),
		 });

	 //call the self-defined matlab function to complete the simulation animation
	 matlabPtr->feval(u"VehAnimation", data_2);

	 system("pause");
	 
	return 0;
}

