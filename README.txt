1.introduce
{
This demo is surbodinate to the ADAS.
It show how to conduct the lateral planning from the current lateral deviation
to back to the lane center after a certain distance, and
how to use MPC to control vehicle track the planning trajectory.

The program organization of this demo:
1) get the discret lane center (3 order polynomia)
2) get the planning trajectory (5 order polynomia)
3) get the projection of current vehicle pose to the trajectory
    and calculate the control error
4) get optmizition control value with MPC
5) update the vehicle state with lateral vehicle dynamic model 
    of linear and 2 freedom degree

Currently, this demo run on the windows system and visual studio IDE.

This demo use the self-defined matlab function to show the simulation
animation. You could open the simuResultAnimation.mp4 firstly to see
the normal run result before runing the program by yourself.
}

2.create the project
{
use the visual studio create the project (named"LatPlanAndCtl_wMPC")
then add the file in the "source" and "mainFunc" folder to the "Source Files"
and add the file in the "include" and "inputPara" folder to the "Header Files"
}

3.config the environment
{
1) Use the  matlab engine (matlab API for C++) establish the contact of 
matlab and C++ (visual studio). (matlab version > 2021b)
the specific operation is following:
https://ww2.mathworks.cn/matlabcentral/answers/406574-how-can-i-compile-a-c-program-that-uses-matlab-engine-c-api-in-visual-studio-2017?s_tid=srchtitle

matlab engine reference:
https://ww2.mathworks.cn/help/matlab/apiref/matlab.engine.matlabengine_cpp.html
https://ww2.mathworks.cn/help/matlab/matlab_external/pass-variables-from-c-to-matlab.html

if you complete the above config steps, and run the test, but it still not work. 
open the visual studio->Tools->Options->Debugging->Symbols and select "Microsoft Symbol Servers" and "NuGet.org Symbol Server"
you could also add another symbols server http://msdl.microsoft.com/download/symbols to make sure the request symbols are downloaded
then run the test program and patiently wait the finish of downloading the symbols.

finally, let the simuAnimation.m file go into the matlab search path. (use the matlab "Set Path")

2) this demo use Eigen library do the matrx calculation.
we can use Eigen library without installing, do the following steps: 
- download the Eigen from https://github.com/PX4/eigen
- unzip the file, suppose you place folder in the D:\ and the folder name is "eigen"
- open the Local Windows Debugger->LatPlanAndCtl_wMPC Debug Properties->C/C++->General
   ->Additional Include Directories->Edit->type in "D:/eigen"

3) this demo use osqpEigen library to solve the quadratic programming problem, and the osqpEigen request osqp library and Eigen.
different from the Eigen, the osqp and the osqpEigen must be installed in C: disk correctly.
when download the osqp and the osqpEigen, make sure they are compatible.
recommand: download the osqp 6.0 and the osqpEigen 6.2.

osqp: https://github.com/osqp/osqp/releases
osqpEigen: https://github.com/robotology/osqp-eigen/releases

compile and install process could refer to: 
https://blog.csdn.net/u010149495/article/details/122075415 (in chinese)

before compile the osqp, you need the qdldl library
download it: https://github.com/osqp/qdldl
and put all the content into the D:\osqp\lin_sys\direct\qdldl\qdldl_sources
(suppose you place folder in the D:\ and the folder name is "\osqp")

when use cmake compile the osqpEigen, the key step is config the Eigen3_DIR and the osqp_DIR
Eigen3_DIR: D:/eigen/bulid (suppose you place folder in the D:\ and the folder name is "eigen")
osqp_DIR: C:/Program Files/osqp/lib/cmake/osqp (if you installed the osqp correctly, the folder will appear in the C:/Program Files)

after complete the compile by cmake, you will get the osqp.sln and OsqpEigen.sln in the respective "build" folder
(when use cmake to complie, you should create the "build" folder)

use the visual studio (2022) open the *.sln (run with administrator), and then find the "INSTALL" in the "Solution Exploer",
then right click->build

after installed the osqp and the OsqpEigen correctly, you will get the 2 folder in the C:/Program Files:
- C:\Program Files\osqp
- C:\Program Files\OsqpEigen

the final step:
open the visual studio->Local Windows Debugger->LatPlanAndCtl_wMPC Debug Properties->Debug->environment->PAth="C:\Program Files\OsqpEigen\bin"
and
go into LatPlanAndCtl_wMPC Debug Properties->C/C++->General
 ->Additional Include Directories ->Edit->type in: 
"C:\Program Files\osqp\include\osqp"
"C:\Program Files\OsqpEigen\include"
and
go into LatPlanAndCtl_wMPC Debug Properties->Linker->General->Additional Library Directories->Edit-> type in: 
"C:\Program Files\osqp\lib"
"C:\Program Files\OsqpEigen\lib"
and
go into LatPlanAndCtl_wMPC Debug Properties->Linker->input->Additional Dependencise->Edit-> type in:
"osqp.lib"
"qdldl.lib"
"OsqpEigend.lib"

note: at the LatPlanAndCtl_wMPC Property Pages: 
the Configuration->select "All Configuration"
the Platform->select "x64"
}

4.run and debug the program.

