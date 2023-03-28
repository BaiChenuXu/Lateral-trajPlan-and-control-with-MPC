# Lateral-trajPlan-and-control-with-MPC
This demo is surbodinate to the ADAS.

It show how to conduct the lateral trajectory planning from the current lateral deviation
to the lane center after a certain distance, and
how to use MPC to control vehicle track the planning trajectory.

The program organization of this demo:
1) get the discret lane center (3 order polynomia)
2) get the planning trajectory (5 order polynomia)
3) get the projection of current vehicle pose to the trajectory
    and calculate the control error
4) get optmizition control value with MPC
5) update the vehicle state with lateral vehicle dynamic model 
   of linear and 2 degree of freedom

Currently, this demo run on the windows system and visual studio IDE.

This demo use the self-defined matlab function to show the simulation
animation. You could open the simuAnimation.mp4 firstly to see
the normal run result before runing the program by yourself.

For more information, please read the README.txt in the package.
