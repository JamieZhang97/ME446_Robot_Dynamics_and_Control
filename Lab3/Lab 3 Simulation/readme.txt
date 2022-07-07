% 3-link manipulator simulation
% Author: Yanran Ding, Mengchao Zhang

TODO:

1. fcns/fcn_controller.m
Write your Inverse Dynamics Controller
 
2. fcns/fcn_gen_traj.m
Generate your fast trajectory 

3.fcns/fcn_controller.m
Tune the gains of PD controller for the fast trajectory

4.fcns/fcn_controller.m & MAIN 
Change these files to choose the right controller and parameter to fullfill the comparision
between PD controller and Inverse Dynamics Controller when a mass is added to the end effector
(You may also want to have a look of fcns/fcn_params.m, but there is nothing need to be changed)

NOTE:set "mass = true" in the top of MAIN.m if you are planning to use controller 6 or 7

If you want to plot something else, you can modify the code in fcns/animateRobot

!!!Please don't change any code not mentioned above unless you understand what you are doing.

INSTRUCTIONS:
- run the *MAIN.m* to initiate the simulation
- change *flag_ctrl* to select the desired controllers
	% 0 - workspace impedance control	
	% 1 - workspace inverse dynamics control
    % 2 - Fun Trajectory (Joint Space Feed Forward + PD)
- change *tfinal* to adjust the simulation time duration

NOTES:
a. include folders fcns and gen to get access to all functions
b. hover the cursor over the *function name* and press CTRL+D to open the function definition
c. The controller is implemented in *MAIN/dyn_manip/fcn_controller*
d. animation and plotting is in *animateRobot*
e. change flag_movie to 1 to make movie in mp4 format

