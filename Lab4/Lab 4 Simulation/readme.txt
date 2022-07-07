% 3-link manipulator simulation
% Author: Yanran Ding

INSTRUCTIONS:
- run the *MAIN.m* to initiate the simulation
- change *flag_ctrl* to select the desired controllers
    % 0 - Task Space PD control
    % 1 - Task Space Impedence control (one or two axes weakened)
    % 2 - Task Space Impedence control (with Rotation)
    % 3 - Straight Line Following 
    % 4 - Straight Line Following (with Rotation)

TODO:
1. fcn_gen_traj.m
   Design the trajectory to let the robot hold at one x,y,z point
2. fcn_controller.m
   Design the Task Space PD controller
3. fcn_controller.m
   Design the Task Space PD controller with one or two axes weakened
4. fcn_controller.m
   Design the Task Space PD control with rotated X axis in frame N (rotate pi/4 about world Z axix to get frame N)
5. fcn_gen_traj.m
   Design the straight line trajectory
6. fcn_controller.m
   Design the controller to finish the Straight Line Following

- change *tfinal* to adjust the simulation time duration

NOTES:
a. include folders fcns and gen to get access to all functions
b. hover the cursor over the *function name* and press CTRL+D to open the function definition
c. The controller is implemented in *MAIN/dyn_manip/fcn_controller*
d. animation and plotting is in *animateRobot*
e. change flag_movie to 1 to make movie in mp4 format

