#include <tistdtypes.h>
#include <coecsl.h>
#include "user_includes.h"
#include "math.h"

// These two offsets are only used in the main file user_CRSRobot.c  You just need to create them here and find the correct offset and then these offset will adjust the encoder readings
float offset_Enc2_rad = -0.46;//-0.37;
float offset_Enc3_rad = 0.18;//0.27;


// Your global varialbes.  

long mycount = 0;

#pragma DATA_SECTION(whattoprint, ".my_vars")
float whattoprint = 0.0;

#pragma DATA_SECTION(whattoprint2, ".my_vars")
float whattoprint2 = 0.0;

#pragma DATA_SECTION(theta1array, ".my_arrs")
float theta1array[100];

#pragma DATA_SECTION(theta2array, ".my_arrs")
float theta2array[100];

#pragma DATA_SECTION(theta3array, ".my_arrs")
float theta3array[100];

long arrayindex = 0;

float printtheta1motor = 0;
float printtheta2motor = 0;
float printtheta3motor = 0;

float theta1DH = 0;
float theta2DH = 0;
float theta3DH = 0;

float theta1rev = 0;
float theta2rev = 0;
float theta3rev = 0;

float position_X = 0;
float position_Y = 0;
float position_Z = 0;
float position_L = 0;


// Assign these float to the values you would like to plot in Simulink
float Simulink_PlotVar1 = 0;
float Simulink_PlotVar2 = 0;
float Simulink_PlotVar3 = 0;
float Simulink_PlotVar4 = 0;


// This function is called every 1 ms
void lab(float theta1motor,float theta2motor,float theta3motor,float *tau1,float *tau2,float *tau3, int error) {


	*tau1 = 0;//base pos right / neg left
	*tau2 = 0.5;//shoulder pos down/
	*tau3 = 0;//elbow  pos down / neg up

	//Motor torque limitation(Max: 5 Min: -5)

	// save past states
	if ((mycount%50)==0) {

		theta1array[arrayindex] = theta1motor;
		theta2array[arrayindex] = theta2motor;
		theta3array[arrayindex] = theta3motor;

		if (arrayindex >= 100) {
			arrayindex = 0;
		} else {
			arrayindex++;
		}

	}

	if ((mycount%500)==0) {
		if (whattoprint > 0.5) {
			serial_printf(&SerialA, "I love robotics\n\r");
		} else {
			printtheta1motor = theta1motor*180/PI; //we want to see the angle printed in degree 
			printtheta2motor = theta2motor*180/PI;
			printtheta3motor = theta3motor*180/PI;
			theta1DH = theta1motor; //after uising the special value at some special position to solve the equation with the parameter a1, a2, a3, b1, b2, we can get the relationship between DH angle and motor angle
            theta2DH = theta2motor-PI/2;
            theta3DH = -theta2motor+theta3motor+PI/2;
            position_X = 10*cos(theta1motor)*(cos(theta3motor)+sin(theta2motor)); //forward kinematics, calculate the position of end-effector by joint angles
            position_Y = 10*sin(theta1motor)*(cos(theta3motor)+sin(theta2motor));
            position_Z = 10*(1+cos(theta2motor)-sin(theta3motor));
            position_L = sqrt(position_X*position_X+position_Y*position_Y+(position_Z-10)*(position_Z-10)); //calculate the horizonal distance between P1 and P3
            theta1rev = atan(position_Y/position_X); //inverse kinematics, calculate the angle of each joints given the position of end-effector
            theta2rev = -acos(position_L/20)-atan((position_Z-10)/sqrt(position_X*position_X+position_Y*position_Y));
            theta3rev = 2*acos(position_L/20);
			SWI_post(&SWI_printf); //Using a SWI to fix SPI issue from sending too many floats.
		}
	    //GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1; // Blink LED on Control Card
	    GpioDataRegs.GPBTOGGLE.bit.GPIO60 = 1; // Blink LED on Emergency Stop Box

		//GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1; // Turn on LED on Emergency Stop Box
		//GpioDataRegs.GPBSET.bit.GPIO60 = 1; // Turn off LED on Emergency Stop Box
	}


	Simulink_PlotVar1 = theta1motor;
	Simulink_PlotVar2 = theta2motor;
	Simulink_PlotVar3 = theta3motor;
	Simulink_PlotVar4 = 0;

	mycount++;
}

void printing(void){
	serial_printf(&SerialA, "Motor Angle:%.2f %.2f %.2f   \n\r",printtheta1motor,printtheta2motor,printtheta3motor);
	serial_printf(&SerialA, "Position:%.2f %.2f %.2f   \n\r",position_X, position_Y, position_Z);
	serial_printf(&SerialA, "DH angle:%.2f %.2f %.2f   \n\r",theta1DH, theta2DH, theta3DH);
	serial_printf(&SerialA, "Revangle:%.2f %.2f %.2f   \n\r",theta1rev,theta2rev, theta3rev);
}

