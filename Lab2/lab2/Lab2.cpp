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

// Assign these float to the values you would like to plot in Simulink
float Simulink_PlotVar1 = 0;
float Simulink_PlotVar2 = 0;
float Simulink_PlotVar3 = 0;
float Simulink_PlotVar4 = 0;

float theta1D=0;
float theta2D=0;
float theta3D=0;

float Theta1_old=0;
float Omega1_old2=0;
float Omega1_old1=0;
float Omega1_raw=0;
float Omega1=0;
float Theta2_old=0;
float Omega2_old2=0;
float Omega2_old1=0;
float Omega2_raw=0;
float Omega2=0;
float Theta3_old=0;
float Omega3_old2=0;
float Omega3_old1=0;
float Omega3_raw=0;
float Omega3=0;

float Kp1=80;
float Kp2=80;
float Kp3=80;
float Kd1=3;
float Kd2=3;
float Kd3=3;
float t1=0;
float t2=0;
float t3=0;
//float Ki1=5;
//float Ki2=10;
//float Ki3=12;
float Ki1=0;
float Ki2=0;
float Ki3=0;
float ik1=0;
float ik2=0;
float ik3=0;
float e1old = 0;
float e1 = 0;
float e2old = 0;
float e2=0;
float e3old=0;
float e3=0;

//float j1=0.0167;
//float j2=0.03;
//float j3=0.0128;

float j1=0.0;
float j2=0.0;
float j3=0.0;

float t=0;

float td0_1 = 0;
float td1_1 = 0;
float td2_1 = 0;
float td0_2 = 0;
float td1_2 = 0;
float td2_2 = 0;
float td0_3 = 0;
float td1_3 = 0;
float td2_3 = 0;
float x_pos = 0;
float y_pos = 0;
float z_pos = 0;
float l_pos = 0;
float theta0_1;
float theta0_2;
float theta0_3;

int flag = 0;
// This function is called every 1 ms
void lab(float theta1motor,float theta2motor,float theta3motor,float *tau1,float *tau2,float *tau3, int error) {
    if ((mycount%500)==0)
    {
        GpioDataRegs.GPBTOGGLE.bit.GPIO60=1; //blink the light 
    }
//trajectory of joint1
    t=mycount/1000.0;
    /*  fun trajectory do not using inverse kinematics, which is not elegent enough
    if(t<=2)
    {
        td0_1 = 0.15*t*t-0.05*t*t*t;
        td1_1 = 0.3*t-0.15*t*t;
        td2_1 = 0.3-0.3*t;
    }
    else if(t<=6)
    {
        td0_1=-0.2+0.45*t-0.15*t*t+0.0125*t*t*t;
        td1_1=0.45-0.3*t+0.0375*t*t;
        td2_1=-0.3+0.075*t;
    }
    else if(t<=8)
    {
        td0_1=16-7.2*t+1.05*t*t-0.05*t*t*t;
        td1_1=-7.2+2.1*t-0.15*t*t;
        td2_1=2.1-0.3*t;
    }
    else
    {
        td0_1=0;
        td1_1=0;
        td2_1=0;
    }
    */
    x_pos = 15;                   //fun trajectory
    y_pos = 2*sin(t/8*2*PI);   
    z_pos = 15+2*cos(t/8*2*PI);
    
	l_pos = sqrt(x_pos*x_pos+y_pos*y_pos+(z_pos-10)*(z_pos-10));  //to simpilify, use a middle parameter

    theta0_1=atan(y_pos/x_pos);          //calculate motor angle use inverse kinematics
    theta0_2=-acos(l_pos/20)-atan((z_pos-10)/sqrt(x_pos*x_pos+y_pos*y_pos));
    theta0_3=2*acos(l_pos/20);

    td0_1=theta0_1;            //transition between DH angle and motor angle
    td0_2=theta0_2+PI/2.0;
    td0_3=theta0_3+theta0_2;

	//Motor torque limitation(Max: 5 Min: -5)
	//Here we will calculate angular velocity for each of the three joints, so I will comment only one paragraph
    //approximate the angular velocity now by using the average angular velocity in one time step 1ms
	Omega1_raw = (theta1motor - Theta1_old)/0.001;
	//To make the angular velocity more accurate, we use the average of the latest three ones     
    Omega1 = (Omega1_raw + Omega1_old1 + Omega1_old2)/3.0;
     //to iterate, we store the motoe angle now as the previous one
    Theta1_old = theta1motor;
	//also the latest and second latest angular velocity stored as the second latest and the third latest
    Omega1_old2 = Omega1_old1;
    Omega1_old1 = Omega1;

    e1=td0_1-theta1motor;    //track error
    ik1=ik1+(e1old+e1)/2.0*0.001;  //integral of each step in I control 
    e1old=e1;     //renew the error
    if(ik1>1||ik1<-1)    //restrict the error with limit, in case the torque is too much
    {
        ik1=0;
    }

	Omega2_raw = (theta2motor - Theta2_old)/0.001;
	Omega2 = (Omega2_raw + Omega2_old1 + Omega2_old2)/3.0;
	Theta2_old = theta2motor;

	Omega2_old2 = Omega2_old1;
	Omega2_old1 = Omega2;

    e2=td0_2-theta2motor;
    ik2=ik2+(e2old+e2)/2.0*0.001;
    e2old=e2;
    if(ik2>1||ik2<-1)
    {
        ik2=0;
    }
    
	//approximate the angular velocity now by using the average angular velocity in one time step 1ms
    Omega3_raw = (theta3motor - Theta3_old)/0.001;
	//To make the angular velocity more accurate, we use the average of the latest three ones     
    Omega3 = (Omega3_raw + Omega3_old1 + Omega3_old2)/3.0; 
    //to iterate, we store the motoe angle now as the previous one
	Theta3_old = theta3motor;   
	//also the latest and second latest angular velocity stored as the second latest and the third latest
	Omega3_old2 = Omega3_old1;  
    Omega3_old1 = Omega3;                    

    e3=td0_3-theta3motor;
    ik3=ik3+(e3old+e3)/2.0*0.001;
    e3old=e3;
    
	if(ik3>1||ik3<-1)
    {
        ik1=0;
    }
	//the control function of each joint
	//Kp1*(theta1D-theta1motor)-Kd1*Omega1+Ki1*ik1;
	t1 = j1*td2_1+Kp1*(td0_1-theta1motor)+Ki1*ik1+Kd1*(td1_1-Omega1);
	        //Kp1*(theta1D-theta1motor)-Kd1*Omega1+Ki1*ik1;
	//restrict the torque in case protective pause
	if(t1>4.9)
	{
	    t1=4.9;
	}
	else if(t1<-4.9)
	{
	    t1=-4.9;
	}

	t2 = j2*td2_2+Kp2*(td0_2-theta2motor)+Ki2*ik2+Kd2*(td1_2-Omega2);
	if(t2>4.9)
	{
	    t2=4.9;
	}
	else if(t2<-4.9)
	{
	    t2=-4.9;
	}

	t3 = j3*td2_3+Kp3*(td0_3-theta3motor)+Ki3*ik3+Kd3*(td1_3-Omega3);
	if(t3>4.9)
	{
	    t3=4.9;
	}
    else if(t3<-4.9)
    {
        t3=-4.9;
    }


	*tau1 =t1;//base pos right / neg left
	*tau2 =t2;
	*tau3 =t3;

	Simulink_PlotVar1 = theta2D;        //transite vcariable to simulink to present
	Simulink_PlotVar2 = theta1motor;
	Simulink_PlotVar3 = theta2motor;
	Simulink_PlotVar4 = theta3motor;

	mycount=(mycount+1)%8000;  //move cycle
}

void printing(void){

}


