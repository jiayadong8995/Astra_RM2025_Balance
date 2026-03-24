#ifndef __CHASSIS_TASK_H
#define __CHASSIS_TASK_H

#include "main.h"
#include "dm4310_drv.h"
#include "pid.h"
#include "VMC_calc.h"
#include "INS_task.h"

#define WHEEL_R 0.0675f;                             //chassis wheel Ratio 
#define MOTOR_RPM_TO_W   0.00349066f               // *2PI/60  /15


//m3508  motor torque to current change
//I = T/0.0236842A
//c=I /20 * 16384 
#define CHASSIS_MOTOR_TORQUE_TO_CURRENT 3458.84f

#define T_WHEEL_RATIO  1.0f

#define ROLL_PID_KP 2.0f
#define ROLL_PID_KI 0.0f 
#define ROLL_PID_KD 1.0f
#define ROLL_PID_MAX_OUT  80.0f
#define ROLL_PID_MAX_IOUT 0.0f

#define TP_PID_KP 9.0f//8 //10
#define TP_PID_KI 0.0f   
#define TP_PID_KD 1.6f
#define TP_PID_MAX_OUT  2.0f
#define TP_PID_MAX_IOUT 0.0f

#define TURN_PID_KP 4.0f
#define TURN_PID_KI 0.0f 
#define TURN_PID_KD 0.4f
#define TURN_PID_MAX_OUT  1.0f
#define TURN_PID_MAX_IOUT 0.0f

typedef struct
{
	Joint_Motor_t joint_motor[4];
	chassis_motor_t wheel_motor[2];
	
	float v_set;//期望速度，单位是m/s
	float x_set;//期望位置，单位是m
	float turn_set;//期望yaw轴弧度
	float roll_set;
	float leg_set;//期望腿长，单位是m
	float last_leg_set;

	float v_filter;//滤波后的车体速度，单位是m/s
	float x_filter;//滤波后的车体位置，单位是m
	
	float myPithR;
	float myPithGyroR;
	float myPithL;
	float myPithGyroL;
	float roll;
	float total_yaw;
	float theta_err;//两腿夹角误差
		
	float turn_T;//yaw轴补偿
	float roll_f0;//roll轴补偿
	float leg_tp;//防劈叉补偿
	
	uint8_t start_flag;//启动标志
	
	uint8_t jump_flag;//跳跃标志
	float jump_leg;
	uint32_t jump_time;
	uint8_t jump_status;	
	
	uint8_t last_recover_flag;
	uint8_t recover_flag;//一种情况下的倒地自起标志
	uint8_t text_jump_true;

} chassis_t;


void Chassis_task(void);
void mySaturate(float *in,float min,float max);

#endif




