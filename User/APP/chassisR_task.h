#ifndef __CHASSISR_TASK_H
#define __CHASSISR_TASK_H

#include "main.h"
#include "dm4310_drv.h"
#include "pid.h"
#include "VMC_calc.h"
#include "INS_task.h"

#define WHEEL_R 0.0675f;                             //chassis wheel Ratio 
#define MOTOR_RPM_TO_W   0.00349066f               // *2PI/60  /15

//������Ť��ת��Ϊ����ϵ�� 3508  -16384~16384 ��Ӧ������Χ-20A~20A ԭʼŤ��ϵ��0.3NM/A  ���ٱȸ�Ϊ15:1���Ϊ0.236842NM/A
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

#define TP_PID_KP 10.0f
#define TP_PID_KI 0.0f   //���û�����
#define TP_PID_KD 1.0f
#define TP_PID_MAX_OUT  2.0f
#define TP_PID_MAX_IOUT 0.0f

#define TURN_PID_KP 4.0f
#define TURN_PID_KI 0.0f //���û�����
#define TURN_PID_KD 0.4f
#define TURN_PID_MAX_OUT  1.0f//��챵���ĶŤ��
#define TURN_PID_MAX_IOUT 0.0f

typedef struct
{
  Joint_Motor_t joint_motor[4];
  chassis_motor_t wheel_motor[2];
	
	float v_set;//�����ٶȣ���λ��m/s
	float x_set;//����λ�ã���λ��m
	float turn_set;//����yaw�ỡ��
	float roll_set;
	float leg_set;//�����ȳ�����λ��m
	float last_leg_set;

	float v_filter;//�˲���ĳ����ٶȣ���λ��m/s
	float x_filter;//�˲���ĳ���λ�ã���λ��m
	
	float myPithR;
	float myPithGyroR;
	float myPithL;
	float myPithGyroL;
	float roll;
	float total_yaw;
	float theta_err;//���ȼн����
		
	float turn_T;//yaw�Ჹ��
	float roll_f0;//roll轴补偿
	float leg_tp;//�����油��
	
	uint8_t start_flag;//������־
	
	uint8_t jump_flag;//��Ծ��־
	uint8_t jump_flag2;//��Ծ��־
	
	uint8_t prejump_flag;//Ԥ��Ծ��־
	uint8_t recover_flag;//һ������µĵ��������־

} chassis_t;


extern void  ChassisR_init(chassis_t *chassis,vmc_leg_t *vmcr_init,vmc_leg_t *vmcl_init);
extern void ChassisR_task(void);
extern void Pensation_init(PidTypeDef *roll,PidTypeDef *Tp,PidTypeDef *turn,PidTypeDef *legr,PidTypeDef *legl);
extern void mySaturate(float *in,float min,float max);
extern void Limit_Int(int16_t *in,int16_t min,int16_t max);
extern void chassisR_feedback_update(chassis_t *chassis,vmc_leg_t *vmc_r,vmc_leg_t *vmc_l,INS_t *ins);
extern void chassisR_control_loop(chassis_t *chassis,vmc_leg_t *vmcr,vmc_leg_t *vmcl,INS_t *ins,float *LQR_K,float *LQR_Kl, PidTypeDef *legr,PidTypeDef *legl);

#endif




