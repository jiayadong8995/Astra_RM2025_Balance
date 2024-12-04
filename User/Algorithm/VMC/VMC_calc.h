#ifndef __VMC_CALC_H
#define __VMC_CALC_H

#include "main.h"
#include "INS_task.h"

#define pi 3.1415926f
#define LEG_PID_KP  280.0f                                 //180
#define LEG_PID_KI  0.0f//
#define LEG_PID_KD  4000.0f                                 //2500
#define LEG_PID_MAX_OUT  50.0f //90N
#define LEG_PID_MAX_IOUT 12.0f


typedef struct  
{
	/*�������ȵĹ����������̶�����*/
	float l5;//AE���� //��λΪm
	float	l1;//��λΪm
	float l2;//��λΪm
	float l3;//��λΪm
	float l4;//��λΪm
	
	float XB,YB;//B�������
	float XD,YD;//D�������
	
	float XC,YC;//C���ֱ������
	float L0,phi0;//C��ļ�����
	float alpha;
	float d_alpha;	
	
	float	lBD;//BD����ľ���
	
	float d_phi0;//����C��Ƕ�phi0�ı任��
	float last_phi0;//��һ��C��Ƕȣ����ڼ���Ƕ�phi0�ı任��d_phi0

	float A0,B0,C0;//�м����
	float phi2,phi3;
	float phi1,phi4;
	
	float j11,j12,j21,j22;//�ѿ����ռ������ؽڿռ�������ſɱȾ���ϵ��
	float torque_set[2];

	float F0;
	float Tp;
	
	float theta;
	float d_theta;//theta��һ�׵���
	float last_d_theta;
	float dd_theta;//theta�Ķ��׵���
	
	float d_L0;//L0��һ�׵���
	float dd_L0;//L0�Ķ��׵���
	float last_L0;
	float last_d_L0;
	float last_d_phi0;
	float last_dd_theta,last_dd_L0;
	
	float FN;//֧����
	
	uint8_t first_flag;
	uint8_t leg_flag;//�ȳ���ɱ�־
} vmc_leg_t;

extern void VMC_init(vmc_leg_t *vmc);//????????

extern void VMC_calc_1_right(vmc_leg_t *vmc,INS_t *ins,float dt);//????theta??d_theta??lqr?????????????L0
extern void VMC_calc_1_left(vmc_leg_t *vmc,INS_t *ins,float dt);
extern void VMC_calc_2(vmc_leg_t *vmc);//???????????????????

extern uint8_t ground_detectionR(vmc_leg_t *vmc,INS_t *ins);//?????????
extern uint8_t ground_detectionL(vmc_leg_t *vmc,INS_t *ins);//?????????

extern float LQR_K_calc(float *coe,float len);
	
#endif




