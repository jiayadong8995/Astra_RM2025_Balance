#include "VMC_calc.h"

void VMC_init(vmc_leg_t *vmc)//给杆长赋值
{
	vmc->l5=0.15f;//AE长度 //单位为m
	vmc->l1=0.15f;//单位为m
	vmc->l2=0.272f;//单位为m
	vmc->l3=0.272f;//单位为m
	vmc->l4=0.15f;//单位为m
}

void VMC_calc_1_right(vmc_leg_t *vmc, INS_t *ins, float dt) {
    static float PitchR = 0.0f;
    static float PithGyroR = 0.0f;

    PitchR = ins->Pitch;
    PithGyroR =ins->Gyro[0];

    vmc->YD = vmc->l4 * arm_sin_f32(vmc->phi4); // D的y坐标
    vmc->YB = vmc->l1 * arm_sin_f32(vmc->phi1); // B的y坐标
    vmc->XD = vmc->l5 + vmc->l4 * arm_cos_f32(vmc->phi4); // D的x坐标
    vmc->XB = vmc->l1 * arm_cos_f32(vmc->phi1); // B的x坐标

    vmc->lBD = sqrtf((vmc->XD - vmc->XB) * (vmc->XD - vmc->XB) + (vmc->YD - vmc->YB) * (vmc->YD - vmc->YB));

    vmc->A0 = 2 * vmc->l2 * (vmc->XD - vmc->XB);
    vmc->B0 = 2 * vmc->l2 * (vmc->YD - vmc->YB);
    vmc->C0 = vmc->l2 * vmc->l2 + vmc->lBD * vmc->lBD - vmc->l3 * vmc->l3;

    float sqrt_term = vmc->A0 * vmc->A0 + vmc->B0 * vmc->B0 - vmc->C0 * vmc->C0;
    if (sqrt_term < 0.0f) {
        sqrt_term = 0.0f; // 防止负数平方根
    }
    vmc->phi2 = 2 * atan2f((vmc->B0 + sqrtf(sqrt_term)), (vmc->A0 + vmc->C0));

    vmc->phi3 = atan2f(vmc->YB - vmc->YD + vmc->l2 * arm_sin_f32(vmc->phi2), vmc->XB - vmc->XD + vmc->l2 * arm_cos_f32(vmc->phi2));

    vmc->XC = vmc->l1 * arm_cos_f32(vmc->phi1) + vmc->l2 * arm_cos_f32(vmc->phi2);
    vmc->YC = vmc->l1 * arm_sin_f32(vmc->phi1) + vmc->l2 * arm_sin_f32(vmc->phi2);

    vmc->L0 = sqrtf((vmc->XC - vmc->l5 / 2.0f) * (vmc->XC - vmc->l5 / 2.0f) + vmc->YC * vmc->YC);
    if (isnan(vmc->L0)) {
        vmc->L0 =vmc->last_L0; // 处理L0为NaN的情况
    }

    vmc->phi0 = atan2f(vmc->YC, (vmc->XC - vmc->l5 / 2.0f)); // phi0用于计算lqr需要的theta
    vmc->alpha = pi / 2.0f - vmc->phi0;

    if (vmc->first_flag == 0) {
        vmc->last_phi0 = vmc->phi0;
        vmc->first_flag = 1;
        vmc->last_d_phi0 = 0.0f;
        vmc->last_L0 = vmc->L0;
        vmc->last_d_L0 = 0.0f;
        vmc->last_dd_L0 = 0.0f;
        vmc->last_d_theta = 0.0f;
        vmc->last_dd_theta = 0.0f;
    }

    if (dt != 0.0f) {
        vmc->d_phi0 = (vmc->phi0 - vmc->last_phi0) / dt; // 计算phi0变化率，d_phi0用于计算lqr需要的d_theta
    } else {
        vmc->d_phi0 = 0.0f; // 处理dt为零的情况
    }

    vmc->d_phi0 = vmc->d_phi0 * 0.5f + vmc->last_d_phi0 * 0.5f; // 对vmc->d_phi0进行一阶滤波
    vmc->last_d_phi0 = vmc->d_phi0;
    vmc->d_alpha = 0.0f - vmc->d_phi0;

    vmc->theta = pi / 2.0f - PitchR - vmc->phi0 -0.05f; // 得到状态变量1
    vmc->d_theta = (-PithGyroR - vmc->d_phi0); // 得到状态变量2

    vmc->last_phi0 = vmc->phi0;

    if (dt != 0.0f) {
        vmc->d_L0 = (vmc->L0 - vmc->last_L0) / dt; // 腿长L0的一阶导数
    } else {
        vmc->d_L0 = 0.0f; // 处理dt为零的情况
    }

    vmc->d_L0 = vmc->d_L0 * 0.5f + vmc->last_d_L0 * 0.5f; // 对vmc->d_L0进行一阶滤波

    if (dt != 0.0f) {
        vmc->dd_L0 = (vmc->d_L0 - vmc->last_d_L0) / dt; // 腿长L0的二阶导数
    } else {
        vmc->dd_L0 = 0.0f; // 处理dt为零的情况
    }

    vmc->dd_L0 = vmc->dd_L0 * 0.5f + vmc->last_dd_L0 * 0.5f; // 对vmc->dd_L0进行一阶滤波
    vmc->last_dd_L0 = vmc->dd_L0;

    vmc->last_d_L0 = vmc->d_L0;
    vmc->last_L0 = vmc->L0;

    if (dt != 0.0f) {
        vmc->dd_theta = (vmc->d_theta - vmc->last_d_theta) / dt;
    } else {
        vmc->dd_theta = 0.0f; // 处理dt为零的情况
    }

    vmc->dd_theta = vmc->dd_theta * 0.5f + vmc->last_dd_theta * 0.5f; // 对vmc->dd_theta进行一阶滤波
    vmc->last_dd_theta = vmc->dd_theta;
    vmc->last_d_theta = vmc->d_theta;
}


void VMC_calc_1_left(vmc_leg_t *vmc, INS_t *ins, float dt) {
    static float PitchL = 0.0f;
    static float PithGyroL = 0.0f;

    PitchL = 0.0f - ins->Pitch;
    PithGyroL = 0.0f - ins->Gyro[0];

    vmc->YD = vmc->l4 * arm_sin_f32(vmc->phi4); // D的y坐标
    vmc->YB = vmc->l1 * arm_sin_f32(vmc->phi1); // B的y坐标
    vmc->XD = vmc->l5 + vmc->l4 * arm_cos_f32(vmc->phi4); // D的x坐标
    vmc->XB = vmc->l1 * arm_cos_f32(vmc->phi1); // B的x坐标

    vmc->lBD = sqrtf((vmc->XD - vmc->XB) * (vmc->XD - vmc->XB) + (vmc->YD - vmc->YB) * (vmc->YD - vmc->YB));

    vmc->A0 = 2 * vmc->l2 * (vmc->XD - vmc->XB);
    vmc->B0 = 2 * vmc->l2 * (vmc->YD - vmc->YB);
    vmc->C0 = vmc->l2 * vmc->l2 + vmc->lBD * vmc->lBD - vmc->l3 * vmc->l3;

    float sqrt_term = vmc->A0 * vmc->A0 + vmc->B0 * vmc->B0 - vmc->C0 * vmc->C0;
    if (sqrt_term < 0.0f) {
        sqrt_term = 0.0f; // 防止负数平方根
    }
    vmc->phi2 = 2 * atan2f((vmc->B0 + sqrtf(sqrt_term)), (vmc->A0 + vmc->C0));

    vmc->phi3 = atan2f(vmc->YB - vmc->YD + vmc->l2 * arm_sin_f32(vmc->phi2), vmc->XB - vmc->XD + vmc->l2 * arm_cos_f32(vmc->phi2));

    vmc->XC = vmc->l1 * arm_cos_f32(vmc->phi1) + vmc->l2 * arm_cos_f32(vmc->phi2);
    vmc->YC = vmc->l1 * arm_sin_f32(vmc->phi1) + vmc->l2 * arm_sin_f32(vmc->phi2);

    vmc->L0 = sqrtf((vmc->XC - vmc->l5 / 2.0f) * (vmc->XC - vmc->l5 / 2.0f) + vmc->YC * vmc->YC);
    if (isnan(vmc->L0)) {
        vmc->L0 = vmc->last_L0; // 处理L0为NaN的情况
    }

    vmc->phi0 = atan2f(vmc->YC, (vmc->XC - vmc->l5 / 2.0f)); // phi0用于计算lqr需要的theta
    vmc->alpha = pi / 2.0f - vmc->phi0;

    if (vmc->first_flag == 0) {
        vmc->last_phi0 = vmc->phi0;
        vmc->first_flag = 1;
    }

    if (dt != 0.0f) {
        vmc->d_phi0 = (vmc->phi0 - vmc->last_phi0) / dt; // 计算phi0变化率，d_phi0用于计算lqr需要的d_theta
    } else {
        vmc->d_phi0 = 0.0f; // 处理dt为零的情况
    }

    vmc->d_alpha = 0.0f - vmc->d_phi0;

    vmc->theta = pi / 2.0f - PitchL - vmc->phi0 + 0.05f; // 得到状态变量1
    vmc->d_theta = (-PithGyroL - vmc->d_phi0); // 得到状态变量2

    vmc->last_phi0 = vmc->phi0;

    if (dt != 0.0f) {
        vmc->d_L0 = (vmc->L0 - vmc->last_L0) / dt; // 腿长L0的一阶导数
    } else {
        vmc->d_L0 = 0.0f; // 处理dt为零的情况
    }

    if (dt != 0.0f) {
        vmc->dd_L0 = (vmc->d_L0 - vmc->last_d_L0) / dt; // 腿长L0的二阶导数
    } else {
        vmc->dd_L0 = 0.0f; // 处理dt为零的情况
    }

    vmc->last_d_L0 = vmc->d_L0;
    vmc->last_L0 = vmc->L0;

    if (dt != 0.0f) {
        vmc->dd_theta = (vmc->d_theta - vmc->last_d_theta) / dt;
    } else {
        vmc->dd_theta = 0.0f; // 处理dt为零的情况
    }

    vmc->last_d_theta = vmc->d_theta;
}


void VMC_calc_2(vmc_leg_t *vmc)//计算期望的关节输出力矩
{
		vmc->j11 = (vmc->l1*arm_sin_f32(vmc->phi0-vmc->phi3)*arm_sin_f32(vmc->phi1-vmc->phi2))/arm_sin_f32(vmc->phi3-vmc->phi2);
		vmc->j12 = (vmc->l1*arm_cos_f32(vmc->phi0-vmc->phi3)*arm_sin_f32(vmc->phi1-vmc->phi2))/(vmc->L0*arm_sin_f32(vmc->phi3-vmc->phi2));
		vmc->j21 = (vmc->l4*arm_sin_f32(vmc->phi0-vmc->phi2)*arm_sin_f32(vmc->phi3-vmc->phi4))/arm_sin_f32(vmc->phi3-vmc->phi2);
		vmc->j22 = (vmc->l4*arm_cos_f32(vmc->phi0-vmc->phi2)*arm_sin_f32(vmc->phi3-vmc->phi4))/(vmc->L0*arm_sin_f32(vmc->phi3-vmc->phi2));
	
		vmc->torque_set[0]=vmc->j11*vmc->F0+vmc->j12*vmc->Tp;//得到RightFront的输出轴期望力矩，F0为五连杆机构末端沿腿的推力 
		vmc->torque_set[1]=vmc->j21*vmc->F0+vmc->j22*vmc->Tp;//得到RightBack的输出轴期望力矩，Tp为沿中心轴的力矩 

}

uint8_t ground_detectionR(vmc_leg_t *vmc,INS_t *ins)
{
	vmc->FN=vmc->F0*arm_cos_f32(vmc->theta)+vmc->Tp*arm_sin_f32(vmc->theta)/vmc->L0+6.0f;
	//腿部机构的力+轮子重力，这里忽略了轮子质量*驱动轮竖直方向运动加速度
	if(vmc->FN<10.0f)
	{//离地了
	  return 1;
	}
	else
	{
	  return 0;	
	}
}

uint8_t ground_detectionL(vmc_leg_t *vmc,INS_t *ins)
{
	vmc->FN=vmc->F0*arm_cos_f32(vmc->theta)+vmc->Tp*arm_sin_f32(vmc->theta)/vmc->L0+6.0f;
	//腿部机构的力+轮子重力，这里忽略了轮子质量*驱动轮竖直方向运动加速度
	if(vmc->FN<10.0f)
	{//离地了
	  return 1;
	}
	else
	{
	  return 0;	
	}
}

float LQR_K_calc(float *coe,float len)
{
   
  return coe[0]*len*len*len+coe[1]*len*len+coe[2]*len+coe[3];
}


