/**
  *********************************************************************
  * @file      chassisR_task.c/h
  * @brief     该任务控制右半部分的电机，分别是两个DM4310和一个DM6215，这三个电机挂载在can1总线上
	*						 从底盘上往下看，右上角的DM4310发送id为6、接收id为3，
	*						 右下角的DM4310发送id为8、接收id为4，
	*						 右边DM轮毂电机发送id为1、接收id为0。
  * @note       
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */
	
	
#include "chassisR_task.h"
#include "fdcan.h"
#include "cmsis_os.h"
#include "can_bsp.h"

// float LQR_K_R[12] = { -10.2956 ,  -0.8519 ,  -5.8988  , -3.2567 ,  14.8631   , 1.9321,        //Q=diag([1 0.1 100 1 400 1]);    R=[1 0;0 0.1];   x收敛比较慢
//   19.8404  ,  2.084  , 20.5351 ,  10.4754  , 35.3454  ,  2.026};   

//float LQR_K_R[12]={ -10.4789  , -0.8803 ,  -6.3869 ,  -3.4437  , 14.9392  ,  1.9456,
//            20.6843  ,  2.7071  , 24.1439 ,  13.3403  , 38.0184  ,  2.6012};

	
//float LQR_K_R[12] = {  -11.9379  , -1.0596 ,  -6.9580  , -4.3912  , 14.6212  ,  2.0554,
//   19.8505 ,   2.5835  , 24.7098  , 13.7964 ,  31.5161 ,   2.2462};               

	 
 
//float LQR_K_R[12] = {  -11.9745 ,  -1.0638 ,  -7.3099  , -4.5178  , 14.9500  ,  2.0731,
//   19.7793  ,  2.4753  , 24.6119  , 14.7456  , 32.4884  ,  2.2881};  //FTC  //B等级

//float LQR_K_R[12] = {-11.6053   ,-1.0195 ,  -7.3914  ,-4.3487  , 19.3667  ,  2.8325,
//   39.8714  ,  5.1642  , 53.9197 ,  27.8288  , 61.0153    4.6712};  //抖但是很跟
	 
//	float LQR_K_R[12] = { -11.1150 ,  -1.0045  , -7.3961  , -4.5927  , 15.8541 ,   2.1941,
//   19.8876   , 2.6035   ,24.6833   ,13.7738,   38.5416   , 2.9966}; //B等级
	
//  float LQR_K_R[12] = {	   -11.3457 ,  -1.1203 ,  -7.5128  , -4.6712  , 15.6784   , 2.1718,
//   20.9443 ,   2.4356  , 24.4303  , 13.6690 ,  36.0841  ,  3.0689}; 
////	 
 float LQR_K_R[12]= {  -12.7333,  -1.2330,  -7.5425,  -4.2937,  12.1349,  1.7394,  
     21.8901,  2.9805,  25.1217,  12.7182,  36.4339,  2.9270}; 

//	 float LQR_K_R[12] = {   -9.9259 ,  -0.8755  , -6.5742  , -3.6314 ,  14.6050 ,   2.0019,
// 19.3083 ,   2.4720  , 25.9726  , 12.7464 ,  40.6651 ,   3.0685};
	 
// float LQR_K_R[12] = { -10.0796 ,  -0.7360  , -5.9466  , -3.4557  , 14.4533  ,  1.9628,
//   18.9697   , 1.9722 ,  23.8148  , 12.1939 , 27.7618 ,   1.8519};

//-13.9730,  -1.6324,  -10.7746,  -6.1374,  15.8659,  2.1031,  
//14.8197,  2.2628,  18.3203,  9.3660,  37.3242,  2.7685

// -14.1476,  -1.4480,  -8.6735,  -4.9782,  11.2627,  1.5513,  
//17.1437,  2.2742,  18.5576,  9.5402,  40.4924,  3.1535


//    float LQR_K_R[12] = { -10.8274  , -0.8838  , -7.1019  , -4.0347  , 14.0390 ,   1.8584,
//    18.1394   , 1.9922 ,  23.1322 ,  11.6385  , 33.1553  ,  2.2341};  // 
	 

//三次多项式拟合系数

 float Poly_Coefficient[12][4]={{-163.4468,181.9159,-102.8506,0.4540},
																{7.2204,-7.6577,-7.4142,0.2838},
																{-263.4673,259.1873,-91.3168,1.3282},
																{-103.0651,104.3180,-42.1816,0.1095},
																{-48.1727,114.9231,-80.8660,23.2305},
																{-2.2523,9.6531,-8.1803,2.8198},
																{310.2912,-230.5733,35.0399,16.8861},
																{52.4124,-49.9754,17.1265,0.4292},
																{-79.3744,189.3593,-133.2433,38.2771},
																{-74.7240,115.5978,-67.6325,19.0430},
																{1229.9965,-1210.0154,426.3123,-6.2009},
																{143.9805,-143.1623,51.4354,-2.5578}};
vmc_leg_t right;
vmc_leg_t left;

extern INS_t INS;

int enable_flag[4]= {1,1,1,1};
int Begin_flag = 0;
uint8_t left_flag = 0;
uint8_t right_flag=0;
float leg_G = 10.0f;
float Joint_T_Max = 2.8f;
																
chassis_t chassis_move;
															
PidTypeDef LegR_Pid;//右腿的腿长pd
PidTypeDef LegL_Pid;//左腿的腿长pd
PidTypeDef Tp_Pid;//防劈叉补偿pd
PidTypeDef Turn_Pid;//转向pd
PidTypeDef Roll_Pid;//横滚角补偿pd

uint32_t CHASSR_TIME=2;				
void ChassisR_task(void)
{
	while(INS.ins_flag==0)
	{//等待加速度收敛
	  osDelay(1);	
	}

  ChassisR_init(&chassis_move,&right,&left);//初始化右边两个关节电机和右边轮毂电机的id和控制模式、初始化腿部
  Pensation_init(&Roll_Pid,&Tp_Pid,&Turn_Pid,&LegR_Pid,&LegL_Pid);//补偿pid初始化
  osDelay(6);

	while(1)
	{	
		chassisR_feedback_update(&chassis_move,&right,&left,&INS);//更新数据	
	  chassisR_control_loop(&chassis_move,&right,&left,&INS,LQR_K_R,LQR_K_R,&LegR_Pid,&LegL_Pid);//控制计算

		osDelay(CHASSR_TIME);
	}
}


void ChassisR_init(chassis_t *chassis,vmc_leg_t *vmcr_init,vmc_leg_t *vmcl_init)
{
	// joint_motor_init(&chassis->joint_motor[0],1,MIT_MODE);//发送id为1,控制模式 MIT
	// joint_motor_init(&chassis->joint_motor[1],2,MIT_MODE);//发送id为2,控制模式 MIT
	// joint_motor_init(&chassis->joint_motor[2],3,MIT_MODE);//发送id为3,控制模式 MIT
	// joint_motor_init(&chassis->joint_motor[3],4,MIT_MODE);//发送id为4,控制模式 MIT
	
	for(int i = 0;i < 2; i++){
	 chassis_move.wheel_motor[i].chassis_x = 0.0f;
	 chassis_move.wheel_motor[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
	}  

	 VMC_init(vmcr_init);//给杆长赋值
	 VMC_init(vmcl_init);//给杆长赋值

	 chassisR_feedback_update(&chassis_move,&right,&left,&INS);//更新数据

	 Begin_flag =1;
	
   //电机使能 motor_enable
	//  enable_flag[0] = enable_motor_mode(&hfdcan1,chassis->joint_motor[0].para.id,chassis->joint_motor[0].mode);
	//  osDelay(1);
	//  enable_flag[1] = enable_motor_mode(&hfdcan1,chassis->joint_motor[1].para.id,chassis->joint_motor[1].mode);
	//  osDelay(1);
	//  enable_flag[2] = enable_motor_mode(&hfdcan1,chassis->joint_motor[2].para.id,chassis->joint_motor[2].mode);
	//  osDelay(1);
	//  enable_flag[3] = enable_motor_mode(&hfdcan1,chassis->joint_motor[3].para.id,chassis->joint_motor[3].mode);
	//  osDelay(1);
}

void Pensation_init(PidTypeDef *roll,PidTypeDef *Tp,PidTypeDef *turn,PidTypeDef *legr,PidTypeDef *legl)
{
	//腿长pid初始化
	const static float roll_pid[3] = {ROLL_PID_KP,ROLL_PID_KI,ROLL_PID_KD};
    const static float legr_pid[3] = {LEG_PID_KP, LEG_PID_KI,LEG_PID_KD};
    const static float legl_pid[3] = {LEG_PID_KP, LEG_PID_KI,LEG_PID_KD};
    //补偿pid初始化：防劈叉补偿、偏航角补偿
	const static float tp_pid[3] = {TP_PID_KP, TP_PID_KI, TP_PID_KD};
	const static float turn_pid[3] = {TURN_PID_KP, TURN_PID_KI, TURN_PID_KD};
	
    PID_init(roll, PID_POSITION, roll_pid, ROLL_PID_MAX_OUT, ROLL_PID_MAX_IOUT);
	PID_init(Tp, PID_POSITION, tp_pid, TP_PID_MAX_OUT,TP_PID_MAX_IOUT);
	PID_init(turn, PID_POSITION, turn_pid, TURN_PID_MAX_OUT, TURN_PID_MAX_IOUT);

	PID_init(legr, PID_POSITION,legr_pid, LEG_PID_MAX_OUT, LEG_PID_MAX_IOUT);//腿长pid
	PID_init(legl, PID_POSITION,legl_pid, LEG_PID_MAX_OUT, LEG_PID_MAX_IOUT);//腿长pid
}

void chassisR_feedback_update(chassis_t *chassis,vmc_leg_t *vmc_r,vmc_leg_t *vmc_l,INS_t *ins)
{
    vmc_r->phi1=pi/2.0f+chassis->joint_motor[0].para.pos;
	vmc_r->phi4=pi/2.0f+chassis->joint_motor[1].para.pos+0.03f;
	vmc_l->phi1=pi/2.0f+chassis->joint_motor[2].para.pos-0.14f;
	vmc_l->phi4=pi/2.0f+chassis->joint_motor[3].para.pos;

	chassis_move.wheel_motor[0].chassis_x = chassis_move.wheel_motor[0].chassis_motor_measure->total_angle /15.0f * WHEEL_R;		
	chassis_move.wheel_motor[1].chassis_x = chassis_move.wheel_motor[1].chassis_motor_measure->total_angle /15.0f * WHEEL_R;

	chassis_move.wheel_motor[0].w_speed = chassis_move.wheel_motor[0].chassis_motor_measure->speed_rpm * MOTOR_RPM_TO_W; 		
	chassis_move.wheel_motor[1].w_speed = chassis_move.wheel_motor[1].chassis_motor_measure->speed_rpm * MOTOR_RPM_TO_W; 
	
	chassis_move.wheel_motor[0].speed = chassis_move.wheel_motor[0].w_speed * WHEEL_R;		
	chassis_move.wheel_motor[1].speed = chassis_move.wheel_motor[1].w_speed * WHEEL_R;		

    chassis->myPithGyroL = - ins->Gyro[0];
	chassis->myPithL = - ins->Pitch-0.05f;
	chassis->myPithR = ins->Pitch+0.05f;
	chassis->myPithGyroR = ins->Gyro[0];
	
	chassis->total_yaw = ins->YawTotalAngle;
	chassis->roll = ins->Roll;
	chassis->theta_err = 0.0f-(vmc_r->theta+vmc_l->theta);
	
//	if(ins->Pitch<(3.1415926f/20.0f)&&ins->Pitch>(-3.1415926f/20.0f))
//	{//根据pitch角度判断倒地自起是否完成
//		chassis->recover_flag=0;
//	}
}

void chassisR_control_loop(chassis_t *chassis,vmc_leg_t *vmcr,vmc_leg_t *vmcl,INS_t *ins,float *LQR_K,float *LQR_Kl, PidTypeDef *legr,PidTypeDef *legl)
{
	VMC_calc_1_right(vmcr,ins,2.0f/1000.0f);//计算theta和d_theta给lqr用，同时也计算右腿长L0,该任务控制周期是4*0.001秒
	VMC_calc_1_left(vmcl,ins,2.0f/1000.0f);//计算theta和d_theta给lqr用，同时也计算左腿长L0,该任务控制周期是4*0.001秒
	
//	for(int i=0;i<12;i++)
//	{
//		LQR_K[i]=LQR_K_calc(&Poly_Coefficient[i][0],vmcr->L0 );	
//	}
//	
//		for(int i=0;i<12;i++)
//	{
//		LQR_Kl[i]=LQR_K_calc(&Poly_Coefficient[i][0],vmcl->L0 );	
//	}
//		
	//chassis->turn_T=PID_Calc(&Turn_Pid, chassis->total_yaw, chassis->turn_set);//yaw轴pid计算     //屎
	
    chassis->turn_T=Turn_Pid.Kp*(chassis->turn_set-chassis->total_yaw)-Turn_Pid.Kd*ins->Gyro[2];//这样计算更稳一点
	
	chassis->roll_f0=Roll_Pid.Kp*(chassis->roll_set-chassis->roll)-Roll_Pid.Kd*ins->Gyro[1];
	
    chassis->leg_tp = PID_Calc(&Tp_Pid, chassis->theta_err,0.0f);//防劈叉pid计算

	chassis->wheel_motor[1].torque_set=(LQR_K[0]*(vmcr->theta -0.0f) 
																	+LQR_K[1]*(vmcr->d_theta -0.0f) 
																	+LQR_K[2]*(chassis->x_filter - chassis->x_set)
																	+LQR_K[3]*(chassis->v_filter - chassis->v_set)
																	+LQR_K[4]*(chassis->myPithR-0.0f)
																	+LQR_K[5]*(chassis->myPithGyroR-0.0f));					
   //右边髋关节输出力矩				
	vmcr->Tp=(LQR_K[6]*(vmcr->theta)
					+LQR_K[7]*(vmcr->d_theta)
		            +LQR_K[8]*( chassis->x_filter - chassis->x_set) 
					+LQR_K[9]*( chassis->v_filter - chassis->v_set) 
					+LQR_K[10]*(chassis->myPithR)
					+LQR_K[11]*(chassis->myPithGyroR));
					
					
					
					
				
   //左边髋轮毂关节输出力矩
    chassis->wheel_motor[0].torque_set=(LQR_Kl[0]*(vmcl->theta -0.0f) 
																	+LQR_Kl[1]*(vmcl->d_theta - 0.0f )
																	+LQR_Kl[2]*(chassis->x_set - chassis->x_filter)
																	+LQR_Kl[3]*(chassis->v_set - chassis->v_filter)
																	+LQR_Kl[4]*(chassis->myPithL-0.0f)
																	+LQR_Kl[5]*(chassis->myPithGyroL-0.0f));
   //左边髋关节输出力矩				
	vmcl->Tp=(LQR_Kl[6]*(vmcl->theta)
					+LQR_Kl[7]*(vmcl->d_theta)
					+LQR_Kl[8]*( chassis->x_set - chassis->x_filter)
                    +LQR_Kl[9]*( chassis->v_set - chassis->v_filter)
					+LQR_Kl[10]*(chassis->myPithL)
					+LQR_Kl[11]*(chassis->myPithGyroL));

					
   //右轮毂电机扭矩设定
   for(int i=0;i<2;i++)
   {
	chassis->wheel_motor[i].torque_set= T_WHEEL_RATIO * chassis->wheel_motor[i].torque_set +0.8f * chassis->turn_T;	//轮毂电机输出力矩
	mySaturate(&chassis->wheel_motor[i].torque_set,-2.0f,2.0f);	
    chassis->wheel_motor[i].give_current = chassis->wheel_motor[i].torque_set * CHASSIS_MOTOR_TORQUE_TO_CURRENT;
	Limit_Int(&chassis->wheel_motor[i].give_current,-8000,8000);
   }

    vmcr->Tp = vmcr->Tp+chassis->leg_tp;//右髋关节输出力矩
	vmcl->Tp = vmcl->Tp+chassis->leg_tp;//左髋关节输出力矩


	chassis->roll_f0 = 0;

    vmcr->F0= leg_G + PID_Calc(legr,vmcr->L0,chassis->leg_set) + chassis->roll_f0 ;//前馈+pd
	vmcl->F0=  leg_G + PID_Calc(legl,vmcl->L0,chassis->leg_set) - chassis->roll_f0;//左前馈+pd

//	right_flag = ground_detectionR(vmcr,ins);//右腿离地检测
	 
//	 if(chassis->recover_flag==0)		
//	 {//倒地自起不需要检测是否离地	 
//		if(right_flag==1&&left_flag==1&&vmcr->leg_flag==0)
//		{//当两腿同时离地并且遥控器没有在控制腿的伸缩时，才认为离地
//				chassis->wheel_motor[1].wheel_T=0.0f;
			//   vmcr->Tp=LQR_K[6]*(vmcr->theta-0.0f)+ LQR_K[7]*(vmcr->d_theta-0.0f); //保证机体与腿部垂直

//				chassis->x_filter=0.0f;
//				chassis->x_set=chassis->x_filter;
//				chassis->turn_set=chassis->total_yaw;
//				vmcr->Tp=vmcr->Tp+chassis->leg_tp;		
//		}
//		else
//		{//没有离地
//			vmcr->leg_flag=0;//置为0
//			
//		}
//	 }
//	 if(chassis->recover_flag==1)
//	 {
//		 vmcr->Tp=0.0f;
//		 chassis->recover_flag=0;
//	 }	 
//	 

	mySaturate(&vmcr->F0,-100.0f,100.0f);//限幅 
	mySaturate(&vmcl->F0,-100.0f,100.0f);//限幅
	
	VMC_calc_2(vmcr);//计算期望的关节输出力矩
	VMC_calc_2(vmcl);

	//额定扭矩
    mySaturate(&vmcr->torque_set[1],-Joint_T_Max,Joint_T_Max);//3	
	mySaturate(&vmcr->torque_set[0],-Joint_T_Max,Joint_T_Max);	
	mySaturate(&vmcl->torque_set[1],-Joint_T_Max,Joint_T_Max);//3
	mySaturate(&vmcl->torque_set[0],-Joint_T_Max,Joint_T_Max);	
}


void mySaturate(float *in,float min,float max)
{
  if(*in < min)
  {
    *in = min;
  }
  else if(*in > max)
  {
    *in = max;
  }
}


void Limit_Int(int16_t *in,int16_t min,int16_t max)
{
  if(*in < min)
  {
    *in = min;
  }
  else if(*in > max)
  {
    *in = max;
  }
}






