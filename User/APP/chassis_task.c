/**
  *********************************************************************
  * @file      chassis_task.c/h
  * @brief     该任务计算轮腿模型
  *			   得出输出电流和力矩后发送给电机控制线程
  * @note       
  * @history
  *
  @verbatim
  =====================================================================

  =====================================================================
  @endverbatim
  *********************************************************************
  */
	
	
#include "chassis_task.h"
#include "fdcan.h"
#include "cmsis_os.h"
#include "can_bsp.h"

//-15.2927   -1.2844   -7.4831   -4.5426   13.4664    1.9690
//   22.4682    2.6679   22.1886   12.1150   34.9348    2.9753
//-14.5354   -1.1578   -7.1281   -4.3071   14.0014    2.0200
//   22.7326    2.6070   23.0702   12.5308   33.2775    2.7684（0.16）
//-17.7188   -1.7148   -8.4302   -5.1792   11.7744    1.7642
//   21.5827    2.8661   19.4008   10.7767   39.3566    3.5794（0.22）
//-18.0188,   -2.0148,   -9.9302,   -6.5892,   11.7744,    1.7642,
//  21.5827,    2.8661,   20.2008,   11.2767,   40.3566,    4.0794（0.22较完善）
static float LQR_K_R[12]= {-12.4527, -1.0044, -12.0831, -9.3426, 13.4064, 2.0690,
19.2682, 2.0679, 26.8886, 17.6150, 33.3348, 3.2053}; 
//{-13.1027, -1.0044, -11.8831, -8.8426, 14.0664, 2.0690,
//20.0682, 2.0679, 26.7886, 15.9150, 33.4348, 3.2053};
////{
//    -17.4503,   -1.9640,   -4.8365,   -4.4086,    6.2068,    1.4190,
//     14.0360,    2.1759,    7.1015,    5.9832,   16.9087,    2.6989
//};//0.20
vmc_leg_t right;
vmc_leg_t left;

extern INS_t INS;

int enable_flag[4]= {1,1,1,1};
int Begin_flag = 0;

float leg_G = 10.4f; //10.0f
float Joint_T_Max = 2.8f;
																
chassis_t chassis_move;
															
PidTypeDef LegR_Pid;//右腿的腿长pd
PidTypeDef LegL_Pid;//左腿的腿长pd
PidTypeDef Tp_Pid;//防劈叉补偿pd
PidTypeDef Turn_Pid;//转向pd
PidTypeDef Roll_Pid;//横滚角补偿pd

uint32_t CHASSIS_TIME=2;

static void Chassis_init(chassis_t *chassis,vmc_leg_t *vmcr_init,vmc_leg_t *vmcl_init);
static void Pensation_init(PidTypeDef *roll,PidTypeDef *Tp,PidTypeDef *turn,PidTypeDef *legr,PidTypeDef *legl);
static void chassis_feedback_update(chassis_t *chassis,vmc_leg_t *vmc_r,vmc_leg_t *vmc_l,INS_t *ins);
static void chassis_control_loop(chassis_t * chassis,\
							      vmc_leg_t *    vmcr,\
							      vmc_leg_t *    vmcl,\
						              INS_t *     ins,\
						              float *  LQR_KR,\
						              float *  LQR_KL,\
							     PidTypeDef *    legr,\
							     PidTypeDef *    legl);
static void chassis_jump_loop(chassis_t * chassis,\
							  vmc_leg_t * vmcr   ,\
							  vmc_leg_t * vmcl   ,\
							 PidTypeDef * legr   ,\
							 PidTypeDef * legl	  );
static void Limit_Int(int16_t *in,int16_t min,int16_t max);
static void chassis_ground_detection(chassis_t * chassis,\
									 vmc_leg_t * vmcr   ,\
									 vmc_leg_t * vmcl   ,\
						                 INS_t * ins     );

void Chassis_task(void)
{
	while(INS.ins_flag==0)
	{//等待加速度收敛
	  osDelay(1);	
	}

    Chassis_init(&chassis_move,&right,&left);//初始化右边两个关节电机和右边轮毂电机的id和控制模式、初始化腿部
    Pensation_init(&Roll_Pid,&Tp_Pid,&Turn_Pid,&LegR_Pid,&LegL_Pid);//补偿pid初始化
    osDelay(6);

	while(1)
	{	
		chassis_feedback_update(&chassis_move,&right,&left,&INS);//更新数据	
	    chassis_control_loop(&chassis_move,&right,&left,&INS,LQR_K_R,LQR_K_R,&LegR_Pid,&LegL_Pid);//控制计算
		osDelay(CHASSIS_TIME);
	}
}


static void Chassis_init(chassis_t *chassis,vmc_leg_t *vmcr_init,vmc_leg_t *vmcl_init)
{
	for(int i = 0;i < 2; i++)
	{
		chassis_move.wheel_motor[i].chassis_x = 0.0f;
		chassis_move.wheel_motor[i].chassis_motor_measure = get_chassis_motor_measure_point(i);
	}  
	VMC_init(vmcr_init);//给杆长赋值
	VMC_init(vmcl_init);//给杆长赋值
	
	chassis_feedback_update(&chassis_move,&right,&left,&INS);//更新数据

	Begin_flag =1;
}

static void Pensation_init(PidTypeDef *roll,PidTypeDef *Tp,PidTypeDef *turn,PidTypeDef *legr,PidTypeDef *legl)
{
	//腿长pid初始化
	const static float roll_pid[3] = {ROLL_PID_KP,ROLL_PID_KI,ROLL_PID_KD};
    const static float legr_pid[3] = {LEG_PID_KP, LEG_PID_KI,LEG_PID_KD};
    const static float legl_pid[3] = {LEG_PID_KP, LEG_PID_KI,LEG_PID_KD};
    //补偿pid初始化：防劈叉补偿、偏航角补偿
	const static float tp_pid[3] = {TP_PID_KP, TP_PID_KI, TP_PID_KD};
	const static float turn_pid[3] = {TURN_PID_KP, TURN_PID_KI, TURN_PID_KD};
	
    PID_init(roll, PID_POSITION, roll_pid, ROLL_PID_MAX_OUT, ROLL_PID_MAX_IOUT);
	PID_init(Tp,   PID_POSITION, tp_pid,   TP_PID_MAX_OUT,   TP_PID_MAX_IOUT);
	PID_init(turn, PID_POSITION, turn_pid, TURN_PID_MAX_OUT, TURN_PID_MAX_IOUT);

	PID_init(legr, PID_POSITION,legr_pid, LEG_PID_MAX_OUT, LEG_PID_MAX_IOUT);//腿长pid
	PID_init(legl, PID_POSITION,legl_pid, LEG_PID_MAX_OUT, LEG_PID_MAX_IOUT);//腿长pid
}

static void chassis_feedback_update(chassis_t *chassis,vmc_leg_t *vmc_r,vmc_leg_t *vmc_l,INS_t *ins)
{
    vmc_r->phi1=pi/2.0f+chassis->joint_motor[0].para.pos-0.024f;
	vmc_r->phi4=pi/2.0f+chassis->joint_motor[1].para.pos-0.0531f;
	vmc_l->phi1=pi/2.0f+chassis->joint_motor[2].para.pos-0.018f;
	vmc_l->phi4=pi/2.0f+chassis->joint_motor[3].para.pos+0.074f;

	chassis_move.wheel_motor[0].chassis_x = chassis_move.wheel_motor[0].chassis_motor_measure->total_angle /15.0f * WHEEL_R;		
	chassis_move.wheel_motor[1].chassis_x = chassis_move.wheel_motor[1].chassis_motor_measure->total_angle /15.0f * WHEEL_R;

	chassis_move.wheel_motor[0].w_speed = chassis_move.wheel_motor[0].chassis_motor_measure->speed_rpm * MOTOR_RPM_TO_W; 		
	chassis_move.wheel_motor[1].w_speed = chassis_move.wheel_motor[1].chassis_motor_measure->speed_rpm * MOTOR_RPM_TO_W; 
	
	chassis_move.wheel_motor[0].speed = chassis_move.wheel_motor[0].w_speed * WHEEL_R;		
	chassis_move.wheel_motor[1].speed = chassis_move.wheel_motor[1].w_speed * WHEEL_R;		

  chassis->myPithGyroL = - ins->Gyro[0];
	chassis->myPithL = - ins->Pitch;//-0.05 //+0.05
	chassis->myPithR = ins->Pitch;//0.05
	chassis->myPithGyroR = ins->Gyro[0];
	
	chassis->total_yaw = ins->YawTotalAngle;
	chassis->roll = ins->Roll;
	chassis->theta_err = 0.0f-(vmc_r->theta+vmc_l->theta);
	
	if(ins->Pitch<0.15f&&ins->Pitch>-0.15f)
	{//根据pitch角度判断倒地自起是否完成
		chassis->recover_flag=0;
	}

}

static void chassis_control_loop( chassis_t * chassis,\
							      vmc_leg_t *    vmcr,\
							      vmc_leg_t *    vmcl,\
						              INS_t *     ins,\
						              float *  LQR_KR,\
						              float *  LQR_KL,\
							     PidTypeDef *    legr,\
							     PidTypeDef *    legl)
{
	
	VMC_calc_1_right(vmcr,ins,2.0f/1000.0f);//计算theta和d_theta给lqr用，同时也计算右腿长L0,该任务控制周期是4*0.001秒
	VMC_calc_1_left(vmcl,ins,2.0f/1000.0f);//计算theta和d_theta给lqr用，同时也计算左腿长L0,该任务控制周期是4*0.001秒
	
  chassis->turn_T=Turn_Pid.Kp*(chassis->turn_set-chassis->total_yaw)-Turn_Pid.Kd*ins->Gyro[2];//这样计算更稳一点
	
	chassis->roll_f0=Roll_Pid.Kp*(chassis->roll_set-chassis->roll)-Roll_Pid.Kd*ins->Gyro[1];
	
    chassis->leg_tp = PID_Calc(&Tp_Pid, chassis->theta_err,0.0f);//防劈叉pid计算

	chassis->wheel_motor[1].torque_set=(LQR_KR[0]*(vmcr->theta) 
									   +LQR_KR[1]*(vmcr->d_theta) 
									   +LQR_KR[2]*(chassis->x_filter - chassis->x_set)
									   +LQR_KR[3]*(chassis->v_filter - 0)
									   +LQR_KR[4]*(chassis->myPithR-0.0f)
									   +LQR_KR[5]*(chassis->myPithGyroR-0.0f));					
	//右边髋关节输出力矩				
	vmcr->Tp=(LQR_KR[6]*(vmcr->theta)
			 +LQR_KR[7]*(vmcr->d_theta)
			 +LQR_KR[8]*( chassis->x_filter - chassis->x_set) 
			 +LQR_KR[9]*( chassis->v_filter - 0) 
			 +LQR_KR[10]*(chassis->myPithR)
			 +LQR_KR[11]*(chassis->myPithGyroR));
					
					
					
					
				
	//左边髋轮毂关节输出力矩
    chassis->wheel_motor[0].torque_set=(LQR_KL[0]*(vmcl->theta ) 
									   +LQR_KL[1]*(vmcl->d_theta)
									   +LQR_KL[2]*(chassis->x_set - chassis->x_filter)
									   +LQR_KL[3]*(0 - chassis->v_filter)
									   +LQR_KL[4]*(chassis->myPithL-0.0f)
									   +LQR_KL[5]*(chassis->myPithGyroL-0.0f));
	//左边髋关节输出力矩				
	vmcl->Tp=(LQR_KL[6] *(vmcl->theta)
			 +LQR_KL[7] *(vmcl->d_theta)
			 +LQR_KL[8] *(chassis->x_set - chassis->x_filter)
			 +LQR_KL[9] *(0 - chassis->v_filter)
			 +LQR_KL[10]*(chassis->myPithL)
			 +LQR_KL[11]*(chassis->myPithGyroL));

					
   //右轮毂电机扭矩设定
   for(int i=0;i<2;i++)
   {
		chassis->wheel_motor[i].torque_set= T_WHEEL_RATIO * chassis->wheel_motor[i].torque_set+0.8f * chassis->turn_T;	//轮毂电机输出力矩
		mySaturate(&chassis->wheel_motor[i].torque_set,-2.0f,2.0f);	
		Limit_Int(&chassis->wheel_motor[i].give_current,-8000,8000);
   }

    vmcr->Tp = vmcr->Tp+chassis->leg_tp;//右髋关节输出力矩
	vmcl->Tp = vmcl->Tp+chassis->leg_tp;//左髋关节输出力矩


	chassis->roll_f0 = 0;

	chassis_jump_loop(chassis,vmcr,vmcl,legr,legl);//有F0的计算过程
   
	chassis_ground_detection(chassis,vmcr,vmcl,ins);//离地检测
	
	VMC_calc_2(vmcr);//计算期望的关节输出力矩
	VMC_calc_2(vmcl);
	 
	chassis->wheel_motor[0].give_current = chassis->wheel_motor[0].torque_set * CHASSIS_MOTOR_TORQUE_TO_CURRENT;
	chassis->wheel_motor[1].give_current = chassis->wheel_motor[1].torque_set * CHASSIS_MOTOR_TORQUE_TO_CURRENT;
	 
	//额定扭矩
	mySaturate(&vmcr->F0,-100.0f,100.0f);//限幅 
    mySaturate(&vmcr->torque_set[1],-Joint_T_Max,Joint_T_Max);//3	
	mySaturate(&vmcr->torque_set[0],-Joint_T_Max,Joint_T_Max);
	
	mySaturate(&vmcl->F0,-100.0f,100.0f);//限幅
	mySaturate(&vmcl->torque_set[1],-Joint_T_Max,Joint_T_Max);//3
	mySaturate(&vmcl->torque_set[0],-Joint_T_Max,Joint_T_Max);
	
}

static void Limit_Int(int16_t *in,int16_t min,int16_t max)
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

static void chassis_jump_loop(chassis_t * chassis,\
							  vmc_leg_t * vmcr   ,\
							  vmc_leg_t * vmcl   ,\
							 PidTypeDef * legr   ,\
							 PidTypeDef * legl	  )
{
	if(chassis->start_flag == 1)
	{
		if(chassis->jump_flag == 1)
		{
			if(chassis->jump_status == 0)
			{
				vmcr->F0= leg_G + PID_Calc(legr,vmcr->L0,0.18f);
				vmcl->F0= leg_G + PID_Calc(legl,vmcl->L0,0.18f);
				if(vmcr->L0<0.185f&&vmcl->L0<0.185f)//0.18
				{
					chassis->jump_time++;
				}
				if(chassis->jump_time>10)
				{
					chassis->jump_time = 0;
					chassis->jump_status = 1;
				}
			}else if(chassis->jump_status == 1)
			{
				vmcr->F0= leg_G + PID_Calc(legr,vmcr->L0,0.3f);//暂时定为0.30，测试跳跃
				vmcl->F0= leg_G + PID_Calc(legl,vmcl->L0,0.3f);
				if(vmcr->L0>0.22f&&vmcl->L0>0.22f)
				{
					chassis->jump_time++;					
				}
				if(chassis->jump_time>10)
				{
					chassis->jump_time = 0;
					chassis->jump_status = 2;
				}
			}else if(chassis->jump_status == 2)
			{
				vmcr->F0= leg_G + PID_Calc(legr,vmcr->L0,0.18f);//右前馈+pd
				vmcl->F0= leg_G + PID_Calc(legl,vmcl->L0,0.18f);//左前馈+pd
				if(vmcr->L0<0.250f&&vmcl->L0<0.250f)
				{
					chassis->jump_time++;			
				}
				if(chassis->jump_time>10)
				{
					chassis->jump_time = 0;
					chassis->jump_status = 3;
				}
//收腿的两个逻辑，在状态2最后加一个时间判断回到设定值，或者在3状态加一个腿长判断，或者判断支持力				
			}else if(chassis->jump_status == 3)
			{
				vmcr->F0= leg_G + PID_Calc(legr,vmcr->L0,chassis->leg_set);//右前馈+pd
				vmcl->F0= leg_G + PID_Calc(legl,vmcl->L0,chassis->leg_set);//左前馈+pd
//				vmcr->F0= leg_G + PID_Calc(legr,vmcr->L0,0.13);
//				vmcl->F0= leg_G + PID_Calc(legl,vmcl->L0,0.13);
//				if(vmcr->L0 < 0.17 && vmcl->L0 < 0.17)
//				{
//				vmcr->F0= leg_G + PID_Calc(legr,vmcr->L0,chassis->leg_set);//右前馈+pd
//				vmcl->F0= leg_G + PID_Calc(legl,vmcl->L0,chassis->leg_set);//左前馈+pd
//				}
			}
		}else if(chassis->jump_flag == 0)
		{
			vmcr->F0= leg_G + PID_Calc(legr,vmcr->L0,chassis->leg_set)+ chassis->roll_f0;//右前馈+pd
			vmcl->F0= leg_G + PID_Calc(legl,vmcl->L0,chassis->leg_set)- chassis->roll_f0;//左前馈+pd
			
			chassis->jump_time = 0;
			chassis->jump_status = 0;			
		}
	}
}

static void chassis_ground_detection(chassis_t * chassis,\
									 vmc_leg_t * vmcr   ,\
									 vmc_leg_t * vmcl   ,\
						                 INS_t * ins     )
{
	static uint8_t left_flag  = 0;
	static uint8_t right_flag = 0;
	
	right_flag = ground_detectionR(vmcr,ins);//右腿离地检测
	left_flag  = ground_detectionL(vmcl,ins);
   
	if(chassis->recover_flag==0)		
	{//倒地自起不需要检测是否离地	 
		if(right_flag==1&&left_flag==1)
		{//当两腿同时离地并且遥控器没有在控制腿的伸缩时，才认为离地
			chassis->wheel_motor[0].torque_set=0.0f;
			chassis->wheel_motor[1].torque_set=0.0f;
			//vmcr->Tp=(LQR_K_R[6]+1)*(vmcr->theta-0.0f)+ (LQR_K_R[7]+0.2)*(vmcr->d_theta-0.0f); //保证机体与腿部垂直(测试是否可以)
			chassis->x_filter=0.0f;
			chassis->x_set=chassis->x_filter;
			chassis->turn_set=chassis->total_yaw;
			vmcr->Tp=vmcr->Tp+chassis->leg_tp;
      chassis->text_jump_true = 1;			
		}else
		{
		chassis->text_jump_true = 0;
		}
	}else if(chassis->recover_flag==1)
	{
		 vmcr->Tp=0.0f;
		 vmcl->Tp=0.0f;
	}
	
}
