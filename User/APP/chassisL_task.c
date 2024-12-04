// /**
//   *********************************************************************
//   * @file      chassisL_task.c/h
//   * @brief     该任务控制左半部分的电机，分别是两个DM4310和一个DM6215，这三个电机挂载在can2总线上
// 	*						 从底盘上往下看，左上角的DM4310发送id为8、接收id为4，
// 	*						 左下角的DM4310发送id为6、接收id为3，
// 	*						 左边DM轮毂电机发送id为1、接收id为0。
//   * @note       
//   * @history
//   *
//   @verbatim
//   ==============================================================================

//   ==============================================================================
//   @endverbatim
//   *********************************************************************
//   */

/*菜鸟总结，开始做轮腿的时候看玺佬的视频，里面说一定要保证can通信的稳定，当时感觉接线和开发板，
电机等硬件没有问题，这个就能保证，但是由于基础不牢，can的接收频率和发送频率的原因，
一托六个电机，虽然每1ms延时发送一次，整个电机的发送频率为1000/6ms，满足发送需求，但是把接收频率给忽视了，
3508电机的接收是广播形式的接收，即接收1kHz接收一次，而达妙的电机是发送后才能收到接收，两类电机的接收冲突了。
*/
	
#include "chassisL_task.h"
 #include "fdcan.h"
// #include "VMC_calc.h"

// #include "INS_task.h"
 #include "cmsis_os.h"
// #include "pid.h"

// vmc_leg_t left;

// //float LQR_K_L[12]={  -1.5171 ,  -0.1347  , -2.4105,   -0.9858 ,   0.8685  ,  0.0783,
// //    1.2392 ,   0.1251 ,   2.9650 ,   1.0868,   10.7689 ,   0.5026};
// //float LQR_K_L[12] = {-12.1366,  -1.3915,   -2.7811,   -2.6474 ,   3.2886,    1.0829};
// extern float LQR_K_R[12];
// //Q=diag([20 0.1 80 110 700 1]);
// //R=[90 0;0 4]; 
// // float LQR_K_L[12]={ 
// //    -2.1954,   -0.2044  , -0.8826,   -1.3245,    1.2784  ,  0.1112,
// //     2.5538,   0.2718  ,  1.5728  ,  2.2893  , 12.1973 ,   0.4578};

// extern float Poly_Coefficient[12][4];
extern int enable_flag[4];
uint32_t systick;
extern vmc_leg_t left;
extern vmc_leg_t right;
// extern chassis_t chassis_move;
		
// // PidTypeDef LegL_Pid;
extern INS_t INS;

extern chassis_t chassis_move;
				
void ChassisL_task(void)
{
    while(INS.ins_flag==0)
	{//等待加速度收敛
	  osDelay(1);	
	}
	
	joint_motor_init(& chassis_move.joint_motor[0],1,MIT_MODE);//发送id为1,控制模式 MIT
	joint_motor_init(& chassis_move.joint_motor[1],2,MIT_MODE);//发送id为2,控制模式 MIT
	joint_motor_init(& chassis_move.joint_motor[2],3,MIT_MODE);//发送id为3,控制模式 MIT
	joint_motor_init(& chassis_move.joint_motor[3],4,MIT_MODE);//发送id为4,控制模式 MIT

	 enable_flag[0] = enable_motor_mode(&hfdcan1, chassis_move.joint_motor[0].para.id, chassis_move.joint_motor[0].mode);
	 osDelay(1); 
	 enable_flag[1] = enable_motor_mode(&hfdcan1, chassis_move.joint_motor[1].para.id, chassis_move.joint_motor[1].mode);
	 osDelay(1);
	 enable_flag[2] = enable_motor_mode(&hfdcan1, chassis_move.joint_motor[2].para.id, chassis_move.joint_motor[2].mode);
	 osDelay(1);
	 enable_flag[3] = enable_motor_mode(&hfdcan1, chassis_move.joint_motor[3].para.id,chassis_move.joint_motor[3].mode);
	 osDelay(1);
   osDelay(2);

	while(1)
	{	
		systick = osKernelSysTick();
	   if(chassis_move.start_flag==0){

			if(systick%2==0)//左腿
			{
				mit_ctrl(&hfdcan1,chassis_move.joint_motor[0].para.id, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//right.torque_set[0]
				mit_ctrl(&hfdcan1,chassis_move.joint_motor[1].para.id, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//right.torque_set[1]
				CAN_cmd_chassis(&hfdcan2,0,0,0,0);		
			}
			else
			{
				mit_ctrl(&hfdcan1,chassis_move.joint_motor[2].para.id, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//left.torque_set[0]
				mit_ctrl(&hfdcan1,chassis_move.joint_motor[3].para.id, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//left.torque_set[1]		
			}
		}
		else if(chassis_move.start_flag==1)	
		{
			if(systick%2==0)
			{
				mit_ctrl(&hfdcan1,chassis_move.joint_motor[0].para.id, 0.0f, 0.0f,0.0f, 0.0f,right.torque_set[0]);//right.torque_set[0]		
				mit_ctrl(&hfdcan1,chassis_move.joint_motor[1].para.id, 0.0f, 0.0f,0.0f, 0.0f,right.torque_set[1]);//right.torque_set[1]
        CAN_cmd_chassis(&hfdcan2,chassis_move.wheel_motor[0].give_current,chassis_move.wheel_motor[1].give_current,0,0);
			}
			else
			{
				mit_ctrl(&hfdcan1,chassis_move.joint_motor[2].para.id, 0.0f, 0.0f,0.0f, 0.0f,left.torque_set[0]);//left.torque_set[0]				
				mit_ctrl(&hfdcan1,chassis_move.joint_motor[3].para.id, 0.0f, 0.0f,0.0f, 0.0f,left.torque_set[1]);//left	.torque_set[1]
			}	
		}
		osDelay(1);
		
	}
}


// 		chassisL_feedback_update(&chassis_move,&left,&INS);//更新数据
// 	  	chassisL_control_loop(&chassis_move,&left,&INS,LQR_K_R,&LegL_Pid);//控制计算		
		
//     if(chassis_move.start_flag==1)	
// 		{
// 			if(enable_flag[0]==0 && enable_flag[1] == 0&&enable_flag[2] == 0&&enable_flag[3] == 0){
// 				mit_ctrl(&hfdcan1,chassis_move.joint_motor[2].para.id, 0.0f, 0.0f,0.0f, 0.0f,left.torque_set[0]);
// 				osDelay(CHASSL_TIME);
// 				mit_ctrl(&hfdcan1,chassis_move.joint_motor[3].para.id, 0.0f, 0.0f,0.0f, 0.0f,left.torque_set[1]);//left.torque_set[1]
// 				osDelay(CHASSL_TIME);
// //			    mit_ctrl(&hfdcan1,chassis_move.joint_motor[2].para.id, 0.0f, 0.0f,0.0f, 0.0f,0.0f);
// //			    osDelay(CHASSL_TIME);
// //		      mit_ctrl(&hfdcan1,chassis_move.joint_motor[3].para.id, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//left.torque_set[1]
// //			    osDelay(CHASSL_TIME);
// 			}
// 		}
// 		else if(chassis_move.start_flag==0)	
// 		{
// 			if(enable_flag[0]==0 && enable_flag[1] == 0&&enable_flag[2] == 0&&enable_flag[3] == 0){
// 				mit_ctrl(&hfdcan1,chassis_move.joint_motor[2].para.id, 0.0f, 0.0f,0.0f, 0.0f,0.0f);
// 				osDelay(CHASSL_TIME);
// 				mit_ctrl(&hfdcan1,chassis_move.joint_motor[3].para.id, 0.0f, 0.0f,0.0f, 0.0f,0.0f);//left.torque_set[1]
// 				osDelay(CHASSL_TIME);
// 	   	}
// 		}
// 	}
// }

// int Begin_flag_L = 0;
// void ChassisL_init(chassis_t *chassis,vmc_leg_t *vmc,PidTypeDef *legl)
// {
//       const static float legl_pid[3] = {LEG_PID_KP, LEG_PID_KI,LEG_PID_KD};
	
// 	  joint_motor_init(&chassis->joint_motor[2],3,MIT_MODE);//发送id为3
// 	  joint_motor_init(&chassis->joint_motor[3],4,MIT_MODE);//发送id为4
	
// 	  VMC_init(vmc);//给杆长赋值
	
// 	  PID_init(legl, PID_POSITION,legl_pid, LEG_PID_MAX_OUT, LEG_PID_MAX_IOUT);//腿长pid
// 	  Begin_flag_L = 1;
		
// 	  enable_flag[2] = enable_motor_mode(&hfdcan1,chassis->joint_motor[2].para.id,chassis->joint_motor[2].mode);
// 	  osDelay(1);
// 	  enable_flag[3] = enable_motor_mode(&hfdcan1,chassis->joint_motor[3].para.id,chassis->joint_motor[3].mode);
// 	  osDelay(1);

// }

// void chassisL_feedback_update(chassis_t *chassis,vmc_leg_t *vmc,INS_t *ins)
// {
	
//     vmc->phi1=pi/2.0f+chassis->joint_motor[2].para.pos;
// 	vmc->phi4=pi/2.0f+chassis->joint_motor[3].para.pos;
		
// 	chassis->myPithL = 0.0f -ins->Pitch;
// 	chassis->myPithGyroL = 0.0f -ins->Gyro[0];
	
// }

// extern uint8_t right_flag;
// uint8_t left_flag;
// extern float leg_G;
// extern float Joint_T_Max ;
// void chassisL_control_loop(chassis_t *chassis,vmc_leg_t *vmcl,INS_t *ins,float *LQR_K,PidTypeDef *leg)
// {
// 	VMC_calc_1_left(vmcl,ins,((float)CHASSL_TIME)*2.0f/1000.0f);//计算theta和d_theta给lqr用，同时也计算左腿长L0,该任务控制周期是3*0.001m秒
	
// //for(int i=0;i<12;i++)
// //{
// //	LQR_K[i]=LQR_K_calc(&Poly_Coefficient[i][0],vmcl->L0 );	
// //}
		
// 	chassis->wheel_motor[0].torque_set=(LQR_K[0]*(vmcl->theta -0.0f) 
// 																	+LQR_K[1]*(vmcl->d_theta - 0.0f )
// 																	+LQR_K[2]*(chassis->x_set - chassis->x_filter)
// 																	+LQR_K[3]*(chassis->v_set - chassis->v_filter)
// 																	+LQR_K[4]*(chassis->myPithL-0.0f)
// 																	+LQR_K[5]*(chassis->myPithGyroL-0.0f));


// 	//左边髋关节输出力矩				
// 	vmcl->Tp=(LQR_K[6]*(vmcl->theta)
// 					+LQR_K[7]*(vmcl->d_theta)
// 					+LQR_K[8]*( chassis->x_set - chassis->x_filter) *1.1f
//                     +LQR_K[9]*( chassis->v_set - chassis->v_filter)*1.1f
// 					+LQR_K[10]*(chassis->myPithL)
// 					+LQR_K[11]*(chassis->myPithGyroL));

	

// 	//轮毂电机扭矩设定
// 	chassis->wheel_motor[0].torque_set= T_WHEEL_RATIO * chassis->wheel_motor[0].torque_set + chassis->turn_T;	//轮毂电机输出力矩
// 	mySaturate(&chassis->wheel_motor[0].torque_set,-1.5f,1.5f);	
// 	chassis->wheel_motor[0].give_current = chassis->wheel_motor[0].torque_set * CHASSIS_MOTOR_TORQUE_TO_CURRENT; //3508电机为电流输入
// 	Limit_Int(&chassis->wheel_motor[0].give_current,-6500,6500);

// 	vmcl->Tp= vmcl->Tp+chassis->leg_tp;//髋关节输出力矩
	
	
//   //vmcl->F0= PID_Calc(leg,vmcl->L0,chassis->leg_set);//前馈+pd

// 	vmcl->F0 = leg_G + PID_Calc(leg,vmcl->L0,chassis->leg_set);//前馈+pd
  	
// //	left_flag = ground_detectionL(vmcl,ins);//左腿离地检测
	
// //	 if(chassis->recover_flag==0)	
// //	 {//倒地自起不需要检测是否离地
// //		if(left_flag==1&&right_flag==1&&vmcl->leg_flag==0)
// //		{//当两腿同时离地并且遥控器没有在控制腿的伸缩时，才认为离地
// //			chassis->wheel_motor[0].wheel_T=0.0f;
// //			vmcl->Tp=LQR_K[6]*(vmcl->theta-0.0f)+ LQR_K[7]*(vmcl->d_theta-0.0f);
// //			
// //			chassis->x_filter=0.0f;//对位移清零
// //			chassis->x_set=chassis->x_filter;
// //			chassis->turn_set=chassis->total_yaw;
// //			vmcl->Tp=vmcl->Tp+chassis->leg_tp;		
// //		}
// //		else
// //		{//没有离地
// //			vmcl->leg_flag=0;//置为0			
// //		}
// //	 }
// //	 else if(chassis->recover_flag==1)
// //	 {
// //		 vmcl->Tp=0.0f;
// //	 }
// //	
// 	mySaturate(&vmcl->F0,-100.0f,100.0f);//限幅 
	
// 	VMC_calc_2(vmcl);//计算期望的关节输出力矩
	
//   //额定扭矩
//     mySaturate(&vmcl->torque_set[1],-Joint_T_Max,Joint_T_Max);	
// 	mySaturate(&vmcl->torque_set[0],-Joint_T_Max,Joint_T_Max);	
			
// }


