 /**
   *********************************************************************
   * @file      motor_control_task.c/h
   * @brief     该任务轮腿底盘部分电机
   * @note       
   * @history
   *
   @verbatim
   ==============================================================================

   ==============================================================================
   @endverbatim
   *********************************************************************
   */

/*
菜鸟总结，开始做轮腿的时候看玺佬的视频，里面说一定要保证can通信的稳定，当时感觉接线和开发板，
电机等硬件没有问题，这个就能保证，但是由于基础不牢，can的接收频率和发送频率的原因，
一托六个电机，虽然每1ms延时发送一次，整个电机的发送频率为1000/6ms，满足发送需求，但是把接收频率给忽视了，
3508电机的接收是广播形式的接收，即接收1kHz接收一次，而达妙的电机是发送后才能收到接收，两类电机的接收冲突了。
*/

#include "motor_control_task.h"
#include "fdcan.h"
#include "cmsis_os.h"

extern int enable_flag[4];
uint32_t systick;
extern vmc_leg_t left;
extern vmc_leg_t right;
extern INS_t INS;

extern chassis_t chassis_move;
				
void motor_control_task(void)
{
    while(INS.ins_flag==0)
	{//等待加速度收敛
	  osDelay(1);	
	}
	
	joint_motor_init(& chassis_move.joint_motor[0],1,MIT_MODE);//发送id为1,控制模式 MIT
	joint_motor_init(& chassis_move.joint_motor[1],2,MIT_MODE);//发送id为2,控制模式 MIT
	joint_motor_init(& chassis_move.joint_motor[2],3,MIT_MODE);//发送id为3,控制模式 MIT
	joint_motor_init(& chassis_move.joint_motor[3],4,MIT_MODE);//发送id为4,控制模式 MIT
  //需要对4个关节电机进行零点重置
	enable_flag[0] = enable_motor_mode(&hfdcan1, chassis_move.joint_motor[0].para.id, chassis_move.joint_motor[0].mode);
	//DM_motor_zeroset(&hfdcan1,chassis_move.joint_motor[0].para.id);
	osDelay(1); 
	enable_flag[1] = enable_motor_mode(&hfdcan1, chassis_move.joint_motor[1].para.id, chassis_move.joint_motor[1].mode);
	//DM_motor_zeroset(&hfdcan1,chassis_move.joint_motor[1].para.id);
	osDelay(1);
	enable_flag[2] = enable_motor_mode(&hfdcan1, chassis_move.joint_motor[2].para.id, chassis_move.joint_motor[2].mode);
	//DM_motor_zeroset(&hfdcan1,chassis_move.joint_motor[2].para.id);
	osDelay(1);
	enable_flag[3] = enable_motor_mode(&hfdcan1, chassis_move.joint_motor[3].para.id,chassis_move.joint_motor[3].mode);
	//DM_motor_zeroset(&hfdcan1,chassis_move.joint_motor[3].para.id);
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




