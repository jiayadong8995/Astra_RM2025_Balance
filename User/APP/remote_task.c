/**
  *********************************************************************
  * @file      remote_task.c/h
  * @brief     该任务是读取并处理遥控数据，
	*            将遥控数据转化为期望的速度、期望的转角、期望的腿长等
  * @note       
  * @history
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  *********************************************************************
  */
	
#include "remote_task.h"
#include "cmsis_os.h"
#include "remote_control.h"

//控制任务周期是10ms
#define REMOTE_TIME 10.0f  //注意单精度浮点数的精度问题

//初始腿长
#define BEGIN_LEG_LENGTH   0.18f   //0.22 //0.14
#define INITIAL_LEG_LENGTH 0.18f//未动   

//腿长增量 腿部伸缩灵敏度
#define RC_TO_ADD_LEG 0.00005f   
//转向增量 转向灵敏度
#define RC_TO_TURN_RATIO  0.0002f //0.00006f

//最大速度
#define VX_MAX  6.5f  //可修改部分
#define RC_TO_VX   VX_MAX/660.0f
//速度斜坡，速度控制灵敏度
#define SPEED_STEP 0.2f

extern chassis_t chassis_move;
extern INS_t INS;
extern RC_ctrl_t rc_ctrl;
extern vmc_leg_t right;			
extern vmc_leg_t left;	
float leg_add = 0;
int leg_set_flag = 0;


static void remote_data_process(RC_ctrl_t *rc_ctrl,chassis_t *chassis,float dt);
static void slope_following(float *target,float *set,float acc);


//创建初始化函数
void remote_init(void)
{
	chassis_move.start_flag = 0;
	chassis_move.x_filter = 0;
	chassis_move.x_set = chassis_move.x_filter;
	chassis_move.v_set = 0.0f;
	chassis_move.turn_set = chassis_move.total_yaw;
	chassis_move.leg_set = BEGIN_LEG_LENGTH;
}

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;

uint8_t buffer[18];

#include "bsp_uart.h"

void remote_task(void)
{	
	//初始化	
	remote_init();
	memset(buffer,5,sizeof(buffer));
	while(1)
	{	
		remote_data_process(&rc_ctrl,&chassis_move,(REMOTE_TIME/1000.0f));//处理数据，设置期望数据
		osDelay(REMOTE_TIME);
		osDelay(50);
	}
}

static void remote_data_process(RC_ctrl_t *rc_ctrl,chassis_t *chassis,float dt)
{
	chassis->last_recover_flag = chassis->recover_flag;
	if(switch_is_mid(rc_ctrl->rc.s[0]))
	{
		chassis->start_flag = 1;
		if(chassis->myPithR > 0.20f ||chassis->myPithR < -0.20f)//原0.28
		{
			if(rc_ctrl->rc.s[1] == 3)
			{
			chassis->recover_flag = 0;
			}else
			{
			chassis->recover_flag = 1;
			}
		}
		
	}
	else if(switch_is_down(rc_ctrl->rc.s[0]))
	{
		chassis->start_flag = 0;
		chassis->recover_flag = 0;
	}
	
	
    if(chassis->start_flag == 1)
    {
		if(switch_is_mid(rc_ctrl->rc.s[1]))
		{
			if( 660 == rc_ctrl->rc.ch[3] )
			{
				chassis->jump_flag = 1;
			}else
			{
				chassis->jump_flag = 0;
			}
		}
		else if(switch_is_down(rc_ctrl->rc.s[1]))
		{
			chassis->jump_flag = 0;
		}
		
		chassis_move.leg_set =  BEGIN_LEG_LENGTH;
//		if(rc_ctrl->rc.s[1] == 3)
//		{
		chassis->turn_set = chassis->turn_set - rc_ctrl->rc.ch[2] * RC_TO_TURN_RATIO;
//		}else{
//		chassis->turn_set = chassis->turn_set - rc_ctrl->rc.ch[0] * RC_TO_TURN_RATIO;
		//}
		//速度斜坡控制
		float vx_speed_cmd = -rc_ctrl->rc.ch[1] * RC_TO_VX;//遥控器相反（现加负号）
		//float vx_speed_cmd = rc_ctrl->rc.ch[1] * RC_TO_VX;//遥控器相反（现加负号）
		slope_following(&vx_speed_cmd,&chassis->v_set,0.1f);//原0.01
		mySaturate(&chassis->v_set,-VX_MAX,VX_MAX);

		chassis->x_set = chassis->x_set + chassis->v_set * (float)REMOTE_TIME/1000.0f;

  	}
	else if(chassis->start_flag == 0)
	{
		chassis->v_set=0.0f;//清零
		chassis->x_set=chassis->x_filter;//保存
		chassis->turn_set=chassis->total_yaw;//保存
	}

	float legLength = (left.L0+right.L0)/2.0f;
	
	//腿长步长限幅
	float leg_cmd = chassis_move.leg_set - legLength;
	if(fabs(leg_cmd) > 0.1f){
	    chassis_move.leg_set = (leg_cmd > 0) ? (legLength + 0.1f) : (legLength - 0.1f); 
	}

	//转向限幅
	if(fabsf(chassis->turn_set - chassis->total_yaw) > 0.3f){
		chassis->turn_set = ((chassis->turn_set - chassis->total_yaw) > 0) ? (chassis->total_yaw + 0.3f) : (chassis->total_yaw - 0.3f);
	}

	mySaturate(&chassis_move.leg_set,0.14,0.35);//限制腿长范围

}

static void slope_following(float *target,float *set,float acc)
{
	if(*target > *set)
	{
		*set = *set + acc ;
		if(*set >= *target)
		*set = *target;
	}
	else if(*target < *set)
	{
		*set = *set - acc ;
		if(*set <= *target)
		*set = *target;
	}
}
