/**
  *********************************************************************
  * @file      ps2_task.c/h
  * @brief     该任务是读取并处理ps2手柄传来的遥控数据，
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
	
#include "ps2_task.h"
#include "cmsis_os.h"
#include "remote_control.h"

//控制任务周期是10ms
#define PS2_TIME 10.0f  //注意单精度浮点数的精度问题

//初始腿长
#define BEGIN_LEG_LENGTH 0.14f    //0.14  
#define INITIAL_LEG_LENGTH 0.18f   

//腿长增量 腿部伸缩灵敏度
#define RC_TO_ADD_LEG 0.00005f   
//转向增量 转向灵敏度
#define RC_TO_TURN_RATIO  0.00006f

//最大速度
#define VX_MAX  0.6f
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
int last_start_flag = 0;
int Left_Remote_rs_flag = 0;

//创建初始化函数
void PS2_init(void){
	 chassis_move.start_flag = 0;
	 chassis_move.x_set = chassis_move.x_filter;
	 chassis_move.v_set = 0.0f;
	 chassis_move.turn_set = chassis_move.total_yaw;
	 chassis_move.leg_set = BEGIN_LEG_LENGTH;
}	
	
void pstwo_task(void)
{	
	//初始化	
	PS2_init();

   while(1)
	 {	 
		   PS2_data_process(&rc_ctrl,&chassis_move,(PS2_TIME/1000.0f));//处理数据，设置期望数据
	     osDelay(PS2_TIME);
	 }
}

void PS2_data_process(RC_ctrl_t *rc_ctrl,chassis_t *chassis,float dt)
{   
//	if(rc_ctrl.rc.s[0] == 2 &&chassis->start_flag==0) 
//	{
//		//手柄上的Start按键被按下
//		chassis->start_flag=1;
//		if(chassis->recover_flag==0
//			&&((chassis->myPithR<((-3.1415926f)/4.0f)&&chassis->myPithR>((-3.1415926f)/2.0f))
//		  ||(chassis->myPithR>(3.1415926f/4.0f)&&chassis->myPithR<(3.1415926f/2.0f))))
//		{
//		  chassis->recover_flag=1;//需要自起
//		}
//	}
//	else if(rc_ctrl.rc.s[0] == 2&&data->key==4&&chassis->start_flag==1) 
//	{
//		//手柄上的Start按键被按下
//		chassis->start_flag=0;
//		chassis->recover_flag=0;
//	}
//	
//	data->last_key=data->key;
//  
//	if(chassis->start_flag==1)
//	{//启动
////		chassis->v_set=((float)(data->ry-128))*(-0.004f);//往前大于0
////		chassis->x_set=chassis->x_set+chassis->v_set*dt;
////		chassis->turn_set=chassis->turn_set+(data->rx-127)*(-0.00025f);//往右大于0
//	  			
//		//腿长变化
//		chassis->leg_set=chassis->leg_set+((float)(data->ly-128))*(-0.000015f); 
//		
//		mySaturate(&chassis->leg_set,0.15f,0.28f);//腿长限幅在0.065m到0.18m之间
//				
//		if(fabsf(chassis->last_leg_set-chassis->leg_set)>0.0001f)
//		{//遥控器控制腿长在变化
//			right.leg_flag=1;	//为1标志着腿长在主动伸缩(不包括自适应伸缩)，根据这个标志可以不进行离地检测，因为当腿长在主动伸缩时，离地检测会误判端为离地了
//      left.leg_flag=1;	 			
//		}
//		chassis->last_leg_set=chassis->leg_set;
//	} 
//	else if(chassis->start_flag==0)
//	{//关闭
   leg_add += rc_ctrl->rc.ch[4] * RC_TO_ADD_LEG;
   
//仅用于测试腿长控制
		if(switch_is_mid(rc_ctrl->rc.s[0]))
		{
			 chassis->start_flag = 1;
		}
		else if(switch_is_down(rc_ctrl->rc.s[0]))
		{
			 chassis->start_flag = 0;
			 chassis->turn_set = chassis->total_yaw;
			 chassis->x_set = chassis->x_filter;
		}
		 if(last_start_flag == 0 && chassis->start_flag == 1){
             Left_Remote_rs_flag = 1;
		 }
  
		
    if(chassis->start_flag == 1)
    {
			 chassis_move.leg_set =  BEGIN_LEG_LENGTH;
			 if(switch_is_mid(rc_ctrl->rc.s[1])){
				if(Left_Remote_rs_flag == 1){  //防止未开机状态下，左遥控器上的RS按键至于中位造成腿部设置过长
				   chassis_move.leg_set = BEGIN_LEG_LENGTH;
				}
				 else{
				 chassis_move.leg_set = INITIAL_LEG_LENGTH + leg_add;
				 leg_add = 0.0f;
				 }
			 }
			 else if(switch_is_down(rc_ctrl->rc.s[1])){
				Left_Remote_rs_flag = 0;
			 }

			chassis->turn_set = chassis->turn_set - rc_ctrl->rc.ch[0] * RC_TO_TURN_RATIO;
			 //速度斜坡控制
	        float vx_speed_cmd = rc_ctrl->rc.ch[1] * RC_TO_VX;
            if(fabs(vx_speed_cmd-chassis->v_set) < 0.01f){
                chassis->v_set = vx_speed_cmd;
			}
			else{	
                chassis->v_set = (vx_speed_cmd > chassis->v_set) ? (chassis->v_set + 0.01f) : (chassis->v_set - 0.01f);
			}
			if(fabs(vx_speed_cmd) > 0.1f ){
				chassis->x_set = chassis->x_filter;
			}  
  	}

    //速度步长控制
	// float vx_cmd = chassis->v_set - chassis->v_filter;
	float legLength = (left.L0+right.L0)/2.0f;
	// float speed_step = -(legLength - 0.15f) * 0.36f + SPEED_STEP;
	// if(fabs(vx_cmd) > speed_step){
	// 	chassis->v_set = (vx_cmd > 0) ? (chassis->v_filter + speed_step) :(chassis->v_filter - speed_step);
	// }
	// chassis->x_set = chassis->x_set + chassis->v_set * dt; 
	// if(rc_ctrl->rc.ch[1] > 1 || rc_ctrl->rc.ch[1] < -1){
	// 	chassis->x_set = chassis->x_filter;
	// }
	
	 //腿长步长限幅
	float leg_cmd = chassis_move.leg_set - legLength;
	if(fabs(leg_cmd) > 0.1f){
	    chassis_move.leg_set = (leg_cmd > 0) ? (legLength + 0.1f) : (legLength - 0.1f); 
	}
	//位移限幅
	float x_cmd = chassis->x_set - chassis->x_filter ;
	if(fabs(x_cmd) > 0.05f){  //0.04
		chassis->x_set = (x_cmd > 0) ? (chassis->x_filter + 0.05f) : (chassis->x_filter - 0.05f);
	}

	//转向限幅
	if(fabsf(chassis->turn_set - chassis->total_yaw) > 0.3f){
		chassis->turn_set = ((chassis->turn_set - chassis->total_yaw) > 0) ? (chassis->total_yaw + 0.3f) : (chassis->total_yaw - 0.3f);
	}

	mySaturate(&chassis_move.leg_set,0.14,0.35);//限制腿长范围
	mySaturate(&chassis->v_set,-VX_MAX,VX_MAX);
//		if(fabsf(chassis->last_leg_set-chassis->leg_set)>0.03f)//为达到较好的阻尼效果，腿长稳态误差比较大，基本维持在0.03f之内
//		{//遥控器控制腿长在变化
//			right.leg_flag=1;	//为1标志着腿长在主动伸缩(不包括自适应伸缩)，根据这个标志可以不进行离地检测，因为当腿长在主动伸缩时，离地检测会误判端为离地了
//      left.leg_flag=1;	 			
//		}
//		 chassis->last_leg_set=chassis->leg_set; 
   last_start_flag = chassis->start_flag ;
}






