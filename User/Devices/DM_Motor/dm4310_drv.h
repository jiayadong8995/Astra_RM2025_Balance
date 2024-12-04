#ifndef __DM4310_DRV_H__
#define __DM4310_DRV_H__
#include "main.h"
#include "fdcan.h"
#include "can_bsp.h"

//速度到角度滤波系数
#define FILTER_COEFFICIENT 0.9f   
#define MOTOR_ECD_TO_RAD  0.000766990394f   		// 2*  PI  /8192 /15

#define MIT_MODE 			0x000
#define POS_MODE			0x100
#define SPEED_MODE		0x200

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f

#define P_MIN2 -12.0f
#define P_MAX2 12.0f
#define V_MIN2 -45.0f
#define V_MAX2 45.0f
#define KP_MIN2 0.0f
#define KP_MAX2 500.0f
#define KD_MIN2 0.0f
#define KD_MAX2 5.0f
#define T_MIN2 -18.0f
#define T_MAX2 18.0f

typedef struct 
{
	uint16_t id;
	uint16_t state;
	int p_int;
	int v_int;
	int t_int;
	int kp_int;
	int kd_int;
	float pos;
	float vel;
	float tor;
	float Kp;
	float Kd;
	float Tmos;
	float Tcoil;
}motor_fbpara_t;

//3508 motor data
/*3508????????????*/
typedef struct
{
    uint16_t ecd;            //0~8191
    int16_t speed_rpm;       
    int16_t given_current;   
    uint8_t temperate;       
    int16_t last_ecd;        
	fp32 total_angle;
	int16_t last_encode;
} chassis_motor_measure_t;

typedef struct
{
  const chassis_motor_measure_t *chassis_motor_measure;
  fp32 speed;  
  fp32 w_speed;      
	// fp32 omg;             
  fp32 torque;          
  fp32 torque_set;      
  int16_t give_current; 
  float chassis_x;
} chassis_motor_t;

typedef struct {
    fp32 speed;           
    fp32 filtered_speed;  
    fp32 angle;           
    uint32_t last_time;    
} MotorData;


typedef struct
{
	uint16_t mode;
	motor_fbpara_t para;
}Joint_Motor_t ;

typedef struct
{
	uint16_t mode;
	float wheel_T;
	
	motor_fbpara_t para;	
}Wheel_Motor_t ;


extern void dm4310_fbdata(Joint_Motor_t *motor, uint8_t *rx_data,uint32_t data_len);
// extern void dm6215_fbdata(Wheel_Motor_t *motor, uint8_t *rx_data,uint32_t data_len);

extern int enable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);
extern void disable_motor_mode(hcan_t* hcan, uint16_t motor_id, uint16_t mode_id);

//关节电机
extern void mit_ctrl(hcan_t* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq);
extern void pos_speed_ctrl(hcan_t* hcan,uint16_t motor_id, float pos, float vel);
extern void speed_ctrl(hcan_t* hcan,uint16_t motor_id, float _vel);

//轮毂电机MIT模式发送命令
extern void mit_ctrl2(hcan_t* hcan, uint16_t motor_id, float pos, float vel,float kp, float kd, float torq);

extern void joint_motor_init(Joint_Motor_t *motor,uint16_t id,uint16_t mode);
extern void wheel_motor_init(Wheel_Motor_t *motor,uint16_t id,uint16_t mode);
	
extern float Hex_To_Float(uint32_t *Byte,int num);//十六进制到浮点数
extern uint32_t FloatTohex(float HEX);//浮点数到十六进制转换

extern float uint_to_float(int x_int, float x_min, float x_max, int bits);
extern int float_to_uint(float x_float, float x_min, float x_max, int bits);

extern void CAN_cmd_chassis(hcan_t *hcan,int16_t motor1,int16_t motor2,int16_t rev1,int16_t rev2);
extern void get_motor_measure(chassis_motor_measure_t *ptr, uint8_t *data, uint32_t data_len);
extern void get_total_angle(chassis_motor_measure_t *p);
extern const chassis_motor_measure_t *get_chassis_motor_measure_point(uint8_t i);
extern float motor_speed_to_angle(MotorData *motors,float *speed);


#endif /* __DM4310_DRV_H__ */



