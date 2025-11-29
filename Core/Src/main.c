/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "can.h"
#include "dma.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "string.h"
#include "cmsis_os.h"
#define GM3508_SPEED_GEAR1 200
#define GM3508_SPEED_GEAR2 200
#define DIANJI_ID  0x209
#define DIAN_ANGLE  90.0f
#define PID_LIMIT    4000     
#define ANGLE_DEADZONE  3.0f      // 角度控制死区
#define SPEED_DEADZONE 7        // 速度控制死区
#define BLOCK 9000  // 堵转检测条件
#define CORRECT  0.5f   // 堵转纠正
#define TASK_HEARTBEAT_TIMEOUT_MS   2000    // 任务心跳超时时间(毫秒)
#define WATCHDOG_TASK_DELAY_MS      500     // 监视任务检查间隔          
#define RC_CH0_MIN        364U         // ch0最小値
#define RC_CH0_MAX        1684U        // ch0最大値
#define RC_CH0_MID        1024U        // ch0中点値
#define RC_CH0_DEAD_ZONE  20U          // ch0死区
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
CAN_RxHeaderTypeDef RxMsg={0};
struct recept{

int16_t angle;
int16_t  speed ;
int16_t  i;
uint16_t temp ;

};
struct recept recept1={
  .angle=0,
	.speed=0,
	.i=0,
	.temp=0
};
struct recept recept2={
  .angle=0,
	.speed=0,
	.i=0,
	.temp=0
};
struct recept recept3={
  .angle=0,
	.speed=0,
	.i=0,
	.temp=0
};
typedef struct {
    // 基础参数
    float base_Kp;             // 基准比例增益
    float base_Ki;             // 基准积分增益
    float base_Kd;             // 基准微分增益
    
    // 速度区域边界 (RPM)
    float speed_low_threshold;   // 低速区上限 (500 RPM)
    float speed_mid_threshold;   // 中速区上限 (2000 RPM)
    float speed_high_threshold;  // 高速区上限 (5000 RPM)
    
    // 误差区域边界 (RPM)
    float error_small_threshold; // 小误差区上限 (100 RPM)
    float error_medium_threshold;// 中等误差区上限 (500 RPM)
    
    // 速度区域参数调整系数
    float low_speed_kp_factor;   // 低速区K
    float low_speed_ki_factor;   
    float low_speed_kd_factor;   
    
    float high_speed_kp_factor;  // 高速区
    float high_speed_ki_factor;  
    float high_speed_kd_factor;  
    
    float very_high_speed_kp_factor;  // 极高速区
    float very_high_speed_ki_factor;  
    float very_high_speed_kd_factor;  
    
    // 误差区域参数调整系数
    float large_error_kp_factor; // 大误差区Kp调整系数
    float large_error_ki_factor; 
    float large_error_kd_factor; 
    
    float small_error_kp_factor; // 小误差区调整系数
    float small_error_ki_factor; 
    float small_error_kd_factor; 
    
    // 参数平滑滤波
    float prev_Kp;
    float prev_Ki;
    float prev_Kd;
    float Kp;                // 当前比例增益
    float Ki;                // 当前积分增益
    float Kd;                // 当前微分增益
    float setpoint;          // 目标设定值
    float integral;          // 积分项
    float derivative;        // 微分项
    float output_limit;      // 输出限幅
    float prev_measured_value; // 上一周期测量值
    float dt;                // 控制周期（秒）
} PID_Controller_f;


typedef __packed struct {
    uint16_t ch0;  // 遥控器
    uint16_t ch1; 
    uint16_t ch2;  
	  uint16_t ch3;  
    uint8_t  s1;   
    uint8_t  s2; 
	uint16_t roll;
	
} DBUS_RC_t;
typedef __packed struct {
    int16_t x;     // 鼠标X轴
    int16_t y;     // 鼠标Y轴
    int16_t z;     // 鼠标Z轴
    uint8_t press_l;// 鼠标左键
    uint8_t press_r;// 鼠标右键
} DBUS_Mouse_t;

typedef __packed struct {
    uint16_t v;    // 键盘按键值（Bit0=W, Bit1=S...Bit7=Ctrl）
} DBUS_Key_t;
int16_t output_gm6020=0;
int16_t output_m2006=0;

uint8_t rx_data[8]={0};
uint8_t  send_data[8]={0};
uint8_t rx_data2[8]={0};

DBUS_RC_t g_dbus_rc = {0}; 
DBUS_Mouse_t g_dbus_mouse = {0};            // 鼠标数据
DBUS_Key_t g_dbus_key = {0};  

uint16_t gm6020_current_target_angle = (192 + 6557) / 2;
uint32_t rc_last_heartbeat = 0;  // 遥控器最后一次心跳时间

uint8_t rc_online_flag = 1;


uint8_t g_dbus_rx_buf[18] = {0}; 
uint8_t g_dbus_idx = 0;                 // DBUS缓冲索引
int16_t m2006_target_speed = 300;  
int16_t m2006_max_speed = 1000;  
uint8_t m2006_direction = 1; //方向

int16_t gm3508_motor1_output = 0; // 电机1输出
int16_t gm3508_motor2_output = 0; // 电机2输出
uint16_t gm3508_target_speed = 0;
typedef enum {
    TASK_ID_DEFAULT,
    TASK_ID_MYTASK02,
    TASK_ID_MYTASK03,
    TASK_ID_MAX
} TaskID;
typedef struct {
    TaskID id;
    const char* name;
    uint32_t last_heartbeat;
    uint32_t timeout_ms;
    uint8_t is_active;
} TaskMonitor;

typedef struct {
    float rin;        // 当前设定值
    float lastRin;    // 上一次设定值
    float perrRin;  	// 前两次设定值变化
    int init_count;//初始化转台
} FFC;

typedef struct 
{  
    float Kp;       // 比例系数  
    float Ki;       // 积分系数  
    float Kd;   
    float Kf_a;
    float Kf_v;	
    float setpoint; // 设定值  
    float integral; // 积分项  
	  float derivative;//
    float prev_error; // 上一次误差  
	  float output_limit; // 输出限幅
	  float prev_prev_error;
	  float last_output;
	  float dt;        // 控制周期
   float prev_measured_value; // 存储上一测量值

	  FFC feedforward;   
} PID_Controller; 

PID_Controller pid_controller = {
	  .Kp = 46.0f,   
    .Ki =1.8f,
    .Kd =1.0f,
	.Kf_a=0.05,
	.Kf_v=0.5,
    .setpoint = 10.0f,
    .integral = 0.0f,
    .prev_error = 0.0f,
	  .dt=0.001f,
	  .last_output=0.0,
	  .prev_prev_error=0.0f,
	
    .output_limit = 6000.0f};
PID_Controller angle_pid_controller = {
    .Kp =50.0f,   
    .Ki = 3.0f,
    .Kd = 5.0f,
		.Kf_a=0.01,
	  .Kf_v=0.1,
    .setpoint =1000.0f,
    .integral = 0.0f,
    .prev_error = 0.0f,
	  .prev_prev_error=0.0f,
	.dt=0.001f,
	  .last_output=0.0,

    .output_limit =6000.0f // 输出限幅
};
PID_Controller current_pid = {
    .Kp =1.6f,   
    .Ki = 0.02f,
    .Kd =1.0,
		.Kf_a=0.05,
    .Kf_v=0.5,
    .setpoint = 0.0f, 
    .integral = 0.0f,
    .prev_error = 0.0f,
	  .prev_prev_error=0.0f,
	  .dt=0.001f,
	  .last_output=0.0,
    .output_limit = 6000.0f
};

PID_Controller pid_controller_m2006 = {
	  .Kp = 3.0f,   
    .Ki =0.1f,
    .Kd =0.8,
	  .Kf_a=0.0,
	  .Kf_v=0.0,
    .setpoint = 100.0f,
    .integral = 0.0f,
    .prev_error = 0.0f,
	  .dt=0.001f,
	  .last_output=0.0,
	  .prev_prev_error=0.0f,
	
    .output_limit = 6000.0f};
PID_Controller current_pid_m2006 = {
    .Kp =2.0f,   
    .Ki = 0.2f,
    .Kd =0.9f,
		.Kf_a=0.00,
    .Kf_v=0.0,
    .setpoint = 0.0f, 
    .integral = 0.0f,
    .prev_error = 0.0f,
	  .prev_prev_error=0.0f,
	  .dt=0.001f,
	  .last_output=0.0,
    .output_limit = 6000.0f
};
PID_Controller gm3508_speed_pid = {
    .Kp = 8.0f,   
    .Ki = 0.2f,    
    .Kd = 0.5f,   
    .setpoint = 0.0f,
    .integral = 0.0f,
    .prev_error = 0.0f,
    .output_limit = 6000.0,
    .dt = 0.001
};

PID_Controller gm3508_current_pid = {
    .Kp = 1.5f,   
    .Ki = 0.05f,   
    .Kd = 0.8f,    
    .setpoint = 0.0f,
    .integral = 0.0f,
    .prev_error = 0.0f,
    .output_limit = 6000.0, 
    .dt = 0.001
};
uint16_t mode=1;//0速度，1角度


uint8_t  send[8]={0};//can1,gm6020发送区
uint8_t  send2[8]={0};//can2,m2006发送区
uint8_t send3[8]={0};//3508发射区
uint8_t send4[8]={0};
// 状态：正常、堵转检测中、回退、恢复尝试
typedef enum {
    NORMAL,
   JAMMED,
   BACKOFF,
    RECOVERING
} MotorState;

MotorState motor_state =NORMAL;
uint32_t start_time = 0;
float back_angle = 100.0f;        // 回退角度（度）
float reangle = 100.0f;      // 恢复时尝试正转的角度
float raw_speed= 0;// 保存原始目标速度
float current_position = 0.0f;    // 当前位置
float start_position = 0.0f;      // 记录开始位置
uint8_t jam_recount = 0;   // 堵塞恢复尝试次数
uint8_t max= 3;      // 最多重试
float m2006_filtered_speed = 0.0f;
static TaskMonitor task_monitors[TASK_ID_MAX];

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
void Watchdog_Init(void);
void Watchdog_Heartbeat(TaskID task_id);
void Watchdog_FeedHardware(void);
void StartWatchdogTask(void const * argument);
void Can_SendMsg(CAN_HandleTypeDef *hcan,uint32_t ID,uint8_t *data)
{
   uint32_t canTxMailbox;
	CAN_TxHeaderTypeDef CAN_TX;
	CAN_TX.DLC=8;
	CAN_TX.IDE=CAN_ID_STD;
	CAN_TX.RTR=CAN_RTR_DATA;
  CAN_TX.StdId=ID;
	
	if ((hcan->Instance->TSR&CAN_TSR_TME1)!=RESET){
	canTxMailbox=CAN_TX_MAILBOX1;
	}
	else if ((hcan->Instance->TSR&CAN_TSR_TME2)!=RESET){
		canTxMailbox=CAN_TX_MAILBOX2;
	}
	else if ((hcan->Instance->TSR&CAN_TSR_TME0)!=RESET){
		canTxMailbox=CAN_TX_MAILBOX0;
	}

	HAL_CAN_AddTxMessage(hcan,&CAN_TX,data,(uint32_t*)&canTxMailbox);

}
void CAN_Filter_Init(CAN_HandleTypeDef *hcan)
{
	
	 if (hcan->Instance == CAN1) {
  
		 CAN_FilterTypeDef FilterConfig;
        FilterConfig.FilterBank = 0;
        FilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
        FilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
        FilterConfig.FilterIdHigh = 0x0000;
        FilterConfig.FilterIdLow = 0x0000;
        FilterConfig.FilterMaskIdHigh = 0x0000;
        FilterConfig.FilterMaskIdLow = 0x0000;
        FilterConfig.FilterFIFOAssignment = CAN_FilterFIFO0;
        FilterConfig.FilterActivation = CAN_FILTER_ENABLE;
        FilterConfig.SlaveStartFilterBank = 14;
		 HAL_CAN_ConfigFilter(hcan, &FilterConfig);
    HAL_CAN_Start(hcan);
    } else if (hcan->Instance == CAN2) {
    CAN_FilterTypeDef FilterConfig2;
        FilterConfig2.FilterBank = 14; 
        FilterConfig2.FilterMode = CAN_FILTERMODE_IDMASK; 
        FilterConfig2.FilterScale = CAN_FILTERSCALE_32BIT;
        FilterConfig2.FilterIdHigh =0x0000 ;
        FilterConfig2.FilterIdLow = 0x0000;
        FilterConfig2.FilterMaskIdHigh = 0x0000;
        FilterConfig2.FilterMaskIdLow = 0x0000;
        FilterConfig2.FilterFIFOAssignment = CAN_FilterFIFO1;
        FilterConfig2.FilterActivation = CAN_FILTER_ENABLE;
        FilterConfig2.SlaveStartFilterBank = 14;
			HAL_CAN_ConfigFilter(hcan, &FilterConfig2);
    HAL_CAN_Start(hcan);
    }
    
}
void FFC_Init(FFC *ffc, float initial_setpoint) {
    ffc->rin = initial_setpoint;
    ffc->lastRin = initial_setpoint;
    ffc->perrRin = initial_setpoint;
    ffc->init_count = 0; 
}
float fconstrain(float val, float min, float max) {
    return (val < min) ? min : (val > max) ? max : val;
}
float FeedforwardController(FFC *vFFC, float Kf_v, float Kf_a, float dt) {
    float result = 0.0f;
    if (vFFC->init_count < 1) {
     
        result = 0.0f;
    } else if (vFFC->init_count < 2) {
  
        result = Kf_v * (vFFC->rin - vFFC->lastRin) / dt;
    } else {
        // 使用完整前馈
        result = Kf_v * (vFFC->rin - vFFC->lastRin) / dt 
               + Kf_a * (vFFC->rin - 2*vFFC->lastRin + vFFC->perrRin) / (dt*dt);
    }
    vFFC->perrRin = vFFC->lastRin;
    vFFC->lastRin = vFFC->rin;

    if (vFFC->init_count < 2) {
        vFFC->init_count++;
    }
    return result;
}


float error;
float output;
	float  angle_continuous=0;
float PID_Compute_local(PID_Controller *pid, float measured_value) //前馈位置
{       pid->feedforward.rin =pid->setpoint;
    float feedforward = FeedforwardController(
			&pid->feedforward,
		pid->Kf_v,
		pid->Kf_a,
		pid->dt);

	error = pid->setpoint - measured_value; // 计算误差  

    if((mode == 1 && fabs(error) < ANGLE_DEADZONE) || 
       (mode == 0 && fabs(error) < SPEED_DEADZONE))
    {
        error = 0;
        pid->integral *= 0.99f; // 积分衰减
    }
    else
    {
        // 积分项计算与限幅
			
        pid->integral += error;
	
        if (pid->integral > pid->output_limit / pid->Ki)
            pid->integral = pid->output_limit / pid->Ki;
        else if (pid->integral < -pid->output_limit / pid->Ki)
            pid->integral = -pid->output_limit / pid->Ki;
    }
 
    pid->derivative = 0.7f * pid->derivative + 0.3f * (error - pid->prev_error);
   pid->prev_error = error;
    float output = pid->Kp * error + 
                   pid->Ki * pid->integral +
                   pid->Kd * pid->derivative;
    
    // 输出限幅
    if(output > pid->output_limit)
        output = pid->output_limit;
    else if(output < -pid->output_limit)
        output = -pid->output_limit;
    return output;  
	}
float PID_Compute_First(PID_Controller *pid, float measured_value) {
    float error = pid->setpoint - measured_value;

    // 计算测量值的变化率，核心
    float y_diff = measured_value - pid->prev_measured_value;
    pid->derivative = 0.7f * pid->derivative + 0.3f * y_diff;
//一节滤波，
    pid->prev_measured_value = measured_value;

    float proportional = pid->Kp * error;
    float derivative_term = -pid->Kd * pid->derivative; // 注意负号
  
    float output_without_integral = proportional + derivative_term;
    int should_integrate = 1;
    float saturation_threshold = pid->output_limit * 0.95f;
    
    // 检查是否达到输出限幅且误差方向同向
    if (output_without_integral >= saturation_threshold && error > 0) {
        should_integrate = 0;
    } 
    else if (output_without_integral <= -saturation_threshold && error < 0) {
        should_integrate = 0;
    }

    if (should_integrate  != 0.0f) {
 
        pid->integral += error * pid->dt;
    } else {
        pid->integral += error * pid->dt * 0.2f;
    }
    float max_integral = pid->output_limit / (pid->Ki * 1.2f);
    if (pid->integral > max_integral)
        pid->integral = max_integral;
    else if (pid->integral < -max_integral)
        pid->integral = -max_integral;
//积分限制
    float output = proportional + (pid->Ki * pid->integral) +derivative_term;
    if(output > pid->output_limit)
        output = pid->output_limit;
    else if(output < -pid->output_limit)
        output = -pid->output_limit;
        
    return output;
}
	
	float PID_Compute_add(PID_Controller *pid, float measured_value) //增量
{    
    float error = pid->setpoint - measured_value; 
    if((mode == 1 && fabs(error) < ANGLE_DEADZONE) || 
       (mode == 0 && fabs(error) < SPEED_DEADZONE))
    {
        error = 0;

        pid->derivative = 0;
        pid->prev_error = 0;
        pid->prev_prev_error = 0;
        return pid->last_output; // 死区时保持上次输出
    }
    else{// 误差增量
    float delta_error = error - pid->prev_error;
    float delta_error2 = error - 2 * pid->prev_error + pid->prev_prev_error;
    pid->derivative = 0.7f * pid->derivative + 0.3f * delta_error;

    // 增量式PID公式：Δu = Kp*Δe + Ki*e + Kd*Δe2
    float delta_output = pid->Kp * delta_error + 
                        pid->Ki * error + 
                        pid->Kd * delta_error2;

    
    float output = pid->last_output + delta_output;
    if(output > pid->output_limit)
        output = pid->output_limit;
    else if(output < -pid->output_limit)
        output = -pid->output_limit;
    pid->prev_prev_error = pid->prev_error;
    pid->prev_error = error;
    pid->last_output = output;

    return output;}  
}
float PID_Compute_full(PID_Controller *pid, float measured_value) 
{ float error = pid->setpoint - measured_value;  

    if((mode == 1 && fabs(error) < ANGLE_DEADZONE) || 
       (mode == 0 && fabs(error) < SPEED_DEADZONE))
    {
        error = 0;
        pid->integral *= 0.99f; 
    }
    else
    {

          float error_diff = error - pid->prev_error;
    pid->derivative = 0.7f * pid->derivative + 0.3f * error_diff;
    pid->prev_error = error; 
            float proportional = pid->Kp * error;
    float derivative_term = pid->Kd * pid->derivative; 
		float 	output_without_integral=proportional + derivative_term;
        // 2. 判断是否需要削弱积分
        int should_integrate = 1;
        
        if (output_without_integral >= pid->output_limit && error > 0) {
            // 已达上限且误差为正
            should_integrate = 0;
        } 
        else if (output_without_integral <= -pid->output_limit && error < 0) {
            // 已达下限且误差为负 
            should_integrate = 0;
        }
        if (should_integrate) {
            // 正常积分
            pid->integral += error * pid->dt;
        } 
        else {

            pid->integral += error * pid->dt * 0.2f; // 部分积分作用

        }
        float max_integral = pid->output_limit / pid->Ki;
        if (pid->integral > max_integral)
            pid->integral = max_integral;
        else if (pid->integral < -max_integral)
            pid->integral = -max_integral;
    }

    float output = pid->Kp * error + 
                   pid->Ki * pid->integral +
                   pid->Kd * pid->derivative ;
    
    if(output > pid->output_limit)
        output = pid->output_limit;
    else if(output < -pid->output_limit)
        output = -pid->output_limit;
        
    return output;
}
 float calculate_transition_weight(float value, float lower_bound, float upper_bound) {
    if (value <= lower_bound) return 0.0f;
    if (value >= upper_bound) return 1.0f;
    
    // 在过渡区域内线性插值
    return (value - lower_bound) / (upper_bound - lower_bound);
}

float PID_Compute_SegmentedVS(PID_Controller_f *pid, float measured_value) {
    // 1. 计算误差
    float error = pid->setpoint - measured_value;
    float abs_error = fabsf(error);
    float abs_speed = fabsf(measured_value);
    // 重置当前参数为基准值
    float current_Kp = pid->base_Kp;
    float current_Ki = pid->base_Ki;
    float current_Kd = pid->base_Kd;

    // 定义过渡区域（边界±50 RPM）
    const float SPEED_TRANSITION = 50.0f;
    
    // 低速区判定 (0 - speed_low_threshold)
    if (abs_speed < pid->speed_low_threshold) {
        current_Kp *= pid->low_speed_kp_factor;
        current_Ki *= pid->low_speed_ki_factor;
        current_Kd *= pid->low_speed_kd_factor;
    }
    // 低速-中速过渡区
    else if (abs_speed < pid->speed_low_threshold + SPEED_TRANSITION) {
        float weight = calculate_transition_weight(
            abs_speed, 
            pid->speed_low_threshold, 
            pid->speed_low_threshold + SPEED_TRANSITION
        );
        
        // 线性插值
        current_Kp = pid->base_Kp * (
            pid->low_speed_kp_factor * (1.0f - weight) + 
            weight
        );
        current_Ki = pid->base_Ki * (
            pid->low_speed_ki_factor * (1.0f - weight) + 
            weight
        );
        current_Kd = pid->base_Kd * (
            pid->low_speed_kd_factor * (1.0f - weight) + 
            weight
        );
    }
    // 中速区判定 (speed_low_threshold - speed_mid_threshold)
    else if (abs_speed < pid->speed_mid_threshold) {
        // 中速区使用基准参数（已初始化）
    }
    // 中速-高速过渡区
    else if (abs_speed < pid->speed_mid_threshold + SPEED_TRANSITION) {
        float weight = calculate_transition_weight(
            abs_speed, 
            pid->speed_mid_threshold, 
            pid->speed_mid_threshold + SPEED_TRANSITION
        );
        
        // 线性插值
        current_Kp = pid->base_Kp * (
            1.0f * (1.0f - weight) + 
            pid->high_speed_kp_factor * weight
        );
        current_Ki = pid->base_Ki * (
            1.0f * (1.0f - weight) + 
            pid->high_speed_ki_factor * weight
        );
        current_Kd = pid->base_Kd * (
            1.0f * (1.0f - weight) + 
            pid->high_speed_kd_factor * weight
        );
    }
    // 高速区判定 (speed_mid_threshold - speed_high_threshold)
    else if (abs_speed < pid->speed_high_threshold) {
        current_Kp *= pid->high_speed_kp_factor;
        current_Ki *= pid->high_speed_ki_factor;
        current_Kd *= pid->high_speed_kd_factor;
    }
    // 高速-极高速过渡区
    else if (abs_speed < pid->speed_high_threshold + SPEED_TRANSITION) {
        float weight = calculate_transition_weight(
            abs_speed, 
            pid->speed_high_threshold, 
            pid->speed_high_threshold + SPEED_TRANSITION
        );
        
        // 线性插值
        current_Kp = pid->base_Kp * (
            pid->high_speed_kp_factor * (1.0f - weight) + 
            pid->very_high_speed_kp_factor * weight
        );
        current_Ki = pid->base_Ki * (
            pid->high_speed_ki_factor * (1.0f - weight) + 
            pid->very_high_speed_ki_factor * weight
        );
        current_Kd = pid->base_Kd * (
            pid->high_speed_kd_factor * (1.0f - weight) + 
            pid->very_high_speed_kd_factor * weight
        );
    }
    // 极高速区判定 (> speed_high_threshold)
    else {
        current_Kp *= pid->very_high_speed_kp_factor;
        current_Ki *= pid->very_high_speed_ki_factor;
        current_Kd *= pid->very_high_speed_kd_factor;
    }
    
    // ===== 2. 基于误差的参数调整（叠加效果）=====
    const float ERROR_TRANSITION = 30.0f;
    
    // 大误差区判定 (> error_medium_threshold)
    if (abs_error > pid->error_medium_threshold) {
        current_Kp *= pid->large_error_kp_factor;
        current_Ki *= pid->large_error_ki_factor;
        current_Kd *= pid->large_error_kd_factor;
    }
    // 大-中误差过渡区
    else if (abs_error > pid->error_medium_threshold - ERROR_TRANSITION) {
        float weight = calculate_transition_weight(
            pid->error_medium_threshold - abs_error,
            0.0f,
            ERROR_TRANSITION
        );
        
        // 线性插值
        float kp_factor = pid->large_error_kp_factor * (1.0f - weight) + weight;
        float ki_factor = pid->large_error_ki_factor * (1.0f - weight) + weight;
        float kd_factor = pid->large_error_kd_factor * (1.0f - weight) + weight;
        
        current_Kp *= kp_factor;
        current_Ki *= ki_factor;
        current_Kd *= kd_factor;
    }
    // 中等误差区判定 (error_small_threshold - error_medium_threshold)
    else if (abs_error > pid->error_small_threshold) {
        // 中等误差区使用当前参数（已由速度区调整）
    }
    // 中-小误差过渡区
    else if (abs_error > pid->error_small_threshold - ERROR_TRANSITION) {
        float weight = calculate_transition_weight(
            pid->error_small_threshold - abs_error,
            0.0f,
            ERROR_TRANSITION
        );
        
        // 线性插值
        float kp_factor = 1.0f * (1.0f - weight) + pid->small_error_kp_factor * weight;
        float ki_factor = 1.0f * (1.0f - weight) + pid->small_error_ki_factor * weight;
        float kd_factor = 1.0f * (1.0f - weight) + pid->small_error_kd_factor * weight;
        
        current_Kp *= kp_factor;
        current_Ki *= ki_factor;
        current_Kd *= kd_factor;
    }
    // 小误差区判定 (< error_small_threshold)
    else {
        current_Kp *= pid->small_error_kp_factor;
        current_Ki *= pid->small_error_ki_factor;
        current_Kd *= pid->small_error_kd_factor;
    }
    
    // ===== 3. 参数平滑滤波（防止突变）=====
    const float FILTER_ALPHA = 0.85f; // 滤波系数，值越大越平滑
    pid->Kp = FILTER_ALPHA * pid->prev_Kp + (1.0f - FILTER_ALPHA) * current_Kp;
    pid->Ki = FILTER_ALPHA * pid->prev_Ki + (1.0f - FILTER_ALPHA) * current_Ki;
    pid->Kd = FILTER_ALPHA * pid->prev_Kd + (1.0f - FILTER_ALPHA) * current_Kd;
    
    // 更新上一次参数值
    pid->prev_Kp = pid->Kp;
    pid->prev_Ki = pid->Ki;
    pid->prev_Kd = pid->Kd;
    // ===== 核心：分段式变结构参数计算 END =====
    
    // 2. 微分先行核心计算
    float y_diff = measured_value - pid->prev_measured_value;
    pid->derivative = 0.7f * pid->derivative + 0.3f * y_diff;
    pid->prev_measured_value = measured_value;
    
    // 3. 遇限削弱积分处理
    float proportional = pid->Kp * error;
    float derivative_term = -pid->Kd * pid->derivative;
    
    // 计算无积分项的输出
    float output_without_integral = proportional + derivative_term;
    
    int should_integrate = 1;
    float saturation_threshold = pid->output_limit * 0.95f;
    
    // 检查是否达到输出限幅且误差方向会加剧饱和
    if (output_without_integral >= saturation_threshold && error > 0) {
        should_integrate = 0;
    } else if (output_without_integral <= -saturation_threshold && error < 0) {
        should_integrate = 0;
    }
    
    // 积分处理
    if (should_integrate && pid->Ki != 0.0f) {
        pid->integral += error * pid->dt;
    } else {
        // 遇限削弱
        pid->integral += error * pid->dt * 0.2f;
    }

    float max_integral = (pid->Ki != 0.0f) ? 
                         pid->output_limit / (pid->Ki * 1.2f) : 0.0f;
    if (pid->integral > max_integral)
        pid->integral = max_integral;
    else if (pid->integral < -max_integral)
        pid->integral = -max_integral;

    float output = proportional + 
                  (pid->Ki * pid->integral) +
                  derivative_term;
    if(output > pid->output_limit)
        output = pid->output_limit;
    else if(output < -pid->output_limit)
        output = -pid->output_limit;
        
    return output;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
if (hcan->Instance==CAN1)
{
     HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&RxMsg,rx_data);

		switch(RxMsg.StdId){
		case 0x20A://can口接受gm6020

			recept1.angle=(rx_data[0]<<8)|rx_data[1];
		  recept1.speed = (rx_data[2] << 8) | rx_data[3];//
		 recept1.i=(rx_data[4] << 8) | rx_data[5];
		recept1.temp=rx_data[6];
		
		
		case 0x200:
			recept3.angle=(rx_data[0]<<8)|rx_data[1];
		  recept3.speed = (rx_data[2] << 8) | rx_data[3];//
		 recept3.i=(rx_data[4] << 8) | rx_data[5];
		
		}
		
}}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{if (hcan->Instance==CAN2)
{
   HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO1,&RxMsg,rx_data2);
   switch(RxMsg.StdId){
		case 0x204://结合电调id，200+id
	

			recept2.angle=(rx_data2[0]<<8)|rx_data2[1];
		  recept2.speed = (rx_data2[2] << 8) | rx_data2[3];//
		 recept2.i=(rx_data2[4] << 8) | rx_data2[5];
	 

   case 0x201:
   recept3.angle=(rx_data2[0]<<8)|rx_data2[1];
		  recept3.speed = (rx_data2[2] << 8) | rx_data2[3];//
		 recept3.i=(rx_data2[4] << 8) | rx_data2[5];

		
	 
}
}
}
float check_jam(float setting_speed,float actual_speed,
                float actual_current,float current_angle,float dt) {
    uint32_t nows = HAL_GetTick();
     static uint32_t jam_detect_start = 0; // 新增：堵转检测计时
    static uint8_t jam_detected = 0;
    int is = ((fabsf(actual_speed) < 10) &&           // 几不动
                    (fabsf(actual_current) > 10000) &&         // 高电流
                    (fabsf(setting_speed) > 200));            

    switch (motor_state) {
        case NORMAL:
         
            if (is) {
                if (jam_detected == 0) {
                    jam_detect_start = nows;
                    jam_detected = 1;
                }
                // 持续100ms以上才判定为真堵转
                if (nows - jam_detect_start > 100 && jam_recount < max) {
                    motor_state = JAMMED;
                    start_time = nows;
                    raw_speed= setting_speed;
                    jam_recount++;
                }
            } else {
                jam_detected = 0; // 清除检测标记
            }
            break;

        case JAMMED:
            motor_state = BACKOFF;
            start_time = nows;
            return -setting_speed * 0.4f;  // 反向退出
            break;

        case BACKOFF:
         
            if (nows - start_time < 200) { 
                return -raw_speed* 0.5f;
            } else {
                motor_state = RECOVERING;
                start_time = nows;
                return raw_speed* 0.3f; // 尝试恢复正转
            }
            break;

        case RECOVERING:
           
            if (nows-start_time < 200) {
                return raw_speed * 0.3f;
            } else {
                motor_state =NORMAL;
            }
        default:
            break;
    }
// 返回setting速度
    return setting_speed;
}

void StartWatchdogTask(void const * argument)
{
    (void)argument;
    
    while (1)
    {
        uint32_t current_time = osKernelSysTick();
        uint8_t ok = 1;
        
        // 检查所有任务
        for (int i = 0; i < TASK_ID_MAX; i++) 
        {
            if (!task_monitors[i].is_active) 
                continue;
                
            uint32_t elapsed = current_time - task_monitors[i].last_heartbeat;
            
            // 考虑系统时间回绕
            if (elapsed > 0x7FFFFFFF) 
            {
                elapsed = 0xFFFFFFFF - task_monitors[i].last_heartbeat + current_time;
            }
            
            // 检查是否超时
            if (elapsed > task_monitors[i].timeout_ms) 
            {
                ok = 0;

            }
        }
        
        // 如果所有任务都正常，喂硬件看门狗
        if (ok) {
            Watchdog_FeedHardware();
        } else {
            // 如果有任务卡住，不喂狗，等待IWDG触发复位
            // 可以选择立即触发软复位
            // NVIC_SystemReset();
        }

        osDelay(WATCHDOG_TASK_DELAY_MS);
    }
}

void Watchdog_Init(void)
{
    // 初始化所有任务监控
    memset(task_monitors, 0, sizeof(task_monitors));
    
    // 配置各个任务
    task_monitors[TASK_ID_DEFAULT].id = TASK_ID_DEFAULT;
    task_monitors[TASK_ID_DEFAULT].name = "Default";
    task_monitors[TASK_ID_DEFAULT].timeout_ms = TASK_HEARTBEAT_TIMEOUT_MS;
    task_monitors[TASK_ID_DEFAULT].is_active = 1;
    
    task_monitors[TASK_ID_MYTASK02].id = TASK_ID_MYTASK02;
    task_monitors[TASK_ID_MYTASK02].name = "task2";
    task_monitors[TASK_ID_MYTASK02].timeout_ms = TASK_HEARTBEAT_TIMEOUT_MS;
    task_monitors[TASK_ID_MYTASK02].is_active = 1;
    
    task_monitors[TASK_ID_MYTASK03].id =TASK_ID_MYTASK03;
    task_monitors[TASK_ID_MYTASK03].name = "task3";
    task_monitors[TASK_ID_MYTASK03].timeout_ms = TASK_HEARTBEAT_TIMEOUT_MS;
    task_monitors[TASK_ID_MYTASK03].is_active = 1;
    
    // 记录当前时间作为初始心跳
    for (int i = 0; i < TASK_ID_MAX; i++) {
        if (task_monitors[i].is_active) {
            task_monitors[i].last_heartbeat = osKernelSysTick();
        }
    }
    
    // 创建监视任务
    osThreadDef(WatchdogTask, StartWatchdogTask, osPriorityAboveNormal, 0, 128);
    osThreadCreate(osThread(WatchdogTask), NULL);
}

// 任务心跳更新
void Watchdog_Heartbeat(TaskID task_id)
{
    if (task_id < TASK_ID_MAX && task_monitors[task_id].is_active) {
        task_monitors[task_id].last_heartbeat = osKernelSysTick();
    }
}

// 喂硬件看门狗
void Watchdog_FeedHardware(void)
{
    HAL_IWDG_Refresh(&hiwdg);
}

// 监视任务 - 检查所有任务心跳
float pre_angle=0;
float angleprocessing(int16_t angle2) {
  
    float current_angle = angle2 * 360.0f / 8192.0f;

    float diff = current_angle - pre_angle;
    if (diff > 180.0f) {
   diff-= 360.0f;  
      //   angle_continuous-= 360.0f;
			
    } else if (diff < -180.0f) {
         diff+= 360.0f;  
     //    angle_continuous+= 360.0f;
    }
  //   angle_continuous = angle_continuous - fmod(angle_continuous, 360.0f) + current_angle;
    pre_angle =  current_angle ;
		angle_continuous+= diff;
    return  angle_continuous;
	}
void DBUS_ParseData(uint8_t *pData) {
    if (pData == NULL) return;
    g_dbus_rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF;
	
    g_dbus_rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
    g_dbus_rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) | ((uint16_t)pData[4] << 10)) & 0x07FF;
    g_dbus_rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF; 
    g_dbus_rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2; 
    g_dbus_rc.s2 = ((pData[5] >> 4) & 0x0003); 
     g_dbus_mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8); 
    g_dbus_mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8); 
    g_dbus_mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);     
    g_dbus_mouse.press_l = pData[12]; 
    g_dbus_mouse.press_r = pData[13]; 
    g_dbus_key.v = ((int16_t)pData[14]);
	g_dbus_rc.roll= ((int16_t)pData[16] | ((int16_t)pData[17] << 8)) & 0x07FF;
	
}

//（ch0正→角度增，ch0负→角度减）
uint16_t DBUS_Ch0_Map_to_GM6020_Angle(void) {
    uint16_t target_angle = 0;
    int32_t ch0_dev = (int32_t)g_dbus_rc.ch0 - RC_CH0_MID; // ch0相对于中点的偏差

    if (abs(ch0_dev) < RC_CH0_DEAD_ZONE) {

        target_angle =192;
    }

    else {
        // 公式：目标角度 = 角度中点 + 偏差比例 × 角度范围半宽
        float scale = (float)(6557 - 192) / (RC_CH0_MAX - RC_CH0_MID);
        target_angle = (uint16_t)(192 + ch0_dev * scale);
    }

    if (target_angle >6557) 
			target_angle = 6557;
    if (target_angle < 192)
			target_angle = 192;

    return target_angle;
}


void   HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart,uint16_t Size)
{if(huart->Instance==USART1){
	if(Size==18){

	 DBUS_ParseData(g_dbus_rx_buf);
          rc_last_heartbeat = HAL_GetTick();
  //  g_dbus_idx = (g_dbus_idx + 1) % 2;  
     
		

  
 HAL_UARTEx_ReceiveToIdle_DMA(&huart1, g_dbus_rx_buf,18);
}
}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
 HAL_UARTEx_ReceiveToIdle_DMA(&huart1, g_dbus_rx_buf,18);

}

// 遥控器状态监控任务 10ms检查一次
void StartRCWatchdogTask(void) {
    
        uint32_t current_time = HAL_GetTick();
	      uint32_t   elapsed;
          if (current_time >= rc_last_heartbeat) {
        elapsed = current_time - rc_last_heartbeat;
    } else {
        elapsed = (0xFFFFFFFF - rc_last_heartbeat) + current_time;
    }
    
    // 超时阈值设为100ms（可根据实际遥控器帧率调整）
    if (elapsed > 100) {
        rc_online_flag = 0; // 遥控器失联
    } else {
        rc_online_flag = 1; // 遥控器在线
    }

    
}

void send_can_gm6020()
{
	float current_i = (float)recept1.i;
		if (mode == 0){
//         measured_value = (float)recept1.speed;
        float current = PID_Compute_local(&pid_controller, (float)recept1.speed);
       //current= block1(active_pid, error, current);
        current_pid.setpoint = current;
       float pid_output= (int16_t)PID_Compute_local(&current_pid, (float)current_i); 
      output_gm6020 =(int16_t)(pid_output);

			send[2] = (output_gm6020 >> 8) ;
     send[3] = (int8_t)output_gm6020 ;
//			send[0]=(2000>>8);
//			send[1]=(uint8_t)2000;
				Can_SendMsg(&hcan1, 0x2FF, send);
		}
        else if (mode == 1) {
						  if (rc_online_flag==0) {
        output_gm6020 = 0;
        memset(send, 0, 8);
        Can_SendMsg(&hcan1, 0x2FF, send);
        return;
    }
        // 2.1 ch0映射为GM6020目标角度
        uint16_t target_angle = DBUS_Ch0_Map_to_GM6020_Angle();
        // 2.2 角度环PID计算（目标=ch0角度，测量=反馈角度）
				uint16_t	target_angle1=angleprocessing(recept1.angle);
        angle_pid_controller.setpoint = (float)target_angle;
       float target_speed = PID_Compute_local(&angle_pid_controller, target_angle1);
   
        current_pid.setpoint = target_speed;
    float pid_output = PID_Compute_local(&current_pid, current_i);
    output_gm6020 = (int16_t)pid_output; 

 
 	send[2] = (output_gm6020 >> 8) ;
      send[3] = (int8_t)output_gm6020 ;   // 低8位
 
    Can_SendMsg(&hcan1, 0x2FF, send);    }  
}
		

void  send_can_m2006(){
	 if (!rc_online_flag) {
		   m2006_filtered_speed = 0.0f; // 失联时清零滤波
        pid_controller_m2006.integral = 0.0f; // 清零积分
        current_pid_m2006.integral = 0.0f;
        pid_controller_m2006.last_output = 0.0f; // 清零上次输出
        current_pid_m2006.last_output = 0.0f;
        output_m2006 = 0;
        memset(send2, 0, 8);
        Can_SendMsg(&hcan1, 0x200, send2);
        return;
    }
	float dt = 0.001f;

	    float current_i = (float)recept2.i;
	 int32_t roll_dev = (int32_t)g_dbus_rc.roll - RC_CH0_MID;
    float speed_ratio =(float) roll_dev / (float)(RC_CH0_MAX - RC_CH0_MID);
     if (abs(roll_dev) < RC_CH0_DEAD_ZONE) {
        roll_dev = 0;
     
        pid_controller_m2006.integral = 0.0f;
        current_pid_m2006.integral = 0.0f;
			   m2006_filtered_speed = 0.0f;
    }   
		speed_ratio = fconstrain(speed_ratio, -1.0f, 1.0f);
		if(roll_dev >25)
				    pid_controller_m2006.setpoint =  m2006_target_speed;
		if(roll_dev <-25)
			pid_controller_m2006.setpoint = - m2006_target_speed;
		if(roll_dev<=25&&roll_dev>=-25){
				pid_controller_m2006.setpoint = 0;
		  pid_controller_m2006.integral = 0.0f; // 清零积分
        current_pid_m2006.integral = 0.0f;}
//speed_ratio * m2006_max_speed;
//		   m2006_filtered_speed = 0.1 * target_speed + (1 - 0.1) * m2006_filtered_speed;
//	    pid_controller_m2006.setpoint =  m2006_filtered_speed;
//float current_angle = ((float)recept2.angle * 360.0f / 8191.0f);
float setting= PID_Compute_add(&pid_controller_m2006, (float)recept2.speed);
float new_speed = check_jam(setting,(float)recept2.speed,(float)recept2.i,0.0f,dt );
current_pid_m2006.setpoint = new_speed;
float pid_output= (int16_t)PID_Compute_add(&current_pid_m2006, (float)current_i); 
output_m2006 =(int16_t)(pid_output);
			send2[6] = (output_m2006 >> 8) ;
      send2[7] = (int8_t)output_m2006 ;
			Can_SendMsg(&hcan2, 0x200, send2);


}
void M2006init(int16_t target_speed, int16_t max_speed) {
    m2006_target_speed = target_speed;
    m2006_max_speed = max_speed;
   
}
//void  send1_can_gm3508(){
//     
//    float current_i = (float)recept3.i;
//	
//float current = PID_Compute_full(&pid_controller, (float)recept3.speed);
//        current_pid.setpoint = current;
//       float pid_output= (int16_t)PID_Compute_full(&current_pid, (float)current_i); 
//int16_t output_gm3508 =-(int16_t)(pid_output);

//			send3[0] = (output_gm3508>> 8) ;
//     send3[1] = (int8_t)output_gm3508 ;
//				Can_SendMsg(&hcan1, 0x200, send3);
//		}
//void  send2_can_gm3508(){
//     
//    float current_i = (float)recept3.i;
//	
//float current = PID_Compute_full(&pid_controller, (float)recept3.speed);
//        current_pid.setpoint = current;
//       float pid_output= (int16_t)PID_Compute_full(&current_pid, (float)current_i); 
//int16_t output2_gm3508  =(int16_t)(pid_output);

//			send4[0] = (output2_gm3508>> 8) ;
//     send4[1] = (int8_t)output2_gm3508  ;
//				Can_SendMsg(&hcan2, 0x200, send4);
//		}

void GM3508_Control(void)
{
    // 遥控器保护
    if (!rc_online_flag) {
        gm3508_target_speed = 0;
        gm3508_motor1_output = 0;
        gm3508_motor2_output = 0;
        gm3508_speed_pid.integral = 0;
        gm3508_current_pid.integral = 0;
        memset(send3, 0, 8);
        memset(send3, 0, 8);
        Can_SendMsg(&hcan1, 0x200, send3);
        Can_SendMsg(&hcan2, 0x201, send3);
    }

    // 2. 解析S1档位，设置目标速度
    switch(g_dbus_rc.s1) {
        case 1: // 低速运行
            gm3508_target_speed = GM3508_SPEED_GEAR1;
            break;
        case 2: // 停止
            gm3508_target_speed = 0;
            gm3508_speed_pid.integral = 0;
            gm3508_current_pid.integral = 0;
            break;
        case 3: // 高速运行
            gm3508_target_speed = GM3508_SPEED_GEAR2;
            break;
        default:
            gm3508_target_speed = 0;
            break;
    }


    gm3508_target_speed = (int16_t)fconstrain(gm3508_target_speed, - 2000, 2000);


    gm3508_speed_pid.setpoint = (float)gm3508_target_speed;
    float motor1_current_target = PID_Compute_full(&gm3508_speed_pid, (float)recept3.speed); // 速度环输出=电流内环目标
    gm3508_current_pid.setpoint = motor1_current_target;
    float motor1_pid_output = PID_Compute_full(&gm3508_current_pid, (float)recept3.i); // 电流内环计算
    gm3508_motor1_output = (int16_t)motor1_pid_output;
    gm3508_motor1_output = -gm3508_motor1_output; // 取反=顺时针旋转

   
    gm3508_speed_pid.setpoint = (float)gm3508_target_speed;
    float motor2_current_target = PID_Compute_full(&gm3508_speed_pid, (float)recept2.speed);
    gm3508_current_pid.setpoint = motor2_current_target;
    float motor2_pid_output = PID_Compute_full(&gm3508_current_pid, (float)recept2.i);
    gm3508_motor2_output = (int16_t)motor2_pid_output; // 正常=逆时针旋转

    send3[0] = (gm3508_motor1_output >> 8) ;
    send3[1] = (uint8_t)gm3508_motor1_output;
    send3[2] = (gm3508_motor2_output >> 8) ;
    send3[3] = (uint8_t)gm3508_motor2_output;

    // 7. 发送CAN指令
    Can_SendMsg(&hcan1, 0x200, send3);
    Can_SendMsg(&hcan2,0x201, send3);
}



		
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_TIM3_Init();
  MX_IWDG_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

 HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
	CAN_Filter_Init(&hcan1);
 HAL_CAN_Start(&hcan1);
 
	 HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING); 
 CAN_Filter_Init(&hcan2);
 HAL_CAN_Start(&hcan2);
//__HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
//__HAL_UART_ENABLE_IT(&huart1, UART_IT_ERR);
HAL_UARTEx_ReceiveToIdle_DMA(&huart1, g_dbus_rx_buf, 18);
  rc_last_heartbeat = HAL_GetTick();
 Watchdog_Init();
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
