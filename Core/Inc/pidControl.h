#pragma once
#include "main.h"
#include "stm32f4xx_hal.h"
#include "mpu6050_SL.h"
#include <stdbool.h>

#define PID_CALCULATE_COMPLETE 1
#define OUT_LOOP 1
#define INTER_LOOP 0
#define M_PI_F 3.1415926


typedef struct
{
  // bool WhetherEuler;
  float desired;  // 期望值
  float error;    // 误差值
  float preError; //先前误差
  float integration; //积分
  float derivation; //微分
  float Kp;         //比例系数
  float Ki;         //积分系数
  float Kd;         //微分系数
  float outP;        //pid比例值
  float outI;        //pid积分值
  float outD;       //pid微分值
  float iLimit;     //积分限制
  float pidResult;  //pid输出结果
  float dt;
	float outputLimit;  //< total PID output limit, absolute value. '0' means no limit.
} pidStruct;

typedef struct
{
	uint32_t timestamp;	/*时间戳*/

	float roll;
	float pitch;
	float yaw;
} attitude_t;

void pidControl(void);
void pidControlGyro(void);
void pidInit(pidStruct *pidStruct, float Kp, float Ki, float Kd, float iLimit,float dT,float outputLimit);
void pidReset(pidStruct* pid);
void pidSetDt(pidStruct* pid, const float dt);
void pidSetKd(pidStruct* pid, const float kd);
void pidSetKi(pidStruct* pid, const float ki);
void pidSetKp(pidStruct* pid, const float kp);
float pidGetDesired(pidStruct* pid);
void pidSetDesired(pidStruct* pid, const float desired);
void pidSetError(pidStruct* pid, const float error);
void pidSetOutputLimit(pidStruct* pid, const float limit);
void pidSetIntegralLimit(pidStruct* pid, const float limit);





