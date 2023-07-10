#include "pidControl.h"

#include <results_holder.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "Dshot.h"
#include "Dshot_tim.h"
#include "FreeRTOS.h"
#include "crsf.h"
#include "math.h"
#include "maths.h"
#include "micros.h"
#include "mpu6050_SL.h"
#include "mpu6050_init.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"
#include "system.h"
#include "task.h"

uint16_t my_motor_value[4] = {0, 0, 0, 0};

static float Thro = 0.0, Roll = 0.0, Pitch = 0.0, Yaw = 0.0;
pidStruct XGyroPid, YGyroPid, ZGyroPid;
pidStruct XEulerPid, YEulerPid, ZEulerPid;
attitude_t rateDesired;
extern uint16_t ThroValue;
extern RcAngleRateData AngleEXPRateData;
extern IMU_Result_data IMU_Result;
extern uint32_t crsfChannelData[CRSF_MAX_CHANNEL];
// static int controlAngleSpeed(void);
// static float pidCalculate(pidStruct *pidStruct, float expectedValue,
                          // float currentValue);
// static void controlGyroSpeed(void);

void attitudeRatePID(void);
void attitudeAnglePID(void);
void controlMotor(void);
float pidUpdate(pidStruct *pid,const float error);


void pidInit(pidStruct *pidStruct, float Kp, float Ki, float Kd, float iLimit,float dT,float outputLimit)
{
  pidStruct->Ki = Ki;
  pidStruct->Kd = Kd;
  pidStruct->Kp = Kp;
  pidStruct->iLimit = iLimit;
  pidStruct->integration = 0;
  pidStruct->dt = dT;
  pidStruct->outputLimit = outputLimit;
}
void pidControl(void) {
  int ret = 0;
  //判断是否为自稳模式
  if (ret == MPU6050_getValue() && crsfChannelData[5] < 992) {
    attitudeAnglePID();
  }
}
void pidControlGyro(void){
  mpu6050_get_gyrodata();
if(IMU_Result.GyroX <600 && IMU_Result.GyroZ < 500 && IMU_Result.GyroY < 500 &&
     IMU_Result.GyroX > -600 && IMU_Result.GyroY > -600 && IMU_Result.GyroZ > -600){
      attitudeRatePID();
      controlMotor();
  }
}



float pidUpdate(pidStruct *pid,const float error)
{
  float output;

  pid->error = error;
  pid->integration += pid->error * pid-> dt;
  //积分限幅
  if(pid->integration > pid->iLimit)
  {
    pid->integration = pid->iLimit;
  }
  else if (pid->integration < -pid->iLimit)
  {
    pid->integration = -pid->iLimit;
  }
 pid->derivation = (pid->error - pid->preError) / pid->dt;

  pid->outP = pid->Kp * pid->error;
  pid->outI = pid->Ki * pid->integration;
  pid->outD = pid->Kd * pid->derivation;


  output = pid->outP + pid->outI + pid->outD;
  //输出限幅
  if(pid->outputLimit !=0 )
  {
    if(output > pid->outputLimit)
      output = pid->outputLimit;
    else if (output < -pid->outputLimit)
        output = -pid->outputLimit;
  }

  pid->preError = pid->error;
  pid->pidResult = output;
  return output;
}
//外环计算
void attitudeAnglePID(void)
{
 rateDesired.yaw   = pidUpdate(&ZGyroPid, AngleEXPRateData.YawEXPAngleRate - IMU_Result.EulerZ);
 rateDesired.pitch = pidUpdate(&XEulerPid, AngleEXPRateData.PitchEXPAngleRate - IMU_Result.EulerX);
 rateDesired.roll  = pidUpdate(&YEulerPid, AngleEXPRateData.RollEXPAngleRate - IMU_Result.EulerY);
}
//内环计算
void attitudeRatePID(void)
{
  //自稳模式 500hz
  if (crsfChannelData[5] < 992) {
    // Yaw   = pidUpdate(&ZGyroPid, rateDesired.yaw - IMU_Result.GyroZ);
    Yaw  = pidUpdate(&ZGyroPid, AngleEXPRateData.YawEXPAngleRate - IMU_Result.GyroZ);
    Pitch = pidUpdate(&XEulerPid, rateDesired.pitch - IMU_Result.GyroX);
    Roll  = pidUpdate(&YEulerPid, rateDesired.roll - IMU_Result.GyroY);
  }
  else if (crsfChannelData[5] >992) {
    Pitch = pidUpdate(&XGyroPid, AngleEXPRateData.PitchEXPAngleRate - IMU_Result.GyroX);
    Roll = pidUpdate(&YGyroPid, AngleEXPRateData.RollEXPAngleRate - IMU_Result.GyroY);
    Yaw  = pidUpdate(&ZGyroPid, AngleEXPRateData.YawEXPAngleRate - IMU_Result.GyroZ);
  }
 }

void controlMotor(void) {
  // 开机油门摇杆下拉初始化电机
  if (crsfChannelData[2] < 185 ) {
    my_motor_value[0] = 0;
    my_motor_value[1] = 0;
    my_motor_value[2] = 0;
    my_motor_value[3] = 0;
  } else if (crsfChannelData[4] > 1300) {
    my_motor_value[0] = 0;
    my_motor_value[1] = 0;
    my_motor_value[2] = 0;
    my_motor_value[3] = 0;
  } else  {
    Thro = ThroValue;
    // Pitch = Pitch / 2.0f;
    // Roll = Roll / 2.0f;
    //PA3 -> M4
    //PA2 -> M3     M4     M2
    //PA0 -> M1         o
    //PA1 -> M2     M3     M1
    my_motor_value[0] = CONSTRAIN((0.6 * Thro + Pitch + Roll + Yaw),ANALOG_MIN,ANALOG_MAX);
    my_motor_value[1] = CONSTRAIN((0.6 * Thro - Pitch + Roll - Yaw),ANALOG_MIN,ANALOG_MAX);
    my_motor_value[2] = CONSTRAIN((0.6 * Thro - Pitch - Roll + Yaw),ANALOG_MIN,ANALOG_MAX);
    my_motor_value[3] = CONSTRAIN((0.6 * Thro + Pitch - Roll - Yaw),ANALOG_MIN,ANALOG_MAX);
    dshot_write(my_motor_value);
    // printf("\nM4= %d,M3= %d,M2= %d,M1= %d\n,Pitch= %d Roll= %d,Yaw= %d\n",my_motor_value[0],my_motor_value[1],my_motor_value[3],my_motor_value[2]
           // ,(int)Pitch,(int)Roll,(int)Yaw);
      }
}

void pidSetIntegralLimit(pidStruct* pid, const float limit)
{
    pid->iLimit = limit;
}

void pidSetOutputLimit(pidStruct* pid, const float limit)
{
	pid->outputLimit = limit;
}

void pidSetError(pidStruct* pid, const float error)
{
	pid->error = error;
}

void pidSetDesired(pidStruct* pid, const float desired)
{
	pid->desired = desired;
}

float pidGetDesired(pidStruct* pid)
{
	return pid->desired;
}

void pidSetKp(pidStruct* pid, const float kp)
{
	pid->Kp = kp;
}

void pidSetKi(pidStruct* pid, const float ki)
{
	pid->Ki = ki;
}

void pidSetKd(pidStruct* pid, const float kd)
{
	pid->Kd = kd;
}

void pidSetDt(pidStruct* pid, const float dt)
{
    pid->dt = dt;
}

void pidReset(pidStruct* pid)
{
	pid->error     = 0;
	pid->preError = 0;
	pid->integration     = 0;
	pid->derivation     = 0;
}
