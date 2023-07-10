#include "mpu6050_init.h"
#include "STM32F4_porting.h"
#include <stdint.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "stm32f4xx_hal.h"
#include "task.h"
#include "inv_mpu.h"
#include "eMPL_outputs.h"
#define get_tick_count get_ms_user

IMU_Result_data IMU_Result;

uint8_t mpu6050_wirte_byte(uint8_t addr, uint8_t reg, uint8_t dat)
{
  return Sensors_I2C_WriteRegister(addr, reg, 1, &dat);
}

uint8_t mpu6050_read_byte(uint8_t addr, uint8_t reg, uint8_t *dat)
{
  return Sensors_I2C_ReadRegister(addr, reg, 1, dat);
}

void mpu6050_sw_reset(void)
{
  mpu6050_wirte_byte(ATK_MS6050_IIC_ADDR, MPU_PWR_MGMT1_REG, 0x80);
  HAL_Delay(100);
  mpu6050_wirte_byte(ATK_MS6050_IIC_ADDR, MPU_PWR_MGMT1_REG, 0x00);

}

uint8_t mpu6050_set_gyro_fsr(uint8_t fsr)
{
  return mpu6050_wirte_byte(ATK_MS6050_IIC_ADDR, MPU_GYRO_CFG_REG, fsr << 3);
}

uint8_t mpu6050_set_accel_fsr(uint8_t fsr)
{
  return mpu6050_wirte_byte(ATK_MS6050_IIC_ADDR, MPU_ACCEL_CFG_REG, fsr << 3);
}

uint8_t mpu6050_set_lpf(uint16_t lpf)
{
  uint8_t dat;
    if (lpf >= 188)
    {
        dat = 1;
    }
    else if (lpf >= 98)
    {
        dat = 2;
    }
    else if (lpf >= 42)
    {
        dat = 3;
    }
    else if (lpf >= 20)
    {
        dat = 4;
    }
    else if (lpf >= 10)
    {
        dat = 5;
    }
    else
    {
        dat = 6;
    }
  return mpu6050_wirte_byte(ATK_MS6050_IIC_ADDR, MPU_CFG_REG, dat);
}

uint8_t mpu6050_set_rate(uint16_t rate)
{
    uint8_t ret;
    uint8_t dat;
    
    if (rate > 1000)
    {
        rate = 1000;
    }
    
    if (rate < 4)
    {
        rate = 4;
    }
    
    dat = 1000 / rate - 1;
    ret = mpu6050_wirte_byte(ATK_MS6050_IIC_ADDR, MPU_SAMPLE_RATE_REG, dat);
    if (ret != ATK_MS6050_EOK)
    {
        return ret;
    }
    
    ret = mpu6050_set_lpf(rate >> 1);
    if (ret != ATK_MS6050_EOK)
    {
        return ret;
    }
    
    return ATK_MS6050_EOK;
}

uint8_t mpu6050_get_temperature(int16_t *temp)
{
    uint8_t dat[2];
    uint8_t ret;
    int16_t raw = 0;
    
    ret = Sensors_I2C_ReadRegister(ATK_MS6050_IIC_ADDR, MPU_TEMP_OUTH_REG, 2, dat);
    if (ret == ATK_MS6050_EOK)
    {
        raw = ((uint16_t)dat[0] << 8) | dat[1];
        *temp = (int16_t)((36.53f + ((float)raw / 340)) * 100);
    }
    
    return ret;
}

/**
 * @brief       ATK-MS6050获取陀螺仪值
 * @param       gx，gy，gz: 陀螺仪x、y、z轴的原始度数（带符号）
 * @retval      ATK_MS6050_EOK : 函数执行成功
 *              ATK_MS6050_EACK: IIC通讯ACK错误，函数执行失败
 */
uint8_t mpu6050_get_gyroscope(int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t dat[6];
    uint8_t ret;
    
    ret =  Sensors_I2C_ReadRegister(ATK_MS6050_IIC_ADDR, MPU_GYRO_XOUTH_REG, 6, dat);
    if (ret == ATK_MS6050_EOK)
    {
        *gx = ((uint16_t)dat[0] << 8) | dat[1];
        *gy = ((uint16_t)dat[2] << 8) | dat[3];
        *gz = ((uint16_t)dat[4] << 8) | dat[5];
    }
    
    return ret;
}

/**
 * @brief       ATK-MS6050获取加速度值
 * @param       ax，ay，az: 加速度x、y、z轴的原始度数（带符号）
 * @retval      ATK_MS6050_EOK : 函数执行成功
 *              ATK_MS6050_EACK: IIC通讯ACK错误，函数执行失败
 */
uint8_t mpu6050_get_accelerometer(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t dat[6];
    uint8_t ret;
    
    ret =  Sensors_I2C_ReadRegister(ATK_MS6050_IIC_ADDR, MPU_ACCEL_XOUTH_REG, 6, dat);
    if (ret == ATK_MS6050_EOK)
    {
        *ax = ((uint16_t)dat[0] << 8) | dat[1];
        *ay = ((uint16_t)dat[2] << 8) | dat[3];
        *az = ((uint16_t)dat[4] << 8) | dat[5];
    }
    
    return ret;
}
// uint8_t mpu6050_init(void)
// {
//   uint8_t id;
//   uint8_t ret;
//   mpu6050_sw_reset();
//   mpu6050_set_gyro_fsr(3);
//   mpu6050_set_accel_fsr(0);
//   mpu6050_set_rate(100);
//   mpu6050_wirte_byte(ATK_MS6050_IIC_ADDR, MPU_INT_EN_REG, 0x00);
//   mpu6050_wirte_byte(ATK_MS6050_IIC_ADDR, MPU_USER_CTRL_REG, 0x00);
//   mpu6050_wirte_byte(ATK_MS6050_IIC_ADDR, MPU_FIFO_EN_REG, 0x00);
//   mpu6050_wirte_byte(ATK_MS6050_IIC_ADDR, MPU_INTBP_CFG_REG, 0x80);
//   mpu6050_read_byte(ATK_MS6050_IIC_ADDR, MPU_DEVICE_ID_REG, &id);
//   if(id != ATK_MS6050_IIC_ADDR)
//   {
//     return ATK_MS6050_EID;
//   }
//   mpu6050_wirte_byte(ATK_MS6050_IIC_ADDR, MPU_PWR_MGMT1_REG, 0x01);
//   mpu6050_wirte_byte(ATK_MS6050_IIC_ADDR, MPU_PWR_MGMT2_REG, 0x00);
//   mpu6050_set_rate(100);
//
//   ret = mpu6050_dmp_init();
//   if(ret != 0)
//   {
//     printf("DMP init Failed!\n");
//   }
//   return ATK_MS6050_EOK;
//
// }
//
// void mpu6050_getData(void)
// {
//     uint8_t ret;
//     float pit, rol, yaw;
//     int16_t temp;
// unsigned long timestamp;
// get_tick_count(&timestamp);
//             /* 获取mpu6050 DMP处理后的数据 */
//         ret  = mpu6050_dmp_get_data(&pit, &rol, &yaw);
//         // ret = mpu6050_get_accelerometer(&acc_x, &acc_y,&acc_z );
//         // ret = mpu6050_get_gyroscope(&gyr_x, &gyr_y, &gyr_z);
// // if (inv_get_sensor_type_gyro(data, &accuracy, (inv_time_t *)&timestamp)) {
// //      IMU_Result.GyroX = data[0] * 1.0 / (1 << 16);
// //      IMU_Result.GyroY = data[1] * 1.0 / (1 << 16);
// //      IMU_Result.GyroZ = data[2] * 1.0 / (1 << 16);
// //       printf("\n角速度(degree/s): %d   %d   %d  \n", (int)IMU_Result.GyroX,
// //              (int)IMU_Result.GyroX, (int)IMU_Result.GyroX);
// //     }
//         // ret = mpu_get_gyro_reg()
//         // ret = mpu6050_get_temperature(&temp);
//         if(ret ==0)
//         {
//                 IMU_Result.EulerX = pit;
//                 IMU_Result.EulerY = rol;
//                 IMU_Result.EulerZ = yaw;
//                 printf("\npit: %d, rol: %d, yaw: %d\n", (int)pit, (int)rol, (int)yaw);
//                 // printf("acc_x: %d, acc_y: %d, acc_z: %d, ", acc_x, acc_y, acc_z);
//                 // printf("\ngyr_x: %d, gyr_y: %d, gyr_z: %d\n", gyr_x, gyr_y, gyr_z);
//                 // printf("temp: %d\r\n", temp);
//         }
//
// }







