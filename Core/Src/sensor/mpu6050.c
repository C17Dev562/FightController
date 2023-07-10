//此源文件暂不使用，不纳入编译，MPU6050的DMP库值Middlewares文件夹
#include "sensor/mpu6050.h"

#include <stdint.h>
#include <stdio.h>

#include "driver/iic.h"
#include "stm32f4xx_hal_i2c.h"
#include "driver/uart.h"
#include "stm32f4xx_hal_uart.h"

uint8_t dataBuffer[6];
uint8_t MPU6050_Address = MPU6050_DEFAULT_ADDRESS << 1;
// i2c function to write a byte to mpu6050
void i2cSend(uint16_t registerAddrs, uint8_t byte) {
  // HAL_I2C_Mem_Write_DMA(&hi2c1, MPU6050_Address , registerAddrs,I2C_MEMADD_SIZE_8BIT,
  //                       &byte, 1);
  HAL_I2C_Mem_Write(&hi2c1,MPU6050_Address,registerAddrs,I2C_MEMADD_SIZE_8BIT,&byte,1,1000);
}

// i2c function to read a byte from mpu6050
uint8_t i2cRead(uint16_t registerAddrs, uint8_t data, uint16_t length) {
  // int16_t MemAddSize;
  // MemAddSize = sizeof(data);
  // HAL_I2C_Mem_Read_DMA(&hi2c1,MPU6050_Address, registerAddrs,I2C_MEMADD_SIZE_8BIT,&data,
  //                      length);
     HAL_I2C_Mem_Read(&hi2c1,MPU6050_DEFAULT_ADDRESS<<1,registerAddrs,I2C_MEMADD_SIZE_8BIT,&data,length,1000);
  return data;
}

// i2c function to read a byte from mpu6050
uint8_t i2cReadSingeByte(uint16_t registerAddrs) {
  uint8_t data;
  // HAL_I2C_Mem_Read_DMA(&hi2c1, MPU6050_DEFAULT_ADDRESS<<1, registerAddrs,I2C_MEMADD_SIZE_8BIT,&data,
  //                      1);
   HAL_I2C_Mem_Read(&hi2c1, MPU6050_DEFAULT_ADDRESS<<1, registerAddrs, I2C_MEMADD_SIZE_8BIT, &data, 1, 1000);
  return data;
}

uint8_t readDeiveAddress(void) {
  uint8_t template;
  // HAL_I2C_Mem_Read_DMA(&hi2c1, MPU6050_DEFAULT_ADDRESS <<1, MPU6050_RA_WHO_AM_I, I2C_MEMADD_SIZE_8BIT,
                       // &template, 1);
   HAL_I2C_Mem_Read(&hi2c1, MPU6050_Address , MPU6050_RA_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &template, 1, 1000);
  return template;
}

uint8_t initMpuDevice(void) {
  // if (0x68 == i2cReadSingeByte(MPU6050_RA_WHO_AM_I)) {
    uint8_t data = 0x00;
    i2cSend(MPU6050_RA_PWR_MGMT_1, data);
    data = 0x07;  // Div frequancy set data rate of 1Khz
    i2cSend(MPU6050_RA_SMPLRT_DIV, data);
    // // Set accelerometer configuration in ACCEL_CONFIG Register
    // // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 ->  2g
    data = 0x00;
    i2cSend(MPU6050_RA_GYRO_CONFIG, data);
    data = 0x00;
    i2cSend(MPU6050_RA_ACCEL_CONFIG, data);
    return data;
  // }
  // return 1;
}

void readAccelData(MPU6050_t *DataStruct) {
  i2cRead(MPU6050_RA_ACCEL_XOUT_H,*dataBuffer, 6);
  DataStruct->Accel_X_RAW = (uint16_t)(dataBuffer[0] << 8 | dataBuffer[1]);
  DataStruct->Accel_Y_RAW = (uint16_t)(dataBuffer[2] << 8 | dataBuffer[3]);
  DataStruct->Accel_Z_RAW = (uint16_t)(dataBuffer[4] << 8 | dataBuffer[5]);

  DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
  DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
  DataStruct->Az = DataStruct->Accel_Z_RAW / 16384.0;
  // printf("accleX data = %d\n", (int)DataStruct->Ax);
  // printf("accleY data = %d\n", (int)DataStruct->Ay);
  // printf("accleZ data = %d\n", (int)DataStruct->Az);
}

void readGyroData(MPU6050_t *DataStruct) {
  // i2cRead(MPU6050_RA_GYRO_XOUT_H, *dataBuffer, 1);
  HAL_I2C_Mem_Read(&hi2c1, MPU6050_Address, MPU6050_RA_GYRO_XOUT_H,I2C_MEMADD_SIZE_8BIT , dataBuffer, 6, 1000);
  DataStruct->Gyro_X_RAW = (uint16_t)(dataBuffer[0] << 8 | dataBuffer[1]);
  DataStruct->Gyro_Y_RAW = (uint16_t)(dataBuffer[2] << 8 | dataBuffer[3]);
  DataStruct->Gyro_Z_RAW = (uint16_t)(dataBuffer[4] << 8 | dataBuffer[5]);

  DataStruct->Gx = (double)((int)(DataStruct->Gyro_X_RAW) / 131.0);
  DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
  DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;
  printf("GyroX data = %d\n",(int)DataStruct->Gx);
  // printf("GyroY data = %.2f\n",DataStruct->Gy);
  // printf("GyroZ data = %.2f\n", DataStruct->Gz);
  printf("test value = %d\n",DataStruct->Gyro_X_RAW);
 // HAL_UART_Transmit_DMA(&UART1_Handler,(uint8_t*)&DataStruct->Gyro_Y_RAW, 1);
}
