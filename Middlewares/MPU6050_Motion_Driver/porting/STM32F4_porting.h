#pragma once

#include "iic.h"
#include "main.h"

#define MPU6050_I2C_Handle hi2c1

#define I2Cx_FLAG_TIMEOUT ((uint32_t)1000) // 超时时间 Timeout duration
#define I2Cx_LONG_TIMEOUT ((uint32_t)(300 * I2Cx_FLAG_TIMEOUT))

int Sensors_I2C_ReadRegister(unsigned char slave_addr, unsigned char reg_addr, unsigned short len,
                             unsigned char *data_ptr);
int Sensors_I2C_WriteRegister(unsigned char slave_addr, unsigned char reg_addr, unsigned short len,
                              unsigned char *data_ptr);

int get_ms_user(unsigned long *count);
