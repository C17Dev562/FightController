#include "STM32F4_porting.h"

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_i2c.h"
#include "FreeRTOS.h"
#include "task.h"
#include "micros.h"
#include "timecouter.h"


static void I2Cx_Error(uint8_t Addr) {
  /* 恢复I2C寄存器为默认值 */
  HAL_I2C_DeInit(&MPU6050_I2C_Handle);
  MX_I2C1_Init();
}

/**
 * @brif 读寄存器，这是提供给上层对借口
 * @param slave_addr: 从机地址
 * @param reg_addr: 寄存器地址
 * @param len: 要读取对长度
 * @param data_ptr: 指向要存储数据对指针
 * @retval 正常为0，不正常为非0
 */
int Sensors_I2C_ReadRegister(unsigned char slave_addr, unsigned char reg_addr,
                             unsigned short len, unsigned char *data_ptr) {
  HAL_StatusTypeDef status = HAL_OK;
  slave_addr <<= 1;
  status =
      // HAL_I2C_Mem_Read(&MPU6050_I2C_Handle, slave_addr, reg_addr,
                       // I2C_MEMADD_SIZE_8BIT, data_ptr, len, I2Cx_FLAG_TIMEOUT);
      HAL_I2C_Mem_Read_DMA(&MPU6050_I2C_Handle, slave_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data_ptr, len);

  /* 检查通讯状态 */
  if (status != HAL_OK) {
    I2Cx_Error(slave_addr);
  }
  while (HAL_I2C_GetState(&MPU6050_I2C_Handle) != HAL_I2C_STATE_READY) {
  }
  /* 检查SENSOR是否就绪进行下一次读写操作 */
  while (HAL_I2C_IsDeviceReady(&MPU6050_I2C_Handle, slave_addr,
                               I2Cx_FLAG_TIMEOUT,
                               I2Cx_FLAG_TIMEOUT) == HAL_TIMEOUT)
    ;
  /* 等待传输结束 */
  while (HAL_I2C_GetState(&MPU6050_I2C_Handle) != HAL_I2C_STATE_READY) {
  }
  return status;
}

/**
 * @brief  写寄存器，这是提供给上层的接口
 * @param  slave_addr: 从机地址
 * @param 	reg_addr:寄存器地址
 * @param len：写入的长度
 *	@param data_ptr:指向要写入的数据
 * @retval 正常为0，不正常为非0
 */
int Sensors_I2C_WriteRegister(unsigned char slave_addr, unsigned char reg_addr, unsigned short len,
                              unsigned char *data_ptr)
{
    HAL_StatusTypeDef status = HAL_OK;
    slave_addr <<= 1;
    status = 
    // HAL_I2C_Mem_Write(&MPU6050_I2C_Handle, slave_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data_ptr, len,
                               // I2Cx_FLAG_TIMEOUT);
    HAL_I2C_Mem_Write_DMA(&MPU6050_I2C_Handle, slave_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, data_ptr, len);
    /* 检查通讯状态 */
    if (status != HAL_OK)
    {
        /* 总线出错处理 */
        I2Cx_Error(slave_addr);
    }
    while (HAL_I2C_GetState(&MPU6050_I2C_Handle) != HAL_I2C_STATE_READY)
    {
    }
    /* 检查SENSOR是否就绪进行下一次读写操作 */
    while (HAL_I2C_IsDeviceReady(&MPU6050_I2C_Handle, slave_addr, I2Cx_FLAG_TIMEOUT, I2Cx_FLAG_TIMEOUT) == HAL_TIMEOUT)
        ;
    /* 等待传输结束 */
    while (HAL_I2C_GetState(&MPU6050_I2C_Handle) != HAL_I2C_STATE_READY)
    {
    }
    return status;
}
int get_ms_user(unsigned long *count) //换壳函数  inline修饰（内联）减少出入栈开销
{
    if (!count)
        return 1;
    // count[0] = HAL_GetTick();
       // *count = getTime();
      *count = xTaskGetTickCount();

    return 0;
}
