#include "main.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "Dshot.h"
#include "FreeRTOS.h"
#include "driver/system.h"
#include "driver/uart.h"
#include "pidControl.h"
#include "portmacro.h"
#include "projdefs.h"
#include "rx/crsf.h"
#include "rx/rx.h"
#include "semphr.h"
#include "sensor/mpu6050.h"
#include "stm32f411xe.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_it.h"
#include "micros.h"
#include "task.h"
#include "timecouter.h"
#include "mpu6050_init.h"
#include "inv_mpu.h"

void SystemClock_Config(void);
void MX_GPIO_Init(void);
void MX_DMA_Init(void);
void MX_UART_Init(void);
void Error_Handler(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);


extern pidStruct XGyroPid, YGyroPid, ZGyroPid;
extern pidStruct XEulerPid, YEulerPid, ZEulerPid;
extern uint8_t currentValue;
//pid内外环 参数
float innerKi = 0.02;
float innerKp = 2.34;
float innerKd = 0.0125; //0.0013
float outerKi = 0.02;
// float outerPitchKi = 0.50;
float outerKp = 1.55;
float outerKd = 0.000;
extern uint16_t my_motor_value[4];
static TaskHandle_t xdshotTask = NULL, xControlTask = NULL, xPIDTask = NULL,xPIDGyro;


void vTask1(void *pvParameters) {
  for (;;) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
    vTaskDelay(500);
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
    vTaskDelay(500);
  }
}

void vTask2(void *pvParameters) {
const TickType_t xTickToReceiverTimeOut = pdMS_TO_TICKS(800);
TickType_t currentTickTime;
TickType_t receiveStart;
 bool whetheronce = false;
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
receiveStart = xTaskGetTickCount();
    do {
      ReceiveCRSFdataFromUart();
      if (whetheronce == true) {
       currentTickTime = xTaskGetTickCount();
          if (currentTickTime - receiveStart > xTickToReceiverTimeOut) {
              my_motor_value[0] =0;
              my_motor_value[1] =0;
              my_motor_value[2] =0;
              my_motor_value[3] =0;
              dshot_write(my_motor_value);
       }
      }

    } while (RX_FRAME_COMPLETE != crsfFrameStatus());
    whetheronce = true;

  }
}
void vTask3(void *pvParameters) {
  //遥控器接受任务：30ms一帧
  //PID外环（角度环）计算：200hz
  //PID内环（角速度环）计算：500hz
  const TickType_t xTickToReceiver = pdMS_TO_TICKS(30);
  const TickType_t xTickToPID = pdMS_TO_TICKS(5);
  const TickType_t xTickToPIDGyro = pdMS_TO_TICKS(2);
  TickType_t receiveStartTick = xTaskGetTickCount();
  TickType_t pidStartTick     = receiveStartTick;
  TickType_t pidGyroTick      = receiveStartTick;
  TickType_t currentTickValue;
  for (;;) {
    dshot_write(my_motor_value);
    currentTickValue = xTaskGetTickCount();
    if (currentTickValue - receiveStartTick > xTickToReceiver) {
      receiveStartTick = currentTickValue;
      xTaskNotifyGive(xControlTask);
    } else if (currentTickValue - pidStartTick > xTickToPID) {
      pidStartTick = currentTickValue;
      xTaskNotifyGive(xPIDTask);
    } else if (currentTickValue - pidGyroTick > xTickToPIDGyro) {
      pidGyroTick = currentTickValue;
      xTaskNotifyGive(xPIDGyro);
    }
  }
}
void vTask4(void *pvParameters) {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    pidControl();
  }
}
void vTask5(void *pvParameters){
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    pidControlGyro();
  }
}
/*
 * MOTOR 1 (PA0),
 * MOTOR 2 (PA1),
 * MOTOR 3 (PA3),
 * MOTOR 4 (PA4),
 * Location Of Motor:
 *       MOTOR 4         MOTOR 2
 *                  O
 *       MOTOR 3         MOTOR 1
 */
int main(void) {
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  //DMA中断设置
  MX_DMA_Init();
  // MX_UART_Init(); //UART Interrupt Initializes;
  // DSHOT协议定时器初始化
  MX_TIM2_Init();
  MX_TIM5_Init();
  dshot_init();
  uart_init();
  cycleCounterInit();
  // timecouter();
  MX_I2C1_Init();
  DWT_Init();
  MPU6050_Init();
  //pid结构体初始化
  pidInit(&XGyroPid, innerKp, innerKi, innerKd, 100.0,0.002,280.0);
  pidInit(&YGyroPid, innerKp, innerKi, innerKd, 100.0,0.002,280.0);
  pidInit(&ZGyroPid, outerKp, outerKi, outerKd, 100.0,0.002,280.0);
  pidInit(&XEulerPid, outerKp, outerKi, outerKd, 30.0,0.005,280.0);
  pidInit(&YEulerPid, outerKp, outerKi, outerKd, 30.0,0.005,280.0);
  pidInit(&ZEulerPid, outerKp, outerKi, outerKd, 30.0,0.005,280.0);
  //
  // xTaskCreate(vTask1, "LED1", 128, NULL, 1, NULL);
  xTaskCreate(vTask2, "Control", 512, NULL, 4, &xControlTask);
  xTaskCreate(vTask3, "dshot", 512, NULL, 1, &xdshotTask);
  xTaskCreate(vTask5, "PID_GyroSpeed",256,NULL,2,&xPIDGyro);
  xTaskCreate(vTask4, "PID", 256, NULL, 3, &xPIDTask);
  vTaskStartScheduler();

  while (1) {
  }

  return 0;
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  /** Initializes the CPU, AHB and APB busses clocks
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;

  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
    Error_Handler();
  }
  /** Enables the Clock Security System
   */
  HAL_RCC_EnableCSS();
}

void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}

void MX_DMA_Init(void) {
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();
  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 2, 1);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 2, 1);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 2, 1);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 2, 1);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA2_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream5_IRQn, 2, 1);
  HAL_NVIC_EnableIRQ(DMA2_Stream5_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 2, 1);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 2, 1);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, 2, 1);
  HAL_NVIC_EnableIRQ(DMA1_Stream7_IRQn);
}

void MX_UART_Init(void) {
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  HAL_NVIC_SetPriority(USART1_IRQn, 2, 1);
}

// Init the GPIO C13 B5 B4
void MX_GPIO_Init(void) {
  GPIO_InitTypeDef GPIO_InitStruct, GPIO_InitStruct2 = {0};
  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  GPIO_InitStruct2.Pin = GPIO_PIN_13;
  GPIO_InitStruct2.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct2.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct2.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct2);

  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM11 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM11) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}
