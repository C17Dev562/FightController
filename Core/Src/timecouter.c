#include "timecouter.h"
#include "main.h"
#include "stm32f411xe.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_tim.h"
#include <stdint.h>

TIM_HandleTypeDef htim10;

void timecouter(void){
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 49999;
  htim10.Init.Period = 65535;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim10)!= HAL_OK) {
   Error_Handler();
  }
}

unsigned long getTime(void)
{
  uint32_t CNTCOUTER = htim10.Instance->CNT;
  unsigned long time = CNTCOUTER/2;
  return time;
}
