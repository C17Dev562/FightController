#include "driver/system.h"
#include  "FreeRTOS.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_rcc.h"
#include "task.h"
#include <stdint.h>

volatile int sysTickPending = 0;
volatile uint32_t sysTickUptime = 0;
volatile uint32_t sysTickValStamp = 0;
static uint32_t cpuClockFrequency = 0;
static volatile uint32_t usTicks = 0;
extern TIM_HandleTypeDef        htim11;
uint32_t microsISR(void);
void SysTick_Handler(void);
void cycleCounterInit(void);

void cycleCounterInit(void) {
  cpuClockFrequency = HAL_RCC_GetSysClockFreq(); 
  usTicks = cpuClockFrequency / 1000000;
}

uint32_t microsISR(void) {
  register uint32_t ms,  cycle_cnt;
  cycle_cnt = SysTick->VAL;
  // cycle_cnt = TIM11->CNT; 
  // if (SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) {
    // Update pending.
    // Record it for multiple calls within the same rollover period
    // (Will be cleared when serviced).
    // Note that multiple rollovers are not considered.

    // sysTickPending = 1;

    // Read VAL again to ensure the value is read after the rollover.

    // cycle_cnt = SysTick->VAL;
  // }
  // ms = sysTickUptime;
  ms = xTaskGetTickCountFromISR() * portTICK_RATE_MS; 
  // ms = HAL_GetTick(); 
  // pending = sysTickPending;
  return (ms  * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
}
// Return system uptime in microseconds (rollover in 70minutes)
//返回 us
// uint32_t micros(void)
// {
//     register uint32_t ms, cycle_cnt;
//     do {
//         ms = xTaskGetTickCount();
//         cycle_cnt = SysTick->VAL;
//     } while (ms != xTaskGetTickCount());
//     return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
// }
// uint32_t micros_HAL(void)
// {
//     register uint32_t ms, cycle_cnt;
//     do {
//         ms = HAL_GetTick(); 
//         cycle_cnt = TIM11->CNT; 
//     } while (ms != HAL_GetTick());
//     return (ms * 1000) + (usTicks * 1000 - cycle_cnt) / usTicks;
// }
