#pragma once
#include "stm32f4xx_hal.h"
typedef uint32_t timeUs_t;
typedef int32_t timeDelta_t;

static inline timeDelta_t cmpTimeUs(timeUs_t a, timeUs_t b) {
  return (timeDelta_t)(a - b);
}
static inline int32_t cmpTimeCycles(uint32_t a, uint32_t b) {
  return (int32_t)(a - b);
}
