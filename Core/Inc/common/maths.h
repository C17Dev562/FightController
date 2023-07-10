#pragma once
#include <stdint.h>

#define CONSTRAIN(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

/* Function copied from Arduino code */
static inline long MAP(long x, long in_min, long in_max,
                       long out_min, long out_max)
{
    long divisor = (in_max - in_min);
    if (divisor == 0)
    {
        return -1; //AVR returns -1, SAM returns 0
    }
    return CONSTRAIN((x - in_min) * (out_max - out_min) / divisor + out_min, out_min, out_max);
}

static inline float MAP_F(float x, float in_min, float in_max,
                          float out_min, float out_max)
{
    return CONSTRAIN((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min, out_min, out_max);
}

static inline uint16_t MAP_U16(uint16_t x, uint16_t in_min, uint16_t in_max,
                               uint16_t out_min, uint16_t out_max)
{
    return CONSTRAIN((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min, out_min, out_max);
};

static inline uint16_t MAP_I16(int16_t x, int16_t in_min, int16_t in_max,
                               int16_t out_min, int16_t out_max)
{
    return CONSTRAIN((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min, out_min, out_max);
};

#define MIN(a,b) \
  __extension__ ({ __typeof__ (a) _a = (a); \
  __typeof__ (b) _b = (b); \
  _a < _b ? _a : _b; })
#define MAX(a,b) \
  __extension__ ({ __typeof__ (a) _a = (a); \
  __typeof__ (b) _b = (b); \
  _a > _b ? _a : _b; })
#define ABS(x) \
  __extension__ ({ __typeof__ (x) _x = (x); \
  _x > 0 ? _x : -_x; })
#define SIGN(x) \
  __extension__ ({ __typeof__ (x) _x = (x); \
  (_x > 0) - (_x < 0); })


  
