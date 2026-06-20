#ifndef _FILTERING_H
#define _FILTERING_H

#include "main.h"
#include <stdint.h>

typedef struct {
    int32_t last_filtered;  // 上一次滤波结果
    uint16_t alpha_int;     // 整数型滤波系数（0~1000，对应α=0~1）
} LowPassFilter;            //一阶低通滤波


typedef struct {
    float integral;
    uint32_t last_cycle;
    uint8_t initialized;
} integrator_t;

typedef struct {
  float amplitude ;     // 幅值
  float frequency ;      // 频率 Hz
  float phase     ;      // 相位
  float offset    ;      // 中心偏置
  float time     ;      // 时间单次累加值
  float time_all;
}sine_sweep ;


extern void LowPassFilter_Init( LowPassFilter *filter, uint16_t alpha_int);
extern int32_t LowPassFilter_Process(LowPassFilter *filter, int32_t raw_data);
extern void DWT_Init(void);
extern void integrator_init(integrator_t *p);
extern float integrator_update(integrator_t *p, float value);
extern void integrator_reset(integrator_t *p);
extern uint16_t float_to_str(char *buf, uint16_t buf_size, float value, int decimals);
extern float normalize_angle(float angle_diff );
extern float sine_target_int( sine_sweep *sin_gain );

#endif
