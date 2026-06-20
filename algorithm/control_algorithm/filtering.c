#include "filtering.h"
#include <stdint.h>
#include <math.h>

#include <string.h>    /* strlen, memcpy */
#include <stdio.h>     /* snprintf  ← 关键 */
#define PI 3.1415926535f
#define SYS_CLOCK_HZ 168000000U   /* 你的主时钟,F407 一般是 168 MHz,确认一下 */
float dt;

//----------------积分
 void DWT_Init(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

 void integrator_init(integrator_t *p)
{
    p->integral = 0.0f;
    p->last_cycle = 0;
    p->initialized = 0;
}

 float integrator_update(integrator_t *p, float value)
{
    uint32_t now = DWT->CYCCNT;
    
    /* 第一次调用,只记时间不积分 */
    if (!p->initialized) {
        p->last_cycle = now;
        p->initialized = 1;
        return p->integral;
    }
    
    uint32_t diff = now - p->last_cycle;   /* 32 位减法,自动处理溢出 */
    p->last_cycle = now;
    
     dt = (float)diff / (float)SYS_CLOCK_HZ;
    
    /* 防御:如果 dt 异常大(比如断点恢复),跳过 */
    if (dt > 0.1f) return p->integral;
    
    p->integral += value * dt;
    return p->integral;
}

 void integrator_reset(integrator_t *p)
{
    p->integral = 0.0f;
    p->initialized = 0;
}




//--------------------------低通滤波
void LowPassFilter_Init( LowPassFilter *filter, uint16_t alpha_int) {
    filter->last_filtered = 0;
    filter->alpha_int = (alpha_int > 1000) ? 1000 : alpha_int;  // 限制范围
}

// 滤波处理（输入原始数据，输出滤波后的值）
int32_t LowPassFilter_Process(LowPassFilter *filter, int32_t raw_data) {
    // 整数运算避免浮点，先乘后除（注意：raw_data和last_filtered范围不能太大，避免溢出）
    filter->last_filtered = (filter->alpha_int * raw_data + (1000 - filter->alpha_int) * filter->last_filtered) / 1000;
    return filter->last_filtered;
}

//-------------------------------符号数据处理

 uint16_t float_to_str(char *buf, uint16_t buf_size, float value, int decimals)
{
    char *p = buf;
    char *end = buf + buf_size;

    /* 处理符号 */
    if (value < 0.0f) {
        if (p < end) *p++ = '-';
        value = -value;
    }

    /* 缩放系数 */
    uint32_t scale = 1;
    for (int i = 0; i < decimals; i++) scale *= 10;

    uint32_t scaled  = (uint32_t)(value * scale + 0.5f);
    uint32_t integer = scaled / scale;
    uint32_t frac    = scaled % scale;

    /* 如果 scaled 是 0,前面的负号要去掉 */
    if (scaled == 0 && p > buf && *(p - 1) == '-') {
        p--;
    }

    int n = snprintf(p, (size_t)(end - p), "%lu.%0*lu",
                     (unsigned long)integer, decimals, (unsigned long)frac);
    if (n < 0) return (uint16_t)(p - buf);

    p += n;
    return (uint16_t)(p - buf);
}
//-----------------------------------角度归一化--------------
// 角度归一化：将任意角度转换到±180°范围内，超过180°自动折回
 float normalize_angle(float angle_diff)   // 只折叠误差,返回 ±180 内的误差
{
    while (angle_diff > 180.0f)  angle_diff -= 360.0f;
    while (angle_diff < -180.0f) angle_diff += 360.0f;
    return angle_diff;
}

//-----------------------------------正弦函数----------------------------

float sine_target_int(sine_sweep *sin_gain)
{
    sin_gain->time_all += sin_gain->time;   // 累加 2ms,直接改结构体里的 time

    float sin_out = sin_gain->offset
                  + sin_gain->amplitude * sinf(2.0f * PI * sin_gain->frequency * sin_gain->time_all + sin_gain->phase);
    return sin_out;
}

