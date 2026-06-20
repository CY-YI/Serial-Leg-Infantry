#include "main.h"
#include "gimbal.h"
#include "math.h"
#include "filtering.h"
#include <time.h>
#include "pid_init.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "bsp_can.h"
#include "pid.h"

/* ============ 业务变量 ============ */
float test_iput_angle;
int test_out;
float test_angle_flag;
float angle_diff;
float angle_flag;
float fake_set;

const imu_angle_t *imu_gimbal;
const motor_system_t *motor_gimbal;

/* ============ 诊断变量(用 VOFA 或 watch 监视) ============ */
volatile uint8_t  gimbal_stage = 0;          /* task 当前执行到哪一步 1-5 */
volatile uint32_t gimbal_loop_count = 0;     /* task 完整循环计数 */
volatile uint32_t gimbal_last_tick = 0;      /* task 最后一次运行时的 HAL tick */
volatile uint32_t gimbal_curr_tick = 0;      /* 当前 HAL tick */
volatile uint32_t gimbal_tick_delta = 0;     /* 两次循环间隔的 tick 差 */
volatile uint32_t can_tx_fail_count = 0;     /* CAN 发送失败计数 */

/* ============ 正弦参数 ============ */
sine_sweep test_angle_sin = {
    .amplitude = 80.0f,
    .frequency = 0.2f,
    .phase     = 1.5078f,       
    .offset    = 50.0f,
    .time      = 0.002f,
    .time_all  = 0.0f
};

/* ============ 任务实现 ============ */
void gimbal_task(void const *pvParameters)
{
    motor_gimbal = get_pitch_gimbal_motor_measure_point();
    
    /* 初始化时刻 */
    gimbal_last_tick = HAL_GetTick();
    
    while(1)
    {
        /* 阶段1:正弦目标计算 */
        gimbal_stage = 1;
        test_angle_flag = sine_target_int(&test_angle_sin);
        
        /* 阶段2:角度误差归一化 */
        gimbal_stage = 2;
        float cur_angle = motor_gimbal->can1_motors.back_left.angle;
        angle_diff = normalize_angle(test_angle_flag - cur_angle);
        fake_set = cur_angle + angle_diff;
        
        /* 阶段3:PID 计算 */
        gimbal_stage = 3;
        test_iput_angle = (float)PID_calc(&all_motor_pid.pid_test_angle, 
                                          motor_gimbal->can1_motors.back_left.angle, 
                                          fake_set);
        test_out = (int)PID_calc(&all_motor_pid.pid_test_speed, 
                                 motor_gimbal->can1_motors.back_left.speed_rpm, 
                                 test_iput_angle);
        
        /* 阶段4:CAN 发送 */
        gimbal_stage = 4;
        CAN_cmd_chassis(0, 0, test_out, 0);
        
        /* 阶段5:循环计数 + tick 监控 */
        gimbal_stage = 5;
        gimbal_loop_count++;
        gimbal_curr_tick = HAL_GetTick();
        gimbal_tick_delta = gimbal_curr_tick - gimbal_last_tick;
        gimbal_last_tick = gimbal_curr_tick;
        
        /* 阶段6:延时(故障时大概率卡在这里) */
        gimbal_stage = 6;
        osDelay(2);
    }
}

