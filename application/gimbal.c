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

/* ============ ҵ����� ============ */
float test_iput_angle;
int test_out;
float test_angle_flag;
float angle_diff;
float angle_flag;
float fake_set;

const imu_angle_t *imu_gimbal;
const motor_system_t *motor_gimbal;

/* ============ ��ϱ���(�� VOFA �� watch ����) ============ */
volatile uint8_t  gimbal_stage = 0;          /* task ��ǰִ�е���һ�� 1-5 */
volatile uint32_t gimbal_loop_count = 0;     /* task ����ѭ������ */
volatile uint32_t gimbal_last_tick = 0;      /* task ���һ������ʱ�� HAL tick */
volatile uint32_t gimbal_curr_tick = 0;      /* ��ǰ HAL tick */
volatile uint32_t gimbal_tick_delta = 0;     /* ����ѭ������� tick �� */
volatile uint32_t can_tx_fail_count = 0;     /* CAN ����ʧ�ܼ��� */

/* ============ ���Ҳ��� ============ */
sine_sweep test_angle_sin = {
    .amplitude = 80.0f,
    .frequency = 0.2f,
    .phase     = 1.5078f,       
    .offset    = 50.0f,
    .time      = 0.002f,
    .time_all  = 0.0f
};

/* ============ ����ʵ�� ============ */
void gimbal_task(void const *pvParameters)
{
    motor_gimbal = get_pitch_gimbal_motor_measure_point();
    
    /* ��ʼ��ʱ�� */
    gimbal_last_tick = HAL_GetTick();
    
    while(1)
    {
        /* �׶�1:����Ŀ����� */
        gimbal_stage = 1;
        test_angle_flag = sine_target_int(&test_angle_sin);
        
        /* �׶�2:�Ƕ�����һ�� */
        gimbal_stage = 2;
        float cur_angle = motor_gimbal->can1_motors.back_left.angle;
        angle_diff = normalize_angle(test_angle_flag - cur_angle);
        fake_set = cur_angle + angle_diff;
        
        /* �׶�3:PID ���� */
        gimbal_stage = 3;
        test_iput_angle = (float)PID_calc(&all_motor_pid.pid_test_angle, 
                                          motor_gimbal->can1_motors.back_left.angle, 
                                          fake_set);
        test_out = (int)PID_calc(&all_motor_pid.pid_test_speed, 
                                 motor_gimbal->can1_motors.back_left.speed_rpm, 
                                 test_iput_angle);
        
        /* �׶�4:CAN ���� */
        gimbal_stage = 4;
        CAN_cmd_chassis(0, 0, test_out, 0);
        
        /* �׶�5:ѭ������ + tick ��� */
        gimbal_stage = 5;
        gimbal_loop_count++;
        gimbal_curr_tick = HAL_GetTick();
        gimbal_tick_delta = gimbal_curr_tick - gimbal_last_tick;
        gimbal_last_tick = gimbal_curr_tick;
        
        /* �׶�6:��ʱ(����ʱ����ʿ�������) */
        gimbal_stage = 6;
        osDelay(2);
    }
}

