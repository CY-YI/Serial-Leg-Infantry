#ifndef BSP_CAN_H
#define BSP_CAN_H


#include "stm32f4xx.h"
#include <stdint.h>
#include "struct_typedef.h"






typedef struct
{
    // ========== 从 CAN 报文直接读取 ==========
    uint16_t ecd;          // 原始编码器值 0~8191
    int16_t  speed_rpm;    // 转子转速 RPM
    int16_t  given_current;      // 实际输出电流
    uint8_t  temperate;         // 温度

    // ========== 计算连续位置（必须） ==========
    uint16_t last_ecd;     // 上一次编码器值
    int32_t  diff;         // 编码器变化量
    int32_t  total_round;  // 总圈数
    int32_t  total_ecd;    // 连续编码值（带圈数，可正可负）

    // ========== 计算角度（可选，非常实用）==========
    float    angle;        // 总角度（°），360°制
    float    total_angle;  // 绝对总角度（可无限大）

} motor_measure_temp;



typedef struct {         
    motor_measure_temp front_left;  // 底盘前左 (ID1)
    motor_measure_temp front_right;   // 底盘前左 (ID2)
    motor_measure_temp back_left;    // 底盘后右 (ID3)
    motor_measure_temp back_right;   // 底盘后右 (ID4)
    motor_measure_temp yaw;          // yaw (ID5)
    motor_measure_temp super_cap;    // 超级电容 (ID6)
} can1_motors_t;

// CAN2 总线的设备结构体（包含 4 个设备）
typedef struct {
    motor_measure_temp pitch;      // pitchI (ID6)
    motor_measure_temp friction_left; // 摩擦轮左 (ID1)
    motor_measure_temp friction_right; // 摩擦轮右 (ID2)
    motor_measure_temp pop_up_wheel; // 拨弹轮 (ID3)
} can2_motors_t;


// 电机系统结构体，管理整个系统的电机信息
typedef struct {
    can1_motors_t can1_motors; // 存储 CAN1 上 6 个设备的信息
    can2_motors_t can2_motors; // 存储 CAN2 上 4 个设备的信息
} motor_system_t;


void get_motor_ecd(motor_measure_temp *v);
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
const motor_system_t *get_pitch_gimbal_motor_measure_point(void);
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
extern void CAN_cmd_yaw(int16_t motor1);
extern void CAN_cmd_pitch(int16_t motor2);
extern void CAN_cmd_friction(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);


extern void can_filter_init(void);

#endif
