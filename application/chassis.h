#ifndef CHASSIS_H
#define CHASSIS_H
#include "main.h"

/**
  ******************************************************************************
  * @file    chassis.h
  * @brief   底盘控制模块（全向轮/麦轮）
  *          框架沿用老版本(橙子工程)：结构体名/函数名/模式枚举保持一致，
  *          解算内核采用 StandardRobotpp 标准逆运动学（三自由度）。
  *          坐标系：vx=前进轴、vy=左移轴、v_rot 逆时针为正（右手系）。
  *          单位体系：RPM（电机轴转速），与 speed_rpm 反馈及 pid_init 参数兼容。
  ******************************************************************************
  */

/* ================= 物理参数（TODO-接线标定：实车测量后更新） ================= */
#define WHEEL_RADIUS            0.08f   /* 轮半径 (m)                                */
#define WHEEL_CENTER_DISTANCE   0.304f  /* 轮心到中心距 (m)，需实车测量              */
#define WHEEL_RATIO             19      /* 3508 减速比                                */

/* ============ 轮序/方向参数区（★接线确定后只改这里，不动解算主体） ============ */
/* direction=±1：某轮装反 -> 该轮宏取反；前后轮序错位 -> 交换同侧两轮宏            */
#define WHEEL_FL_DIRECTION      (1)     /* 前左                                       */
#define WHEEL_FR_DIRECTION      (1)     /* 前右                                       */
#define WHEEL_BL_DIRECTION      (1)     /* 后左                                       */
#define WHEEL_BR_DIRECTION      (1)     /* 后右                                       */

/* ================= 运动参数 ================= */
#define MAX_GIVEN_SPEED         7500    /* 单轮目标转速上限 (RPM)，老版沿用          */
#define V_REVOLVE               5000    /* 小陀螺自旋等效轮速 (RPM)≈7rad/s 车体转速   */
#define V_MAX                   4000    /* 小陀螺键盘平移满速 (RPM)，老版 v_max      */
#define RAMP_DEADLINE           66      /* 斜坡死区（老版原样）                       */
#define RAMP_COEFF              6       /* 斜坡每周期步进（老版原样）                 */
#define RC_MAX_RANGE            660     /* 遥控通道行程                               */
#define RC_DEADLINE             5       /* 摇杆死区                                   */
#define KEYBOARD_MAX_SPEED      660     /* 键盘 W/S/A/D 附加速度（老版原样）          */

/* ================= FOLLOW（底盘跟随云台） ================= */
#define FOLLOW_ROT_MAX_RPM      2000    /* 跟随修正的旋转等效轮速上限 (RPM)          */
#define YAW_MID_ENC_FALLBACK    1300    /* yaw 电机中值硬编码回退（老版 A6020_M），
                                           上电自标定失败时使用                      */
#define YAW_CALI_WINDOW_MS      500     /* 上电静止采集中值的窗口时长 (ms)           */

/* ================= 功率限制（占位，RMUL 前接裁判系统） ================= */
#define POWER_LIMIT_PARAM       6.0f    /* limit = buffer^2 * 此系数（老版原样）      */
#define POWER_BUFFER_FIXED      50.0f   /* 裁判系统未接入前写死 50 -> 限功率分支禁用  */

/* ================= 通用宏（老版本同名） ================= */
#define ABS(x)          ((x) > 0 ? (x) : (-(x)))
#define FlagCheck(z)    ((z) > 0 ? 1 : 0)

/* ================= 数据结构（老版本同名） ================= */
typedef struct
{
    int16_t RemoteLefUpSpeed;           /* 左上轮解算给定速度 (RPM)                  */
    int16_t RemoteLefDoSpeed;           /* 左下轮解算给定速度 (RPM)                  */
    int16_t RemoteRigUpSpeed;           /* 右上轮解算给定速度 (RPM)                  */
    int16_t RemoteRigDoSpeed;           /* 右下轮解算给定速度 (RPM)                  */
} RemoteParam_Struct;

typedef struct
{
    int16_t LimtOutValue;               /* 功率限制后的输出总和（预留）              */
    float   MergeFinalOutValue;         /* 四电机输出绝对值和（预留）                */
    int16_t LefUpFinalOutValue;         /* 左上轮最终输出（CAN 电流指令）            */
    int16_t LefDoFinalOutValue;         /* 左下轮最终输出                            */
    int16_t RigUpFinalOutValue;         /* 右上轮最终输出                            */
    int16_t RigDoFinalOutValue;         /* 右下轮最终输出                            */
} ChassisParam_Struct;

typedef enum
{
    CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,   /* 底盘跟随云台相对角度（默认模式）          */
    CHASSIS_VECTOR_revolve,             /* 小陀螺自旋                                */
    CHASSIS_LOCK,                       /* 锁底盘（掉线/拨杆 DOWN/标定中）           */
} chassis_mode_e;

/* ================= 诊断变量（VOFA 监视，仿 gimbal.c 惯例） ================= */
extern volatile uint8_t  chassis_stage;         /* 任务执行阶段 1-6                    */
extern volatile uint32_t chassis_loop_count;    /* 循环计数                            */
extern volatile uint32_t chassis_tick_delta;    /* 两次循环 tick 差                    */
extern float chassis_vx;                        /* 解算：前进分量 (RPM)                */
extern float chassis_vy;                        /* 解算：左移分量 (RPM)                */
extern float chassis_v_rot;                     /* 解算：旋转等效轮速 (RPM)            */
extern float chassis_yaw_delta;                 /* yaw 相对中值角度 (°)，FOLLOW 用     */
extern float yaw_mid_deg;                       /* yaw 中值角度 (°)，上电标定结果      */

/* ================= 接口函数（老版本同名） ================= */
void chassis_task(void const *pvParameters);
void chassis_set_mode(chassis_mode_e *chassis_mode);   /* 模式状态机                  */
void chassis_set_control(void);                        /* FOLLOW 模式目标解算         */
void chassis_set_v(void);                              /* revolve 模式目标解算        */
void ChassisController(void);                          /* 限幅 + 麦轮逆运动学         */
void ChassisPidCtr(int16_t LefUpTarspd, int16_t RigUpTarspd,
                   int16_t LefDoTarspd, int16_t RigDoTarspd);
int16_t Ramp_function(int16_t delta_Min, int16_t target,
                      int16_t now, int16_t ramp_Coeff, uint8_t mode);
void chassis_yaw_mid_calibrate(void);                  /* yaw 电机中值上电标定        */

#endif
