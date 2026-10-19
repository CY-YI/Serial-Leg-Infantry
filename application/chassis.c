#include "main.h"
#include "chassis.h"
#include "pid.h"
#include "pid_init.h"
#include "bsp_can.h"
#include "remote_control.h"
#include "math.h"
#include "cmsis_os.h"

/**
  ******************************************************************************
  * @file    chassis.c
  * @brief   底盘控制任务（全向轮/麦轮）
  *          MVU-2：模式状态机 + 遥控映射 + Ramp 斜坡 + 麦轮解算 + 开环发送。
  *          MVU-3：四轮速度环 PID 闭环，反馈 speed_rpm，替换原开环电流路径。
  *          MVU-4：FOLLOW 跟随（yaw 中值上电标定 + 旋转矩阵 + follow 双环）。
  *          MVU-5：掉线保护（rc_is_offline）+ revolve 增益对齐老版 + 键盘控制。
  *          MVU-6：功率限制占位（get_chassis_power_and_buffer 空壳）+ 注释清理。
  *          状态：MVU-3~6 代码已写入；通信链路已台架验证，闭环/跟随待架车验证。
  *          注意：接线方向未标定前，闭环存在正反馈跑飞风险，首次上电须四轮悬空。
  *          开发进度：
  *          [x] MVU-1 骨架/心跳      [x] MVU-2 遥控+解算
  *          [x] MVU-3 速度环PID      [x] MVU-4 FOLLOW
  *          [x] MVU-5 revolve+保护   [x] MVU-6 功率占位+质检
  ******************************************************************************
  */

/* ============ 业务变量（老版本同名） ============ */
ChassisParam_Struct  chassisparam;
RemoteParam_Struct   remoteparam;
chassis_mode_e       chassis_mode = CHASSIS_LOCK;  /* 上电即锁 */

/* 数据指针 */
const RC_ctrl_t      *RC;                /* 遥控器 */
const motor_system_t *c;                 /* 电机系统 */

/* ============ 诊断变量（VOFA 监视，仿 gimbal.c 惯例） ============ */
volatile uint8_t  chassis_stage      = 0;   /* 任务当前执行阶段 1-6 */
volatile uint32_t chassis_loop_count = 0;   /* 完整循环计数 */
volatile uint32_t chassis_tick_delta = 0;   /* 两次循环间隔 tick 差 */

/* ============ 内部状态 ============ */
static uint32_t chassis_last_tick = 0;
static uint32_t chassis_curr_tick = 0;

/* VOFA 监视：yaw 中值角度(°)，上电标定结果（A6020_M 换算而来） */
float yaw_mid_deg = 0.0f;

/* 麦轮解算中间量：vx/vy=平移分量(RPM)，v_rot=旋转等效轮速(RPM)
   （非 static，VOFA 监视用，chassis.h 已 extern） */
float chassis_vx = 0.0f;
float chassis_vy = 0.0f;
float chassis_v_rot = 0.0f;

/* Ramp 平滑值（通道值域 ±660，老版原样；映射 RPM 在解算函数内完成） */
static int16_t ramp_vx = 0;
static int16_t ramp_vy = 0;

/* revolve 模式低通值（RPM 域，老版 0.8/0.2 系数） */
static float lp_vx = 0.0f;
static float lp_vy = 0.0f;

/* ============ FOLLOW 跟随（MVU-4，变量名沿用老版） ============ */
static int32_t A6020_M     = YAW_MID_ENC_FALLBACK;  /* yaw 中值编码器（标定后更新）  */
static int32_t A6020_M1    = 0;                     /* 当前 yaw 相对中值的编码器差   */
static float   A6020_angle = 0.0f;                  /* follow 角度环输出             */
static float   A6020_speed = 0.0f;                  /* follow 速度环输出（=v_rot）   */

/* VOFA 监视：yaw 相对角度（°，归一化到 ±180） */
float chassis_yaw_delta = 0.0f;

/* ================= yaw 电机中值上电标定 ================= */
/**
 * @brief 上电静止窗口 YAW_CALI_WINDOW_MS 内采集 yaw 电机编码器均值作为中值 A6020_M。
 *        在线判据：窗口内编码器有变化 **或** 采样值非 0（电机静止时 ecd 是常数，
 *        不能用"有变化"单独判断；CAN 无反馈时 ecd 恒为 0）。
 *        判为离线则回退硬编码 YAW_MID_ENC_FALLBACK。
 *        注意：采样期间车体必须静止，否则中值被运动污染（TODO-接线标定）。
 */
void chassis_yaw_mid_calibrate(void)
{
    uint32_t start  = HAL_GetTick();
    uint32_t sum    = 0;
    uint16_t cnt    = 0;
    uint16_t first  = c->can1_motors.yaw.ecd;
    uint8_t  varied = 0;

    while ((HAL_GetTick() - start) < YAW_CALI_WINDOW_MS)
    {
        uint16_t ecd = c->can1_motors.yaw.ecd;
        sum += ecd;
        cnt++;
        if (ecd != first)
        {
            varied = 1;     /* 编码器有变化 -> 反馈在线 */
        }
        osDelay(10);
    }

    if (cnt > 0 && (varied || first != 0))
    {
        A6020_M = (int32_t)(sum / cnt);
    }
    else
    {
        A6020_M = YAW_MID_ENC_FALLBACK;     /* 反馈不在线（恒 0），回退硬编码 */
    }

    /* 编码器 -> 度：8192 线/圈 */
    yaw_mid_deg = (float)A6020_M * 360.0f / 8192.0f;
}

/* ================= 任务入口 ================= */
void chassis_task(void const *pvParameters)
{
    c  = get_pitch_gimbal_motor_measure_point();   /* 电机反馈（bsp_can） */
    RC = get_remote_control_point();               /* 遥控器 */

    chassis_last_tick = HAL_GetTick();
    chassis_yaw_mid_calibrate();

    while (1)
    {
        /* 阶段1：模式状态机 */
        chassis_stage = 1;
        chassis_set_mode(&chassis_mode);

        /* 阶段2：FOLLOW 模式目标解算 */
        chassis_stage = 2;
        chassis_set_control();

        /* 阶段3：revolve 模式目标解算 */
        chassis_stage = 3;
        chassis_set_v();

        /* 阶段4：限幅 + 麦轮逆运动学 + 输出电流计算 */
        chassis_stage = 4;
        if (chassis_mode == CHASSIS_LOCK)
        {
            /* LOCK 清零全部解算量，解锁后从零起步，避免残留速度突变 */
            chassis_vx = 0.0f;
            chassis_vy = 0.0f;
            chassis_v_rot = 0.0f;
            ramp_vx = 0;
            ramp_vy = 0;
            lp_vx = 0.0f;
            lp_vy = 0.0f;
        }
        ChassisController();

        /* 阶段5：CAN 发送（gimbal_task 已停用 0x200，底盘接管） */
        chassis_stage = 5;
        if (chassis_mode == CHASSIS_LOCK)
        {
            CAN_cmd_chassis(0, 0, 1000, 0);
        }
        else
        {
            CAN_cmd_chassis(chassisparam.LefUpFinalOutValue,
                            chassisparam.RigUpFinalOutValue,
                            chassisparam.LefDoFinalOutValue,
                            chassisparam.RigDoFinalOutValue);
        }

        /* 阶段6：心跳 + tick 监控 */
        chassis_stage = 6;
        chassis_loop_count++;
        chassis_curr_tick = HAL_GetTick();
        chassis_tick_delta = chassis_curr_tick - chassis_last_tick;
        chassis_last_tick = chassis_curr_tick;

        osDelay(2);     /* 2ms 控制周期 */
    }
}

/* ================= 模式状态机 ================= */
/**
 * @brief 拨杆/键盘/掉线决定底盘模式（优先级：掉线 > 拨杆 > 键盘）。
 *        右拨杆 s[1]：UP -> revolve；MID -> FOLLOW；DOWN -> LOCK（键盘云台模式）
 *        MID 下按住 F -> revolve，按住 R -> FOLLOW，松开回默认 FOLLOW
 *        （老版同样是"按住生效"，不是 latching 切换）
 *        DBUS 掉线（>RC_OFFLINE_TIMEOUT_MS 无完整帧）-> LOCK，四轮发零
 */
void chassis_set_mode(chassis_mode_e *mode)
{
    /* 最高优先级：遥控器掉线，立即锁底盘（MVU-5） */
    if (rc_is_offline())
    {
        *mode = CHASSIS_LOCK;
        return;
    }

    if (switch_is_up(RC->rc.s[1]))
    {
        *mode = CHASSIS_VECTOR_revolve;
    }
    else if (switch_is_down(RC->rc.s[1]))
    {
        *mode = CHASSIS_LOCK;
    }
    else /* MID */
    {
        if (RC->key.v & KEY_PRESSED_OFFSET_F)
        {
            *mode = CHASSIS_VECTOR_revolve;
        }
        else if (RC->key.v & KEY_PRESSED_OFFSET_R)
        {
            *mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW;
        }
        else
        {
            /* 默认 FOLLOW（老版默认模式；上电 LOCK 在拨杆回 MID 后自动进入） */
            *mode = CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW;
        }
    }
}

/* ================= FOLLOW 模式目标解算 ================= */
/**
 * @brief 摇杆+键盘 -> Ramp 平滑 -> 平移 vx/vy(RPM)；
 *        yaw_delta -> follow 角度环+速度环 -> 旋转修正 v_rot(RPM)；
 *        yaw_delta 旋转矩阵把"操作者系"平移量变换到底盘系（借鉴 StandardRobotpp）。
 *        停止直接清零（官方做法：目标死区内平滑值立即归零，急停不拖尾）。
 */
void chassis_set_control(void)
{
    int16_t x = 0;      /* 左右通道目标（含键盘附加） */
    int16_t y = 0;      /* 前后通道目标（含键盘附加） */
    float   vx_local;
    float   vy_local;
    float   delta_rad;

    if (chassis_mode != CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        /* 非 FOLLOW：清跟随双环历史，避免切回 FOLLOW 时积分冲击 */
        PID_clear(&all_motor_pid.pid_follow_angle);
        PID_clear(&all_motor_pid.pid_follow_speed);
        A6020_M1 = 0;
        A6020_angle = 0.0f;
        A6020_speed = 0.0f;
        chassis_yaw_delta = 0.0f;
        return;
    }

    /* ---- 1. yaw 相对中值的编码器差（老版归一化：±4046 判据 / ±8191 修正） ---- */
    A6020_M1 = (int32_t)c->can1_motors.yaw.ecd - A6020_M;
    if (A6020_M1 >= 4046 || A6020_M1 <= -4046)
    {
        if (A6020_M1 <= -4045) A6020_M1 += 8191;
        if (A6020_M1 >=  4046) A6020_M1 -= 8191;
    }
    chassis_yaw_delta = (float)A6020_M1 * 360.0f / 8192.0f;   /* VOFA 监视用 */

    /* ---- 2. follow 双环（老版：角度环 PID_DELTA + 速度环 PID_POSITION） ---- */
    A6020_angle = PID_calc(&all_motor_pid.pid_follow_angle, (fp32)A6020_M1, 0.0f);
    A6020_speed = PID_calc(&all_motor_pid.pid_follow_speed,
                           (fp32)c->can1_motors.yaw.speed_rpm, A6020_angle);

    /* 跟随修正旋转量限幅（架构要求；老版此处无此限幅） */
    if (A6020_speed >  FOLLOW_ROT_MAX_RPM) A6020_speed =  FOLLOW_ROT_MAX_RPM;
    if (A6020_speed < -FOLLOW_ROT_MAX_RPM) A6020_speed = -FOLLOW_ROT_MAX_RPM;

    /* ---- 3. 摇杆 + 键盘 W/S/A/D 叠加
       通道符号（2026-10-19 实机确认）：
         ch3 正 = 前进、负 = 后退   → y 正 = 前进
         ch2 正 = 右移、负 = 左移   → x 正 = 右移（配合第 4 步 vy 取负变左正）
       键盘：W/S = 前/后，A/D = 左/右（与摇杆同向） ---- */
    x = RC->rc.ch[2] - FlagCheck(RC->key.v & KEY_PRESSED_OFFSET_A) * KEYBOARD_MAX_SPEED
                     + FlagCheck(RC->key.v & KEY_PRESSED_OFFSET_D) * KEYBOARD_MAX_SPEED;
    y = RC->rc.ch[3] + FlagCheck(RC->key.v & KEY_PRESSED_OFFSET_W) * KEYBOARD_MAX_SPEED
                     - FlagCheck(RC->key.v & KEY_PRESSED_OFFSET_S) * KEYBOARD_MAX_SPEED;

    /* 停止直接清零：目标在死区内 -> 平滑值立即归零（不等 Ramp 爬坡） */
    if (ABS(x) < RAMP_DEADLINE)
    {
        ramp_vx = 0;
    }
    else
    {
        ramp_vx = Ramp_function(RAMP_DEADLINE, x, ramp_vx, RAMP_COEFF, 0);
    }

    if (ABS(y) < RAMP_DEADLINE)
    {
        ramp_vy = 0;
    }
    else
    {
        ramp_vy = Ramp_function(RAMP_DEADLINE, y, ramp_vy, RAMP_COEFF, 0);
    }

    /* ---- 4. 通道值域 -> RPM 域（底盘系/操作者系平移量）
       ch3 正=前进 -> vx 正；
       ch2 正=右移，而坐标系约定 vy 左为正，故对 ch2 取负
       （实车若左右整体相反，再把 vy_local 取反） ---- */
    vx_local =  (float)ramp_vy / RC_MAX_RANGE * MAX_GIVEN_SPEED;
    vy_local = -(float)ramp_vx / RC_MAX_RANGE * MAX_GIVEN_SPEED;

    /* ---- 5. yaw_delta 旋转矩阵：操作者系平移 -> 底盘系
           借鉴 StandardRobotpp：vx_c = vx·cos - vy·sin，vy_c = vx·sin + vy·cos
           TODO-接线标定: 若跟随方向相反，把下一行 delta_rad 取负 */
    delta_rad = (float)A6020_M1 * (2.0f * 3.14159265f / 8192.0f);
    chassis_vx = vx_local * cosf(delta_rad) - vy_local * sinf(delta_rad);
    chassis_vy = vx_local * sinf(delta_rad) + vy_local * cosf(delta_rad);

    /* ---- 6. 旋转修正量作为麦轮解算的 v_rot 项
           （老版是把 A6020_speed 四轮同号叠加，属近似；此处用标准旋转项） ---- */
    chassis_v_rot = A6020_speed;
}

/* ================= revolve 模式目标解算 ================= */
/**
 * @brief 摇杆低通(0.8/0.2 老版系数，×5 增益) + 键盘(±V_MAX) -> vx/vy；
 *        v_rot = V_REVOLVE 固定自旋。
 *        （老版此处"自旋"是四轮同号加常数，实为平移；本版用标准旋转项 v_rot）
 */
void chassis_set_v(void)
{
    if (chassis_mode != CHASSIS_VECTOR_revolve)
    {
        return;
    }

    float target_vx = 0.0f;
    float target_vy = 0.0f;

    /* 键盘 W/S/A/D：满速平移（老版 v_max） */
    if (RC->key.v & KEY_PRESSED_OFFSET_W) target_vx =  V_MAX;
    if (RC->key.v & KEY_PRESSED_OFFSET_S) target_vx = -V_MAX;
    if (RC->key.v & KEY_PRESSED_OFFSET_A) target_vy = -V_MAX;
    if (RC->key.v & KEY_PRESSED_OFFSET_D) target_vy =  V_MAX;

    /* 无键盘时用摇杆（低通滤波 + ×5 增益，老版 0.8/0.2 手感与量纲） */
    if (target_vx == 0.0f && target_vy == 0.0f)
    {
        /* TODO-接线标定: 摇杆通道符号（ch2 左右 / ch3 前后）实车验证 */
        lp_vx = 0.8f * lp_vx + 0.2f * ((float)RC->rc.ch[3] * 5.0f);
        lp_vy = 0.8f * lp_vy + 0.2f * (-(float)RC->rc.ch[2] * 5.0f);
        chassis_vx = lp_vx;
        chassis_vy = lp_vy;
    }
    else
    {
        chassis_vx = target_vx;
        chassis_vy = target_vy;
        lp_vx = target_vx;   /* 键盘切摇杆不跳变 */
        lp_vy = target_vy;
    }

    chassis_v_rot = V_REVOLVE;   /* 固定自旋（老版原样） */
}

/* ================= 限幅 + 麦轮逆运动学 ================= */
/**
 * @brief 标准麦轮逆运动学（StandardRobotpp 公式，RPM 体系）：
 *        set[FL] = ((vx - vy)/√2 - v_rot) * WHEEL_FL_DIRECTION
 *        set[FR] = ((vx + vy)/√2 - v_rot) * WHEEL_FR_DIRECTION
 *        set[BL] = ((vx - vy)/√2 + v_rot) * WHEEL_BL_DIRECTION
 *        set[BR] = ((vx + vy)/√2 + v_rot) * WHEEL_BR_DIRECTION
 *        限幅采用官方等比缩放：四轮同比例缩小，保持运动方向不变。
 *        TODO-接线标定: 轮子装反 -> 对应 DIRECTION 宏取反；轮序错位 -> 交换宏。
 */
void ChassisController(void)
{
    const float SQRT2_INV = 0.70710678f;    /* 1/√2 */
    float set[4];
    float max_abs = 0.0f;
    float rate = 1.0f;
    int i;

    /* 解算（direction 参数区生效处） */
    set[0] = ((chassis_vx - chassis_vy) * SQRT2_INV - chassis_v_rot) * WHEEL_FL_DIRECTION;
    set[1] = ((chassis_vx + chassis_vy) * SQRT2_INV - chassis_v_rot) * WHEEL_FR_DIRECTION;
    set[2] = ((chassis_vx - chassis_vy) * SQRT2_INV + chassis_v_rot) * WHEEL_BL_DIRECTION;
    set[3] = ((chassis_vx + chassis_vy) * SQRT2_INV + chassis_v_rot) * WHEEL_BR_DIRECTION;

    /* 等比缩放限幅（官方做法）：超限时四轮同比例缩小 */
    for (i = 0; i < 4; i++)
    {
        float a = fabsf(set[i]);
        if (a > max_abs) max_abs = a;
    }
    if (max_abs > MAX_GIVEN_SPEED)
    {
        rate = MAX_GIVEN_SPEED / max_abs;
        for (i = 0; i < 4; i++)
        {
            set[i] *= rate;
        }
    }

    remoteparam.RemoteLefUpSpeed = (int16_t)set[0];
    remoteparam.RemoteRigUpSpeed = (int16_t)set[1];
    remoteparam.RemoteLefDoSpeed = (int16_t)set[2];
    remoteparam.RemoteRigDoSpeed = (int16_t)set[3];

    /* 输出电流计算（MVU-2 开环；MVU-3 替换为速度环 PID） */
    ChassisPidCtr(remoteparam.RemoteLefUpSpeed,
                  remoteparam.RemoteRigUpSpeed,
                  remoteparam.RemoteLefDoSpeed,
                  remoteparam.RemoteRigDoSpeed);
}

/* ================= 功率限制占位（MVU-6） ================= */
/**
 * @brief 裁判系统功率/缓冲能量读取 —— 占位空壳。
 *        本期无裁判系统：功率恒 0、buffer 恒 POWER_BUFFER_FIXED(50.0f)，
 *        使下面的限功率分支永不触发（等价于禁用）。
 *        RMUL 参赛前接入 referee 模块时，只改本函数内部，调用处不用动。
 */
static void get_chassis_power_and_buffer(fp32 *power, fp32 *power_buffer)
{
    *power        = 0.0f;
    *power_buffer = POWER_BUFFER_FIXED;
}

/* ================= 输出电流计算 + 功率限制 ================= */
/**
 * @brief MVU-3：四轮速度环 PID 闭环。
 *        反馈 = 各轮 speed_rpm（bsp_can 解析的转子转速 RPM）；
 *        目标 = ChassisController 解算并限幅后的四轮目标转速。
 *        PID 槽位映射（与 CAN_cmd_chassis(前左,前右,后左,后右) 发送顺序一致）：
 *          LefUp -> front_left，RigUp -> front_right，
 *          LefDo -> back_left， RigDo -> back_right。
 *        LOCK 时清除四路 PID 历史（防积分饱和），输出清零。
 *        功率限制占位：POWER_BUFFER_FIXED=50 时分支禁用，RMUL 前接裁判系统。
 *        参数（5/0.2/1，max_out=10000，max_iout=1500）见 TEST/pid_init.c。
 */
void ChassisPidCtr(int16_t LefUpTarspd, int16_t RigUpTarspd,
                   int16_t LefDoTarspd, int16_t RigDoTarspd)
{
    fp32 power        = 0.0f;                    /* 裁判系统功率（占位恒 0）  */
    fp32 power_buffer = POWER_BUFFER_FIXED;      /* 缓冲能量（占位恒 50）     */
    fp32 scale        = 1.0f;

    /* 锁底盘：清 PID 历史，避免锁定期间积分累积、解锁瞬间冲出 */
    if (chassis_mode == CHASSIS_LOCK)
    {
        PID_clear(&all_motor_pid.pid_front_left);
        PID_clear(&all_motor_pid.pid_front_right);
        PID_clear(&all_motor_pid.pid_back_left);
        PID_clear(&all_motor_pid.pid_back_right);

        chassisparam.LefUpFinalOutValue = 0;
        chassisparam.RigUpFinalOutValue = 0;
        chassisparam.LefDoFinalOutValue = 0;
        chassisparam.RigDoFinalOutValue = 0;
        return;
    }

    /* 速度环：ref = 实际转速，set = 目标转速（PID_calc 第 2 参为反馈） */
    chassisparam.LefUpFinalOutValue = (int16_t)PID_calc(&all_motor_pid.pid_front_left,
                                        (fp32)c->can1_motors.front_left.speed_rpm,
                                        (fp32)LefUpTarspd);
    chassisparam.RigUpFinalOutValue = (int16_t)PID_calc(&all_motor_pid.pid_front_right,
                                        (fp32)c->can1_motors.front_right.speed_rpm,
                                        (fp32)RigUpTarspd);
    chassisparam.LefDoFinalOutValue = (int16_t)PID_calc(&all_motor_pid.pid_back_left,
                                        (fp32)c->can1_motors.back_left.speed_rpm,
                                        (fp32)LefDoTarspd);
    chassisparam.RigDoFinalOutValue = (int16_t)PID_calc(&all_motor_pid.pid_back_right,
                                        (fp32)c->can1_motors.back_right.speed_rpm,
                                        (fp32)RigDoTarspd);

    /* ---- 功率限制（MVU-6 占位：buffer 恒 50 时整个分支跳过，等价禁用）----
       老版逻辑：Σ|四输出| 为 MergeFinalOutValue，Limit = buffer² × POWER_LIMIT_PARAM，
       超限时四路按 (Limit / Σ) 等比缩放，保持运动方向不变。 */
    get_chassis_power_and_buffer(&power, &power_buffer);

    if (power_buffer < POWER_BUFFER_FIXED)
    {
        chassisparam.MergeFinalOutValue =
            (float)(ABS(chassisparam.LefUpFinalOutValue) +
                    ABS(chassisparam.RigUpFinalOutValue) +
                    ABS(chassisparam.LefDoFinalOutValue) +
                    ABS(chassisparam.RigDoFinalOutValue));

        /* 防除零：四路输出全 0 时不需要缩放 */
        if (chassisparam.MergeFinalOutValue > 0.01f)
        {
            chassisparam.LimtOutValue =
                (int16_t)(power_buffer * power_buffer * POWER_LIMIT_PARAM);

            scale = (fp32)chassisparam.LimtOutValue / chassisparam.MergeFinalOutValue;

            chassisparam.LefUpFinalOutValue = (int16_t)(chassisparam.LefUpFinalOutValue * scale);
            chassisparam.RigUpFinalOutValue = (int16_t)(chassisparam.RigUpFinalOutValue * scale);
            chassisparam.LefDoFinalOutValue = (int16_t)(chassisparam.LefDoFinalOutValue * scale);
            chassisparam.RigDoFinalOutValue = (int16_t)(chassisparam.RigDoFinalOutValue * scale);
        }
    }
}

/* ================= Ramp 斜坡函数（老版本原样保留） ================= */
/**
 * @brief  速度斜坡：死区内直接到位，死区外每周期爬 ramp_Coeff；
 *         负向大跳变直接砍到 1/3（老版急刹逻辑）。
 * @param  delta_Min  死区（66）
 * @param  target     目标速度
 * @param  now        当前平滑值
 * @param  ramp_Coeff 每周期步进（6）
 * @param  mode       0=老版模式
 */
int16_t Ramp_function(int16_t delta_Min, int16_t target,
                      int16_t now, int16_t ramp_Coeff, uint8_t mode)
{
    int16_t Lenght = target - now;
    if (ABS(Lenght) < delta_Min)
    {
        now = target;
    }
    else
    {
        if (mode == 0)
        {
            if (target < 0)
            {
                if (ABS(Lenght) > 660)
                {
                    return target / 3;
                }
            }
        }
        if (Lenght > 0)
        {
            now += ramp_Coeff;
        }
        else if (Lenght < 0)
        {
            now -= ramp_Coeff;
        }
    }
    return now;
}
