# -*- coding: utf-8 -*-
"""
底盘控制仿真脚本（MVU-0）— 纯 Python 标准库，本地运行

仿真 1：四轮速度环 PID 阶跃响应
  - 被控对象：3508+C620 一阶惯性 G(s)=K/(tau*s+1)
  - PID 逻辑与固件 algorithm/control_algorithm/pid.c 一致：
    位置式、Iout 每周期累加 Ki*error（dt 隐含在 Ki 中）、D 用 error 差分
  - 周期 dt=0.002s（500Hz，与 chassis_task 一致）
  - 考察：超调、稳态误差、稳定时间；tau 扫参验证鲁棒性

仿真 2：麦轮逆运动学符号验证
  - 对照公式：set[i] = ((vx∓vy)/√2 ∓ v_rot) * direction[i]
  - 给定若干 (vx,vy,v_rot) 组合，人工核对四轮符号是否符合预期

仿真 3（占位）：FOLLOW 模式在 MVU-4 时扩展

实物标定注意：
  - K/tau 为理论估计值（K≈0.55 RPM/单位电流，tau≈0.08s），
    实车值需用阶跃实验辨识（下发改电流指令，VOFA 记录 RPM 响应拟合）；
  - 实物上电安全限幅（固件 chassis.h 参数区已设）：
    MAX_GIVEN_SPEED=7500 RPM、PID max_out=12000（C620 量程 16384）、
    先架车空转再落地。

用法：python tools/sim_chassis.py
"""
from __future__ import print_function
import math

# ================= 仿真参数区（对应 chassis.h 参数区） =================
DT          = 0.002      # 控制周期 (s)
SETPOINT    = 7500.0     # 阶跃目标 RPM（= MAX_GIVEN_SPEED）
KP, KI, KD  = 5.0, 0.2, 1.0          # 速度环 PID（pid_init.c 同款）
MAX_OUT     = 12000.0    # PID 输出限幅（电流值）
# 积分限幅扫参：100=老版值（仿真显示稳态误差~270RPM），1500=建议修正值
MAX_IOUT_SWEEP = (100.0, 1500.0)

# 电机模型 G(s)=K/(tau*s+1)
# K：稳态增益 RPM/单位电流（16384 单位 = 20A）。
#    官方手册数据推算：空载 482rpm(输出轴)@0.78A -> 电机轴 K≈14；
#    额定 469rpm@3N·m/10A -> K≈1.1。K 随负载 1~14 大幅变化（线性模型局限）。
#    仿真取 5（轻负载中间值）；实车辨识：发阶跃电流、VOFA 记录 RPM 稳态值拟合。
K_MOTOR     = 5.0       # 稳态增益 RPM/单位电流
# tau 主工况 0.05s = 官方机械时间常数 49ms；0.08/0.15 为带负载鲁棒性扫参
TAU_SWEEP   = (0.05, 0.08, 0.15)     # 时间常数扫参 (s)

SQRT2_INV   = 1.0 / math.sqrt(2.0)   # 1/√2


class PidPosition(object):
    """复刻固件 pid.c 的位置式 PID（dt 隐含在 Ki 累加中）"""

    def __init__(self, kp, ki, kd, max_out, max_iout):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.max_out, self.max_iout = max_out, max_iout
        self.pout = self.iout = self.dout = 0.0
        self.err = [0.0, 0.0, 0.0]      # 0最新 1上次 2上上次

    def calc(self, fdb, setpoint):
        self.err[2] = self.err[1]
        self.err[1] = self.err[0]
        self.err[0] = setpoint - fdb
        self.pout = self.kp * self.err[0]
        self.iout += self.ki * self.err[0]
        if self.iout > self.max_iout:
            self.iout = self.max_iout
        elif self.iout < -self.max_iout:
            self.iout = -self.max_iout
        self.dout = self.kd * (self.err[0] - self.err[1])
        out = self.pout + self.iout + self.dout
        if out > self.max_out:
            out = self.max_out
        elif out < -self.max_out:
            out = -self.max_out
        return out


def motor_step(speed, current, tau):
    """一阶惯性电机模型一步（欧拉法）"""
    return speed + (DT / tau) * (K_MOTOR * current - speed)


def sim_velocity_loop(tau, max_iout):
    """仿真 1：单轮速度环阶跃响应，返回指标字典"""
    pid = PidPosition(KP, KI, KD, MAX_OUT, max_iout)
    speed = 0.0
    t = 0.0
    t_rise = t_settle = None
    peak = 0.0
    first_cross = None          # 首次到达 90% 时间
    for k in range(3000):       # 6 秒仿真
        t = k * DT
        curr = pid.calc(speed, SETPOINT)
        speed = motor_step(speed, curr, tau)
        if speed > peak:
            peak = speed
        if first_cross is None and speed >= 0.9 * SETPOINT:
            first_cross = t
        if t_rise is None and speed >= 0.1 * SETPOINT:
            t_rise = t
        if t_settle is None and t > 0.2 and abs(speed - SETPOINT) / SETPOINT < 0.02:
            t_settle = t
    return {
        'tau': tau,
        'max_iout': max_iout,
        'rise10_90': None if t_rise is None or first_cross is None else round(first_cross - t_rise, 3),
        'overshoot_pct': round((peak - SETPOINT) / SETPOINT * 100.0, 1),
        'steady_err_rpm': round(speed - SETPOINT, 1),
        'settle_2pct_s': None if t_settle is None else round(t_settle, 3),
    }


def sim_mecanum_check():
    """仿真 2：麦轮解算符号验证（对照 §4 公式手算核对）"""
    cases = [
        ('前进 vx=+7500',           (7500.0, 0.0, 0.0)),
        ('后退 vx=-7500',           (-7500.0, 0.0, 0.0)),
        ('左移 vy=+7500',           (0.0, 7500.0, 0.0)),
        ('右移 vy=-7500',           (0.0, -7500.0, 0.0)),
        ('原地逆时针 v_rot=2000',   (0.0, 0.0, 2000.0)),
        ('斜向+自旋',               (3000.0, 4000.0, 1500.0)),
    ]
    directions = [1, 1, 1, 1]        # 参数区默认值，接线标定后改
    print('\n===== 仿真2：麦轮解算符号验证 =====')
    print('(左前, 右前, 左后, 右后) 单位 RPM；direction 全 1')
    for name, (vx, vy, vrot) in cases:
        s0 = ((vx - vy) * SQRT2_INV - vrot) * directions[0]
        s1 = ((vx + vy) * SQRT2_INV - vrot) * directions[1]
        s2 = ((vx - vy) * SQRT2_INV + vrot) * directions[2]
        s3 = ((vx + vy) * SQRT2_INV + vrot) * directions[3]
        print('%-22s -> (%8.1f, %8.1f, %8.1f, %8.1f)' % (name, s0, s1, s2, s3))
    print('公式自洽核对（vx=前进轴，vy=左移轴，vrot 逆时针为正）：')
    print('  前进/后退 = 四轮同号；左移 = 左前右后为负、右前左后为正；')
    print('  原地逆时针 = 左前右前为负、左后右后为正。')
    print('TODO-接线标定：实车验证——推杆向前，若车向左/右斜行，')
    print('  交换同侧前后轮 direction 宏；若某轮反转，该轮 direction 取反。')


def main():
    print('===== 仿真1：四轮速度环 PID 阶跃响应（0 -> %.0f RPM） =====' % SETPOINT)
    print('PID(%.1f/%.2f/%.1f)  max_out=%.0f  dt=%.3fs' % (KP, KI, KD, MAX_OUT, DT))
    for max_iout in MAX_IOUT_SWEEP:
        print('\n--- max_iout = %.0f ---' % max_iout)
        print('%-8s %-12s %-12s %-12s %-12s' % ('tau(s)', '上升10-90(s)', '超调(%)', '稳态误差(RPM)', '稳定±2%(s)'))
        for tau in TAU_SWEEP:
            r = sim_velocity_loop(tau, max_iout)
            print('%-8.2f %-12s %-12s %-12s %-12s' % (
                r['tau'],
                r['rise10_90'] if r['rise10_90'] is not None else '-',
                r['overshoot_pct'],
                r['steady_err_rpm'],
                r['settle_2pct_s'] if r['settle_2pct_s'] is not None else '>6',
            ))
    print('\n判据：超调 < 20%%、稳态误差 < 50 RPM、稳定时间 < 1s 即认为参数可用；')
    print('实车若振荡则优先降 KD/KP，若响应慢则升 KP。')
    print('稳态需求电流 = %.0f / K_MOTOR，若接近 max_out=%.0f 说明限幅吃紧，' %
          (SETPOINT, MAX_OUT))
    print('需降低 MAX_GIVEN_SPEED 或确认 max_iout 是否够补偿稳态误差。')

    sim_mecanum_check()


if __name__ == '__main__':
    main()
