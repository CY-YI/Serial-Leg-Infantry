# -*- coding: utf-8 -*-
"""修复并追加 文档管理/03_老版本框架参考.md 的第 6 节。

2026-10-07: 首次追加时因 bash 命令替换吞掉了反引号内容导致损坏，
本脚本截去损坏部分并重新写入正确的第 6 节（UTF-8 带 BOM）。
"""
import io
import os

DOC = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'docs', 'legacy_chassis_reference.md')
# 第 5 节结束行号（1 起），其后内容全部丢弃
KEEP_UNTIL_LINE = 347

SECTION6 = u'''

---

## 6. 标准全向轮实现参考（StandardRobotpp，Polarbear 战队开源）

> 来源：D:\\串步\\StandardRobotpp-master（2024-2025 开源，麦轮/全向轮底盘成熟实现）
> 用途：补充老版本麦轮解算缺失的三自由度公式与 FOLLOW 模式正确算法

### 6.1 模块划分（比老版更清晰）

- `chassis_task.c`：任务骨架（__weak 弱函数：Publish / Init / HandleException /
  SetMode / Observer / Reference / Console / SendCmd，按底盘类型由具体实现覆盖）
- `chassis_mecanum.c/h`：麦轮实现（CHASSIS_TYPE==CHASSIS_MECANUM_WHEEL 时编译）
- `chassis_omni.c/h`：全向轮实现（CHASSIS_TYPE==CHASSIS_OMNI_WHEEL 时编译）
- `robot_param_mecannum.h`：**全部参数集中在一个头文件**（物理参数 / PID / 遥控参数 / 电机方向）
- `motor.c/h`：电机抽象层（Motor_s 统一 3508 / 2006 / 6020 / 赛博齿轮）

### 6.2 标准麦轮逆运动学（ChassisConsole，修正老版 bug 的关键）

```c
// vx,vy 单位 m/s；wz 单位 rad/s；R=WHEEL_RADIUS 轮半径；L=WHEEL_CENTER_DISTANCE 轮心到中心距
// 右手系：前进=x，左=y，逆时针旋转 wz 为正；direction=±1 为每轮接线方向
set[0] = ( sqrt(2)/2*(vx - vy) - L*wz ) / R * ratio * direction;   // 左前
set[1] = ( sqrt(2)/2*(vx + vy) - L*wz ) / R * ratio * direction;   // 右前
set[2] = ( sqrt(2)/2*(vx - vy) + L*wz ) / R * ratio * direction;   // 左后
set[3] = ( sqrt(2)/2*(vx + vy) + L*wz ) / R * ratio * direction;   // 右后
```

与老版对比：

| 项目 | 老版（橙子） | StandardRobotpp |
|------|-------------|-----------------|
| 自由度 | 仅平移 2 自由度（pwm3/4 复制 pwm1/2，bug） | 完整 3 自由度（平移+旋转） |
| 旋转分量 | FOLLOW 用速度环输出近似叠加 | 解算含 L*wz 项，数学正确 |
| 单位 | 遥控值直接映射 RPM（±7500） | 物理单位 m/s、rad/s，换算在电机层 |
| 接线方向 | 写死在解算符号里 | **每轮 direction(±1) 参数化**，改宏即可 |

### 6.3 FOLLOW 模式（底盘跟随云台）正确算法

```c
// yaw_delta = loop_fp32_constrain(yaw电机当前位置 - YAW_MID, -PI, PI)  单位 rad
// 1) 平移速度旋转到云台/世界系（解决"推杆方向随车头转"问题）
reference.vx = rc_vx * cos(yaw_delta) - rc_vy * sin(yaw_delta);
reference.vy = rc_vx * sin(yaw_delta) + rc_vy * cos(yaw_delta);
// 2) 旋转分量用 PID 把偏差归零（底盘自动转正）
reference.wz = PID_calc(&follow_pid, 0, yaw_delta);   // 200/0.3/0，max_out=2.0 rad/s
```

老版做法是把 follow 速度环输出直接加到四轮 RPM（近似），
StandardRobotpp 的旋转矩阵 + wz PID 才是标准做法，新版建议采用。

### 6.4 SPIN 小陀螺模式

```c
// 平移同 FOLLOW（经 yaw_delta 旋转，保证推杆方向与车头无关）
// 旋转固定值：mecanum 版  移动中 3 rad/s，静止 8 rad/s
//            ：omni 版    移动中 6.6+等级*0.5，静止 7.6+等级*0.6
```

### 6.5 模式切换（ChassisSetMode）

```c
if (DBUS掉线 || s[0]==DOWN || 云台未初始化完成) mode = CHASSIS_LOCK;
else if (s[0]==MID) mode = (spin_flag ? CHASSIS_SPIN : CHASSIS_FOLLOW);  // SHIFT 键翻转 spin_flag
else if (s[0]==UP)   mode = CHASSIS_SPIN;
```

老版用右拨杆 s[1] 切模式 + F/R 键，StandardRobotpp 用左拨杆 s[0] + SHIFT 键。
新版按老版习惯（s[1]）保留，LOCK 条件值得借鉴（云台未初始化完成也锁底盘）。

### 6.6 键盘控制（W/S/A/D）

- 键盘优先于摇杆：有键盘输入时目标速度 = 低通滤波爬升（alpha 0.005）× 最大速度
- 无键盘输入时用摇杆（死区 ±5，/660 归一化 × 最大速度 2 m/s）
- omni 版用 x_time/y_time 计数线性爬升（每周期 0.004/0.002）

### 6.7 参数集中化（robot_param_mecannum.h，移植借鉴重点）

```c
#define WHEEL_1_ID (1)             // CAN ID
#define WHEEL_1_CAN (1)            // CAN 口
#define WHEEL_1_MOTOR_TYPE DJI_M3508
#define WHEEL_1_DIRECTION (1)      // ★ 接线方向：轮子装反只改这里 ±1
#define WHEEL_1_RATIO (19)         // 3508 减速比 19:1
#define WHEEL_1_MODE (0)
#define WHEEL_RADIUS 0.08f         // 轮半径 (m)
#define WHEEL_CENTER_DISTANCE 0.304f  // 轮心到中心距 (m)
```

**这正是「接线确定后只改参数区」的落地样板**：方向 / 轮序 / 尺寸全部宏参数化，
接线变了改宏，不动解算主体。
'''


def main():
    with io.open(DOC, 'r', encoding='utf-8-sig') as f:
        lines = f.readlines()
    if len(lines) < KEEP_UNTIL_LINE:
        print('ERROR: 文件行数 %d 小于截断点 %d，拒绝操作' % (len(lines), KEEP_UNTIL_LINE))
        return
    head = lines[:KEEP_UNTIL_LINE]
    # 去掉头部末尾多余空行，保证与第 6 节之间恰好一个空行
    while head and head[-1].strip() == '':
        head.pop()
    head.append(u'\n')
    new_content = u''.join(head) + SECTION6
    with io.open(DOC, 'w', encoding='utf-8-sig') as f:
        f.write(new_content)
    print('OK: 保留前 %d 行，追加第 6 节，共 %d 行' % (len(head), len(head) + SECTION6.count(u'\n')))


if __name__ == '__main__':
    main()
