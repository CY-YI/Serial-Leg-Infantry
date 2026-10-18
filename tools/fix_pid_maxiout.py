# -*- coding: utf-8 -*-
"""TEST/pid_init.c：底盘四个速度环 PID 的 max_iout 由 100 改为 1500。

依据：tools/sim_chassis.py 仿真结论——max_iout=100 时稳态误差约 -270 RPM
（积分输出不足以补偿稳态电流需求），1500 时误差归零、稳定时间 0.2s。
只改底盘四个 PID_init 调用，其他 PID 不动。保留 UTF-8 BOM。

幂等：可重复执行。注释统一放在 "//底盘" 行之后。
"""
import io
import os

PID_INIT = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        '..', 'TEST', 'pid_init.c')

COMMENT = (
    '    // 底盘 max_iout 由 100 改为 1500：tools/sim_chassis.py 仿真结论——\n'
    '    // 100 时稳态误差约 -270 RPM（积分输出不足以补偿稳态电流），1500 时误差归零\n'
)


def main():
    with io.open(PID_INIT, 'r', encoding='utf-8-sig') as f:
        lines = f.readlines()

    # 1) 删除残留的旧注释行（幂等）
    lines = [l for l in lines if 'max_iout 由 100 改为 1500' not in l
             and '100 时稳态误差约' not in l]

    # 2) 底盘四个 PID_init 的 max_iout 行（特征：参数为 100，且属于底盘段）
    changed = 0
    in_chassis_section = False
    for i, l in enumerate(lines):
        if l.strip() == '//底盘':
            in_chassis_section = True
        if in_chassis_section and l.strip().startswith('//云台'):
            in_chassis_section = False
        if in_chassis_section and '100);' in l:
            lines[i] = l.replace('100);', '1500);', 1)
            changed += 1

    # 3) 在 "//底盘" 行后插入注释
    for i, l in enumerate(lines):
        if l.strip() == '//底盘':
            lines.insert(i + 1, COMMENT)
            break

    if changed != 4:
        print('WARN: 底盘段修改了 %d 行（预期 4 行），请人工检查' % changed)
    with io.open(PID_INIT, 'w', encoding='utf-8-sig') as f:
        f.writelines(lines)
    print('OK: 底盘 max_iout 修改 %d 行，注释已移至 //底盘 之后' % changed)


if __name__ == '__main__':
    main()
