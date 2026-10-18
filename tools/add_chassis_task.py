# -*- coding: utf-8 -*-
"""在 Core/Src/freertos.c 的 RTOS_THREADS 块内注册 chassis 任务。

在 gimbalHandle 创建行之后插入 chassis 任务定义与创建（保留 UTF-8 BOM）。
锚点行内容含制表符，直接文本替换易失配，故用行匹配方式。
"""
import io
import os

FREERTOS = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        '..', 'Core', 'Src', 'freertos.c')
ANCHOR = 'gimbalHandle = osThreadCreate(osThread(gimbal), NULL);'

INSERT = [
    '',
    '  // 定义并创建 chassis 任务（底盘控制，2ms 周期）',
    '  osThreadDef(chassis, chassis_task, osPriorityHigh, 0, 512);',
    '  chassisHandle = osThreadCreate(osThread(chassis), NULL);',
]


def main():
    with io.open(FREERTOS, 'r', encoding='utf-8-sig') as f:
        lines = f.readlines()
    idx = None
    for i, line in enumerate(lines):
        if line.strip() == ANCHOR:
            idx = i
            break
    if idx is None:
        print('ERROR: 未找到锚点行')
        return
    # 检查是否已插入过（幂等）
    if any('osThreadDef(chassis' in l for l in lines):
        print('SKIP: chassis 任务已存在')
        return
    lines[idx + 1:idx + 1] = [s + '\n' for s in INSERT]
    with io.open(FREERTOS, 'w', encoding='utf-8-sig') as f:
        f.writelines(lines)
    print('OK: 已在第 %d 行后插入 chassis 任务' % (idx + 1))


if __name__ == '__main__':
    main()
