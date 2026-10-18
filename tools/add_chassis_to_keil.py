# -*- coding: utf-8 -*-
"""把 application/chassis.c 与 chassis.h 登记进 Keil 工程 (.uvprojx)。

背景：VSCode 的 "Keil UVision Project" 视图显示的是 .uvprojx 中已登记的文件，
磁盘上的新文件不会自动出现，也不会参与编译，需在 Keil 工程里登记。

本脚本在 application 组的 INS_task.c 之后插入登记块（格式与现有条目一致）。
- 默认试运行，只打印将插入的内容；
- --apply 才写入，写入前自动备份为 .uvprojx.bak；
- 幂等：已登记则跳过；锚点不唯一则拒绝操作。
- 保持文件原有编码（UTF-8 无 BOM）与行尾风格。
"""
import io
import os
import shutil
import sys

UVPROJX = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                       '..', 'MDK-ARM', 'Serial Leg Infantry.uvprojx')
ANCHOR_NAME = '<FileName>INS_task.c</FileName>'

NEW_FILES = [
    ('chassis.c', '1'),
    ('chassis.h', '5'),
]


def build_block(nl):
    lines = []
    for name, ftype in NEW_FILES:
        lines.append('            <File>' + nl)
        lines.append('              <FileName>' + name + '</FileName>' + nl)
        lines.append('              <FileType>' + ftype + '</FileType>' + nl)
        lines.append('              <FilePath>..\\application\\' + name + '</FilePath>' + nl)
        lines.append('            </File>' + nl)
    return lines


def main():
    apply_mode = '--apply' in sys.argv

    with io.open(UVPROJX, 'r', encoding='utf-8', newline='') as f:
        lines = f.readlines()

    # 幂等检查
    if any('<FileName>chassis.c</FileName>' in l for l in lines):
        print('SKIP: chassis.c 已在工程中，无需重复登记')
        return

    # 定位锚点（INS_task.c 条目）
    idx = None
    for i, l in enumerate(lines):
        if ANCHOR_NAME in l:
            idx = i
            break
    if idx is None:
        print('ERROR: 未找到锚点 %s，拒绝操作' % ANCHOR_NAME)
        return

    # 校验条目结构：FileName / FileType / FilePath / </File>
    ok = (idx + 3 < len(lines)
          and '<FileType>' in lines[idx + 1]
          and '<FilePath>' in lines[idx + 2]
          and '</File>' in lines[idx + 3])
    if not ok:
        print('ERROR: 锚点结构异常（idx=%d），拒绝操作' % idx)
        return

    nl = '\r\n' if lines[idx].endswith('\r\n') else '\n'
    block = build_block(nl)

    if not apply_mode:
        print('== 试运行（未写入任何文件）==')
        print('目标：%s' % os.path.abspath(UVPROJX))
        print('插入位置：application 组，INS_task.c 条目之后（第 %d 行后）' % (idx + 4))
        print('将插入内容：')
        sys.stdout.write(''.join(block))
        print('\n行尾风格：%s' % ('CRLF' if nl == '\r\n' else 'LF'))
        print('确认后执行：python tools/add_chassis_to_keil.py --apply')
        return

    # 备份 + 写入
    bak = UVPROJX + '.bak'
    shutil.copy2(UVPROJX, bak)
    lines[idx + 4:idx + 4] = block
    with io.open(UVPROJX, 'w', encoding='utf-8', newline='') as f:
        f.writelines(lines)
    print('OK: 已登记 chassis.c / chassis.h 到 application 组')
    print('备份: %s' % os.path.abspath(bak))
    print('下一步：打开 Keil 重新加载工程，即可在 application 组看到这两个文件并参与编译')


if __name__ == '__main__':
    main()
