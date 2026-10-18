# -*- coding: utf-8 -*-
"""文档搬迁后的引用更新（docs/ -> 文档管理/）。

2026-10-08：文档统一收进「文档管理」文件夹并中文化命名，本脚本更新：
1. 01_底盘需求文档.md / 02_底盘架构设计.md 中指向旧 docs/ 路径的引用
2. tools/append_chassis_doc.py 中的旧路径常量
3. CLAUDE.md 目录职责表补充「文档管理/」一行

默认试运行显示对照，--apply 才写入；按原文件 BOM 状态写回（不增不减）。
"""
import io
import os
import sys

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')

# 文件替换规则：{文件相对路径: [(旧串, 新串), ...]}
REPLACEMENTS = {
    os.path.join('文档管理', '01_底盘需求文档.md'): [
        ('docs/legacy_chassis_reference.md', '03_老版本框架参考.md'),
    ],
    os.path.join('文档管理', '02_底盘架构设计.md'): [
        ('docs/chassis_requirement.md', '01_底盘需求文档.md'),
        ('docs/legacy_chassis_reference.md', '03_老版本框架参考.md'),
    ],
    os.path.join('tools', 'append_chassis_doc.py'): [
        ('docs/legacy_chassis_reference.md', '文档管理/03_老版本框架参考.md'),
    ],
}

# CLAUDE.md 插入规则
CLAUDE_ANCHOR = '| `tools/` | 维护脚本 | 编码/注释修复工具 |\n'
CLAUDE_INSERT = '| `文档管理/` | 项目文档 | 需求/架构/调研/参考文档集中存放，勿散落别处 |\n'


def read_text(path):
    """读取文件，返回 (内容, 是否有BOM)"""
    with open(path, 'rb') as f:
        raw = f.read()
    has_bom = raw.startswith(b'\xef\xbb\xbf')
    text = raw.decode('utf-8-sig' if has_bom else 'utf-8')
    return text, has_bom


def write_text(path, text, has_bom):
    with open(path, 'wb') as f:
        if has_bom:
            f.write(b'\xef\xbb\xbf')
        f.write(text.encode('utf-8'))


def main():
    apply_mode = '--apply' in sys.argv
    changed = 0

    for rel, rules in REPLACEMENTS.items():
        path = os.path.join(ROOT, rel)
        text, has_bom = read_text(path)
        new = text
        for old, rep in rules:
            if old in new:
                new = new.replace(old, rep)
        if new != text:
            changed += 1
            if apply_mode:
                write_text(path, new, has_bom)
            print('[%s] %s' % ('已改' if apply_mode else '待改', rel))
            for old, rep in rules:
                if old in text:
                    print('   %s -> %s' % (old, rep))
        else:
            print('[跳过] %s（无待替换内容）' % rel)

    # CLAUDE.md
    claude = os.path.join(ROOT, 'CLAUDE.md')
    text, has_bom = read_text(claude)
    if '文档管理/' in text and CLAUDE_INSERT in text:
        print('[跳过] CLAUDE.md（已含文档管理行）')
    elif CLAUDE_ANCHOR in text:
        changed += 1
        if apply_mode:
            write_text(claude, text.replace(CLAUDE_ANCHOR, CLAUDE_ANCHOR + CLAUDE_INSERT, 1), has_bom)
        print('[%s] CLAUDE.md（目录表插入文档管理行）' % ('已改' if apply_mode else '待改'))
    else:
        print('[警告] CLAUDE.md 未找到锚点，跳过')

    if not apply_mode:
        print('\n共 %d 处待修改。确认后执行：python tools/update_doc_refs.py --apply' % changed)
    else:
        print('\n完成，共修改 %d 个文件' % changed)


if __name__ == '__main__':
    main()
