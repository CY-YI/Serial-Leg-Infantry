# -*- coding: utf-8 -*-
"""把 CLAUDE.md 的工程规约同步到 AGENTS.md（保持两份 agent 入口文件一致）。

用途：AGENTS.md 是通用 agent 入口文件（给非 Claude 的 agent 工具读），
内容应与 CLAUDE.md 保持一致，仅第一行标题不同。
CLAUDE.md 每次更新后运行本脚本同步。

- 幂等：内容已一致时跳过；
- 无 BOM（与 CLAUDE.md 一致，md 文档不需要 BOM）；
- 覆盖前先做差异检查，打印前 3 处差异行供核对。
"""
import io
import os

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
CLAUDE = os.path.join(ROOT, 'CLAUDE.md')
AGENTS = os.path.join(ROOT, 'AGENTS.md')

TITLE_CLAUDE = '# CLAUDE.md — Serial Leg Infantry 工程速查'
TITLE_AGENTS = '# AGENTS.md — Serial Leg Infantry 工程速查'


def main():
    claude = io.open(CLAUDE, encoding='utf-8', newline='').read()
    expected = claude.replace(TITLE_CLAUDE, TITLE_AGENTS, 1)

    current = io.open(AGENTS, encoding='utf-8', newline='').read()

    if current == expected:
        print('SKIP: AGENTS.md 与 CLAUDE.md 已一致')
        return

    # 打印差异行（前 5 处）供核对
    cur_lines = current.splitlines()
    exp_lines = expected.splitlines()
    diffs = 0
    for i in range(max(len(cur_lines), len(exp_lines))):
        c = cur_lines[i] if i < len(cur_lines) else '<无>'
        e = exp_lines[i] if i < len(exp_lines) else '<无>'
        if c != e:
            diffs += 1
            if diffs <= 5:
                print('差异@%d:\n  当前: %s\n  同步: %s' % (i + 1, c, e))
    if diffs > 5:
        print('... 共 %d 处差异' % diffs)

    with io.open(AGENTS, 'w', encoding='utf-8', newline='') as f:
        f.write(expected)
    print('OK: AGENTS.md 已同步为 CLAUDE.md 最新内容（共 %d 处差异）' % diffs)


if __name__ == '__main__':
    main()
