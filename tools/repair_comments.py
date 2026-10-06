# -*- coding: utf-8 -*-
"""
============================================================================
  注释修复工具 —— 用 .bak 备份里的完好中文，修复当前文件里被损坏的注释
============================================================================
  背景：
      gimbal.c / gimbal.h 这两个文件的注释曾经被"用错误编码打开后保存"，
      原始中文已经变成了 U+FFFD 替换字符（显示成一个方块或问号），
      在文件里是永久性损坏，光换编码救不回来。

  修复思路：
      工程里的 .bak 备份是 GBK 编码的旧版本，中文完好无损。
      但备份是旧版本，代码可能落后于当前文件（比如 amplitude 从 40 改成了 80），
      所以【不能直接覆盖】，必须做外科手术式合并：

          - 代码部分 → 用"当前文件"的（保留你最新的修改）
          - 注释部分 → 用"备份文件"的（拿回完好的中文）

  匹配方式：
      给两个文件的每一行做标记：
          有代码的行 -> 用"剥掉注释后的纯代码"作为标记
          纯注释的行 -> 统一标记为 C
          空行       -> 统一标记为 B
      然后整体做 difflib 序列对齐。代码相同的行才会配对，
      所以即使中间插入了空行，对齐也不会错位（第一版就是栽在这里）。

  安全措施：
      - 默认试运行：只打印"改前 / 改后"对照，不动文件
      - 加 --apply 才真正写入
      - 写入前自动备份到 _encoding_backup/repair_*
      - 写完立刻读回校验，并检查是否还有残留损坏字符
      - 自动保留原文件的换行符风格（LF / CRLF）

  用法：
      python tools/repair_comments.py            # 试运行，看对照表
      python tools/repair_comments.py --apply    # 真正修复
============================================================================
"""

import argparse
import datetime
import difflib
import os
import re
import shutil
import sys

# 需要修复的文件与对应的备份文件
TARGETS = [
    ('application/gimbal.c', 'application/gimbal.c.bak'),
    ('application/gimbal.h', 'application/gimbal.h.bak'),
]

BOM = b'\xef\xbb\xbf'
BAD_CHAR = '�'          # U+FFFD 替换字符，损坏的标志
BACKUP_DIR_NAME = '_encoding_backup'


# ---------------------------------------------------------------------------
# 基础工具
# ---------------------------------------------------------------------------

def load_lines(path):
    """读文件并自动识别编码，返回 (行列表, 换行符)"""
    data = open(path, 'rb').read()
    if data.startswith(BOM):
        data = data[len(BOM):]
    text = None
    for enc in ('utf-8', 'gbk'):
        try:
            text = data.decode(enc)
            break
        except UnicodeDecodeError:
            continue
    if text is None:
        raise ValueError('无法识别编码：' + path)

    eol = '\r\n' if '\r\n' in text else '\n'
    return text.splitlines(), eol


def scan_line(line, in_block):
    """
    扫描一行，返回 (该行注释起始下标 或 None, 到行末是否仍处于块注释中)

    关键点（第一版的 bug 就在这）：
        遇到 /* 之后不能立刻返回 True 就完事，
        必须继续往后找同一行上的 */，因为 /* ... */ 可能在同一行就闭合了。
        否则这种单行块注释会让状态机误判"块注释延续到下一行"，
        然后后续每一行的注释检测都跟着错乱。
    """
    i = 0
    n = len(line)
    in_string = False
    first_comment = None

    while i < n:
        ch = line[i]
        nxt = line[i + 1] if i + 1 < n else ''

        if in_block:
            if ch == '*' and nxt == '/':       # 块注释在同一行闭合
                in_block = False
                i += 2
                continue
            i += 1
            continue

        if in_string:
            if ch == '\\':                      # 转义，跳过下一个字符
                i += 2
                continue
            if ch == '"':
                in_string = False
            i += 1
            continue

        if ch == '"':
            in_string = True
            i += 1
            continue

        if ch == '/' and nxt == '/':            # 行注释，一直到行尾
            if first_comment is None:
                first_comment = i
            break

        if ch == '/' and nxt == '*':            # 块注释开始
            if first_comment is None:
                first_comment = i
            in_block = True
            i += 2
            continue                            # 继续找 */，正确更新状态

        i += 1

    return first_comment, in_block


def code_only(line):
    """取一行去掉注释和空白后的纯代码"""
    start, _ = scan_line(line, False)
    code = line if start is None else line[:start]
    return re.sub(r'\s+', '', code)


def line_key(line):
    """给一行生成用于对齐的标记"""
    if not line.strip():
        return 'B'                    # 空行
    c = code_only(line)
    return c if c else 'C'            # 有代码用代码，纯注释用 C


# ---------------------------------------------------------------------------
# 核心：合并修复
# ---------------------------------------------------------------------------

def repair(cur_lines, bak_lines):
    """
    返回 (修复后的行列表, 修复明细, 无法修复的说明)
    """
    cur_keys = [line_key(l) for l in cur_lines]
    bak_keys = [line_key(l) for l in bak_lines]

    # 整体序列对齐（空行和纯注释行也参与，靠结构对齐不会错位）
    matcher = difflib.SequenceMatcher(None, cur_keys, bak_keys, autojunk=False)
    pair = {}
    for a, b, size in matcher.get_matching_blocks():
        for k in range(size):
            pair[a + k] = b + k

    # 预先算好备份每行行首是否处于块注释中
    bak_in_block = {}
    in_block = False
    for i, line in enumerate(bak_lines):
        bak_in_block[i] = in_block
        _, in_block = scan_line(line, in_block)

    result = list(cur_lines)
    details = []
    unrepaired = []

    for i, line in enumerate(cur_lines):
        if BAD_CHAR not in line:
            continue                        # 这行没坏，跳过

        j = pair.get(i)
        if j is None:
            unrepaired.append((i + 1, line, '在备份里找不到对应的行'))
            continue

        bak_line = bak_lines[j]
        if BAD_CHAR in bak_line:
            unrepaired.append((i + 1, line, '备份里这行也是坏的'))
            continue

        bak_start, _ = scan_line(bak_line, bak_in_block.get(j, False))
        if bak_start is None:
            unrepaired.append((i + 1, line, '备份里这行没有注释可取'))
            continue

        # 当前的代码 + 备份的注释
        cur_start, _ = scan_line(line, False)
        cur_code_part = line if cur_start is None else line[:cur_start]
        fixed = cur_code_part + bak_line[bak_start:]

        if BAD_CHAR in fixed:
            unrepaired.append((i + 1, line, '拼接后仍有损坏字符'))
            continue

        if fixed != line:
            result[i] = fixed
            details.append((i + 1, line, fixed))

    return result, details, unrepaired


# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(description='用 .bak 备份修复损坏的中文注释')
    parser.add_argument('--apply', action='store_true',
                        help='真正写入（不加则只试运行）')
    parser.add_argument('--root', default=None, help='工程根目录，默认脚本上一级')
    args = parser.parse_args()

    root = args.root or os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    print('=' * 74)
    print('  注释修复工具  ' + ('【正式执行】' if args.apply else '【试运行·不改文件】'))
    print('=' * 74)

    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    files_fixed = 0

    for cur_rel, bak_rel in TARGETS:
        cur_path = os.path.join(root, cur_rel)
        bak_path = os.path.join(root, bak_rel)

        print()
        print('=' * 74)
        print('  文件：' + cur_rel)
        print('  备份：' + bak_rel)
        print('=' * 74)

        if not os.path.exists(cur_path):
            print('  跳过：当前文件不存在')
            continue
        if not os.path.exists(bak_path):
            print('  跳过：备份文件不存在')
            continue

        cur_lines, cur_eol = load_lines(cur_path)
        bak_lines, _ = load_lines(bak_path)

        broken = sum(1 for l in cur_lines if BAD_CHAR in l)
        total_bad = sum(l.count(BAD_CHAR) for l in cur_lines)
        print('  当前文件 %d 行，其中 %d 行含损坏（共 %d 个损坏字符）'
              % (len(cur_lines), broken, total_bad))

        fixed_lines, details, unrepaired = repair(cur_lines, bak_lines)

        print('  可修复：%d 行' % len(details))
        if unrepaired:
            print('  无法修复：%d 行' % len(unrepaired))
            for ln, content, why in unrepaired[:6]:
                print('      第%d行（%s）: %s' % (ln, why, content.strip()[:52]))
            if len(unrepaired) > 6:
                print('      …… 其余 %d 行省略' % (len(unrepaired) - 6))
        print()

        print('  逐行对照（最多显示 12 条）：')
        print('  ' + '-' * 70)
        for ln, before, after in details[:12]:
            print('  第 %d 行' % ln)
            print('    改前: ' + before.strip()[:66])
            print('    改后: ' + after.strip()[:66])
        if len(details) > 12:
            print('  …… 其余 %d 条省略' % (len(details) - 12))
        print()

        if not details:
            print('  没有需要修复的内容。')
            continue

        if not args.apply:
            continue

        # ---- 真正写入 ----
        backup_path = os.path.join(root, BACKUP_DIR_NAME,
                                   'repair_%s_%s' % (stamp, os.path.basename(cur_rel)))
        os.makedirs(os.path.dirname(backup_path), exist_ok=True)
        shutil.copy2(cur_path, backup_path)

        text = cur_eol.join(fixed_lines) + cur_eol
        with open(cur_path, 'wb') as f:
            f.write(BOM + text.encode('utf-8'))

        # 立刻读回校验
        check = open(cur_path, 'rb').read()
        if not check.startswith(BOM):
            print('  [失败] 写回后没有 BOM，请用备份还原！')
            continue
        left = check[len(BOM):].decode('utf-8').count(BAD_CHAR)
        if left:
            print('  [警告] 写回后仍残留 %d 个损坏字符' % left)
        else:
            print('  [完成] 已修复 %d 行，写回校验通过，残留损坏 0 个' % len(details))
            print('         修复前的文件已备份到：' + os.path.relpath(backup_path, root))
            files_fixed += 1

    print()
    print('=' * 74)
    if args.apply:
        print('  处理完成，共修复 %d 个文件。' % files_fixed)
        print('  建议：在 VSCode 里重新打开这些文件，确认中文注释正常。')
    else:
        print('  以上是试运行结果，没有修改任何文件。')
        print('  确认对照表无误后，执行下面这条命令真正修复：')
        print('      python tools/repair_comments.py --apply')
    print('=' * 74)


if __name__ == '__main__':
    try:
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    except Exception:
        pass
    main()
