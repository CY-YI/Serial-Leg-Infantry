# -*- coding: utf-8 -*-
"""
============================================================================
  工程注释编码统一工具
============================================================================
  作用：
      把工程里的 GBK / UTF-8(无BOM) 源文件，统一转换成 UTF-8 带BOM 编码，
      解决中文注释在 VSCode 里显示乱码的问题。

  为什么选 UTF-8 带BOM：
      - VSCode 默认按 UTF-8 打开     -> 中文注释正常显示
      - Keil MDK AC5 识别 BOM 头     -> 自动按 UTF-8 读，不会乱码
      - Git / GitHub 网页            -> 中文正常显示
      （如果只存 UTF-8 不加 BOM，Keil 会按系统 GBK 解析，中文照样乱码）

  安全措施：
      - 默认是"试运行"：只报告不写文件，确认没问题后加 --apply 才真正改
      - 全程按字节读写，完整保留原来的 CRLF / LF 换行符，不会整文件重排
      - 解码失败的文件自动跳过并报告，绝不瞎猜
      - 修改前自动把原文件备份到 _encoding_backup/ 目录，随时可还原
      - 工程本身是 git 仓库，也可以 git checkout 一键还原

  风险提示：
      本脚本会额外扫描"字符串常量里的中文"。注释里的中文换编码没影响，
      但字符串常量（如 OLED 显示的中文）换编码后，编译进去的字节会变，
      这类文件会在报告里单独列为【需留意】。

  用法：
      python tools/fix_encoding.py            # 试运行，只看报告
      python tools/fix_encoding.py --apply    # 真正执行转换
============================================================================
"""

import argparse
import os
import sys
import shutil
import datetime

# ---------------------------------------------------------------------------
# 配置区
# ---------------------------------------------------------------------------

# 只处理这些后缀的源文件
SOURCE_EXTS = ('.c', '.h', '.cpp', '.hpp', '.cc', '.hh')

# 这些目录不是我们自己写的代码（第三方库/编译产物/备份），一律跳过
SKIP_DIRS = {
    '.git', 'Drivers', 'Middlewares', 'build', 'DebugConfig',
    'RTE', '.cmsis', '.eide', '__pycache__', 'tools', '_encoding_backup',
}

BOM = b'\xef\xbb\xbf'          # UTF-8 字节序标记
BACKUP_DIR_NAME = '_encoding_backup'


# ---------------------------------------------------------------------------
# 编码判断
# ---------------------------------------------------------------------------

def detect(data):
    """
    判断一段字节是什么编码。
    返回 (编码名, 解码后的文本 或 None)
    编码名取值：ascii / utf8bom / utf8 / gbk / unknown
    """
    if data.startswith(BOM):
        try:
            return 'utf8bom', data[len(BOM):].decode('utf-8')
        except UnicodeDecodeError:
            return 'unknown', None

    # 全是 ASCII 字符 -> 纯英文文件，不需要处理
    try:
        return 'ascii', data.decode('ascii')
    except UnicodeDecodeError:
        pass

    # 优先试 UTF-8（严格的，能过说明确实是 UTF-8）
    try:
        return 'utf8', data.decode('utf-8')
    except UnicodeDecodeError:
        pass

    # 再试 GBK
    try:
        return 'gbk', data.decode('gbk')
    except UnicodeDecodeError:
        return 'unknown', None


# ---------------------------------------------------------------------------
# 扫描：找出字符串常量里的中文（这是换编码时唯一有实际风险的地方）
# ---------------------------------------------------------------------------

def find_cjk_in_string_literals(text):
    """
    扫描源码，返回"出现在字符串常量里的中文"的行号列表。
    做法：逐字符走一遍，维护当前状态（普通代码 / 行注释 / 块注释 / 字符串），
          只在"字符串"状态里发现中文才记录下来。
    """
    hits = []
    line = 1
    state = 'code'          # code / line_comment / block_comment / string / char
    i = 0
    n = len(text)

    def is_cjk(ch):
        return '一' <= ch <= '鿿'

    while i < n:
        ch = text[i]
        nxt = text[i + 1] if i + 1 < n else ''

        if ch == '\n':
            line += 1
            if state == 'line_comment':
                state = 'code'
            i += 1
            continue

        if state == 'code':
            if ch == '/' and nxt == '/':
                state = 'line_comment'; i += 2; continue
            if ch == '/' and nxt == '*':
                state = 'block_comment'; i += 2; continue
            if ch == '"':
                state = 'string'; i += 1; continue
            if ch == "'":
                state = 'char'; i += 1; continue

        elif state == 'block_comment':
            if ch == '*' and nxt == '/':
                state = 'code'; i += 2; continue

        elif state == 'string':
            if ch == '\\':          # 转义字符，跳过下一个字符
                i += 2; continue
            if ch == '"':
                state = 'code'; i += 1; continue
            if is_cjk(ch):
                if not hits or hits[-1] != line:
                    hits.append(line)

        elif state == 'char':
            if ch == '\\':
                i += 2; continue
            if ch == "'":
                state = 'code'; i += 1; continue

        i += 1

    return hits


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------

def collect_files(root):
    """遍历工程，收集需要处理的源文件列表"""
    result = []
    for dirpath, dirnames, filenames in os.walk(root):
        # 原地修剪要跳过的目录，os.walk 就不会往下走了
        dirnames[:] = [d for d in dirnames if d not in SKIP_DIRS]
        for fn in filenames:
            if fn.lower().endswith(SOURCE_EXTS):
                result.append(os.path.join(dirpath, fn))
    return sorted(result)


def main():
    parser = argparse.ArgumentParser(
        description='把工程源文件统一转成 UTF-8 带BOM 编码')
    parser.add_argument('--apply', action='store_true',
                        help='真正执行转换（不加此参数则只试运行不改文件）')
    parser.add_argument('--root', default=None,
                        help='工程根目录，默认是脚本所在目录的上一级')
    args = parser.parse_args()

    root = args.root or os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    root = os.path.abspath(root)

    print('=' * 74)
    print('  工程注释编码统一工具  ' + ('【正式执行】' if args.apply else '【试运行·不改文件】'))
    print('  工程根目录：' + root)
    print('=' * 74)
    print()

    files = collect_files(root)

    # 分类统计
    todo = []        # 需要转换的：(路径, 原编码, 文本)
    done_already = 0  # 已经是 UTF-8 带BOM，跳过
    ascii_only = 0    # 纯英文，跳过
    failed = []      # 解码失败，跳过
    risky = []       # 字符串常量里有中文，需留意

    for path in files:
        data = open(path, 'rb').read()
        enc, text = detect(data)
        rel = os.path.relpath(path, root)

        if enc == 'ascii':
            ascii_only += 1
            continue
        if enc == 'utf8bom':
            done_already += 1
            continue
        if enc == 'unknown':
            failed.append(rel)
            continue

        # 到这里 enc 一定是 gbk 或 utf8（都不带 BOM），需要转
        lines = find_cjk_in_string_literals(text)
        if lines:
            risky.append((rel, lines))
        todo.append((path, rel, enc, text))

    # ---- 逐条报告 ----
    print('需要转换的文件：%d 个' % len(todo))
    print('-' * 74)
    for path, rel, enc, text in todo:
        size_kb = os.path.getsize(path) / 1024
        print('  [%s -> UTF-8+BOM]  %-52s %6.1f KB' % (enc.upper(), rel, size_kb))
    print()

    if risky:
        print('【需留意】字符串常量里含中文的文件（换编码后编译结果字节会变）：')
        print('-' * 74)
        for rel, lines in risky:
            shown = ', '.join(str(l) for l in lines[:10])
            more = ' ...共%d处' % len(lines) if len(lines) > 10 else ''
            print('  %s' % rel)
            print('      行号：%s%s' % (shown, more))
        print()
        print('  说明：这类通常是 OLED/串口打印的中文。转换后如果显示异常，')
        print('        说明原代码是按 GBK 字节直接送显示的，需要单独处理。')
        print()

    print('-' * 74)
    print('汇总：共扫描 %d 个源文件' % len(files))
    print('      需转换        ：%d 个' % len(todo))
    print('      已是 UTF-8+BOM：%d 个（无需处理）' % done_already)
    print('      纯英文 ASCII  ：%d 个（无需处理）' % ascii_only)
    print('      无法解码跳过  ：%d 个' % len(failed))
    if failed:
        for rel in failed:
            print('          - ' + rel)
    print()

    if not args.apply:
        print('以上是试运行结果，没有修改任何文件。')
        print('确认无误后，执行下面这条命令真正开始转换：')
        print('    python tools/fix_encoding.py --apply')
        print()
        return

    # ---- 正式执行 ----
    backup_root = os.path.join(root, BACKUP_DIR_NAME)
    stamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')

    print('开始转换……（原文件备份到 %s/）' % BACKUP_DIR_NAME)
    print('-' * 74)

    ok = 0
    for path, rel, enc, text in todo:
        # 1) 备份原文件
        backup_path = os.path.join(backup_root, stamp, rel)
        os.makedirs(os.path.dirname(backup_path), exist_ok=True)
        shutil.copy2(path, backup_path)

        # 2) 写入 UTF-8 带BOM（用二进制模式，换行符原样保留）
        new_bytes = BOM + text.encode('utf-8')
        with open(path, 'wb') as f:
            f.write(new_bytes)

        # 3) 立刻读回来校验一遍，确保真的写对了
        check = open(path, 'rb').read()
        if not check.startswith(BOM) or check[len(BOM):].decode('utf-8') != text:
            print('  [失败] %s 写回校验不通过，请用备份还原！' % rel)
            continue
        ok += 1
        print('  [完成] %s' % rel)

    print()
    print('=' * 74)
    print('转换结束：成功 %d 个，失败 %d 个' % (ok, len(todo) - ok))
    print('原文件备份位置：%s' % os.path.join(BACKUP_DIR_NAME, stamp))
    print('如需还原，把备份目录里的文件复制回原位置覆盖即可。')
    print('=' * 74)


if __name__ == '__main__':
    try:
        sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    except Exception:
        pass
    main()
