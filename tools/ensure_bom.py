# -*- coding: utf-8 -*-
"""ensure_bom.py —— 确保指定源文件为 UTF-8 带 BOM。

背景：本工程用 Keil ARMCC V5.06 (AC5)，在中文 Windows 上按系统 GBK 解析源文件；
UTF-8 无 BOM 会导致中文注释在 Keil 里乱码。而某些编辑器/工具（如整文件重写）
会把 BOM 头丢掉，这个脚本用来定点补回。

与 tools/fix_encoding.py 的区别：
  - fix_encoding.py 扫描全工程并做编码转换（范围大，会动很多文件）；
  - 本脚本只处理命令行显式给出的文件，且**只增删 3 字节 BOM**，绝不改动内容。

用法：
  python tools/ensure_bom.py <文件1> [文件2 ...]          # 试运行，只报告
  python tools/ensure_bom.py --apply <文件1> [文件2 ...]  # 实际写入 BOM
"""

import sys
import os

BOM = b"\xef\xbb\xbf"


def check_one(path):
    """返回 (状态, 说明)。状态取值：ok / missing / not_utf8 / not_found"""
    if not os.path.isfile(path):
        return "not_found", "文件不存在"
    with open(path, "rb") as f:
        head = f.read(3)
        f.seek(0)
        raw = f.read()
    if head == BOM:
        return "ok", "已有 BOM"
    # 确认是合法 UTF-8（无 BOM 时也能解码），避免把 GBK 文件误当 UTF-8 加 BOM
    try:
        raw.decode("utf-8")
    except UnicodeDecodeError as exc:
        return "not_utf8", "不是合法 UTF-8：%s" % exc
    if raw.startswith(b"\xef\xbb\xbf"):
        return "ok", "已有 BOM"
    return "missing", "缺少 BOM（UTF-8 无 BOM）"


def main(argv):
    apply = False
    files = []
    for arg in argv[1:]:
        if arg == "--apply":
            apply = True
        else:
            files.append(arg)

    if not files:
        print("用法: python tools/ensure_bom.py [--apply] <文件...>")
        return 2

    changed = 0
    pending = 0
    failed = 0
    for path in files:
        status, msg = check_one(path)
        if status == "missing":
            pending += 1
            if apply:
                with open(path, "r+b") as f:
                    raw = f.read()
                    f.seek(0)
                    f.write(BOM + raw)
                changed += 1
                print("[已补] %s  <- %s" % (path, msg))
            else:
                print("[待补] %s  <- %s（加 --apply 生效）" % (path, msg))
        elif status == "ok":
            print("[跳过] %s  <- %s" % (path, msg))
        else:
            failed += 1
            print("[失败] %s  <- %s" % (path, msg))

    print("-" * 60)
    if apply:
        print("完成：补 BOM %d 个，失败 %d 个" % (changed, failed))
        return 1 if failed else 0
    print("试运行结束：待补 %d 个，失败 %d 个（未写入任何文件）" % (pending, failed))
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
