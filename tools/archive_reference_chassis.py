# -*- coding: utf-8 -*-
"""归档外部参考代码到工程内（防外部路径丢失，便于交接）。

- 橙子老版本 chassis.c/h（GBK）      -> tools/reference_legacy/（转 UTF-8 带BOM）
- StandardRobotpp chassis_mecanum.c/h -> tools/reference_polarbear/（补 BOM）
- 各目录生成 README.md 说明来源与用途

幂等：目标文件已存在则跳过。默认直接执行（仅新增文件，不覆盖任何现有文件）。
"""
import io
import os

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')

LEGACY_SRC = os.path.join('D:\\', 'shaobinjiqiren', '大烧饼', '步兵min(橙子)', 'APP')
LEGACY_DST = os.path.join(ROOT, 'tools', 'reference_legacy')
LEGACY_FILES = ['chassis.c', 'chassis.h']

POLARBEAR_SRC = os.path.join('D:\\', '串步', 'StandardRobotpp-master', 'application', 'chassis')
POLARBEAR_DST = os.path.join(ROOT, 'tools', 'reference_polarbear')
POLARBEAR_FILES = ['chassis_mecanum.c', 'chassis_mecanum.h']

LEGACY_README = """# 橙子老版本底盘参考（归档）

来源：`D:\\shaobinjiqiren\\大烧饼\\步兵min(橙子)\\APP\\chassis.c / chassis.h`（2026-10-08 归档）
原文件为 GBK 编码，此处已转 UTF-8 带 BOM 便于阅读（编码转换不改代码内容）。

用途：新版 chassis 的框架模板（函数名 / 结构体名 / 2ms 数据流）。
**不参与编译**，仅供阅读对照；差异与已知 bug 分析见工程根 `文档管理/03_老版本框架参考.md`。
"""

POLARBEAR_README = """# StandardRobotpp 底盘参考（归档）

来源：`D:\\串步\\StandardRobotpp-master\\application\\chassis\\chassis_mecanum.c / .h`（2026-10-08 归档）
原工程为 Polarbear 战队开源（GitHub）。原文件 UTF-8 无 BOM，归档时补 BOM。

用途：标准三自由度麦轮逆运动学与 FOLLOW 算法参考。
**不参与编译**，仅供阅读对照；分析见工程根 `文档管理/04_资料调研报告.md`
与 `文档管理/03_老版本框架参考.md` 第 6 节。
"""


def decode_gbk(path):
    """GBK 解码。橙子 chassis.h 为历史遗留混合编码（GBK 中文 + 少量损坏字节），
    逐级降级：gbk -> gb18030 -> gbk(errors=replace)。代码部分为 ASCII 不受影响，
    仅个别注释字符会显示为 U+FFFD（README 已注明）。"""
    with open(path, 'rb') as f:
        raw = f.read()
    for enc in ('gbk', 'gb18030'):
        try:
            return raw.decode(enc)
        except UnicodeDecodeError:
            continue
    print('[注意] %s 含无法解码字节，已用 U+FFFD 替换（仅影响个别注释字符）' % path)
    return raw.decode('gbk', errors='replace')


def decode_utf8(path):
    with open(path, 'rb') as f:
        raw = f.read()
    if raw.startswith(b'\xef\xbb\xbf'):
        raw = raw[3:]
    return raw.decode('utf-8')


def save_utf8bom(path, text):
    with io.open(path, 'w', encoding='utf-8-sig', newline='') as f:
        f.write(text)


def archive(src_dir, dst_dir, files, decoder, readme_text, label):
    os.makedirs(dst_dir, exist_ok=True)
    for name in files:
        src = os.path.join(src_dir, name)
        dst = os.path.join(dst_dir, name)
        if os.path.exists(dst):
            print('[跳过] %s/%s（已存在）' % (label, name))
            continue
        if not os.path.exists(src):
            print('[警告] 源文件不存在: %s' % src)
            continue
        try:
            text = decoder(src)
        except Exception as e:
            print('[失败] %s 解码错误: %s' % (src, e))
            continue
        save_utf8bom(dst, text)
        print('[归档] %s/%s (%d 字符, UTF-8 带BOM)' % (label, name, len(text)))
    readme = os.path.join(dst_dir, 'README.md')
    if not os.path.exists(readme):
        save_utf8bom(readme, readme_text)
        print('[归档] %s/README.md' % label)


def main():
    archive(LEGACY_SRC, LEGACY_DST, LEGACY_FILES, decode_gbk, LEGACY_README, 'reference_legacy')
    archive(POLARBEAR_SRC, POLARBEAR_DST, POLARBEAR_FILES, decode_utf8, POLARBEAR_README, 'reference_polarbear')
    print('完成')


if __name__ == '__main__':
    main()
