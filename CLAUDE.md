# CLAUDE.md — Serial Leg Infantry 工程速查

> 大疆 RoboMaster 串腿步兵机器人固件。STM32F407 + FreeRTOS + Keil MDK。
> 维护者：@CY
> 项目文档入口：`文档管理/00_总览.md`（需求 / 架构 / 进度 / 调研 / 交接）

---

## 一、⚠️ 编码约定（最重要，违反会永久损坏文件）

**所有源文件必须保存为 UTF-8 带 BOM。**

原因：工程用 **Keil ARMCC V5.06 (AC5)** 编译器，它在中文 Windows 上按系统 GBK 解析源文件。
- 存成 GBK → VSCode 默认按 UTF-8 打开 → 中文注释乱码
- 存成 UTF-8 不带 BOM → Keil 按 GBK 解析 → 中文注释同样乱码
- 存成 **UTF-8 带 BOM** → 两头都能正确识别 ✅

`.vscode/settings.json` 已设 `"files.encoding": "utf8bom"` 兜底。

### 绝对不要做的事

**不要用错误的编码打开文件后点保存。** 编辑器会把无法解码的字节替换成 `�`(U+FFFD) 写回文件，
**原始中文就永久丢失了**，换编码救不回来。2026-10-06 的 `gimbal.c` / `gimbal.h` 就是这么坏的
（分别损坏 151 和 136 个字符），最后是靠 `.bak` 备份才恢复的。

看到注释里出现 `�` 或 `？` 方块，**立刻停止保存**，先确认编辑器右下角显示的编码是否正确。

### 相关工具

| 脚本 | 用途 |
|------|------|
| `tools/fix_encoding.py` | 扫描全工程编码，把 GBK/UTF-8无BOM 统一转成 UTF-8带BOM。默认试运行，`--apply` 才写入，自动备份 |
| `tools/repair_comments.py` | 用 `.bak` 备份里完好的中文，修复已被 U+FFFD 损坏的注释（代码用当前的，注释用备份的） |

---

## 二、工具链与构建

| 项目 | 说明 |
|------|------|
| 芯片 | STM32F407 |
| 编译器 | Keil ARMCC **V5.06 update 7 (AC5)**，`<uAC6>0` |
| 主工程 | `MDK-ARM/Serial Leg Infantry.uvprojx` |
| 备用构建 | EIDE（VSCode 插件），配置在 `MDK-ARM/.eide/eide.yml` |
| CubeMX 工程 | `Serial Leg Infantry.ioc`（根目录） |

**编译方式**：Keil 图形界面手动编译；**也可用命令行 armcc 做快速质检**
（`D:\keil5\ARM\ARMCC\Bin\armcc.exe`，V5.06 update 7 build 960，
完整参数见 `文档管理/05_交接文档.md` §五 第 3 条）。命令行能查语法/类型错误，
链接与最终产物仍需 Keil。

> `Core/` 下的文件由 CubeMX 生成。**改代码只能写在 `/* USER CODE BEGIN ... */` 和
> `/* USER CODE END ... */` 之间**，否则重新生成代码时会被冲掉。

---

## 三、目录职责

| 目录 | 职责 | 说明 |
|------|------|------|
| `Core/` | CubeMX 生成的外设初始化 | 只往 USER CODE 块里加代码 |
| `application/` | 应用层业务逻辑 | 云台、摩擦轮、遥控器、INS 任务 |
| `bsp/` | 板级支持包 | CAN / SPI / I2C / USART / Flash / LED 等 |
| `bsp/devices/` | 具体芯片驱动 | BMI088(IMU)、IST8310(磁力计)、OLED |
| `algorithm/` | 算法 | `control_algorithm/` PID+滤波，`optimization_algorithm/` AHRS |
| `TEST/` | 测试代码 | **参与编译**，不是废代码，改动需谨慎 |
| `Drivers/` `Middlewares/` | HAL 库 + FreeRTOS + DSP | **第三方库，不要改** |
| `tools/` | 维护脚本 | 编码/注释修复工具 |
| `文档管理/` | 项目文档 | 需求/架构/调研/参考文档集中存放，勿散落别处 |
| `MDK-ARM/` | Keil 工程与构建产物 | |

> `algorithm/optimization_algorithm/AHRS.lib` 和 `arm_cortexM4lf_math.lib` 是**预编译库**，
> 只有头文件，看不到源码。

---

## 四、FreeRTOS 任务架构

任务在 `Core/Src/freertos.c` 的 `MX_FREERTOS_Init()` 里创建（位于 USER CODE 块内）。

| 任务 | 入口函数 | 优先级 | 栈(字) | 说明 |
|------|---------|--------|--------|------|
| `defaultTask` | `StartDefaultTask` | Normal | 128 | 空转占位，只 `osDelay(1)` |
| `imuTask` | `INS_task` | **Realtime** | 1024 | 姿态解算，优先级最高 |
| `gimbal` | `gimbal_task` | High | 1024 | 云台控制 |
| `chassis` | `chassis_task` | High | 512 | 底盘控制（2ms 周期，开发中见 MVU-3） |

关键配置（`Core/Inc/FreeRTOSConfig.h`）：tick 1000Hz、最大优先级 7、堆 24576 字节。

**新增任务**：定义和创建都写在 `/* USER CODE BEGIN RTOS_THREADS */` 块内。

---

## 五、调试约定

**调试工具**：VOFA+（波形）、J-Link/ST-Link（硬件调试）、逻辑分析仪（总线抓包）、CAN 分析仪。

**VOFA+ 监控变量**（定义在 `application/gimbal.c` 与 `application/chassis.c`）：

| 变量 | 含义 |
|------|------|
| `gimbal_stage` | 当前任务执行阶段 1-5 |
| `gimbal_loop_count` | 完整循环计数 |
| `gimbal_tick_delta` | 两次循环间隔的 tick 差 |
| `can_tx_fail_count` | CAN 发送失败计数 |
| `test_iput_angle` | PID 角度环输出 |
| `angle_diff` | 目标角度与实际角度差值 |
| `chassis_stage` | 底盘任务执行阶段 1-6 |
| `chassis_loop_count` | 底盘循环计数（正常约 500/s） |
| `chassis_tick_delta` | 底盘两次循环 tick 差（正常 ≈2ms） |
| `chassis_vx` / `chassis_vy` / `chassis_v_rot` | 底盘解算中间量（RPM） |

**问题记录**：所有问题都记在 `ISSUES.md`（已有完整的模板和分类标签），
修复后补根因分析和解决方案。遇到新坑请顺手记录。

---

## 六、其他注意事项

- **`.bak` 文件不要删。** 工程里散落着 `xxx.c.bak` / `.bak2`，它们是 GBK 编码的旧版本，
  2026-10-06 就是靠它们救回了被损坏的注释。它们已被 `.gitignore` 排除，只存在本地。
- **`.bak` 文件本身是 GBK 编码**，这是正常的（它们是备份，不参与编译）。
  用 VSCode 打开可能显示乱码，属预期现象。
- **`*.bak` 后缀的文件不会被编码工具处理**（工具只认 `.c/.h/.cpp/.hpp`）。
- 仓库 `core.autocrlf=true` 而文件是 LF，提交时会刷 `LF will be replaced by CRLF` 警告，
  属既有现象，不影响使用。

---

## 七、交互约定

- **全程使用中文**交流，包括工具调用中会显示给用户看的描述文字。
- 执行维护类操作时，优先把逻辑写成 `tools/` 下的脚本文件再调用，
  这样权限弹窗里只显示一行易读的命令，用户也能自己审查脚本。
- 破坏性操作（批量改文件、覆盖）必须先试运行、显示对照，确认后再执行。
- **用 Write 工具整体覆盖源文件后必须补回 BOM**（Write 不保留 BOM）：
  检查 `head -c 3 文件 | xxd` 应为 `efbbbf`；缺失时用 `tools/fix_encoding.py` 或手动补。
- Python 脚本向终端输出中文时，命令加 `PYTHONIOENCODING=utf-8` 前缀；
  不要用 bash 内嵌反引号写含代码片段的文本（会被命令替换吞掉，此前损坏过文档）。
