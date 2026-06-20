# GPOPS-II + MATLAB — OpenCode Skill

[English](#english) | [中文](#chinese)

---

## English

An [OpenCode](https://opencode.ai) skill providing expert-level guidance for **GPOPS-II v1.0** optimal control with **MATLAB** — hp-adaptive Radau pseudospectral methods + SNOPT solver. Covers everything from setup gotchas to production-grade warm start and EKF closed-loop patterns.

### What it covers

- **Setup**: critical field names (silent failures), `maxiteration` vs `maxiterations`, mesh tolerance tuning
- **SNOPT solver**: which parameters actually work (spoiler: only `tolerance`), hard-coded parameter table, exit code reference
- **Patterns**: warm start interpolation, EKF closed-loop guidance, C¹-smooth obstacle penalty design
- **Troubleshooting**: 15+ common errors with fixes
- **Parallel**: parfor + evalc transparency violation fix with generic wrapper pattern
- **API reference**: full output structure, continuous/endpoint function signatures, mesh history

### Install

```bash
mkdir -p ~/.config/opencode/skills/gpops2-matlab
cp SKILL.md REFERENCE.md ~/.config/opencode/skills/gpops2-matlab/
```

Restart OpenCode. The skill loads automatically when relevant topics are detected.

### Files

| File | Purpose |
|------|---------|
| `SKILL.md` | Quick reference — always loaded when skill activates |
| `REFERENCE.md` | Deep reference — loaded on demand by the LLM |

### Requirements

- MATLAB R2025a (or compatible)
- GPOPS-II v1.0 with SNOPT solver
- [OpenCode](https://opencode.ai)

> **Note**: This skill was developed and tested on WSL/Linux. All code patterns are platform-independent and work on Windows and macOS as well.

### Path conventions

| Placeholder | Typical value |
|-------------|---------------|
| `<MATLAB_ROOT>` | `/usr/local/MATLAB/R2025a` |
| `<GPOPS2_ROOT>` | `/usr/local/MATLAB/R2025a/toolbox/gpops2/Gpops` |

Replace these placeholders with your actual installation paths.

### License

MIT

---

## 中文

一个 [OpenCode](https://opencode.ai) 技能，为 **GPOPS-II v1.0** + **MATLAB** 最优控制提供专家级指导——hp-自适应 Radau 伪谱法 + SNOPT 求解器。涵盖从配置陷阱到生产级暖启动和 EKF 闭环模式的全套内容。

### 覆盖范围

- **配置**：关键字段名（静默失败）、`maxiteration` vs `maxiterations`、网格容差调优
- **SNOPT 求解器**：哪些参数真正生效（仅 `tolerance`）、硬编码参数表、退出码参考
- **模式**：暖启动插值、EKF 闭环制导、C¹ 光滑障碍惩罚设计
- **排错**：15+ 常见错误及修复方案
- **并行**：parfor + evalc 透明性违规修复（通用包装器模式）
- **API 参考**：完整输出结构、连续/端点函数签名、网格历史

### 安装

```bash
mkdir -p ~/.config/opencode/skills/gpops2-matlab
cp SKILL.md REFERENCE.md ~/.config/opencode/skills/gpops2-matlab/
```

重启 OpenCode。检测到相关话题时自动加载。

### 文件

| 文件 | 用途 |
|------|------|
| `SKILL.md` | 快速参考——技能激活时始终加载 |
| `REFERENCE.md` | 深度参考——由 LLM 按需读取 |

### 依赖

- MATLAB R2025a（或兼容版本）
- GPOPS-II v1.0 + SNOPT 求解器
- [OpenCode](https://opencode.ai)

> **注意**：本项目在 WSL/Linux 下开发与测试。所有代码模式均与平台无关，同样适用于 Windows 和 macOS。

### 路径约定

| 占位符 | 典型值 |
|--------|--------|
| `<MATLAB_ROOT>` | `/usr/local/MATLAB/R2025a` |
| `<GPOPS2_ROOT>` | `/usr/local/MATLAB/R2025a/toolbox/gpops2/Gpops` |

请替换为实际的安装路径。

### 许可

MIT
