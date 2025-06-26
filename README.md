# 🤖 Lampbot - 6轴智能机械臂台灯

[![STM32](https://img.shields.io/badge/STM32-F407IGH-blue.svg)](https://www.st.com/en/microcontrollers-microprocessors/stm32f407ig.html)
[![License](https://img.shields.io/badge/license-MIT-green.svg)](LICENSE)
[![RoboMaster](https://img.shields.io/badge/RoboMaster-Competition-red.svg)](https://www.robomaster.com/)

> 🎯 一个基于STM32F407的6轴机械臂智能台灯系统，集成语音控制、光照跟随、坐姿提醒等多种功能

## ✨ 主要特性

### 🦾 机械臂控制
- 🔧 **6轴自由度控制** - 基于DH参数的运动学解算
- 🎯 **精确定位** - 支持DM4310和M2006电机混合控制
- 🔄 **多种运动模式** - 插补运动、点到点运动
- 📐 **运动学仿真** - MATLAB/GeoGebra仿真验证

### 💡 智能灯光系统
- 🌟 **亮度调节** - 语音控制灯光亮度 (0-100%)
- 🌈 **色温调节** - 智能色温控制 (2700K-6500K)
- 👀 **光照跟随** - 机械臂自动跟随用户位置提供最佳照明
- 🎨 **氛围灯效** - 多种灯光展示模式

### 🎤 语音交互
- 🗣️ **语音识别** - 支持中文语音指令识别
- 🤖 **智能问答** - 集成AI助手功能
- 🎛️ **语音控制** - 通过语音控制机械臂动作和灯光

### 🏥 健康提醒
- 💺 **坐姿提醒** - 智能检测并提醒用户保持正确坐姿
- 👁️ **远眺提醒** - 定时提醒用户进行眼部休息
- 📊 **健康数据** - 记录用户使用习惯和健康数据

### 🌐 网络功能
- 🖥️ **Web控制面板** - 浏览器端控制界面
- 👨‍👩‍👧‍👦 **家长监控** - 家长端监控界面
- 📱 **远程控制** - 支持远程监控和控制

## 🏗️ 系统架构

```
├── 🎯 Core/                    # STM32核心代码
│   ├── Inc/                   # 头文件
│   │   ├── Arm.h             # 机械臂主控制
│   │   ├── Arm_Calc.h        # 运动学计算
│   │   └── ...
│   └── Src/                   # 源文件
├── 📚 UserLib/                # 用户库
│   ├── DM4310.h/.c           # DM4310电机驱动
│   ├── M2006.h/.c            # M2006电机驱动
│   └── Light.h/.c            # 灯光控制
├── 🔧 UserTasks/              # 用户任务
├── 🧮 Simulation/             # 仿真文件
│   ├── *.m                   # MATLAB仿真
│   └── *.ggb                 # GeoGebra仿真
└── 📖 Docs/                   # 项目文档
```

## 🚀 快速开始

### 📋 系统要求
- **MCU**: STM32F407IGH6
- **开发环境**: STM32CubeIDE / CLion
- **编译器**: ARM GCC
- **调试器**: ST-Link V2

### 🔧 硬件连接
```
📡 CAN总线 → DM4310电机 (关节1-3)
📡 CAN总线 → M2006电机 (关节4-6)  
🔌 UART → 语音模块
💡 PWM → LED灯光控制
🔘 GPIO → 按键/开关
```

### ⚡ 编译&烧录
```bash
# 1. 克隆项目
git clone <repository-url>

# 2. 打开项目 (CLion)
# File → Open → 选择项目文件夹

# 3. 编译项目
cmake --build cmake-build-debug --target all

# 4. 烧录到MCU
# 使用ST-Link或通过IDE直接烧录
```

### 🎮 基本使用

#### 🤖 机械臂姿态控制
```c
// 切换到坐姿提醒模式
Arm_Set_Posture(Remind_Sitting_Posture);

// 切换到远眺提醒模式  
Arm_Set_Posture(Remind_Looking_Forward_Posture);

// 光照跟随模式
Arm_Set_Posture(Light_Tracing_Posture);
```

#### 💡 灯光控制
```c
// 设置亮度 (0-100%)
Light_Set_Brightness(&light1, 80);

// 设置色温 (2700K-6500K)
Light_Set_Temperature(&light1, 5000);
```

## 🎮 遥控器控制模式

根据代码中的`Arm_Task()`函数，支持以下遥控器挡位控制模式：

| S1挡位 | S2挡位 | 功能模式 | 描述 |
|--------|--------|----------|------|
| 1️⃣ | 1️⃣ | `Arm_Remote_Mode()` | 🎛️ 机械臂手动遥控模式 |
| 1️⃣ | 3️⃣ | `Arm_Light_Remote()` | 💡 灯光手动遥控模式 |
| 3️⃣ | 1️⃣ | `Arm_Remind_Sitting()` | 💺 坐姿提醒动作 |
| 3️⃣ | 2️⃣ | `Arm_Light_Tracing_Present()` | 👀 光照跟随展示模式 |
| 3️⃣ | 3️⃣ | `Arm_Looking_Forward()` | 👁️ 远眺提醒动作 |
| 2️⃣ | 2️⃣ | `Arm_Turn_Itself_Off()` | 🔌 自动关机序列 |
| 2️⃣ | 3️⃣ | `Arm_Quick_Turn_Itself_Off()` | ⚡ 快速关机序列 |
| 其他 | 其他 | `Arm_Back()` | 🏠 返回基础待机姿态 |

> 📝 **注意**: 拨杆开关从上到下对应数值: 1️⃣→3️⃣→2️⃣

### 🎛️ 详细控制说明

#### 🤖 机械臂手动遥控模式 (S1=1, S2=1)
- **CH0**: Yaw轴控制 (左右旋转)
- **CH1**: Pitch1轴控制 (第一关节俯仰)
- **CH2**: Pitch2轴控制 (第二关节俯仰)
- **CH3**: Pitch3轴控制 (第三关节俯仰)
- **滚轮**: 灯光亮度调节

#### 💡 灯光手动遥控模式 (S1=1, S2=3)
- **CH1**: 色温调节 (暖光↔冷光)
- **CH3**: 亮度调节 (亮↔暗)

## 🎯 支持的姿态模式

| 模式 | 描述 | 触发方式 |
|------|------|----------|
| 🏠 `Base_Posture` | 基础待机姿态 | 系统初始化后 |
| 💺 `Remind_Sitting_Posture` | 坐姿提醒动作 | 语音指令/定时触发 |
| 👁️ `Remind_Looking_Forward_Posture` | 远眺提醒动作 | 语音指令/定时触发 |
| 💡 `Light_Tracing_Posture` | 光照跟随模式 | 自动检测/手动切换 |
| 📖 `Book_Follow_Posture` | 阅读跟随模式 | 检测到阅读行为 |
| 🔌 `Turn_Itself_Off_*` | 自动关机序列 | 长时间无操作 |

## 🎤 语音指令

| 指令类型 | 示例指令 | 功能 |
|----------|----------|------|
| 💡 灯光控制 | "调亮一点" / "调暗一点" | 调节亮度 |
| 🌈 色温控制 | "暖光模式" / "冷光模式" | 调节色温 |
| 🦾 姿态控制 | "坐姿提醒" / "远眺提醒" | 切换机械臂姿态 |
| 🔄 模式切换 | "跟随模式" / "待机模式" | 切换工作模式 |

## 🧮 运动学参数

基于标准DH参数建立的6轴机械臂运动学模型：

| 关节 | θ (°) | d (mm) | a (mm) | α (°) |
|------|-------|--------|--------|-------|
| 1 | θ₁ | d₁ | 0 | 90 |
| 2 | θ₂ | 0 | a₂ | 0 |
| 3 | θ₃ | 0 | a₃ | 0 |
| 4 | θ₄ | d₄ | 0 | 90 |
| 5 | θ₅ | 0 | 0 | -90 |
| 6 | θ₆ | d₆ | 0 | 0 |

## 🔧 开发进度

- [x] ✅ 机械臂运动学解算
- [x] ✅ DM4310/M2006电机驱动
- [x] ✅ CAN通讯协议
- [x] ✅ 灯光控制系统
- [x] ✅ 语音识别接入
- [x] ✅ 坐姿/远眺提醒功能
- [x] ✅ 自动关机功能
- [x] ✅ 光照跟随算法
- [ ] 🔄 Web控制界面 (开发中)
- [ ] 🔄 家长监控系统 (开发中)
- [ ] 📋 入座自启动功能 (计划中)

## 🤝 贡献指南

1. Fork 本项目
2. 创建特性分支 (`git checkout -b feature/AmazingFeature`)
3. 提交更改 (`git commit -m 'Add some AmazingFeature'`)
4. 推送到分支 (`git push origin feature/AmazingFeature`)
5. 开启 Pull Request

### 📝 代码规范
- 遵循项目中的 [OOP规范](Docs/OOP.md)
- 使用小写字母+下划线命名文件
- 函数命名模仿HAL库风格
- 优先使用外部头文件

## 📄 许可证

本项目采用 MIT 许可证 - 查看 [LICENSE](LICENSE) 文件了解详情

___
<div align="center">
  <p>🌟 如果这个项目对你有帮助，请给我们一个 Star！</p>
  <p>Made with ❤️ by RoboMaster Team</p>
</div>
