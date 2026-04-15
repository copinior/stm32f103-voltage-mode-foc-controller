# STM32F103 无电流采样电压型 FOC 多环控制工程

[English README](README.md)

## 演示视频

- Bilibili: https://www.bilibili.com/video/BV165QEBjEX6/

基于 STM32CubeMX 生成并持续扩展的 STM32F103xE 电机控制工程，集成了 FreeRTOS 任务调度、AS5600 角度反馈、无电流采样的电压型 FOC 控制、本地串口命令接口，以及可选的语音命令桥接链路。

## 项目定位

该项目对应一个可独立运行的嵌入式电机控制工程：

- 在 STM32F103xE 上完成三相电压型 FOC 电机控制
- 通过 AS5600 获取转子机械角度反馈
- 通过本地串口进行命令输入和状态观测
- 通过 USART3 接收来自 ESP8266 桥接节点的语音命令
- 支持位置、速度以及多环级联控制路径
- 使用 FreeRTOS 将采样、命令处理和状态打印拆分为独立任务

控制链路包含电角度对齐和 SVPWM 调制，但不包含相电流采样与真实电流闭环。若启用语音识别，上游前端由独立的 ESP32 / ESP8266 工程提供。

## 运行职责

- 初始化 PWM、定时器、串口和 RTOS 对象
- 在 TIM3 中断路径中执行电机控制主循环
- 采集编码器数据并更新运行时状态
- 统一处理本地 UART 命令与桥接语音命令
- 执行位置、速度、速度-力矩级联、位置-速度-力矩级联控制路径
- 打印运行状态，便于调试、校准和链路观察

## 仓库结构

```text
Core/                   CubeMX 生成的应用入口、中断钩子、RTOS 任务
Drivers/motor/          AS5600 驱动、FOC 控制、电机命令解析与控制服务
Drivers/voice_bridge/   USART3 语音桥接帧解析与命令分发
algorithm/              通用滤波辅助模块
Middlewares/            CMSIS-DSP 与 FreeRTOS 源码
cmake/                  GCC 工具链与 STM32CubeMX 的 CMake glue
MDK-ARM/                可选的 Keil MDK 工程文件
stm32f103_voltage_mode_foc_controller.ioc
                        STM32CubeMX 硬件配置源文件
```

主要运行链路依赖：

- `Drivers/motor`
- `Drivers/voice_bridge`
- `Core/Src/freertos.c`

## 处理流程

主流程：

1. 系统启动后初始化 GPIO、TIM1 PWM、TIM3、USART1、USART3 和 RTOS 对象
2. TIM3 提供控制周期中断，`SensorTask` 负责刷新编码器采样
3. `CmdTask` 轮询本地串口控制台和可选的语音桥接输入
4. 命令被转换为位置、速度、停止、复位或校准请求
5. `motor_control_service` 更新电压型 FOC 主循环使用的控制模式
6. `TelemetryTask` 在收到请求时打印电机状态

## 语音命令映射

语音桥接支持命令 ID `0..15`，用于与上游语音识别工程保持一致。其中命令 `0` 和 `1` 作为保留命令位，在 STM32 侧不参与应用层动作分发。

| command_id | 含义 |
| --- | --- |
| 0 | 预留 |
| 1 | 预留 |
| 2 | 加速 |
| 3 | 减速 |
| 4 | 急停 |
| 5 | 停止 |
| 6 | 左转 |
| 7 | 右转 |
| 8 | 左掉头 |
| 9 | 右掉头 |
| 10 | 倒车 |
| 11 | 切换到位置模式 |
| 12 | 切换到速度模式 |
| 13 | 打印状态 |
| 14 | 系统复位 |
| 15 | 启动校准 |

## 控制说明

- `dq` 轴电压指令经反 Park 变换和 SVPWM 转换为三相 PWM 输出
- 实现不使用相电流采样
- 由于使用的电机驱动硬件部分缺少adc电流采样，力矩/电流相关驱动选择通过电压指令路径模拟电流/力矩环
- 控制栈采用受限传感条件下的多环控制架构

## 硬件平台

- 控制对象为 `2804` 外转子无刷电机
- 控制板平台基于 `STM32F103 + SimpleFOC Mini`
- 转子位置反馈使用 `AS5600` 磁编码器
- TIM1 输出三路中心对齐 PWM，频率为 `40 kHz`
- STM32 侧仅需配置三路 PWM 通道，由`SimpleFOC Mini` 板级驱动侧提供互补驱动拓扑
- TIM3 作为主控制周期中断，控制更新频率为 `500 Hz`
- `USART1` 用于本地串口命令与日志输出，`USART3` 用于可选语音桥接输入

## 依赖环境

- 目标 MCU：`STM32F103xE`
- STM32CubeMX 工程源：[`stm32f103_voltage_mode_foc_controller.ioc`](stm32f103_voltage_mode_foc_controller.ioc)
- 开放构建链路：`CMake 3.22+`、Ninja、GNU Arm Embedded Toolchain
- 可选 Keil 工程：`MDK-ARM/stm32f103_voltage_mode_foc_controller.uvprojx`

## 配置说明

- `Core/Inc/app_config.h` 控制是否启用语音桥接
- `Core/Src/usart.c` 中默认配置了 USART1 和 USART3，波特率均为 `115200`
- `Drivers/voice_bridge/voice_bridge.h` 中定义了桥接帧格式和命令范围

关闭语音桥接后，工程仍可通过本地 UART 控制台进行电机控制和状态打印。
