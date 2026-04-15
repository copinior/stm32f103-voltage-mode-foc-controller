# STM32F103 Voltage-Mode FOC Controller

[中文说明](README.zh-CN.md)

An STM32F103xE motor-control project generated from STM32CubeMX and extended with FreeRTOS scheduling, AS5600 feedback, voltage-mode FOC control without phase-current sensing, a local UART command interface, and an optional voice-command bridge.

## Overview

This project corresponds to a standalone embedded motor-control system:

- three-phase voltage-mode FOC control on STM32F103xE
- AS5600-based rotor angle feedback
- local UART command input and runtime status observation
- optional voice-command ingress from an ESP8266 bridge over USART3
- multi-loop control paths covering position, speed, and cascaded control modes
- FreeRTOS task split for sensing, command dispatch, and status printing

The control stack includes electrical-angle alignment and SVPWM modulation, but it does not implement phase-current sensing or a true inner current loop. When voice recognition is enabled, the upstream frontend is provided by a separate ESP32 / ESP8266 project.

## Runtime Responsibilities

- initialize PWM, timers, UART, and RTOS objects
- run the motor control loop from the TIM3 interrupt path
- sample encoder data and update shared runtime state
- dispatch local UART commands and bridged voice commands through a unified control service
- execute voltage-mode position, speed, speed-torque, and position-speed-torque control paths
- print runtime status for debugging, calibration, and link observation

## Repository Layout

```text
Core/                   CubeMX-generated application entry, ISR hooks, RTOS tasks
Drivers/motor/          AS5600 driver, FOC loop, motor command parser, control service
Drivers/voice_bridge/   USART3 frame parser and voice-command dispatcher
algorithm/              shared filter helpers
Middlewares/            CMSIS-DSP and FreeRTOS sources
cmake/                  GCC toolchain and STM32CubeMX CMake glue
MDK-ARM/                optional Keil MDK project files
stm32f103_voltage_mode_foc_controller.ioc
                        STM32CubeMX hardware configuration source
```

The main runtime path depends on:

- `Drivers/motor`
- `Drivers/voice_bridge`
- `Core/Src/freertos.c`

## Processing Flow

Main processing flow:

1. System startup initializes GPIO, TIM1 PWM, TIM3, USART1, USART3, and RTOS objects
2. TIM3 drives the control-period interrupt, while `SensorTask` refreshes encoder samples
3. `CmdTask` polls both the local UART console and the optional voice bridge
4. Commands are translated into position, speed, stop, reset, or calibration requests
5. `motor_control_service` updates the active control mode used by the voltage-mode FOC loop
6. `TelemetryTask` prints motor status when requested

## Voice Command Map

The voice bridge accepts command IDs `0..15` to remain aligned with the upstream speech-recognition project. Command IDs `0` and `1` are reserved and are not mapped to application-level motion actions on the STM32 side.

| command_id | meaning |
| --- | --- |
| 0 | reserved |
| 1 | reserved |
| 2 | accelerate |
| 3 | decelerate |
| 4 | emergency stop |
| 5 | stop |
| 6 | turn left |
| 7 | turn right |
| 8 | U-turn left |
| 9 | U-turn right |
| 10 | reverse |
| 11 | switch to position mode |
| 12 | switch to speed mode |
| 13 | print status |
| 14 | reset system |
| 15 | start calibration |

## Control Notes

- `dq`-axis voltage commands are converted into three-phase PWM outputs through inverse Park transformation and SVPWM
- the implementation does not use phase-current sensing
- because the motor driver hardware does not provide ADC current sampling, torque/current-related control paths are approximated through voltage-command paths
- the control stack uses a multi-loop architecture under constrained sensing conditions

## Hardware Platform

- the target plant is a `2804` outrunner BLDC motor
- the control platform is based on `STM32F103` and a `SimpleFOC Mini` board
- rotor position feedback is provided by an `AS5600` magnetic encoder
- TIM1 is configured for three center-aligned PWM channels, with a PWM frequency of `40 kHz`
- the STM32 side only needs to configure three PWM channels; the complementary drive topology is provided by the `SimpleFOC Mini` power stage
- TIM3 drives the main control-period interrupt at `500 Hz`
- `USART1` is used for the local command console and runtime logs, while `USART3` is reserved for the optional voice-command bridge

## Requirements

- target MCU: `STM32F103xE`
- STM32CubeMX project source: [`stm32f103_voltage_mode_foc_controller.ioc`](stm32f103_voltage_mode_foc_controller.ioc)
- CMake `3.22+`, Ninja, and GNU Arm Embedded Toolchain for the open-toolchain build path
- optional Keil MDK-ARM project: `MDK-ARM/stm32f103_voltage_mode_foc_controller.uvprojx`

## Configuration

- `Core/Inc/app_config.h` controls whether the voice bridge is enabled
- `Core/Src/usart.c` configures both USART1 and USART3 at `115200`
- `Drivers/voice_bridge/voice_bridge.h` defines the bridge frame format and command range

With the voice bridge disabled, the project still supports local UART command control and status output.
