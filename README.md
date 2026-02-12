# STM32 自研四轴飞行器系统 (STM32-Drone-System)

## 📖 项目简介
本项目是一个从零开发的四轴飞行器系统，包含**飞控端 (Flight Controller)**、**遥控器端 (Remote Control)** 以及配套的 **PCB 硬件设计**。
项目旨在实现一个稳定、可控的微型空心杯无人机，核心基于 STM32F4 和 STM32F1 系列微控制器，采用 NRF24L01 进行无线通信，并实现了串级 PID 姿态控制算法。

## 📂 仓库结构
* **Drone_Code/**: 飞控端固件，基于 STM32F411CEU6。
* **Remote_Code/**: 遥控器端固件，基于 STM32F103C8T6。
* **PCB_Design/**: 嘉立创 EDA (EasyEDA) 设计的原理图与 PCB 源文件。

## 🛠️ 技术栈 (Tech Stack)

### 1. 硬件平台
* **飞控主控**: STM32F411CEU6 (Cortex-M4, 100MHz)
* **遥控主控**: STM32F103C8T6 (Cortex-M3, 72MHz)
* **传感器**: MPU6050 (6轴 IMU: 陀螺仪 + 加速度计)
* **无线通信**: NRF24L01+ (2.4G SPI 接口)
* **动力系统**: 8520 空心杯电机 + SI2302 MOS 管驱动
* **显示交互**: 0.96寸 OLED (I2C)

### 2. 软件与算法
* **开发环境**: Keil MDK-ARM
* **底层驱动**:
    * 模拟 I2C 驱动 MPU6050 (针对 GPIO 翻转速度优化，读取稳定)
    * 硬件 SPI 驱动 NRF24L01 (基于标准库的高速读写)
    * TIM 定时器输出 1kHz 高频 PWM 控制电机
* **核心算法**:
    * **姿态解算**: Mahony 互补滤波算法 (融合陀螺仪与加速度计数据)
    * **飞行控制**: 串级 PID 控制 (内环角速度环 + 外环角度环)
    * **通信协议**: 基于 Enhanced ShockBurst™ 模式 (启用硬件 CRC 校验与自动重发机制，降低 MCU 负载)
