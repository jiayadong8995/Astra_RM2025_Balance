# Astra RM2025 Balance Chassis

Wheel-legged balance robot chassis firmware based on STM32H723VGT, developed for RoboMaster 2025.

Secondary development based on [DaMiao open-source wheel-legged robot](https://gitee.com/kit-miao/wheel-legged).

## Hardware

- MCU: STM32H723VGT
- IMU: BMI088
- Motors: DM4310 (leg joints) + DJI M3508 (drive wheels)
- Communication: FDCAN, UART (DBUS remote control)

## Software Architecture

FreeRTOS tasks:

| Task | Priority | Description |
|------|----------|-------------|
| INS_Task | Realtime | IMU attitude estimation (Mahony filter + Kalman) |
| Chassis_Task | AboveNormal | VMC leg kinematics and LQR balance control |
| Motor_Control_Task | AboveNormal | DM4310 & M3508 motor CAN communication |
| Observe_Task | High | Velocity/state estimation (Kalman filter) |
| Remote_Task | AboveNormal | Remote control input parsing and robot state machine |

## Key Algorithms

- **VMC** (Virtual Model Control): 5-bar linkage forward/inverse kinematics
- **LQR**: Linear-quadratic regulator for balance and velocity control
- **Kalman Filter**: Velocity and state estimation
- **Mahony Filter**: IMU attitude fusion

## Build

Keil MDK-ARM project located in `MDK-ARM/CtrlBoard-H7_IMU.uvprojx`.
