# STM32F407 CAN motor control

Keil MDK and STM32CubeMX project for three DM4310 motors on CAN2 and three M2006 motors on CAN1.

## Project files

- `can.ioc`: STM32CubeMX configuration
- `MDK-ARM/can.uvprojx`: Keil uVision project
- `application/`: motor protocol and control task
- `bsp/`, `Src/`, `Inc/`: board and application code
- `Drivers/`, `Middlewares/`: required STM32 HAL, CMSIS, and FreeRTOS sources

Open `MDK-ARM/can.uvprojx` in Keil uVision to build. Build output and local debugger settings are excluded from Git.

Before running motors, verify each motor's CAN ID, Master ID, control mode, and MIT parameter ranges against its settings in the Damiao debugging tool.
