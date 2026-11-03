# 14.CAN: DM4310 IDs 1 and 2 gravity experiment

Open the original `../MDK-ARM/can.uvprojx`. It now compiles this folder's
`gravity.c`, `gravity_model.c`, and `gravity_log.c` directly.
The original `application/motor_task.c` remains the CubeMX `motorstart_task`
entry. It services gravity every 2 ms and runs a separate logger task.
IDs 1 and 2 are exclusively owned by gravity. ID 3 is enabled independently
of the remote switches and receives a zero-gain, zero-torque MIT command
every control cycle. Its feedback angle is read by the six-angle stream.
The former independent ID 1 MIT command channel has been removed.
The earlier `can_gravity.uvprojx` and `gravity_task.c` are legacy alternate
entry files; do not add `gravity_task.c` to the original project because it
defines a second `motorstart_task`.

## Switch mapping

The reference DJI DBUS frame has `s[0]` as left; values are
UP=1, MID=3, DOWN=2. The right switch is not used by this experiment.

- Left DOWN: enable all three motors and send all-zero MIT control fields.
- Left MID: IDs 1 and 2 run fitted gravity feed-forward plus Kd damping;
  ID 3 keeps receiving all-zero MIT control fields.
- Left UP: IDs 1 and 2 run joint-space automatic sampling with position hold;
  ID 3 keeps receiving all-zero MIT control fields.
- Invalid/offline DBUS disables IDs 1 and 2. Stale/faulty CAN feedback
  or an out-of-range ID 1/2 joint disables IDs 1 and 2. ID 3 receives only
  zero-output commands while enabled; it has no gravity compensation.

## Before enabling movement

`GRAVITY_CONFIG_READY` starts at 0. In `gravity_config.h`, set the actual
joint safety ranges, home positions, speed and torque limits, and check both
positive directions. Only then set `GRAVITY_CONFIG_READY` to 1. The initial
model coefficients and `gravity_output_scale[]` are zero. Check the actual
DM4310 MIT P/V/T ranges and Master IDs in the unchanged `application/dm4310.h`.

The model assumes the two motor angles form a planar serial pair. If gearing,
mechanical offsets, reversed installation, or nonparallel joint axes change
that relationship, change the model features and motor-to-joint conversion
before fitting. Distal motor group pose must remain fixed for this baseline:
the current log and model contain only motor 1 and 2 angles. If distal motor
angles move the center of mass, record their calibrated joint angles and add
them to the model before claiming compensation across those poses.

The sampled Halton poses cover the configured rectangular angle range. A
straight path between two individually safe poses can still hit an obstacle.
Check the entire reachable path with the real mechanism supported before
setting `GRAVITY_CONFIG_READY=1`.

## Data path

CAN2's existing receive callback writes `dm4310_feedback[]`. The gravity
control task reads IDs 1 and 2 every 2 ms. Sampling uses a shared smooth
trajectory, requires both axes to settle, then averages 32 *fresh* feedback
pairs. Forward and reverse passes use the same `pair_id`; the last pose has
no reverse sample and is ignored by the fitter.

USART1_TX on PA9 emits `GRAV2,v1` records at 115200 8N1. Fields are
`seq,pair_id,F/R,tick_ms,q1_mrad,q2_mrad,t1_mNm,t2_mNm`. The control task
queues each record; a lower-priority task transmits it. Save the serial stream
to a text file, then run from this directory:

```powershell
py -3 -m pip install numpy
py -3 fit_gravity_2axis.py --log .\gravity.log
```

Inspect `fits/<timestamp>/report.json` and test independent poses. To install
the coefficients, rerun with `--install`, rebuild in Keil, then flash. Start
`gravity_output_scale[0]` and `[1]` near zero and raise them gradually while
supporting the arm. Keep software torque limits conservative.

The feedback torque is the drive's reported estimate during position hold;
it is not an independent load-cell measurement. Friction, cable forces and
position-loop effects can bias the fit. F/R averaging only reduces some
direction-dependent friction.

## CubeMX and DBUS

The original `../can.ioc` now reserves the gravity peripherals. It keeps
CAN1/CAN2, USART6, and the STM32F407IGH6 clock setup,
and reserves PC11/USART3_RX with DMA1 Stream1 Channel 4, and PA9/USART1_TX.
`Src/main.c` calls `MX_DMA_Init`, `MX_USART1_UART_Init`, and
`MX_USART3_UART_Init`; `Src/usart.c` configures both UARTs with HAL and links
USART3 to DMA1 Stream1. The reference example's `remote_control.c/.h` and
`bsp_rc.c/.h` are now in `application/` and `bsp/boards/`. They decode DBUS
and operate the DMA double buffer after HAL initialization. Gravity and motor
tasks read the decoded switches directly. The reference `USART3_IRQHandler`
is defined in `application/remote_control.c`; no second USART3 handler is
compiled. `gravity_log.c` transmits through
`HAL_UART_Transmit(&huart1, ...)`. If CubeMX regenerates source files, retain
the DBUS IDLE handler and verify the 9-bit word length with even parity.

The DBUS receiver follows `9.remote_control_dma`: 36-byte DMA double buffer,
USART IDLE interrupt, 18-byte frame and the same switch bit positions. The
USART3 setting uses 100000 baud, 8 data bits, even parity, one stop bit.
On STM32F4 this means M=1 (9-bit frame including parity). The reference
example's `UART_WORDLENGTH_8B` with parity would leave only seven data bits.

With `GRAVITY_CONFIG_READY=0`, any valid left-switch position plus valid DBUS
permits zero-output enable of IDs 1 and 2; requested MID/UP functions remain
locked until the joint ranges and home positions are configured.
Support the arm mechanically: zero output or a feedback/DBUS fault can let it fall.
ID 3 is included in the existing six-angle stream. Its MIT command has
Kp=Kd=tff=0, so the application requests no torque from it. This does not
guarantee zero physical torque from the drive itself.

The original Keil project builds with ARM Compiler 5. Hardware behavior and
CubeMX source regeneration have not been verified on the physical board.
