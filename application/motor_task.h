#ifndef MOTOR_TASK_H
#define MOTOR_TASK_H

#include "struct_typedef.h"
#include "dm4310.h"
#include "CAN_receive.h"

/* M2006/C610 current command range. */
#define M2006_CURRENT_MIN (-10000)
#define M2006_CURRENT_MAX 10000

/* These setters are intended for RTOS task context, not interrupt context. */
void motor_task_set_m2006_current(uint8_t motor_index, int16_t current);

/* Entry configured as the external motor_task in CubeMX. */
void motorstart_task(void *argument);

#endif
