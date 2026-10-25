#include "motor_task.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include "can.h"
#include "bsp_can.h"

#define MOTOR_CONTROL_PERIOD_MS 1U
#define FRAME_GAP_MS 1U

static int16_t m2006_current_command[M2006_MOTOR_COUNT];
static dm4310_mit_command_t dm4310_command[DM4310_MOTOR_COUNT];

static int16_t clamp_m2006_current(int16_t current)
{
    if (current > M2006_CURRENT_MAX)
    {
        return M2006_CURRENT_MAX;
    }
    if (current < M2006_CURRENT_MIN)
    {
        return M2006_CURRENT_MIN;
    }
    return current;
}

static HAL_StatusTypeDef send_m2006_current(void)
{
    CAN_TxHeaderTypeDef tx_header;
    uint8_t data[8] = {0};
    uint32_t mailbox;
    uint8_t i;

    tx_header.StdId = 0x200U;
    tx_header.ExtId = 0U;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = 8U;
    tx_header.TransmitGlobalTime = DISABLE;

    for (i = 0U; i < M2006_MOTOR_COUNT; i++)
    {
        uint16_t current = (uint16_t)m2006_current_command[i];
        data[2U * i] = (uint8_t)(current >> 8);
        data[2U * i + 1U] = (uint8_t)current;
    }

    return HAL_CAN_AddTxMessage(&hcan1, &tx_header, data, &mailbox);
}

void motor_task_set_m2006_current(uint8_t motor_index, int16_t current)
{
    if (motor_index >= M2006_MOTOR_COUNT)
    {
        return;
    }

    taskENTER_CRITICAL();
    m2006_current_command[motor_index] = clamp_m2006_current(current);
    taskEXIT_CRITICAL();
}

void motor_task_set_dm4310_mit(uint8_t motor_id,
                               const dm4310_mit_command_t *command)
{
    uint8_t motor_index;

    if ((command == NULL) ||
        (motor_id < DM4310_MOTOR1_ID) ||
        (motor_id > DM4310_MOTOR3_ID))
    {
        return;
    }

    motor_index = (uint8_t)(motor_id - DM4310_MOTOR1_ID);
    taskENTER_CRITICAL();
    dm4310_command[motor_index] = *command;
    taskEXIT_CRITICAL();
}

void motor_task_init(void)
{
    uint8_t i;

    for (i = 0U; i < M2006_MOTOR_COUNT; i++)
    {
        m2006_current_command[i] = 0;
    }

    for (i = 0U; i < DM4310_MOTOR_COUNT; i++)
    {
        dm4310_command[i].position_rad = 0.0f;
        dm4310_command[i].velocity_rad_s = 0.0f;
        dm4310_command[i].kp = 0.0f;
        dm4310_command[i].kd = 0.0f;
        dm4310_command[i].torque_nm = 0.0f;
    }

    for (i = 0U; i < DM4310_MOTOR_COUNT; i++)
    {
        (void)dm4310_enable(&hcan2,
                            (uint8_t)(DM4310_MOTOR1_ID + i),
                            DM4310_MODE_MIT);
        osDelay(1U);
    }
}


void motorstart_task(void *argument)
{
    uint32_t next_wake;
    uint8_t i;
    (void)argument;

     
    osDelay(3000U);
    motor_task_init();
    next_wake = osKernelGetTickCount();
    for (;;)
    {
//        (void)send_m2006_current();

        for (i = 0U; i <DM4310_MOTOR_COUNT; i++)
        {
            const dm4310_mit_command_t *command = &dm4310_command[i];
            (void)dm4310_mit_control(&hcan2,
                                     (uint8_t)(DM4310_MOTOR1_ID + i),
                                     command->position_rad,
                                     command->velocity_rad_s,
                                     command->kp,
                                     command->kd,
                                     command->torque_nm);
        next_wake += FRAME_GAP_MS;      /* 每发一帧，推进一个发送时间点 */
        (void)osDelayUntil(next_wake);
        }

        next_wake += MOTOR_CONTROL_PERIOD_MS;
        (void)osDelayUntil(next_wake);
    }
}
