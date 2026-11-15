#include "motor_task.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include "can.h"
#include "bsp_can.h"
#include "gravity.h"
#include "gravity_log.h"

#define MOTOR_CONTROL_PERIOD_MS 2U

static int16_t m2006_current_command[M2006_MOTOR_COUNT];

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

void motor_task_init(void)
{
    uint8_t i;

    for (i = 0U; i < M2006_MOTOR_COUNT; i++)
    {
        m2006_current_command[i] = 0;
    }

    /* IDs 1 and 2 are controlled only by gravity_service_2ms(). */
}

static void motor_gravity_logger_task(void *argument)
{
    (void)argument;
    for (;;)
    {
        gravity_log_service();
        vTaskDelay(pdMS_TO_TICKS(2U));
    }
}

static void dis_output(void)//id3电机输出为0，无重力补偿
{
    static uint8_t id3_enabled;

    if (id3_enabled == 0U)
    {
        if (dm4310_enable(&hcan2, DM4310_MOTOR3_ID, DM4310_MODE_MIT) != HAL_OK)
            return;
        id3_enabled = 1U;
    }
    /* Keep ID 3 enabled and request zero output in every remote state. */
    (void)dm4310_mit_control(&hcan2, DM4310_MOTOR3_ID,
                             0.0f, 0.0f, 0.0f, 0.4f, 0.0f);
}

void motorstart_task(void *argument)
{
    uint32_t next_wake;
    (void)argument;

     
    gravity_init();
    if (xTaskCreate(motor_gravity_logger_task, "gravity_log", 768U, NULL,
                    tskIDLE_PRIORITY + 1U, NULL) != pdPASS)
    {
        Error_Handler();
    }
    next_wake = osKernelGetTickCount();
    for (;;)
    {
        send_m2006_current();
        gravity_service_2ms();
        dis_output();
        next_wake += MOTOR_CONTROL_PERIOD_MS;
        (void)osDelayUntil(next_wake);
    }
}
