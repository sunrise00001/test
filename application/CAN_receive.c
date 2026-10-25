#include "CAN_receive.h"
#include "can.h"
#include "dm4310.h"

/* CAN 接收中断写入，主循环或控制任务读取。 */
motor_measure_t motor_2006[M2006_MOTOR_COUNT];
uint8_t motor_index;

/* 按大端格式拆解 M2006 的 8 字节反馈帧。 */
static void decode_2006_feedback(volatile motor_measure_t *motor, const uint8_t data[8])
{
    motor->last_ecd = motor->ecd;
    motor->ecd = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
    motor->speed_rpm = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
    motor->given_current = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
    motor->temperature = data[6];
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U)
    {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
        {
            break;
        }

        if ((rx_header.IDE != CAN_ID_STD) ||
            (rx_header.RTR != CAN_RTR_DATA) ||
            (rx_header.DLC != 8U))
        {
            continue;
        }

        if ((hcan->Instance == CAN1) &&
            (rx_header.StdId >= CAN_2006_1_ID) &&
            (rx_header.StdId <= CAN_2006_3_ID))
        {
            motor_index = (uint8_t)(rx_header.StdId - CAN_2006_1_ID);
            decode_2006_feedback(&motor_2006[motor_index], rx_data);
        }

        else if (hcan->Instance == CAN2)
        {
            dm4310_decode_feedback((uint16_t)rx_header.StdId, rx_data);
        }
    }
}

const volatile motor_measure_t *get_2006_motor_measure_point(uint8_t motor_index)
{
    if (motor_index >= M2006_MOTOR_COUNT)
    {
        return (const volatile motor_measure_t *)0;
    }

    return &motor_2006[motor_index];
}
