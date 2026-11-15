#include "CAN_receive.h"
#include "can.h"
#include "dm4310.h"

/* CAN 接收中断写入，主循环或控制任务读取。 */
volatile motor_measure_t motor_2006[M2006_MOTOR_COUNT];
uint8_t motor_index;
float angle[3] = {0.0f, 0.0f, 0.0f};

/* 按大端格式拆解 M2006 的 8 字节反馈帧。 */
static void decode_2006_feedback(volatile motor_measure_t *motor, const uint8_t data[8])
{
    uint16_t new_ecd = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);

    if (motor->ecd_initialized == 0U)
    {
        /* 首帧只确定本次上电的相对零点，不与静态初值 0 作差。 */
        motor->last_ecd = new_ecd;
        motor->ecd = new_ecd;
        motor->total_ecd = 0;
        motor->ecd_initialized = 1U;
    }
    else
    {
        int32_t delta = (int32_t)new_ecd - (int32_t)motor->ecd;

        /* 相邻反馈的转子位移须小于半圈，才能确定跨零方向。 */
        if (delta > 4096)
        {
            delta -= 8192;
        }
        else if (delta < -4096)
        {
            delta += 8192;
        }

        motor->last_ecd = motor->ecd;
        motor->ecd = new_ecd;
        motor->total_ecd += delta;
    }
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
            get_2006_motor_relative_angle_deg(motor_index, 1.0f, &angle[motor_index]);
        }

        else if (hcan->Instance == CAN2)
        {
            dm4310_decode_feedback((uint16_t)rx_header.StdId, rx_data);
        }
    }
}

const volatile motor_measure_t *get_2006_motor_measure_point(uint8_t motor_index)//2006电机数据接口，但因为64位数据类型的total_ecd，需要额外处理，以地址方式把角度传出
{
    if (motor_index >= M2006_MOTOR_COUNT)
    {
        return (const volatile motor_measure_t *)0;
    }

    return &motor_2006[motor_index];
}

uint8_t get_2006_motor_relative_count(uint8_t motor_index, int64_t *count)//防止中断data被修改，使用关中断的方式读取数据
{
    uint32_t primask;

    if ((motor_index >= M2006_MOTOR_COUNT) || (count == NULL))
    {
        return 0U;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (motor_2006[motor_index].ecd_initialized == 0U)
    {
        __set_PRIMASK(primask);
        return 0U;
    }
    *count = motor_2006[motor_index].total_ecd;
    __set_PRIMASK(primask);
    return 1U;
}

uint8_t get_2006_motor_relative_angle_deg(uint8_t motor_index,
                                           float reduction_ratio,//传动比
                                           float *angle_deg)//编码器转换角度
{
    int64_t count;//关闭中断得到一次编码器值做计算，防止被中断修改

    if ((angle_deg == NULL) || !(reduction_ratio > 0.0f) ||
        (get_2006_motor_relative_count(motor_index, &count) == 0U))
    {
        return 0U;
    }

    *angle_deg = (float)count * (360.0f / (8192.0f * reduction_ratio));
    return 1U;
}
