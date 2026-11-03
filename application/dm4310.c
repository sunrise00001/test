#include "dm4310.h"

volatile dm4310_feedback_t dm4310_feedback[DM4310_MOTOR_COUNT];

#define DM4310_KP_MIN 0.0f
#define DM4310_KP_MAX 500.0f
#define DM4310_KD_MIN 0.0f
#define DM4310_KD_MAX 5.0f

static const uint16_t dm4310_master_id[DM4310_MOTOR_COUNT] =
{
    DM4310_MOTOR1_MASTER_ID,
    DM4310_MOTOR2_MASTER_ID,
    DM4310_MOTOR3_MASTER_ID
};  

static float dm4310_uint_to_float(uint16_t value, float minimum, float maximum, uint8_t bits)
{
    uint32_t integer_max = (1UL << bits) - 1UL;

    return ((float)value * (maximum - minimum) / (float)integer_max) + minimum;
}

static float dm4310_clamp(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

static uint16_t dm4310_float_to_uint(float value, float minimum, float maximum, uint8_t bits)
{
    uint32_t integer_max = (1UL << bits) - 1UL;

    value = dm4310_clamp(value, minimum, maximum);
    return (uint16_t)((value - minimum) * (float)integer_max / (maximum - minimum));
}

static void dm4310_pack_float_little_endian(float value, uint8_t data[4])//小端排序
{
    union
    {
        float value;
        uint8_t byte[4];
    } converter;

    converter.value = value;
    data[0] = converter.byte[0];
    data[1] = converter.byte[1];
    data[2] = converter.byte[2];
    data[3] = converter.byte[3];
}

static uint8_t dm4310_motor_id_valid(uint8_t motor_id)//id限制
{
    return (uint8_t)((motor_id >= DM4310_MOTOR1_ID) &&
                     (motor_id <= DM4310_MOTOR3_ID));
}

static HAL_StatusTypeDef dm4310_send(CAN_HandleTypeDef *hcan,
                                     uint16_t std_id,
                                     uint8_t dlc,
                                     const uint8_t data[8])
{
    CAN_TxHeaderTypeDef tx_header;
    uint32_t mailbox;

    tx_header.StdId = std_id;
    tx_header.ExtId = 0U;
    tx_header.IDE = CAN_ID_STD;
    tx_header.RTR = CAN_RTR_DATA;
    tx_header.DLC = dlc;
    tx_header.TransmitGlobalTime = DISABLE;

    return HAL_CAN_AddTxMessage(hcan, &tx_header, (uint8_t *)data, &mailbox);
}

static HAL_StatusTypeDef dm4310_special_command(CAN_HandleTypeDef *hcan,
                                                 uint8_t motor_id,
                                                 dm4310_control_mode_e mode,
                                                 uint8_t command)//构建多模式命令
{
    uint8_t data[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU,
                       0xFFU, 0xFFU, 0xFFU, 0xFFU};

    if (!dm4310_motor_id_valid(motor_id))
    {
        return HAL_ERROR;
    }

    data[7] = command;
    return dm4310_send(hcan, (uint16_t)((uint16_t)mode + motor_id), 8U, data);
}

void dm4310_decode_feedback(uint16_t std_id, const uint8_t data[8])
{
    uint8_t motor_id = data[0] & 0x0FU;
    uint8_t motor_index;
    volatile dm4310_feedback_t *feedback;

    if ((motor_id < DM4310_MOTOR1_ID) || (motor_id > DM4310_MOTOR3_ID))
    {
        return;
    }

    motor_index = (uint8_t)(motor_id - DM4310_MOTOR1_ID);

    /* Master ID 是反馈报文的标准帧 ID，不等同于电机 CAN ID。 */
    if (std_id != dm4310_master_id[motor_index])
    {
        return;
    }

    feedback = &dm4310_feedback[motor_index];
    feedback->motor_id = motor_id;
    feedback->state = data[0] >> 4;
    feedback->position_raw = (uint16_t)(((uint16_t)data[1] << 8) | data[2]);
    feedback->velocity_raw = (uint16_t)(((uint16_t)data[3] << 4) | (data[4] >> 4));
    feedback->torque_raw = (uint16_t)((((uint16_t)data[4] & 0x0FU) << 8) | data[5]);
    feedback->position_rad = dm4310_uint_to_float(feedback->position_raw,
                                                  DM4310_P_MIN,
                                                  DM4310_P_MAX,
                                                  16U);
    feedback->velocity_rad_s = dm4310_uint_to_float(feedback->velocity_raw,
                                                    DM4310_V_MIN,
                                                    DM4310_V_MAX,
                                                    12U);
    feedback->torque_nm = dm4310_uint_to_float(feedback->torque_raw,
                                               DM4310_T_MIN,
                                               DM4310_T_MAX,
                                               12U);
    feedback->mos_temperature = data[6];
    feedback->rotor_temperature = data[7];
    feedback->rx_count++;
}

const volatile dm4310_feedback_t *get_dm4310_feedback_point(uint8_t motor_id)
{
    if ((motor_id < DM4310_MOTOR1_ID) || (motor_id > DM4310_MOTOR3_ID))
    {
        return (const volatile dm4310_feedback_t *)0;
    }

    return &dm4310_feedback[motor_id - DM4310_MOTOR1_ID];
}

HAL_StatusTypeDef dm4310_mit_control(CAN_HandleTypeDef *hcan,
                                     uint8_t motor_id,
                                     float position_rad,
                                     float velocity_rad_s,
                                     float kp,
                                     float kd,
                                     float torque_nm)
{
    uint16_t position;
    uint16_t velocity;
    uint16_t kp_value;
    uint16_t kd_value;
    uint16_t torque;
    uint8_t data[8];

    if (!dm4310_motor_id_valid(motor_id))
    {
        return HAL_ERROR;
    }

    position = dm4310_float_to_uint(position_rad, DM4310_P_MIN, DM4310_P_MAX, 16U);
    velocity = dm4310_float_to_uint(velocity_rad_s, DM4310_V_MIN, DM4310_V_MAX, 12U);
    kp_value = dm4310_float_to_uint(kp, DM4310_KP_MIN, DM4310_KP_MAX, 12U);
    kd_value = dm4310_float_to_uint(kd, DM4310_KD_MIN, DM4310_KD_MAX, 12U);
    torque = dm4310_float_to_uint(torque_nm, DM4310_T_MIN, DM4310_T_MAX, 12U);

    data[0] = (uint8_t)(position >> 8);
    data[1] = (uint8_t)position;
    data[2] = (uint8_t)(velocity >> 4);
    data[3] = (uint8_t)((velocity << 4) | (kp_value >> 8));
    data[4] = (uint8_t)kp_value;
    data[5] = (uint8_t)(kd_value >> 4);
    data[6] = (uint8_t)((kd_value << 4) | (torque >> 8));
    data[7] = (uint8_t)torque;

    return dm4310_send(hcan, motor_id, 8U, data);
}

HAL_StatusTypeDef dm4310_position_speed_control(CAN_HandleTypeDef *hcan,
                                                uint8_t motor_id,
                                                float position_rad,
                                                float velocity_rad_s)
{
    uint8_t data[8];

    if (!dm4310_motor_id_valid(motor_id))
    {
        return HAL_ERROR;
    }

    dm4310_pack_float_little_endian(position_rad, &data[0]);
    dm4310_pack_float_little_endian(velocity_rad_s, &data[4]);
    return dm4310_send(hcan, (uint16_t)(0x100U + motor_id), 8U, data);
}

HAL_StatusTypeDef dm4310_speed_control(CAN_HandleTypeDef *hcan,
                                       uint8_t motor_id,
                                       float velocity_rad_s)
{
    uint8_t data[8] = {0U};

    if (!dm4310_motor_id_valid(motor_id))
    {
        return HAL_ERROR;
    }

    dm4310_pack_float_little_endian(velocity_rad_s, data);
    return dm4310_send(hcan, (uint16_t)(0x200U + motor_id), 4U, data);
}

HAL_StatusTypeDef dm4310_position_speed_torque_control(CAN_HandleTypeDef *hcan,
                                                       uint8_t motor_id,
                                                       float position_rad,
                                                       float velocity_limit_rad_s,
                                                       float current_limit_ratio)
{
    uint16_t velocity_limit;
    uint16_t current_limit;
    uint8_t data[8];

    if (!dm4310_motor_id_valid(motor_id))
    {
        return HAL_ERROR;
    }

    velocity_limit_rad_s = dm4310_clamp(velocity_limit_rad_s, 0.0f, 100.0f);
    current_limit_ratio = dm4310_clamp(current_limit_ratio, 0.0f, 1.0f);
    velocity_limit = (uint16_t)(velocity_limit_rad_s * 100.0f);
    current_limit = (uint16_t)(current_limit_ratio * 10000.0f);

    dm4310_pack_float_little_endian(position_rad, &data[0]);
    data[4] = (uint8_t)velocity_limit;
    data[5] = (uint8_t)(velocity_limit >> 8);
    data[6] = (uint8_t)current_limit;
    data[7] = (uint8_t)(current_limit >> 8);

    return dm4310_send(hcan, (uint16_t)(0x300U + motor_id), 8U, data);
}

HAL_StatusTypeDef dm4310_enable(CAN_HandleTypeDef *hcan,
                                uint8_t motor_id,
                                dm4310_control_mode_e mode)
{
    return dm4310_special_command(hcan, motor_id, mode, 0xFCU);
}

HAL_StatusTypeDef dm4310_disable(CAN_HandleTypeDef *hcan,
                                 uint8_t motor_id,
                                 dm4310_control_mode_e mode)
{
    return dm4310_special_command(hcan, motor_id, mode, 0xFDU);
}

HAL_StatusTypeDef dm4310_save_zero(CAN_HandleTypeDef *hcan,
                                   uint8_t motor_id,
                                   dm4310_control_mode_e mode)
{
    return dm4310_special_command(hcan, motor_id, mode, 0xFEU);
}

HAL_StatusTypeDef dm4310_clear_error(CAN_HandleTypeDef *hcan,
                                     uint8_t motor_id,
                                     dm4310_control_mode_e mode)
{
    return dm4310_special_command(hcan, motor_id, mode, 0xFBU);
}

