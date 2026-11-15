#ifndef DM4310_H
#define DM4310_H

#include "struct_typedef.h"
#include "stm32f4xx_hal.h"

#define DM4310_MOTOR_COUNT 3U

/* 电机的 CAN ID，必须与达妙调试助手中的 CAN ID 一致。 */
#define DM4310_MOTOR1_ID 1U
#define DM4310_MOTOR2_ID 2U
#define DM4310_MOTOR3_ID 3U

/*
 * 反馈帧标准 ID（调试助手中的 Master ID）。官方默认值为 0。
 */
#define DM4310_MOTOR1_MASTER_ID 0x11U
#define DM4310_MOTOR2_MASTER_ID 0x000U
#define DM4310_MOTOR3_MASTER_ID 0x13U

/* 必须与电机内部配置的 PMAX、VMAX、TMAX 一致。 */
#define DM4310_P_MIN (-12.5f)
#define DM4310_P_MAX (12.5f)
#define DM4310_V_MIN (-30.0f)
#define DM4310_V_MAX (30.0f)
#define DM4310_T_MIN (-10.0f)
#define DM4310_T_MAX (10.0f)

typedef struct
{
    uint8_t motor_id;
    uint8_t state;
    uint16_t position_raw;
    uint16_t velocity_raw;
    uint16_t torque_raw;
    float position_rad;
    float velocity_rad_s;
    float torque_nm;
    uint8_t mos_temperature;
    uint8_t rotor_temperature;
    uint32_t rx_count;
} dm4310_feedback_t;

/* 不同控制模式使用不同的 CAN 帧 ID 偏移。 */
typedef enum
{
    DM4310_MODE_MIT = 0x000U,
    DM4310_MODE_POSITION_SPEED = 0x100U,
    DM4310_MODE_SPEED = 0x200U,
    DM4310_MODE_POSITION_SPEED_TORQUE = 0x300U
} dm4310_control_mode_e;

//调试接收电机反馈结构体
extern volatile dm4310_feedback_t dm4310_feedback[DM4310_MOTOR_COUNT];


extern void dm4310_decode_feedback(uint16_t std_id, const uint8_t data[8]);

/* 按电机 ID（1~3）获取反馈，ID 越界时返回空指针。 */
extern const volatile dm4310_feedback_t *get_dm4310_feedback_point(uint8_t motor_id);

extern HAL_StatusTypeDef dm4310_mit_control(CAN_HandleTypeDef *hcan,
                                            uint8_t motor_id,
                                            float position_rad,
                                            float velocity_rad_s,
                                            float kp,
                                            float kd,
                                            float torque_nm);

extern HAL_StatusTypeDef dm4310_position_speed_control(CAN_HandleTypeDef *hcan,
                                                       uint8_t motor_id,
                                                       float position_rad,
                                                       float velocity_rad_s);

extern HAL_StatusTypeDef dm4310_speed_control(CAN_HandleTypeDef *hcan,
                                              uint8_t motor_id,
                                              float velocity_rad_s);

extern HAL_StatusTypeDef dm4310_position_speed_torque_control(CAN_HandleTypeDef *hcan,
                                                              uint8_t motor_id,
                                                              float position_rad,
                                                              float velocity_limit_rad_s,
                                                              float current_limit_ratio);

extern HAL_StatusTypeDef dm4310_enable(CAN_HandleTypeDef *hcan,
                                       uint8_t motor_id,
                                       dm4310_control_mode_e mode);
extern HAL_StatusTypeDef dm4310_disable(CAN_HandleTypeDef *hcan,
                                        uint8_t motor_id,
                                        dm4310_control_mode_e mode);
extern HAL_StatusTypeDef dm4310_save_zero(CAN_HandleTypeDef *hcan,
                                          uint8_t motor_id,
                                          dm4310_control_mode_e mode);
extern HAL_StatusTypeDef dm4310_clear_error(CAN_HandleTypeDef *hcan,
                                            uint8_t motor_id,
                                            dm4310_control_mode_e mode);

#endif
