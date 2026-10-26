#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

/* 机械臂使用的 M2006 电机数量。 */
#define M2006_MOTOR_COUNT 3U

/* 三个 M2006 电调的反馈 CAN ID。 */
typedef enum
{
    CAN_2006_1_ID = 0x201,
    CAN_2006_2_ID = 0x202,
    CAN_2006_3_ID = 0x203
} can_msg_id_e;

/*
 * M2006 反馈帧格式（DLC = 8）：
 * data[0..1]：转子机械角度，范围 0~8191
 * data[2..3]：转速，单位 rpm，有符号数
 * data[4..5]：实际转矩电流，有符号数
 * data[6]   ：电机温度，单位摄氏度
 * data[7]   ：保留
 */
typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperature;
    uint16_t last_ecd;
    /* 从本次上电收到的首帧起累计的转子编码器计数，首帧位置为零。 */
    int64_t total_ecd;
    uint8_t ecd_initialized;// 0: 未初始化，1: 已初始化
} motor_measure_t;

/*
 * 获取指定 M2006 电机的最新反馈值。
 * motor_index 的有效范围为 0~2，分别对应 CAN ID 0x201~0x203；
 * 参数越界时返回空指针。该数据由 CAN 接收中断更新。
 */
extern const volatile motor_measure_t *get_2006_motor_measure_point(uint8_t motor_index);

extern uint8_t get_2006_motor_relative_count(uint8_t motor_index, int64_t *count);

/* 根据实际传动比换算输出轴相对角度（度）；未初始化或参数无效时返回 0。 */
extern uint8_t get_2006_motor_relative_angle_deg(uint8_t motor_index,
                                                  float reduction_ratio,
                                                  float *angle_deg);

#endif
