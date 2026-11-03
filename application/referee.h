#ifndef _REFEREE_H_
#define _REFEREE_H_
#include <stdint.h>
#include <string.h>
#pragma pack(1)
typedef struct
{
    uint8_t SOF;                        //SOF 0xA5
    uint16_t Data_Length;               //2
    uint8_t Seq;                        //1
    uint8_t CRC8;                   //CRC8 1
}Data_Head_t;
#pragma pack()

#pragma pack(1)
typedef struct
{
    Data_Head_t Data_Head;//5
    uint16_t cmd_id;      //2
    union{
        uint8_t data[30];
        struct
        {
            float angle[6];
            uint8_t button_arm_up:1;
            uint8_t button_arm_down:1;
            uint8_t button_pump_open_trig:1;
            uint8_t button_pump_close_trig:1;
            uint8_t button_resv_1_up:1;
            uint8_t button_resv_1_down:1;
            uint8_t button_resv_2_up:1;
            uint8_t button_resv_2_down:1;
            uint8_t resv[3];
        }mt_ang;
    }message;             //30
    uint16_t CRC_16;      //2
}self_controller_data_t;

typedef union
{
    uint8_t data[sizeof(self_controller_data_t)];
    self_controller_data_t self_data;
}self_controller_data_u;
#pragma pack()

typedef char referee_frame_must_be_39_bytes[(sizeof(self_controller_data_t) == 39U) ? 1 : -1];

extern self_controller_data_u self_controller_data;

void init_controller_data(void);
uint16_t Get_CRC16_Check_Sum_UI(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
uint8_t Get_CRC8_Check_Sum_UI(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8);

void fresh_mt(float angle[6], uint8_t button_value);

#endif
