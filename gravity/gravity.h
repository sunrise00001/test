#ifndef GRAVITY_H
#define GRAVITY_H

#include <stdint.h>

typedef enum {
    GRAVITY_MODE_SAFE = 0,
    GRAVITY_MODE_NO_FORCE = 1,
    GRAVITY_MODE_SAMPLING = 2,
    GRAVITY_MODE_COMPENSATION = 3
} gravity_mode_t;

typedef enum {
    GRAVITY_SAMPLE_IDLE = 0,
    GRAVITY_SAMPLE_MOVE = 1,
    GRAVITY_SAMPLE_STABLE = 2,
    GRAVITY_SAMPLE_ACCUMULATE = 3,
    GRAVITY_SAMPLE_DONE = 4,
    GRAVITY_SAMPLE_FAULT = 5
} gravity_sample_state_t;

typedef struct {
    volatile uint32_t loop_count;
    volatile uint32_t sample_count;
    volatile uint32_t can_send_fail_count;
    volatile uint32_t fault_count;
    volatile uint8_t mode;
    volatile uint8_t sample_state;
    volatile uint8_t left_switch;
    volatile uint8_t feedback_valid;
    volatile uint16_t pair_id;
    volatile uint8_t pass_reverse;
    volatile float q_rad[2];
    volatile float velocity_rad_s[2];
    volatile float feedback_torque_nm[2];
    volatile float target_rad[2];
    volatile float command_torque_nm[2];
    volatile float ramp;
} gravity_debug_t;

extern volatile gravity_debug_t gravity_debug;
extern volatile float gravity_output_scale[2];

void gravity_init(void);
void gravity_service_2ms(void);

#endif
