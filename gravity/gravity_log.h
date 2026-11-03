#ifndef GRAVITY_LOG_H
#define GRAVITY_LOG_H

#include <stdint.h>

typedef struct {
    uint32_t seq;
    uint16_t pair_id;
    char pass;
    uint32_t tick_ms;
    int32_t q1_mrad;
    int32_t q2_mrad;
    int32_t t1_mnm;
    int32_t t2_mnm;
} gravity_log_record_t;

void gravity_log_init(void);
void gravity_log_push(const gravity_log_record_t *record);
void gravity_log_service(void);
extern volatile uint32_t gravity_log_drop_count;
extern volatile uint32_t gravity_log_tx_error_count;

#endif
