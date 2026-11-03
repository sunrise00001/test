#include "gravity_log.h"
#include "usart.h"
#include <stdio.h>

/* USART1_TX PA9, 115200 8N1. Only the low-priority logger task transmits. */
#define LOG_QUEUE_SIZE 16U
static gravity_log_record_t queue[LOG_QUEUE_SIZE];
static volatile uint8_t head;
static volatile uint8_t tail;
volatile uint32_t gravity_log_drop_count;
volatile uint32_t gravity_log_tx_error_count;

void gravity_log_init(void)
{
    /* MX_USART1_UART_Init() runs before the RTOS starts. */
}

void gravity_log_push(const gravity_log_record_t *record)
{
    uint8_t next = (uint8_t)((head + 1U) % LOG_QUEUE_SIZE);
    if (next == tail) {
        gravity_log_drop_count++;
        return;
    }
    queue[head] = *record;
    head = next;
}

void gravity_log_service(void)
{
    gravity_log_record_t record;
    char line[128];
    int length;
    if (tail == head) return;
    record = queue[tail];
    tail = (uint8_t)((tail + 1U) % LOG_QUEUE_SIZE);
    length = snprintf(line, sizeof(line),
                      "GRAV2,v1,%lu,%u,%c,%lu,%ld,%ld,%ld,%ld\r\n",
                      (unsigned long)record.seq,
                      (unsigned int)record.pair_id, record.pass,
                      (unsigned long)record.tick_ms,
                      (long)record.q1_mrad, (long)record.q2_mrad,
                      (long)record.t1_mnm, (long)record.t2_mnm);
    if (length <= 0 || length >= (int)sizeof(line)) return;
    if (HAL_UART_Transmit(&huart1, (uint8_t *)line, (uint16_t)length, 50U) != HAL_OK)
        gravity_log_tx_error_count++;
}
