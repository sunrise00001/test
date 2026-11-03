#include "gravity.h"
#include "gravity_config.h"
#include "gravity_log.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"

static void gravity_control_task(void *argument)
{
    TickType_t wake;
    (void)argument;
    gravity_init();
    wake = xTaskGetTickCount();
    for (;;) {
        gravity_service_2ms();
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(GRAVITY_CONTROL_PERIOD_MS));
    }
}

static void gravity_logger_task(void *argument)
{
    (void)argument;
    for (;;) {
        gravity_log_service();
        vTaskDelay(pdMS_TO_TICKS(2U));
    }
}

/* AI写的重力补偿任务 */
void motorstart_task(void *argument)
{
    BaseType_t control_ok;
    BaseType_t logger_ok;
    (void)argument;
    vTaskDelay(pdMS_TO_TICKS(3000U));
    control_ok = xTaskCreate(gravity_control_task, "gravity", 768U, NULL,
                             tskIDLE_PRIORITY + 4U, NULL);
    logger_ok = xTaskCreate(gravity_logger_task, "gravity_log", 768U, NULL,
                            tskIDLE_PRIORITY + 1U, NULL);
    if (control_ok != pdPASS || logger_ok != pdPASS) {
        taskDISABLE_INTERRUPTS();
        for (;;) { }
    }
    vTaskDelete(NULL);
}
