/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "CAN_receive.h"
#include "dm4310.h"
#include "referee.h"
#include "usart.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ANGLE_SEND_PERIOD_MS 20U
#define RAD_TO_DEG 57.29577951308232f
/* Set these to the actual M2006 rotor-to-joint ratios for joint angles. */
#define M2006_RATIO_1 1.0f
#define M2006_RATIO_2 1.0f
#define M2006_RATIO_3 1.0f
float angles[6];

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId_t defaultTaskHandle;
osThreadId_t motor_taskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static uint8_t angle_get_feedback(float angles[6]);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument); // for v2
extern void motorstart_task(void *argument); // for v2

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
osKernelInitialize(); // Initialize CMSIS-RTOS

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .priority = (osPriority_t) osPriorityNormal,
    .stack_size = 1024
  };
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* definition and creation of motor_task */
  const osThreadAttr_t motor_task_attributes = {
    .name = "motor_task",
    .priority = (osPriority_t) osPriorityNormal3,
    .stack_size = 1024
  };
  motor_taskHandle = osThreadNew(motorstart_task, NULL, &motor_task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  uint32_t next_wake;
  (void)argument;

  init_controller_data();
  next_wake = osKernelGetTickCount();
  for(;;)
  {
    if (angle_get_feedback(angles) != 0U)
    {
      fresh_mt(angles, 0U);
      (void)HAL_UART_Transmit(&huart6, self_controller_data.data,
                              sizeof(self_controller_data_t), 100U); 
    }
    next_wake += ANGLE_SEND_PERIOD_MS;
    (void)osDelayUntil(next_wake);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void vApplicationStackOverflowHook(TaskHandle_t task, char *task_name)
{
  (void)task;
  (void)task_name;
  taskDISABLE_INTERRUPTS();
  for (;;) { }
}


/* Frame order: M2006 IDs 0x201..0x203, then DM4310 IDs 1..3. */
static uint8_t angle_get_feedback(float angles[6])
{
  static const float ratio[3] = {M2006_RATIO_1, M2006_RATIO_2, M2006_RATIO_3};
  uint8_t i;
  uint32_t primask;
  const volatile dm4310_feedback_t *feedback;

  for (i = 0U; i < M2006_MOTOR_COUNT; ++i)
  {
    if (get_2006_motor_relative_angle_deg(i, ratio[i], &angles[i]) == 0U)
    {
      return 0U;
    }
  }

  for (i = 0U; i < DM4310_MOTOR_COUNT; ++i)
  {
    feedback = get_dm4310_feedback_point((uint8_t)(DM4310_MOTOR1_ID + i));
    primask = __get_PRIMASK();
    __disable_irq();
    if (feedback->rx_count == 0U)
    {
      __set_PRIMASK(primask);
      return 0U;
    }
    angles[M2006_MOTOR_COUNT + i] = feedback->position_rad;
    __set_PRIMASK(primask);
    angles[M2006_MOTOR_COUNT + i] *= RAD_TO_DEG;
  }
   
  return 1U;
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
