/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "crc.h"
#include "Application/test/test.h"
#include "Application/SystemClock/SystemClock.h"
#include "Application/Sensor/INS/ins.h"
#include "Application/Filter/QAHRS/qeskf.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId Controller01Handle;
osThreadId Controller02Handle;
osThreadId Sensor01Handle;
osThreadId Sensor02Handle;
osThreadId OnlineCheckHandle;
osThreadId SerialHandle;
osThreadId SystemClockTaskHandle;
osMessageQId myQueue01Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartController01(void const * argument);
void StartController02(void const * argument);
void StartSensor01(void const * argument);
void StartSensor02(void const * argument);
void StartOnlineCheck(void const * argument);
void StartSerial(void const * argument);
void StartSystemClockTask(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of myQueue01 */
  osMessageQDef(myQueue01, 16, uint32_t);
  myQueue01Handle = osMessageCreate(osMessageQ(myQueue01), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Controller01 */
  osThreadDef(Controller01, StartController01, osPriorityIdle, 0, 512);
  Controller01Handle = osThreadCreate(osThread(Controller01), NULL);

  /* definition and creation of Controller02 */
  osThreadDef(Controller02, StartController02, osPriorityIdle, 0, 512);
  Controller02Handle = osThreadCreate(osThread(Controller02), NULL);

  /* definition and creation of Sensor01 */
  osThreadDef(Sensor01, StartSensor01, osPriorityIdle, 0, 512);
  Sensor01Handle = osThreadCreate(osThread(Sensor01), NULL);

  /* definition and creation of Sensor02 */
  osThreadDef(Sensor02, StartSensor02, osPriorityIdle, 0, 512);
  Sensor02Handle = osThreadCreate(osThread(Sensor02), NULL);

  /* definition and creation of OnlineCheck */
  osThreadDef(OnlineCheck, StartOnlineCheck, osPriorityIdle, 0, 256);
  OnlineCheckHandle = osThreadCreate(osThread(OnlineCheck), NULL);

  /* definition and creation of Serial */
  osThreadDef(Serial, StartSerial, osPriorityIdle, 0, 512);
  SerialHandle = osThreadCreate(osThread(Serial), NULL);

  /* definition and creation of SystemClockTask */
  osThreadDef(SystemClockTask, StartSystemClockTask, osPriorityIdle, 0, 128);
  SystemClockTaskHandle = osThreadCreate(osThread(SystemClockTask), NULL);

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
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {

    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartController01 */
/**
* @brief Function implementing the Controller01 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartController01 */
void StartController01(void const * argument)
{
  /* USER CODE BEGIN StartController01 */
  QESKF_Init();
  Ins_Init(0);
  /* Infinite loop */
  for(;;)
  {
    Ins_Task();
    osDelay(1);
  }
  /* USER CODE END StartController01 */
}

/* USER CODE BEGIN Header_StartController02 */
/**
* @brief Function implementing the Controller02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartController02 */
void StartController02(void const * argument)
{
  /* USER CODE BEGIN StartController02 */
  UNUSED(argument);
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartController02 */
}

/* USER CODE BEGIN Header_StartSensor01 */
/**
* @brief Function implementing the Sensor01 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensor01 */
void StartSensor01(void const * argument)
{
  /* USER CODE BEGIN StartSensor01 */
  UNUSED(argument);
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartSensor01 */
}

/* USER CODE BEGIN Header_StartSensor02 */
/**
* @brief Function implementing the Sensor02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensor02 */
void StartSensor02(void const * argument)
{
  /* USER CODE BEGIN StartSensor02 */
  UNUSED(argument);
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartSensor02 */
}

/* USER CODE BEGIN Header_StartOnlineCheck */
/**
* @brief Function implementing the OnlineCheck thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartOnlineCheck */
void StartOnlineCheck(void const * argument)
{
  /* USER CODE BEGIN StartOnlineCheck */
  UNUSED(argument);
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartOnlineCheck */
}

/* USER CODE BEGIN Header_StartSerial */
/**
* @brief Function implementing the Serial thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSerial */
void StartSerial(void const * argument)
{
  /* USER CODE BEGIN StartSerial */
  UNUSED(argument);
        Test_Init();
        #define BUFFER_SIZE    114
        static const uint32_t dataBuffer[BUFFER_SIZE] =
        {
                0x00001021, 0x20423063, 0x408450a5, 0x60c670e7, 0x9129a14a, 0xb16bc18c,
                0xd1ade1ce, 0xf1ef1231, 0x32732252, 0x52b54294, 0x72f762d6, 0x93398318,
                0xa35ad3bd, 0xc39cf3ff, 0xe3de2462, 0x34430420, 0x64e674c7, 0x44a45485,
                0xa56ab54b, 0x85289509, 0xf5cfc5ac, 0xd58d3653, 0x26721611, 0x063076d7,
                0x569546b4, 0xb75ba77a, 0x97198738, 0xf7dfe7fe, 0xc7bc48c4, 0x58e56886,
                0x78a70840, 0x18612802, 0xc9ccd9ed, 0xe98ef9af, 0x89489969, 0xa90ab92b,
                0x4ad47ab7, 0x6a961a71, 0x0a503a33, 0x2a12dbfd, 0xfbbfeb9e, 0x9b798b58,
                0xbb3bab1a, 0x6ca67c87, 0x5cc52c22, 0x3c030c60, 0x1c41edae, 0xfd8fcdec,
                0xad2abd0b, 0x8d689d49, 0x7e976eb6, 0x5ed54ef4, 0x2e321e51, 0x0e70ff9f,
                0xefbedfdd, 0xcffcbf1b, 0x9f598f78, 0x918881a9, 0xb1caa1eb, 0xd10cc12d,
                0xe16f1080, 0x00a130c2, 0x20e35004, 0x40257046, 0x83b99398, 0xa3fbb3da,
                0xc33dd31c, 0xe37ff35e, 0x129022f3, 0x32d24235, 0x52146277, 0x7256b5ea,
                0x95a88589, 0xf56ee54f, 0xd52cc50d, 0x34e224c3, 0x04817466, 0x64475424,
                0x4405a7db, 0xb7fa8799, 0xe75ff77e, 0xc71dd73c, 0x26d336f2, 0x069116b0,
                0x76764615, 0x5634d94c, 0xc96df90e, 0xe92f99c8, 0xb98aa9ab, 0x58444865,
                0x78066827, 0x18c008e1, 0x28a3cb7d, 0xdb5ceb3f, 0xfb1e8bf9, 0x9bd8abbb,
                0x4a755a54, 0x6a377a16, 0x0af11ad0, 0x2ab33a92, 0xed0fdd6c, 0xcd4dbdaa,
                0xad8b9de8, 0x8dc97c26, 0x5c644c45, 0x3ca22c83, 0x1ce00cc1, 0xef1fff3e,
                0xdf7caf9b, 0xbfba8fd9, 0x9ff86e17, 0x7e364e55, 0x2e933eb2, 0x0ed11ef0
        };

        uint32_t uwExpectedCRCValue = 0x379E9F06;
  /* Infinite loop */
  for(;;)
  {
      Test_Task();
          __IO uint32_t CRCValue = 0;
          static uint8_t fuck_right = 0;
          CRCValue = HAL_CRC_Calculate(&hcrc, (uint32_t *)dataBuffer, BUFFER_SIZE);

          if (CRCValue == uwExpectedCRCValue) {
                  fuck_right = 1;
          }
    osDelay(1);
  }
  /* USER CODE END StartSerial */
}

/* USER CODE BEGIN Header_StartSystemClockTask */
/**
* @brief Function implementing the SystemClockTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSystemClockTask */
void StartSystemClockTask(void const * argument)
{
  /* USER CODE BEGIN StartSystemClockTask */
  UNUSED(argument);
  SystemClock_Init();
  /* Infinite loop */
  for(;;)
  {
      SystemClock_Task();
      osDelay(5);
  }
  /* USER CODE END StartSystemClockTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
