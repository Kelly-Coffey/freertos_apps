/**
  ******************************************************************************
  * File Name          : app_TIM3_encoder.c
  * Description        : This file provides code for the configuration
  *                      of the STM32 Timer3 with Hall Effect sensor input
  ******************************************************************************
  *
  * COPYRIGHT 2021 Kelly-Coffey
  *
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  ******************************************************************************
  */

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "app_TIM3_encoder.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>

#include "math.h"

#include "FreeRTOS.h"
#include "portable.h"
#include "queue.h"
#include "task.h"
#include "timers.h"

/* Private variables ---------------------------------------------------------*/
static uint8_t verbose = 1;  /* Verbose output to UART terminal ON/OFF. */

struct encoder ENCODER_1;
QueueHandle_t encoderQueueHandle;
float previousPosition = 0;
const maximumPosition = 4096



/* Private function prototypes -----------------------------------------------*/
static void MX_Encoder_Init(void);
static void MX_Encoder_Process(void);

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim3;

void MX_Encoder_Init(void)
{
  /* USER CODE BEGIN SV */

  /* USER CODE END SV */

  /* USER CODE BEGIN MEMS_Init_PreTreatment */

  /* USER CODE END MEMS_Init_PreTreatment */

  /* Initialize the peripherals and the MEMS components */

  MX_TIM3encoder_Init();

  /* USER CODE BEGIN MEMS_Init_PostTreatment */

  /* USER CODE END MEMS_Init_PostTreatment */
}

/*
 * LM background task
 */
void MX_Encoder_Process(void)
{
  /* USER CODE BEGIN MEMS_Process_PreTreatment */

  /* USER CODE END MEMS_Process_PreTreatment */

  MX_TIM3encoder_Process();

  /* USER CODE BEGIN MEMS_Process_PostTreatment */

  /* USER CODE END MEMS_Process_PostTreatment */
}

/**
  * @brief  Initialize the DataLogTerminal application
  * @retval None
  */
void MX_TIM3encoder_Init(void)
{

	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);

}


/**
  * @brief  Process of the DataLogTerminal application
  * @retval None
  */
void MX_TIM3encoder_Process(void)
{
	float currentPosition;
	const float radspertick = maximumPosition/(2*3.14759);

	currentPosition = (float) TIM3->CNT;

	ENCODER_1.position = (previousPosition - currentPosition)*radspertick;

	ENCODER_1.radspsec  = ((previousPosition - currentPosition)*radspertick/0.100) ;
	  if (! xQueueSend(encoderQueueHandle,&ENCODER_1,100)){
		  printf("Failed to write sensor data to QueueHandle\n");
		}
}

#ifdef __cplusplus
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
