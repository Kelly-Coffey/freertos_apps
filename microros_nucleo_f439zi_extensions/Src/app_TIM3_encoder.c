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
#include "cmsis_os.h"
#include "semphr.h"

/* Private variables ---------------------------------------------------------*/
static uint8_t verbose = 1;  /* Verbose output to UART terminal ON/OFF. */

struct encoder ENCODER_1;
QueueHandle_t encoderQueueHandle;
float previousPosition = 0;

//https://sdrobots.com/tech-thursday-029-encoder-cpr-resisted/
const int MotorCPR = 6707;
extern TIM_HandleTypeDef htim3;


/* Private function prototypes -----------------------------------------------*/
static void MX_TIM3encoder_Init(void);
static void MX_TIM3encoder_Process(void);

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim3;
extern osMutexId_t Tim3MutexHandle;

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
	  if (HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL) != HAL_OK)
	  {
	    Error_Handler();
	  }
}


/**
  * @brief  Process of the DataLogTerminal application
  * @retval None
  */
void MX_TIM3encoder_Process(void)
{
	float currentPosition;
	const float radspertick = (2*3.14759)/MotorCPR; // Update CubeMX TIM Counter Period setting
	int forward, backward;

	//
	xSemaphoreTake(Tim3MutexHandle, portMAX_DELAY);
	currentPosition = (float) htim3.Instance->CNT;
	xSemaphoreGive(Tim3MutexHandle);

	ENCODER_1.position = (currentPosition)*radspertick;


	// MOTOR DIRECTIONS DIRECT FROM HARDWARE PINS
	forward = HAL_GPIO_ReadPin(GPIOE,HB0_P_EN_Pin);
	backward = HAL_GPIO_ReadPin(GPIOE,HB0_N_EN_Pin);


	if (forward == 1 && backward == 0){
		//FORWARD
		if (currentPosition == previousPosition)
			ENCODER_1.radspsec = 0;
		else {
			if (currentPosition > previousPosition)
				ENCODER_1.radspsec  = (currentPosition-previousPosition)*radspertick;
			else
				ENCODER_1.radspsec  = (currentPosition + MotorCPR - previousPosition)*radspertick;
			}
	}

	if (forward == 0 && backward == 1){
		//BACKWARD
		if (currentPosition == previousPosition)
			ENCODER_1.radspsec = 0;
		else {
			if (currentPosition < previousPosition)
				ENCODER_1.radspsec  = (currentPosition - previousPosition)*radspertick;
			else
				ENCODER_1.radspsec  = (currentPosition - MotorCPR - previousPosition)*radspertick;
			}
	}

	if ( (forward == 0 && backward == 0) || (forward == 1 && backward == 1) ){
		//BACKWARD
		if (currentPosition == previousPosition)
			ENCODER_1.radspsec = 0;
		else {
			ENCODER_1.radspsec  = (currentPosition - previousPosition)*radspertick;
			}
	}

	previousPosition = currentPosition;
	  if (! xQueueSend(encoderQueueHandle,&ENCODER_1,100)){
		  printf("Failed to write sensor data to QueueHandle\n");
		}
}


#ifdef __cplusplus
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
