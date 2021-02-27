/**
  ******************************************************************************
  * File Name          : app_btn8962ta_motor.c
  * Description        : This file provides code for the configuration
  *                      of the STM32 Timer1 and BTN8962TA Half-Bridge or Full-Bridge
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
#include "app_btn8962ta_motor.h"
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

struct motorctrl MOTORCTRL_1;
QueueHandle_t motorctrlQueueHandle;


/* Private function prototypes -----------------------------------------------*/
static void MX_BTN8962TA_Init(void);
static void MX_BTN8962TA_Process(void);
float shortestSignedDistanceBetweenCircularValues(float origin, float target);

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim3;
extern osMutexId_t Tim3MutexHandle;
void MX_MOTOR_Init(void)
{
  /* USER CODE BEGIN SV */

  /* USER CODE END SV */

  /* USER CODE BEGIN MEMS_Init_PreTreatment */

  /* USER CODE END MEMS_Init_PreTreatment */

  /* Initialize the peripherals and the MEMS components */

  MX_BTN8962TA_Init();

  /* USER CODE BEGIN MEMS_Init_PostTreatment */

  /* USER CODE END MEMS_Init_PostTreatment */
}

/*
 * LM background task
 */
void MX_MOTOR_Process(void)
{
  /* USER CODE BEGIN MEMS_Process_PreTreatment */

  /* USER CODE END MEMS_Process_PreTreatment */

  MX_BTN8962TA_Process();

  /* USER CODE BEGIN MEMS_Process_PostTreatment */

  /* USER CODE END MEMS_Process_PostTreatment */
}

/**
  * @brief  Initialize the DataLogTerminal application
  * @retval None
  */
void MX_BTN8962TA_Init(void)
{

	HAL_GPIO_WritePin(GPIOE,HB0_P_EN_Pin,GPIO_PIN_RESET);   // BOTH MOTOR DIRECTIONS ARE DISABLED
	HAL_GPIO_WritePin(GPIOE,HB0_N_EN_Pin,GPIO_PIN_RESET);

	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1); // turn on complementary channel

	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,500); // Set motor power 50% as default

}


/**
  * @brief  Process of the DataLogTerminal application
  * @retval None
  */
void MX_BTN8962TA_Process(void)
{
	const int MaxMagnitude = 1000;
	int magnitude;
	//ENCODER CODE
	const int MotorCPR = 6707;
	const float radspertick = (2*3.14759)/MotorCPR; // Update CubeMX TIM Counter Period setting
	float currentPosition;
	float direction;

	if(xQueueReceive(motorctrlQueueHandle, &MOTORCTRL_1, 90)){

		//
		xSemaphoreTake(Tim3MutexHandle, portMAX_DELAY);
		currentPosition = (float) htim3.Instance->CNT;
		xSemaphoreGive(Tim3MutexHandle);

		direction = shortestSignedDistanceBetweenCircularValues((currentPosition)*radspertick, MOTORCTRL_1.target);
		MOTORCTRL_1.direction = 0;
		if (direction > 0.08) MOTORCTRL_1.direction = 1;
		if (direction < -0.08) MOTORCTRL_1.direction = 2;

		magnitude = 500; //50% of MaxMagnitude

		switch ((int)MOTORCTRL_1.direction){
			case 1:
				magnitude = MOTORCTRL_1.power;              // direct due to P complementary PWN output
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,magnitude); // set PWM_P motor power
				HAL_GPIO_WritePin(GPIOE,HB0_P_EN_Pin,GPIO_PIN_SET);   // MOTOR DIRECTIONS FORWARDS
				HAL_GPIO_WritePin(GPIOE,HB0_N_EN_Pin,GPIO_PIN_RESET);
				break;

			case 2:
				magnitude = 1000 - MOTORCTRL_1.power;		// inverse due to N complementary PWN output
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,(magnitude)); // set PWM_N motor power
				HAL_GPIO_WritePin(GPIOE,HB0_P_EN_Pin,GPIO_PIN_RESET);   // MOTOR DIRECTIONS BACKWARDS
				HAL_GPIO_WritePin(GPIOE,HB0_N_EN_Pin,GPIO_PIN_SET);
				break;

			default:
				HAL_GPIO_WritePin(GPIOE,HB0_P_EN_Pin,GPIO_PIN_RESET);   // BOTH MOTOR DISABLED
				HAL_GPIO_WritePin(GPIOE,HB0_N_EN_Pin,GPIO_PIN_RESET);
		}
	}
	else {
		xSemaphoreTake(Tim3MutexHandle, portMAX_DELAY);
		currentPosition = (float) htim3.Instance->CNT;
		xSemaphoreGive(Tim3MutexHandle);
// BOBBY First measure the location and direction
		direction = shortestSignedDistanceBetweenCircularValues((currentPosition)*radspertick, MOTORCTRL_1.target);
		MOTORCTRL_1.direction = 0;
		if (direction > 0.08) MOTORCTRL_1.direction = 1;
		if (direction < -0.08) MOTORCTRL_1.direction = 2;

// BOBBY Second calculate output power based on a controller input
		MOTORCTRL_1.power = 500; //50% of MaxMagnitude

		switch ((int)MOTORCTRL_1.direction){
			case 1:
				magnitude = MOTORCTRL_1.power;              // direct due to P complementary PWN output
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,magnitude); // set PWM_P motor power
				HAL_GPIO_WritePin(GPIOE,HB0_N_EN_Pin,GPIO_PIN_RESET);
				HAL_GPIO_WritePin(GPIOE,HB0_P_EN_Pin,GPIO_PIN_SET);   // MOTOR DIRECTION FORWARDS
				break;

			case 2:
				magnitude = MOTORCTRL_1.power;		// inverse due to N complementary PWN output
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,(magnitude)); // set PWM_N motor power
				HAL_GPIO_WritePin(GPIOE,HB0_N_EN_Pin,GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOE,HB0_P_EN_Pin,GPIO_PIN_RESET);   // MOTOR DIRECTION BACKWARDS
				break;

			default:
				HAL_GPIO_WritePin(GPIOE,HB0_P_EN_Pin,GPIO_PIN_RESET);   // BOTH MOTOR DISABLED
				HAL_GPIO_WritePin(GPIOE,HB0_N_EN_Pin,GPIO_PIN_RESET);
	}
 }
}

#define MAX_VALUE 3.14159*2

float shortestSignedDistanceBetweenCircularValues(float origin, float target){
// This function requires both the origin and target to be in a range from 0 to 2*PI
  float signedDiff = 0.0;
  float raw_diff = origin > target ? origin - target : target - origin;
  float mod_diff = fmod(raw_diff, MAX_VALUE); //equates rollover values. E.g 0 == 2* PI radians in a circle

  if(mod_diff > (MAX_VALUE/2) ){
    //There is a shorter path in opposite direction
    signedDiff = (MAX_VALUE - mod_diff);
    if(target>origin) signedDiff = signedDiff * -1;
  } else {
    signedDiff = mod_diff;
    if(origin>target) signedDiff = signedDiff * -1;
  }
  return signedDiff;
}

#ifdef __cplusplus
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
