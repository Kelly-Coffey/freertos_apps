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

/* Private variables ---------------------------------------------------------*/
static uint8_t verbose = 1;  /* Verbose output to UART terminal ON/OFF. */

struct motorctrl MOTORCTRL_1;
QueueHandle_t MOTORCTRL_Queue_Handle;


/* Private function prototypes -----------------------------------------------*/
static void MX_BTN8962TA_Init(void);
static void MX_BTN8962TA_Process(void);

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;

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
	const int STOP = 0;
	const int FORWARD = 1;
	const int BACKWARD = 2;
	int magnitude;

	if(xQueueReceive(MOTORCTRL_Queue_Handle, &MOTORCTRL_1, 90)){

		switch ((int)MOTORCTRL_1.direction){
			case 1:
				magnitude = MOTORCTRL_1.power;              // direct due to P complementary PWN output
				HAL_GPIO_WritePin(GPIOE,HB0_P_EN_Pin,GPIO_PIN_SET);   // MOTOR DIRECTIONS FORWARDS
				HAL_GPIO_WritePin(GPIOE,HB0_N_EN_Pin,GPIO_PIN_RESET);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,magnitude); // set PWM_P motor power
				break;

			case 2:
				magnitude = 1000 - MOTORCTRL_1.power;		// inverse due to N complementary PWN output
				HAL_GPIO_WritePin(GPIOE,HB0_P_EN_Pin,GPIO_PIN_RESET);   // MOTOR DIRECTIONS BACKWARDS
				HAL_GPIO_WritePin(GPIOE,HB0_N_EN_Pin,GPIO_PIN_SET);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1,magnitude); // set PWM_N motor power
				break;

			default:
				HAL_GPIO_WritePin(GPIOE,HB0_P_EN_Pin,GPIO_PIN_RESET);   // BOTH MOTOR DISABLED
				HAL_GPIO_WritePin(GPIOE,HB0_N_EN_Pin,GPIO_PIN_RESET);
		}
	}
}

#ifdef __cplusplus
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
