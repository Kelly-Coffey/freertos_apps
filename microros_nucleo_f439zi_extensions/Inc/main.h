/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "app.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern uint8_t SensorReadRequest;

struct pcc {
		float accelDataX;
		float accelDataY;
	 	float accelDataZ;
		float gyroDataX;
		float gyroDataY;
	 	float gyroDataZ;
		float magDataX;
		float magDataY;
	 	float magDataZ;
	 	float tempData;
	 	float presData;
	 	float humdData;
} pcc;

extern struct pcc PCC_1;

struct motorctrl {
	 float power;  // 0 to 1000
	 float direction; // stop, forward , backward
	 float target;  // in degrees radians TODO need to determine best 0 degree relative to hardware
} motorctrl;

extern struct motorctrl MOTORCTRL_1;

struct encoder {
	 float position;  // degrees in radians
	 float radspsec;  //radians per second
} encoder;

extern struct encoder ENCODER_1;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define HB0_P_EN_Pin GPIO_PIN_0
#define HB0_P_EN_GPIO_Port GPIOE
#define HB0_N_EN_Pin GPIO_PIN_1
#define HB0_N_EN_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
