/**
  ******************************************************************************
  * @file           : mems_manager.h
  * @brief          : mems interface between sensors and FreeRTOS
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_X_CUBE_MEMS1_H
#define __APP_X_CUBE_MEMS1_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/

/* Exported defines ----------------------------------------------------------*/
 extern RTC_HandleTypeDef hrtc;
 extern uint8_t SensorReadRequest;
/* Exported functions --------------------------------------------------------*/
 void MX_MEMS_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_X_CUBE_MEMS1_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
