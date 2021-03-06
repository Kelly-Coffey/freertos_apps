/**
  ******************************************************************************
  * File Name          : stmicroelectronics_x-cube-mems1_7_2_0.c
  * Description        : This file provides code for the configuration
  *                      of the STMicroelectronics.X-CUBE-MEMS1.7.2.0 instances.
  ******************************************************************************
  *
  * COPYRIGHT 2021 STMicroelectronics
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
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
#include "app_x-cube-mems1.h"
#include "main.h"
#include <stdio.h>

#include "iks01a2_motion_sensors.h"
#include "iks01a2_env_sensors.h"
#include "math.h"

#include "FreeRTOS.h"
#include "portable.h"
#include "queue.h"
#include "task.h"
#include "timers.h"

/* Private variables ---------------------------------------------------------*/
static uint8_t verbose = 1;  /* Verbose output to UART terminal ON/OFF. */
static IKS01A2_MOTION_SENSOR_Capabilities_t MotionCapabilities[IKS01A2_MOTION_INSTANCES_NBR];
static IKS01A2_ENV_SENSOR_Capabilities_t EnvCapabilities[IKS01A2_ENV_INSTANCES_NBR];

struct pcc PCC_1;
extern QueueHandle_t sensorQueueHandle;

/* Private function prototypes -----------------------------------------------*/
static void Accelero_Sensor_Handler(uint32_t Instance);
static void Gyro_Sensor_Handler(uint32_t Instance);
static void Magneto_Sensor_Handler(uint32_t Instance);
static void Temp_Sensor_Handler(uint32_t Instance);
static void Hum_Sensor_Handler(uint32_t Instance);
static void Press_Sensor_Handler(uint32_t Instance);
static void MX_IKS01A2_Init(void);
static void MX_IKS01A2_Process(void);

void MX_MEMS_Init(void)
{
  /* USER CODE BEGIN SV */

  /* USER CODE END SV */

  /* USER CODE BEGIN MEMS_Init_PreTreatment */

  /* USER CODE END MEMS_Init_PreTreatment */

  /* Initialize the peripherals and the MEMS components */

  MX_IKS01A2_Init();

  /* USER CODE BEGIN MEMS_Init_PostTreatment */

  /* USER CODE END MEMS_Init_PostTreatment */
}

/*
 * LM background task
 */
void MX_MEMS_Process(void)
{
  /* USER CODE BEGIN MEMS_Process_PreTreatment */

  /* USER CODE END MEMS_Process_PreTreatment */

  MX_IKS01A2_Process();

  /* USER CODE BEGIN MEMS_Process_PostTreatment */

  /* USER CODE END MEMS_Process_PostTreatment */
}

/**
  * @brief  Initialize the DataLogTerminal application
  * @retval None
  */
void MX_IKS01A2_Init(void)
{
  int i;

  IKS01A2_MOTION_SENSOR_Init(IKS01A2_LSM6DSL_0, MOTION_ACCELERO | MOTION_GYRO);

  IKS01A2_MOTION_SENSOR_Init(IKS01A2_LSM303AGR_ACC_0, MOTION_ACCELERO);

  IKS01A2_MOTION_SENSOR_Init(IKS01A2_LSM303AGR_MAG_0, MOTION_MAGNETO);

  for(i = 0; i < IKS01A2_MOTION_INSTANCES_NBR; i++)
    {  IKS01A2_MOTION_SENSOR_GetCapabilities(i, &MotionCapabilities[i]);
    }
/* TODO Create micro-ROS 2 message and message log for MEM capability

  for(i = 0; i < IKS01A2_MOTION_INSTANCES_NBR; i++)
  {
    IKS01A2_MOTION_SENSOR_GetCapabilities(i, &MotionCapabilities[i]);
    snprintf(dataOut, MAX_BUF_SIZE,
             "\r\nMotion Sensor Instance %d capabilities: \r\n ACCELEROMETER: %d\r\n GYROSCOPE: %d\r\n MAGNETOMETER: %d\r\n LOW POWER: %d\r\n",
             i, MotionCapabilities[i].Acc, MotionCapabilities[i].Gyro, MotionCapabilities[i].Magneto, MotionCapabilities[i].LowPower);
    printf("%s", dataOut);
    floatToInt(MotionCapabilities[i].AccMaxOdr, &out_value_odr, 3);
    snprintf(dataOut, MAX_BUF_SIZE, " MAX ACC ODR: %d.%03d Hz, MAX ACC FS: %d\r\n", (int)out_value_odr.out_int,
             (int)out_value_odr.out_dec, (int)MotionCapabilities[i].AccMaxFS);
    printf("%s", dataOut);
    floatToInt(MotionCapabilities[i].GyroMaxOdr, &out_value_odr, 3);
    snprintf(dataOut, MAX_BUF_SIZE, " MAX GYRO ODR: %d.%03d Hz, MAX GYRO FS: %d\r\n", (int)out_value_odr.out_int,
             (int)out_value_odr.out_dec, (int)MotionCapabilities[i].GyroMaxFS);
    printf("%s", dataOut);
    floatToInt(MotionCapabilities[i].MagMaxOdr, &out_value_odr, 3);
    snprintf(dataOut, MAX_BUF_SIZE, " MAX MAG ODR: %d.%03d Hz, MAX MAG FS: %d\r\n", (int)out_value_odr.out_int,
             (int)out_value_odr.out_dec, (int)MotionCapabilities[i].MagMaxFS);
    printf("%s", dataOut);
  }
*/


  IKS01A2_ENV_SENSOR_Init(IKS01A2_HTS221_0, ENV_TEMPERATURE | ENV_HUMIDITY);

  IKS01A2_ENV_SENSOR_Init(IKS01A2_LPS22HB_0, ENV_TEMPERATURE | ENV_PRESSURE);

  for(i = 0; i < IKS01A2_ENV_INSTANCES_NBR; i++)
  {  IKS01A2_ENV_SENSOR_GetCapabilities(i, &EnvCapabilities[i]);
  }
  /* TODO Create micro-ROS 2 message and message log for MEM capability

  for(i = 0; i < IKS01A2_ENV_INSTANCES_NBR; i++)
  {
    IKS01A2_ENV_SENSOR_GetCapabilities(i, &EnvCapabilities[i]);
    snprintf(dataOut, MAX_BUF_SIZE,
             "\r\nEnvironmental Sensor Instance %d capabilities: \r\n TEMPERATURE: %d\r\n PRESSURE: %d\r\n HUMIDITY: %d\r\n LOW POWER: %d\r\n",
             i, EnvCapabilities[i].Temperature, EnvCapabilities[i].Pressure, EnvCapabilities[i].Humidity, EnvCapabilities[i].LowPower);
    printf("%s", dataOut);
    floatToInt(EnvCapabilities[i].TempMaxOdr, &out_value_odr, 3);
    snprintf(dataOut, MAX_BUF_SIZE, " MAX TEMP ODR: %d.%03d Hz\r\n", (int)out_value_odr.out_int,
             (int)out_value_odr.out_dec);
    printf("%s", dataOut);
    floatToInt(EnvCapabilities[i].PressMaxOdr, &out_value_odr, 3);
    snprintf(dataOut, MAX_BUF_SIZE, " MAX PRESS ODR: %d.%03d Hz\r\n", (int)out_value_odr.out_int,
             (int)out_value_odr.out_dec);
    printf("%s", dataOut);
    floatToInt(EnvCapabilities[i].HumMaxOdr, &out_value_odr, 3);
    snprintf(dataOut, MAX_BUF_SIZE, " MAX HUM ODR: %d.%03d Hz\r\n", (int)out_value_odr.out_int, (int)out_value_odr.out_dec);
    printf("%s", dataOut);
  } */

}


/**
  * @brief  Process of the DataLogTerminal application
  * @retval None
  */
void MX_IKS01A2_Process(void)
{
  int i;

  for(i = 0; i < IKS01A2_MOTION_INSTANCES_NBR; i++)
  {
    if(MotionCapabilities[i].Acc)
    {
      Accelero_Sensor_Handler(i);
    }
    if(MotionCapabilities[i].Gyro)
    {
      Gyro_Sensor_Handler(i);
    }
    if(MotionCapabilities[i].Magneto)
    {
      Magneto_Sensor_Handler(i);
    }
  }

  for(i = 0; i < IKS01A2_ENV_INSTANCES_NBR; i++)
  {
    if(EnvCapabilities[i].Humidity)
    {
      Hum_Sensor_Handler(i);
    }
    if(EnvCapabilities[i].Temperature)
    {
      Temp_Sensor_Handler(i);
    }
    if(EnvCapabilities[i].Pressure)
    {
      Press_Sensor_Handler(i);
    }
  }
  if (! xQueueSend(sensorQueueHandle,&PCC_1,100)){
	  printf("Failed to write sensor data to PCC queue\n");
	}

}

/**
  * @brief  Handles the accelerometer axes data getting/sending
  * @param  Instance the device instance
  * @retval None
  */
static void Accelero_Sensor_Handler(uint32_t Instance)
{
  float odr;
  int32_t fullScale;
  IKS01A2_MOTION_SENSOR_Axes_t acceleration;
  uint8_t whoami;

  if (IKS01A2_MOTION_SENSOR_GetAxes(Instance, MOTION_ACCELERO, &acceleration))
  {
	PCC_1.accelDataX =0;
	PCC_1.accelDataY =0;
	PCC_1.accelDataZ =0;
  }
  else
  {
    PCC_1.accelDataX =(int)acceleration.x;
    PCC_1.accelDataY =(int)acceleration.y;
    PCC_1.accelDataZ =(int)acceleration.z;
  }

  /* TODO Create micro-ROS 2 message and message log for MEM capability
  if (verbose == 1)
  {
    if (IKS01A2_MOTION_SENSOR_ReadID(Instance, &whoami))
    {
      snprintf(dataOut, MAX_BUF_SIZE, "WHOAMI[%d]: Error\r\n", (int)Instance);
    }
    else
    {
      snprintf(dataOut, MAX_BUF_SIZE, "WHOAMI[%d]: 0x%x\r\n", (int)Instance, (int)whoami);
    }

    printf("%s", dataOut);

    if (IKS01A2_MOTION_SENSOR_GetOutputDataRate(Instance, MOTION_ACCELERO, &odr))
    {
      snprintf(dataOut, MAX_BUF_SIZE, "ODR[%d]: ERROR\r\n", (int)Instance);
    }
    else
    {
      floatToInt(odr, &out_value, 3);
      snprintf(dataOut, MAX_BUF_SIZE, "ODR[%d]: %d.%03d Hz\r\n", (int)Instance, (int)out_value.out_int,
               (int)out_value.out_dec);
    }

    printf("%s", dataOut);

    if (IKS01A2_MOTION_SENSOR_GetFullScale(Instance, MOTION_ACCELERO, &fullScale))
    {
      snprintf(dataOut, MAX_BUF_SIZE, "FS[%d]: ERROR\r\n", (int)Instance);
    }
    else
    {
      snprintf(dataOut, MAX_BUF_SIZE, "FS[%d]: %d g\r\n", (int)Instance, (int)fullScale);
    }

    printf("%s", dataOut);
  }*/

}

/**
  * @brief  Handles the gyroscope axes data getting/sending
  * @param  Instance the device instance
  * @retval None
  */
static void Gyro_Sensor_Handler(uint32_t Instance)
{
  float odr;
  int32_t fullScale;
  IKS01A2_MOTION_SENSOR_Axes_t angular_velocity;
  uint8_t whoami;

  if (IKS01A2_MOTION_SENSOR_GetAxes(Instance, MOTION_GYRO, &angular_velocity))
  {
	    PCC_1.gyroDataX =0;
	    PCC_1.gyroDataY =0;
	    PCC_1.gyroDataZ =0;
  }
  else
  {
	    PCC_1.gyroDataX =(int)angular_velocity.x;
	    PCC_1.gyroDataY =(int)angular_velocity.y;
	    PCC_1.gyroDataZ =(int)angular_velocity.z;
  }


  /* TODO Create micro-ROS 2 message and message log for MEM capability
  if (verbose == 1)
  {
    if (IKS01A2_MOTION_SENSOR_ReadID(Instance, &whoami))
    {
      snprintf(dataOut, MAX_BUF_SIZE, "WHOAMI[%d]: Error\r\n", (int)Instance);
    }
    else
    {
      snprintf(dataOut, MAX_BUF_SIZE, "WHOAMI[%d]: 0x%x\r\n", (int)Instance, (int)whoami);
    }

    printf("%s", dataOut);

    if (IKS01A2_MOTION_SENSOR_GetOutputDataRate(Instance, MOTION_GYRO, &odr))
    {
      snprintf(dataOut, MAX_BUF_SIZE, "ODR[%d]: ERROR\r\n", (int)Instance);
    }
    else
    {
      floatToInt(odr, &out_value, 3);
      snprintf(dataOut, MAX_BUF_SIZE, "ODR[%d]: %d.%03d Hz\r\n", (int)Instance, (int)out_value.out_int,
               (int)out_value.out_dec);
    }

    printf("%s", dataOut);

    if (IKS01A2_MOTION_SENSOR_GetFullScale(Instance, MOTION_GYRO, &fullScale))
    {
      snprintf(dataOut, MAX_BUF_SIZE, "FS[%d]: ERROR\r\n", (int)Instance);
    }
    else
    {
      snprintf(dataOut, MAX_BUF_SIZE, "FS[%d]: %d dps\r\n", (int)Instance, (int)fullScale);
    }

    printf("%s", dataOut);
  }*/

}

/**
  * @brief  Handles the magneto axes data getting/sending
  * @param  Instance the device instance
  * @retval None
  */
static void Magneto_Sensor_Handler(uint32_t Instance)
{
  float odr;
  int32_t fullScale;
  IKS01A2_MOTION_SENSOR_Axes_t magnetic_field;
  uint8_t whoami;

  if (IKS01A2_MOTION_SENSOR_GetAxes(Instance, MOTION_MAGNETO, &magnetic_field))
  {
	    PCC_1.magDataX =0;
	    PCC_1.magDataY =0;
	    PCC_1.magDataZ =0;
  }
  else
  {
	    PCC_1.magDataX =(int)magnetic_field.x;
	    PCC_1.magDataY =(int)magnetic_field.y;
	    PCC_1.magDataZ =(int)magnetic_field.z;
  }

  /* TODO Create micro-ROS 2 message and message log for MEM capability
  if (verbose == 1)
  {
    if (IKS01A2_MOTION_SENSOR_ReadID(Instance, &whoami))
    {
      snprintf(dataOut, MAX_BUF_SIZE, "WHOAMI[%d]: Error\r\n", (int)Instance);
    }
    else
    {
      snprintf(dataOut, MAX_BUF_SIZE, "WHOAMI[%d]: 0x%x\r\n", (int)Instance, (int)whoami);
    }

    printf("%s", dataOut);

    if (IKS01A2_MOTION_SENSOR_GetOutputDataRate(Instance, MOTION_MAGNETO, &odr))
    {
      snprintf(dataOut, MAX_BUF_SIZE, "ODR[%d]: ERROR\r\n", (int)Instance);
    }
    else
    {
      floatToInt(odr, &out_value, 3);
      snprintf(dataOut, MAX_BUF_SIZE, "ODR[%d]: %d.%03d Hz\r\n", (int)Instance, (int)out_value.out_int,
               (int)out_value.out_dec);
    }

    printf("%s", dataOut);

    if (IKS01A2_MOTION_SENSOR_GetFullScale(Instance, MOTION_MAGNETO, &fullScale))
    {
      snprintf(dataOut, MAX_BUF_SIZE, "FS[%d]: ERROR\r\n", (int)Instance);
    }
    else
    {
      snprintf(dataOut, MAX_BUF_SIZE, "FS[%d]: %d gauss\r\n", (int)Instance, (int)fullScale);
    }

    printf("%s", dataOut);
  } */

}

/**
  * @brief  Handles the temperature data getting/sending
  * @param  Instance the device instance
  * @retval None
  */
static void Temp_Sensor_Handler(uint32_t Instance)
{
  float odr;
  float temperature;
  uint8_t whoami;

  if (IKS01A2_ENV_SENSOR_GetValue(Instance, ENV_TEMPERATURE, &temperature))
  {
	  PCC_1.tempData=0;
  }
  else
  {
	  PCC_1.tempData=temperature;
  }

  /* TODO Create micro-ROS 2 message and message log for MEM capability

  if (verbose == 1)
  {
    if (IKS01A2_ENV_SENSOR_ReadID(Instance, &whoami))
    {
      snprintf(dataOut, MAX_BUF_SIZE, "WHOAMI[%d]: Error\r\n", (int)Instance);
    }
    else
    {
      snprintf(dataOut, MAX_BUF_SIZE, "WHOAMI[%d]: 0x%x\r\n", (int)Instance, (int)whoami);
    }

    printf("%s", dataOut);

    if (IKS01A2_ENV_SENSOR_GetOutputDataRate(Instance, ENV_TEMPERATURE, &odr))
    {
      snprintf(dataOut, MAX_BUF_SIZE, "ODR[%d]: Error\r\n", (int)Instance);
    }
    else
    {
      floatToInt(odr, &out_value, 3);
      snprintf(dataOut, MAX_BUF_SIZE, "ODR[%d]: %d.%03d Hz\r\n", (int)Instance, (int)out_value.out_int,
               (int)out_value.out_dec);
    }

    printf("%s", dataOut);
  } */

}

/**
  * @brief  Handles the pressure sensor data getting/sending
  * @param  Instance the device instance
  * @retval None
  */
static void Press_Sensor_Handler(uint32_t Instance)
{
  float odr;
  float pressure;
  uint8_t whoami;

  if (IKS01A2_ENV_SENSOR_GetValue(Instance, ENV_PRESSURE, &pressure))
  {
	  PCC_1.presData=0;
  }
  else
  {
	  PCC_1.presData=pressure;
  }

  /* TODO Create micro-ROS 2 message and message log for MEM capability

  if (verbose == 1)
  {
    if (IKS01A2_ENV_SENSOR_ReadID(Instance, &whoami))
    {
      snprintf(dataOut, MAX_BUF_SIZE, "WHOAMI[%d]: Error\r\n", (int)Instance);
    }
    else
    {
      snprintf(dataOut, MAX_BUF_SIZE, "WHOAMI[%d]: 0x%x\r\n", (int)Instance, (int)whoami);
    }

    printf("%s", dataOut);

    if (IKS01A2_ENV_SENSOR_GetOutputDataRate(Instance, ENV_PRESSURE, &odr))
    {
      snprintf(dataOut, MAX_BUF_SIZE, "ODR[%d]: Error\r\n", (int)Instance);
    }
    else
    {
      floatToInt(odr, &out_value, 3);
      snprintf(dataOut, MAX_BUF_SIZE, "ODR[%d]: %d.%03d Hz\r\n", (int)Instance, (int)out_value.out_int,
               (int)out_value.out_dec);
    }

    printf("%s", dataOut);
  }*/

}

/**
  * @brief  Handles the humidity data getting/sending
  * @param  Instance the device instance
  * @retval None
  */
static void Hum_Sensor_Handler(uint32_t Instance)
{
  float odr;
  float humidity;
  uint8_t whoami;

  if (IKS01A2_ENV_SENSOR_GetValue(Instance, ENV_HUMIDITY, &humidity))
  {
	  PCC_1.humdData=0;
  }
  else
  {
	  PCC_1.humdData=humidity;
  }

  /* TODO Create micro-ROS 2 message and message log for MEM capability

  if (verbose == 1)
  {
    if (IKS01A2_ENV_SENSOR_ReadID(Instance, &whoami))
    {
      snprintf(dataOut, MAX_BUF_SIZE, "WHOAMI[%d]: Error\r\n", (int)Instance);
    }
    else
    {
      snprintf(dataOut, MAX_BUF_SIZE, "WHOAMI[%d]: 0x%x\r\n", (int)Instance, (int)whoami);
    }

    printf("%s", dataOut);

    if (IKS01A2_ENV_SENSOR_GetOutputDataRate(Instance, ENV_HUMIDITY, &odr))
    {
      snprintf(dataOut, MAX_BUF_SIZE, "ODR[%d]: Error\r\n", (int)Instance);
    }
    else
    {
      floatToInt(odr, &out_value, 3);
      snprintf(dataOut, MAX_BUF_SIZE, "ODR[%d]: %d.%03d Hz\r\n", (int)Instance, (int)out_value.out_int,
               (int)out_value.out_dec);
    }

    printf("%s", dataOut);
  } */

}

#ifdef __cplusplus
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
