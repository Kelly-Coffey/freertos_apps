/**
  ******************************************************************************
  * File Name          : stmicroelectronics_x-cube-mems1_7_2_0.c
  * Description        : This file provides code for the configuration
  *                      of the STMicroelectronics.X-CUBE-MEMS1.7.2.0 instances.
  ******************************************************************************
  *
  * COPYRIGHT 2020 STMicroelectronics
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

#include "stm32f4xx_hal.h"
#include "iks01a2_mems_control.h"
#include "bsp_ip_conf.h"

#include "FreeRTOS.h"
#include "portable.h"
#include "queue.h"
#include "task.h"
#include "timers.h"

/* Private typedef -----------------------------------------------------------*/

 /* Enable sensor masks */
 #define PRESSURE_SENSOR       0x00000001U
 #define TEMPERATURE_SENSOR    0x00000002U
 #define HUMIDITY_SENSOR       0x00000004U
 #define UV_SENSOR             0x00000008U /* for future use */
 #define ACCELEROMETER_SENSOR  0x00000010U
 #define GYROSCOPE_SENSOR      0x00000020U
 #define MAGNETIC_SENSOR       0x00000040U

 float samplePeriod = 0.2f; // replace this value with actual sample period in seconds

 // Data From accelerometer
  FusionVector3 DataFromAccelerometer = {
      .axis.x = 0.0f, /* replace this value with actual accelerometer x axis measurement in lsb */
      .axis.y = 0.0f, /* replace this value with actual accelerometer y axis measurement in lsb */
      .axis.z = 1.0f, /* replace this value with actual accelerometer z axis measurement in lsb */
  };

 // Data From gyroscope
  FusionVector3 DataFromGyroscope = {
      .axis.x = 0.0f, /* replace this value with actual accelerometer x axis measurement in lsb */
      .axis.y = 0.0f, /* replace this value with actual accelerometer y axis measurement in lsb */
      .axis.z = 1.0f, /* replace this value with actual accelerometer z axis measurement in lsb */
  };

 // Data From magnetometer
   FusionVector3 DataFromMagnetometer = {
       .axis.x = 0.0f, /* replace this value with actual accelerometer x axis measurement in lsb */
       .axis.y = 0.0f, /* replace this value with actual accelerometer y axis measurement in lsb */
       .axis.z = 1.0f, /* replace this value with actual accelerometer z axis measurement in lsb */
   };


 //Humidity Measurement
 float hum_value;

 //Temperature Measurement
 float temp_value;

//Pressure Measurement
 float press_value;

 /**
  * @brief  Serial message structure definition
  */
#define TMsg_MaxLen             256
 typedef struct
 {
   uint32_t Len;
   uint8_t Data[TMsg_MaxLen];
 } TMsg;

/* Private define ------------------------------------------------------------*/
#define ALGO_FREQ    100U /* Algorithm frequency [Hz] */

#define ALGO_PERIOD  10   /* Algorithm period [ms] */

#define MOTION_FX_ENGINE_DELTATIME  0.01f

#define FROM_MG_TO_G         0.001f
#define FROM_G_TO_MG         1000.0f
#define FROM_MDPS_TO_DPS     0.001f
#define FROM_DPS_TO_MDPS     1000.0f
#define FROM_MGAUSS_TO_UT50  (0.1f/50.0f)
#define FROM_UT50_TO_MGAUSS  500.0f

#define DWT_LAR_KEY  0xC5ACCE55 /* DWT register unlock key */

/* Public variables ----------------------------------------------------------*/
volatile uint8_t DataLoggerActive = 0;
volatile uint32_t SensorsEnabled = 0;
uint8_t Enabled6X = 0;

char lib_version[35];
int lib_version_len;

/* Extern variables ----------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
struct pcc PCC_1;
QueueHandle_t PCC_Queue_Handle;
uint8_t SensorReadRequest;
static MOTION_SENSOR_Axes_t AccValue;
static MOTION_SENSOR_Axes_t GyrValue;
static MOTION_SENSOR_Axes_t MagValue;
uint32_t SensorsAreEnabled = 0;


/* Private function prototypes -----------------------------------------------*/
static void MX_DataLogFusion_Init(void);
static void MX_DataLogFusion_Process(void);

static void Init_Sensors(void);
static void Disable_Sensors(void);
static void Enable_Sensors(void);
static void FX_Data_Handler(void);

void MX_MEMS_Init(void)
{
  /* USER CODE BEGIN SV */

  /* USER CODE END SV */

  /* USER CODE BEGIN MEMS_Init_PreTreatment */

  /* USER CODE END MEMS_Init_PreTreatment */

  /* Initialize the peripherals and the MEMS components */

  MX_DataLogFusion_Init();

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

  MX_DataLogFusion_Process();

  /* USER CODE BEGIN MEMS_Process_PostTreatment */

  /* USER CODE END MEMS_Process_PostTreatment */
}

/* Exported functions --------------------------------------------------------*/
/**

/**
 * @brief  Configures the current date
 * @param  y the year value to be set
 * @param  m the month value to be set
 * @param  d the day value to be set
 * @param  dw the day-week value to be set
 * @retval None
 */
void RTC_DateRegulate(uint8_t y, uint8_t m, uint8_t d, uint8_t dw)
{
  RTC_DateTypeDef sdatestructure;

  sdatestructure.Year    = y;
  sdatestructure.Month   = m;
  sdatestructure.Date    = d;
  sdatestructure.WeekDay = dw;

  if (HAL_RTC_SetDate(&hrtc, &sdatestructure, FORMAT_BIN) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}

/**
 * @brief  Configures the current time
 * @param  hh the hour value to be set
 * @param  mm the minute value to be set
 * @param  ss the second value to be set
 * @retval None
 */
void RTC_TimeRegulate(uint8_t hh, uint8_t mm, uint8_t ss)
{
  RTC_TimeTypeDef stimestructure;

  stimestructure.Hours          = hh;
  stimestructure.Minutes        = mm;
  stimestructure.Seconds        = ss;
  stimestructure.SubSeconds     = 0;
  stimestructure.TimeFormat     = RTC_HOURFORMAT12_AM;
  stimestructure.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  stimestructure.StoreOperation = RTC_STOREOPERATION_RESET;

  if (HAL_RTC_SetTime(&hrtc, &stimestructure, FORMAT_BIN) != HAL_OK)
  {
    /* Initialization Error */
    Error_Handler();
  }
}


/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Initialize the DataLogFusion application
  * @retval None
  */
static void MX_DataLogFusion_Init(void)
{

  /* Initialize (disabled) Sensors */
  Init_Sensors();
  /* Sensor Fusion API initialization function */

  /* Enable magnetometer calibration */
  /* Test if calibration data are available */
  /* If calibration data are available load HI coeficients */

}



/**
  * @brief  Process of the DataLogFusion application
  * @retval None
  */
static void MX_DataLogFusion_Process(void)
{
	if (SensorsAreEnabled == 0) {
		Enable_Sensors();
	}

    /* Acquire data from enabled sensors and fill Msg stream */

    if ((SensorsEnabled & ACCELEROMETER_SENSOR) == ACCELEROMETER_SENSOR)
    {
    	BSP_SENSOR_ACC_GetAxes(&AccValue);
    }

    if ((SensorsEnabled & GYROSCOPE_SENSOR) == GYROSCOPE_SENSOR)
    {
    	BSP_SENSOR_GYR_GetAxes(&GyrValue);
    }

    if ((SensorsEnabled & MAGNETIC_SENSOR) == MAGNETIC_SENSOR)
    {
      BSP_SENSOR_MAG_GetAxes(&MagValue);
    }


    if ((SensorsEnabled & HUMIDITY_SENSOR) == HUMIDITY_SENSOR)
    {
      BSP_SENSOR_HUM_GetValue(&hum_value);
    }

    if ((SensorsEnabled & TEMPERATURE_SENSOR) == TEMPERATURE_SENSOR)
    {
      BSP_SENSOR_TEMP_GetValue(&temp_value);
    }


    if ((SensorsEnabled & PRESSURE_SENSOR) == PRESSURE_SENSOR)
    {
      BSP_SENSOR_PRESS_GetValue(&press_value);
    }
    FX_Data_Handler();

}

/**
 * @brief  Initialize all sensors
 * @param  None
 * @retval None
 */
static void Init_Sensors(void)
{
  BSP_SENSOR_ACC_Init();
  BSP_SENSOR_GYR_Init();
  BSP_SENSOR_MAG_Init();
  BSP_SENSOR_PRESS_Init();
  BSP_SENSOR_TEMP_Init();
  BSP_SENSOR_HUM_Init();
  Disable_Sensors();
}

/**
 * @brief  Disable all sensors
 * @param  None
 * @retval None
 */
static void Disable_Sensors(void)
{
  BSP_SENSOR_ACC_Disable();
  BSP_SENSOR_GYR_Disable();
  BSP_SENSOR_MAG_Disable();
  BSP_SENSOR_PRESS_Disable();
  BSP_SENSOR_TEMP_Disable();
  BSP_SENSOR_HUM_Disable();
  SensorsEnabled = 0;
  SensorsAreEnabled = 0;
}


/**
 * @brief  Enable all sensors
 * @param  None
 * @retval None
 */
static void Enable_Sensors(void)
{
   BSP_SENSOR_ACC_Enable();
   BSP_SENSOR_GYR_Enable();
   BSP_SENSOR_MAG_Enable();
   BSP_SENSOR_PRESS_Enable();
   BSP_SENSOR_TEMP_Enable();
   BSP_SENSOR_HUM_Enable();
   SensorsEnabled = (PRESSURE_SENSOR |TEMPERATURE_SENSOR |HUMIDITY_SENSOR |UV_SENSOR |ACCELEROMETER_SENSOR |GYROSCOPE_SENSOR |MAGNETIC_SENSOR);
//   SensorsEnabled = (ACCELEROMETER_SENSOR |GYROSCOPE_SENSOR);
   SensorsAreEnabled = 1;
}

/**
 * @brief  Sensor Fusion data handler
 * @param  Msg the Sensor Fusion data part of the stream
 * @retval None
 */
static void FX_Data_Handler(void)
{

  if ((SensorsEnabled & ACCELEROMETER_SENSOR) == ACCELEROMETER_SENSOR)
  {
     if ((SensorsEnabled & GYROSCOPE_SENSOR) == GYROSCOPE_SENSOR)
    {
      if ((SensorsEnabled & MAGNETIC_SENSOR) == MAGNETIC_SENSOR)
      {
        /* Convert angular velocity from [mdps] to [dps] */
    	DataFromGyroscope.axis.x = (float)GyrValue.x * FROM_MDPS_TO_DPS;
    	DataFromGyroscope.axis.y = (float)GyrValue.y * FROM_MDPS_TO_DPS;
    	DataFromGyroscope.axis.z = (float)GyrValue.z * FROM_MDPS_TO_DPS;

        /* Convert acceleration from [mg] to [g] */
    	DataFromAccelerometer.axis.x = (float)AccValue.x * FROM_MG_TO_G;
    	DataFromAccelerometer.axis.y = (float)AccValue.y * FROM_MG_TO_G;
    	DataFromAccelerometer.axis.z = (float)AccValue.z * FROM_MG_TO_G;

        /* Convert magnetic field intensity from [mGauss] to [uT / 50] */
        DataFromMagnetometer.axis.x = (float)MagValue.x * FROM_MGAUSS_TO_UT50;
        DataFromMagnetometer.axis.y = (float)MagValue.y * FROM_MGAUSS_TO_UT50;
        DataFromMagnetometer.axis.z = (float)MagValue.z * FROM_MGAUSS_TO_UT50;

        PCC_1.accelDataX = DataFromAccelerometer.axis.x;
        PCC_1.accelDataY = DataFromAccelerometer.axis.y;
        PCC_1.accelDataZ = DataFromAccelerometer.axis.z;

        //PCC_1.accelDataX = DataFromAccelerometer.axis.x;
        //PCC_1.accelDataY = DataFromAccelerometer.axis.y;
        //PCC_1.accelDataZ = DataFromAccelerometer.axis.z;
        //PCC_1.gyroDataX = DataFromGyroscope.axis.x;
        //PCC_1.gyroDataY = DataFromGyroscope.axis.y;
        //PCC_1.gyroDataZ = DataFromGyroscope.axis.z;

      }
    }
  }

  if (! xQueueSend(PCC_Queue_Handle,&PCC_1,1000)){
	  printf("Failed to write sensor data to PCC queue\n");
	}
}


#ifdef __cplusplus
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
