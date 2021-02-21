/**
  ******************************************************************************
  * File Name          : app_TIM3_Encode.h
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __APP_TIM3_ENCODER_H
#define __APP_TIM3_ENCODER_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
/* Exported defines ----------------------------------------------------------*/

/* Exported functions --------------------------------------------------------*/
void MX_Encoder_Init(void);
void MX_Encoder_Process(void);

#ifdef __cplusplus
}
#endif

#endif /* __APP_TIM3_ENCODER_H */

/************************ (C) COPYRIGHT Kelly-Coffey *****END OF FILE****/
