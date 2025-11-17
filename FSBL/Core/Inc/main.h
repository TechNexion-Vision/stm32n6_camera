/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
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

#if defined ( __ICCARM__ )
#  define CMSE_NS_CALL  __cmse_nonsecure_call
#  define CMSE_NS_ENTRY __cmse_nonsecure_entry
#else
#  define CMSE_NS_CALL  __attribute((cmse_nonsecure_call))
#  define CMSE_NS_ENTRY __attribute((cmse_nonsecure_entry))
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32n6xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32n6xx_ll_bus.h"
#include "stm32n6xx_ll_rcc.h"
#include "stm32n6xx_ll_system.h"
#include "stm32n6xx_ll_utils.h"
#include "stm32n6xx_ll_gpio.h"
#include "stm32n6xx_ll_exti.h"
#include "stm32n6xx_ll_usart.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* Function pointer declaration in non-secure*/
#if defined ( __ICCARM__ )
typedef void (CMSE_NS_CALL *funcptr)(void);
#else
typedef void CMSE_NS_CALL (*funcptr)(void);
#endif

/* typedef for non-secure callback functions */
typedef funcptr funcptr_NS;

/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

#define RX_BUF_SIZE					128U

#define COMMAND_I2C_DATA_BUF_SIZE	16U

#define IO_EXPANDER_ADDRESS			(0x27 << 1)
#define TEVS_ADDRESS				(0x48 << 1)

#define TEVS_BOOT_TIME				250U
#define TEVS_DEF_DATA_FREQ			800U

#define TEVS_BOOT_STATE_NORMAL		0x08

#define PREVIEW_WIDTH 				640U
#define PREVIEW_HEIGHT 				480U
#define PREVIEW_FORMAT_UYVY			0x50
#define PREVIEW_SENSOR_MODE			1U
#define PREVIEW_FPS	 				60U
#define PREVIEW_HINF_CTRL			0x12
#define FRAME_BUFFER_SIZE 			(PREVIEW_WIDTH * PREVIEW_HEIGHT * 2)
#define BUFFER_ADDRESS  			0x34200000
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* IRQ Handler treatment functions */
void UART_CharReception_Callback(void);
void UART_Error_Callback(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
#define LED_GREEN_Pin 				GPIO_PIN_1
#define LED_GREEN_GPIO_Port 		GPIOO
#define LED_RED_Pin 				GPIO_PIN_10
#define LED_RED_GPIO_Port 			GPIOG

/* Define host command register of TEVS information page */
#define HOST_COMMAND_TEVS_INFO_VERSION_MSB 						0x3000
#define HOST_COMMAND_TEVS_INFO_VERSION_LSB 						0x3002
#define HOST_COMMAND_TEVS_BOOT_STATE 							0x3004
#define HOST_COMMAND_TEVS_SENSOR_CHIP_ID                        0x3008
#define HOST_COMMAND_TEVS_MODEL_NUMBER_0                        0x300C
#define HOST_COMMAND_TEVS_MODEL_NUMBER_1                        0x300E
#define HOST_COMMAND_TEVS_MODEL_NUMBER_2                        0x3010

/* Define host command register of ISP control page */
#define HOST_COMMAND_ISP_CTRL_PREVIEW_WIDTH 					0x3100
#define HOST_COMMAND_ISP_CTRL_PREVIEW_HEIGHT 					0x3102
#define HOST_COMMAND_ISP_CTRL_PREVIEW_FORMAT 					0x3104
#define HOST_COMMAND_ISP_CTRL_PREVIEW_SENSOR_MODE 				0x3106
#define HOST_COMMAND_ISP_CTRL_PREVIEW_THROUGHPUT 				0x3108
#define HOST_COMMAND_ISP_CTRL_PREVIEW_MAX_FPS 					0x310A
#define HOST_COMMAND_ISP_CTRL_PREVIEW_EXP_TIME_UPPER_MSB 		0x310C
#define HOST_COMMAND_ISP_CTRL_PREVIEW_EXP_TIME_UPPER_LSB 		0x310E
#define HOST_COMMAND_ISP_CTRL_PREVIEW_EXP_TIME_MAX_MSB 			0x3110
#define HOST_COMMAND_ISP_CTRL_PREVIEW_EXP_TIME_MAX_LSB 			0x3112
#define HOST_COMMAND_ISP_CTRL_PREVIEW_HINF_CTRL 				0x3114
#define HOST_COMMAND_ISP_CTRL_AE_MODE 							0x3116
#define HOST_COMMAND_ISP_CTRL_EXP_TIME_MSB 						0x3118
#define HOST_COMMAND_ISP_CTRL_EXP_TIME_LSB 						0x311A
#define HOST_COMMAND_ISP_CTRL_EXP_TIME_MAX_MSB 					0x311C
#define HOST_COMMAND_ISP_CTRL_EXP_TIME_MAX_LSB 					0x311E
#define HOST_COMMAND_ISP_CTRL_EXP_TIME_MIN_MSB 					0x3120
#define HOST_COMMAND_ISP_CTRL_EXP_TIME_MIN_LSB 					0x3122
#define HOST_COMMAND_ISP_CTRL_EXP_GAIN						 	0x3124
#define HOST_COMMAND_ISP_CTRL_EXP_GAIN_MAX 						0x3126
#define HOST_COMMAND_ISP_CTRL_EXP_GAIN_MIN 						0x3128
#define HOST_COMMAND_ISP_CTRL_CURRENT_EXP_TIME_MSB 				0x312A
#define HOST_COMMAND_ISP_CTRL_CURRENT_EXP_TIME_LSB 				0x312C
#define HOST_COMMAND_ISP_CTRL_CURRENT_EXP_GAIN 					0x312E
#define HOST_COMMAND_ISP_CTRL_BACKLIGHT_COMPENSATION 			0x3130
#define HOST_COMMAND_ISP_CTRL_BACKLIGHT_COMPENSATION_MAX 		0x3132
#define HOST_COMMAND_ISP_CTRL_BACKLIGHT_COMPENSATION_MIN 		0x3134
#define HOST_COMMAND_ISP_CTRL_AWB_MODE 							0x3136
#define HOST_COMMAND_ISP_CTRL_AWB_TEMP 							0x3138
#define HOST_COMMAND_ISP_CTRL_AWB_TEMP_MAX 						0x313A
#define HOST_COMMAND_ISP_CTRL_AWB_TEMP_MIN 						0x313C
#define HOST_COMMAND_ISP_CTRL_BRIGHTNESS 						0x313E
#define HOST_COMMAND_ISP_CTRL_BRIGHTNESS_MAX 					0x3140
#define HOST_COMMAND_ISP_CTRL_BRIGHTNESS_MIN 					0x3142
#define HOST_COMMAND_ISP_CTRL_CONTRAST 							0x3144
#define HOST_COMMAND_ISP_CTRL_CONTRAST_MAX 						0x3146
#define HOST_COMMAND_ISP_CTRL_CONTRAST_MIN 						0x3148
#define HOST_COMMAND_ISP_CTRL_SATURATION 						0x314A
#define HOST_COMMAND_ISP_CTRL_SATURATION_MAX 					0x314C
#define HOST_COMMAND_ISP_CTRL_SATURATION_MIN 					0x314E
#define HOST_COMMAND_ISP_CTRL_GAMMA 							0x3150
#define HOST_COMMAND_ISP_CTRL_GAMMA_MAX 						0x3152
#define HOST_COMMAND_ISP_CTRL_GAMMA_MIN 						0x3154
#define HOST_COMMAND_ISP_CTRL_DENOISE 							0x3156
#define HOST_COMMAND_ISP_CTRL_DENOISE_MAX 						0x3158
#define HOST_COMMAND_ISP_CTRL_DENOISE_MIN 						0x315A
#define HOST_COMMAND_ISP_CTRL_SHARPEN 							0x315C
#define HOST_COMMAND_ISP_CTRL_SHARPEN_MAX 						0x315E
#define HOST_COMMAND_ISP_CTRL_SHARPEN_MIN 						0x3160
#define HOST_COMMAND_ISP_CTRL_FLIP 								0x3162
#define HOST_COMMAND_ISP_CTRL_EFFECT 							0x3164
#define HOST_COMMAND_ISP_CTRL_ZOOM_TYPE 						0x3166
#define HOST_COMMAND_ISP_CTRL_ZOOM_TIMES 						0x3168
#define HOST_COMMAND_ISP_CTRL_ZOOM_TIMES_MAX 					0x316A
#define HOST_COMMAND_ISP_CTRL_ZOOM_TIMES_MIN 					0x316C
#define HOST_COMMAND_ISP_CTRL_CT_X 								0x316E
#define HOST_COMMAND_ISP_CTRL_CT_Y 								0x3170
#define HOST_COMMAND_ISP_CTRL_CT_MAX 							0x3172
#define HOST_COMMAND_ISP_CTRL_CT_MIN 							0x3174
#define HOST_COMMAND_ISP_CTRL_SYSTEM_START 						0x3176
#define HOST_COMMAND_ISP_CTRL_ISP_RESET 						0x3178
#define HOST_COMMAND_ISP_CTRL_TRIGGER_MODE 						0x317A
#define HOST_COMMAND_ISP_CTRL_FLICK_CTRL					 	0x317C
#define HOST_COMMAND_ISP_CTRL_MIPI_FREQ 						0x317E
#define HOST_COMMAND_ISP_CTRL_JPEG_QUAL							0x3180
#define HOST_COMMAND_ISP_CTRL_PREVIEW_MIPI_CTRL 				0x3182

/* Define host command register of ISP bootdata page */
#define HOST_COMMAND_ISP_BOOTDATA_1								0x4000

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
