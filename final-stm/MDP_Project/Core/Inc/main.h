/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

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
#define OLED_SCL_Pin GPIO_PIN_5
#define OLED_SCL_GPIO_Port GPIOE
#define OLED_SDA_Pin GPIO_PIN_6
#define OLED_SDA_GPIO_Port GPIOE
#define AIN2_Pin GPIO_PIN_2
#define AIN2_GPIO_Port GPIOA
#define AIN1_Pin GPIO_PIN_3
#define AIN1_GPIO_Port GPIOA
#define BIN1_Pin GPIO_PIN_4
#define BIN1_GPIO_Port GPIOA
#define BIN2_Pin GPIO_PIN_5
#define BIN2_GPIO_Port GPIOA
#define OLED_RST_Pin GPIO_PIN_7
#define OLED_RST_GPIO_Port GPIOE
#define OLED_DC_Pin GPIO_PIN_8
#define OLED_DC_GPIO_Port GPIOE
#define TRIG_Pin GPIO_PIN_9
#define TRIG_GPIO_Port GPIOE
#define LED3_Pin GPIO_PIN_10
#define LED3_GPIO_Port GPIOE
#define ServoPWM_Pin GPIO_PIN_14
#define ServoPWM_GPIO_Port GPIOE
#define Buzzer_Pin GPIO_PIN_10
#define Buzzer_GPIO_Port GPIOB
#define PWMA_Pin GPIO_PIN_6
#define PWMA_GPIO_Port GPIOC
#define PWMB_Pin GPIO_PIN_7
#define PWMB_GPIO_Port GPIOC

/* USER CODE BEGIN Private defines */
#define WHEEL_RADIUS 0.034 //in Meters
#define WHEEL_BASE 0.1405 // in Meters
#define PI 3.141592654
#define WHEEL_CIRCUMFERENCE 0.2 //in Meters Formula: 2 * PI * WHEEL_RADIUS
#define PULSE_PER_REVOLUTION 330 //1 wheel rotation = 1500~1600ticks, (Datasheet = 330)
#define TICKS_PER_REVOLUTION 1500 // 1 wheel rotation = Roughly 1545 Ticks; formula : 330/(WHEEL_CIRCUMFERENCE)

#define DIR_FORWARD 1
#define DIR_BACKWARD 0

#define LEFT_MAX 95 //78 is the maximum, 108 latest
#define CENTER_LEFT 160
#define CENTER_RIGHT 140
#define CENTER 145
#define RIGHT_MAX 240
#define TURN_TIME 500

#define PWM_MAX 7000
#define PWM_MIN 0

//UARTS Defines
#define ARX_BUFFER_SIZE 6
#define CMD_BUFFER_SIZE 12

//IR Defines
#define IR_CONST_A 25644.81557
#define IR_CONST_B 260.4233354
#define IR_SAMPLE 100
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
