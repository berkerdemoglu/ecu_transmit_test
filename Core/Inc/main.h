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

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

#include "stm32g4xx_nucleo.h"
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef union {
    float sensor_float;
    uint32_t sensor_int;
    uint8_t bytes[4];
} can_message_four;

typedef union {
    uint64_t sensor_int;  // we might not need this
    double sensor_double;  // we might not need this
    struct {
        can_message_four first;
        can_message_four second;
    };
    uint8_t bytes[8];
} can_message_eight;


// Race modes
enum RaceMode {
	MODE_PIT_LIMITER,
	MODE_RACE,
	MODE_ECO,
	MODE_SENSOR_READING,
	MODE_GYMKHANA
};

enum RainState {
	STATE_NO_RAIN,
	STATE_RAIN
};

struct RaceState {
	enum RainState rain_state;
	enum RaceMode race_mode;
};


// Throttle
#define THROTTLE_BUFFER_SIZE 32
struct Throttle {
	float adc_sum;
	float buffer[THROTTLE_BUFFER_SIZE];
	uint8_t buffer_index;

	can_message_four throttle_value;
	float hysteresis;
	float hysteresis_min;

	uint8_t throttle_activated;  // flag
};

// Steering angle
#define STEERING_BUFFER_SIZE 32
struct SteeringAngle {
	can_message_four steering_value;
	float steering_output;

	float adc_sum;
	float buffer[STEERING_BUFFER_SIZE];
	uint8_t buffer_index;
};

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void race_state_init(struct RaceState* rs);
void handle_button_press(struct RaceState* rs, uint8_t button_index);

void convert_float_display(can_message_four* msg_in, can_message_four* msg_out, int decimal_points);

void send_CAN_message(uint16_t address, can_message_eight* msg);
void send_turn_on_inverter(void);
void send_velocity_ref_inverter(struct Throttle* th);
void send_throttle_steering_display(struct Throttle* th, struct SteeringAngle* sa);

// Throttle functions
void throttle_init(struct Throttle* thr);
void convert_adc_throttle(struct Throttle* th, uint16_t raw_adc_value);

// Steering angle functions
void steering_angle_init(struct SteeringAngle* sa);
void steering_angle_avg(struct SteeringAngle* sa, float value);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/

/* USER CODE BEGIN Private defines */
// Aswin throttle values (?)
#define SPEED_REFERENCE 1500.0f  // TODO: remove
#define MAX_RPM 1500.0f

// The macros below are to be used in the convert function
#define DECIMAL_POINT_0 1
#define DECIMAL_POINT_1 10
#define DECIMAL_POINT_2 100
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
