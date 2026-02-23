/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "eth.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"
#include <stdio.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hc_sr_04.h"
#include "i2c_lcd.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct us_sensor_str distance_sensor;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

void distance_display(uint32_t *giv_dist, uint32_t *acc_dist){
	lcd_clear();
	lcd_put_cursor(0,0);
	lcd_send_string("G.D.:");
	char *lcdout = (char*)giv_dist;
	lcd_put_cursor(0,6);
	lcd_send_string(lcdout);
	lcd_put_cursor(0,10);
	lcd_send_string("cm");

	lcd_put_cursor(1,0);
	lcd_send_string("A.D.:");

	lcd_put_cursor(1,10);
	lcd_send_string("cm");

	char buffer[12];
	itoa(*acc_dist, buffer, 10);
	lcd_put_cursor(1,6);
	lcd_send_string("   ");
	lcd_put_cursor(1,6);
	lcd_send_string(buffer);
	HAL_Delay(150);
}
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t rx_indx;
uint8_t rx_data[3];
float giv_dist;
uint8_t rx_buffer[100];
uint8_t transfer_cplt;
float distances;

#define PWM_MAX 7500
#define MIN_PWM 2400
#define TOLERANCE_CM 3.0f
typedef struct {
	float Kp;
	float Kd;
	float target_distance;
	float prev_error;
	uint32_t last_time;
} PD_Controller;

PD_Controller robotPD = {0};
int32_t current_pwm_output = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

float uint8_to_float_ascii(uint8_t bytes[3]) {
    // Assuming bytes contain ASCII digits: '3', '0', '0'
	if(bytes[0] != '.' && bytes[1] != '.' && bytes[2] != '.'){
		int value = (bytes[0] - '0') * 100 +
                	(bytes[1] - '0') * 10 +
					(bytes[2] - '0');
		return (float)value;
	}
	else if(bytes[2] == '.'){
		int value = (bytes[0] - '0') * 10 + (bytes[1] - '0') ;
		return (float)value;
	}
	else if(bytes[1] == '.'){
		int value = (bytes[0] - '0');
		return (float)value;
	}
}

void SetRobotSpeed(int32_t speed)
{
	if (speed > PWM_MAX) speed = PWM_MAX;
	if (speed < -PWM_MAX) speed = -PWM_MAX;

	if (speed > 0 && speed < MIN_PWM) speed = MIN_PWM;
	if (speed < 0 && speed > -MIN_PWM) speed = -MIN_PWM;

	if (speed == 0) {
	        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
	        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
	        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
	        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
	        return;
	    }

	if (speed > 0) {
	        // Przód
	        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, speed); // L Przód
	        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 0);
	        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, speed); // R Przód
	        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 0);
	    } else {
	        // Tył
	        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
	        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, -speed);// L Tył
	        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 0);
	        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, -speed);// R Tył
	    }
}

// Obliczanie wartości PWM na podstawie odległości
float Calculate_PD(PD_Controller *pd, uint32_t *current_distance)
{
    uint32_t now = HAL_GetTick();
    float dt = (now - pd->last_time) / 1000.0f; // Czas w sekundach
    if (dt <= 0.0f) dt = 0.001f;

    // Błąd = aktualna odległość - pożądana odległość
    float current_distance_f = *current_distance;
    float error = current_distance_f  - pd->target_distance;

    if (error > -TOLERANCE_CM && error < TOLERANCE_CM){
    	pd->prev_error = error;
		pd->last_time = now;

		return 0.0f;
    }

    // Pochodna = jak szybko zmienia się błąd
    float derivative = (error - pd->prev_error) / dt;

    // Wynik PD
    float output = (pd->Kp * error) + (pd->Kd * derivative);

    pd->prev_error = error;
    pd->last_time = now;

    return output;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ETH_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  hc_sr04_init(&distance_sensor, &htim1, &htim2, TIM_CHANNEL_3);

  lcd_init();

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

  robotPD.Kp = 170.0f;             // Modyfikuj w Live Expressions!
  robotPD.Kd = 20.0f;              // Modyfikuj w Live Expressions!
  robotPD.target_distance = 20.0f;   // TU WKLEIĆ TARGET OD OSKARA CHYBA
  robotPD.last_time = HAL_GetTick();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  static uint32_t last_pd_calc_time = 0;

	  HAL_UART_Receive_IT(&huart2, rx_data, 3);
	  giv_dist = uint8_to_float_ascii(rx_data);

	  robotPD.target_distance = giv_dist;


	  if (HAL_GetTick() - last_pd_calc_time > 50)
	       {
	           last_pd_calc_time = HAL_GetTick();

	           // 1. Oblicz nowe PWM na podstawie zmiennej z czujnika
	           float calculated_pwm = Calculate_PD(&robotPD, &distance_sensor.distance_cm);

	           // 2. Przekaż do silników
	           current_pwm_output = (int32_t)calculated_pwm;
	           SetRobotSpeed(current_pwm_output);
	       }

	  distance_display(rx_data, &distance_sensor.distance_cm);

	  distances = (float)distance_sensor.distance_cm;

	  char wifi_buffer[50];
	  sprintf(wifi_buffer, "%.1f\n", distances);
	  HAL_UART_Transmit(&huart2, (uint8_t*)wifi_buffer, strlen(wifi_buffer), 1000);


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 6;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(TIM1 == htim->Instance)
	{
		uint32_t echo_us;

		echo_us = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
		distance_sensor.distance_cm = hc_sr04_convert_us_to_cm(echo_us);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /* Prevent unused argument(s) compilation warning */
  UNUSED(huart);

  /* NOTE : This function should not be modified, when the callback is needed,
            the HAL_UART_RxCpltCallback can be implemented in the user file.
   */
  	  HAL_UART_Transmit(&huart3, rx_data, 8, 10);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
