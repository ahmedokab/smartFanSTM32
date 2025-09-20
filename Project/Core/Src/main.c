/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <math.h>
#include "./BME280/bme280.h"
#include "ssd1306.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BETA 3435.0      // typical value from Vishay NTC datasheet (B25/85)
#define T0   298.15      // 25 °C in Kelvin
#define R0   10000.0     // 10kΩ at 25°C
#define R_FIXED 10000.0  // the fixed series resistor in your divider

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

float read_temperature(void) {
    // Start ADC
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
    uint32_t adc_val = HAL_ADC_GetValue(&hadc1);

    // Converting ADC value recorded to voltage using formula
    float v_out = (adc_val / 4095.0) * 3.3;

    // Calculate NTC resistance, using resistance formula
    float r_ntc = (R_FIXED * v_out) / (3.3 - v_out);

    // using the beta steinhart eq
    float tempK = 1.0 / ((1.0/T0) + (1.0/BETA) * log(r_ntc / R0));
    float tempC = tempK - 273.15;

    return tempC; // returning temp in celsius because its better
}


float temperature;
float humidity;
float pressure;

struct bme280_dev dev;
struct bme280_data comp_data;
int8_t result;

char hum_string[32];
char temp_string[32];
char press_string[32];
char duty_string[32];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int8_t user_i2c_read(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  if(HAL_I2C_Master_Transmit(&hi2c1, (id << 1), &reg_addr, 1, HAL_MAX_DELAY) != HAL_OK) return -1;
  if(HAL_I2C_Master_Receive(&hi2c1, (id << 1), data, len, HAL_MAX_DELAY) != HAL_OK) return -1;
  return 0;
}

// I2C write function for BME280
int8_t user_i2c_write(uint8_t id, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  uint8_t buf[16];   // using static buffer instead of malloc to avoid memory leaks
  buf[0] = reg_addr;
  memcpy(&buf[1], data, len);

  if(HAL_I2C_Master_Transmit(&hi2c1, (id << 1), buf, len + 1, HAL_MAX_DELAY) != HAL_OK) return -1;
  return 0;
}

/* Delay function for BME280 driver */
void user_delay_ms(uint32_t period)
{
    HAL_Delay(period);
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
  int NTCMode = 0; //mode 1 is for ntc thermistor and voltage divider, mode 0 is for BME sensor. Both displayed on OLED

  result = bme280_init(&dev);
  if (result != BME280_OK) {
      printf("BME280 init failed: %d\r\n", result);
  }




  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  uint32_t duty = 0;


  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  dev.dev_id = BME280_I2C_ADDR_PRIM;
  dev.intf = BME280_I2C_INTF;
  dev.read = user_i2c_read;
  dev.write = user_i2c_write;
  dev.delay_ms = user_delay_ms;
  result = bme280_init(&dev);

  dev.settings.osr_h = BME280_OVERSAMPLING_1X;
   dev.settings.osr_p = BME280_OVERSAMPLING_16X;
   dev.settings.osr_t = BME280_OVERSAMPLING_2X;
   dev.settings.filter = BME280_FILTER_COEFF_16;
   result = bme280_set_sensor_settings(
       BME280_OSR_PRESS_SEL | BME280_OSR_TEMP_SEL |
       BME280_OSR_HUM_SEL | BME280_FILTER_SEL, &dev);

   SSD1306_Init();

   SSD1306_GotoXY(12, 12);
   SSD1306_Puts("Ahmed's", &Font_11x18, 1);
   SSD1306_GotoXY(10, 35);
   SSD1306_Puts("Weather Station", &Font_7x10, 1);
   SSD1306_UpdateScreen();
   HAL_Delay(2000);
   SSD1306_Clear();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // code utilizing NTC Thermistor


	  if(NTCMode){
      float temp = read_temperature();

      // Map temperature to fan duty
      uint32_t duty = 0;
      if (temp < 20) duty = 0;          // Fan OFF
      else if (temp < 24) duty = 500;   // 50% duty cycle, fan on half of the time
      else if (temp < 35) duty = 800;   // 70% duty cycle
      else duty = 1000;                 // 100%

      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);

      printf("Temp = %.2f °C, Fan Duty = %lu\r\n", temp, duty);
      sprintf(temp_string, "Temp: %.1f \xB0""C", temp);
      sprintf(duty_string, "Temp: %.1f \xB0""C", duty);

      HAL_Delay(1000); // 1 sec update

      // Draw to OLED
      SSD1306_Clear();
      SSD1306_GotoXY(0, 0);
      SSD1306_Puts(temp_string, &Font_7x10, 1);
      SSD1306_UpdateScreen();
    	      SSD1306_GotoXY(0, 20);
    	      SSD1306_Puts(duty, &Font_7x10, 1);

  	  }
	  else {
	      result = bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);
	      dev.delay_ms(100);
	      result = bme280_get_sensor_data(BME280_ALL, &comp_data, &dev);


	      // Convert fixed-point to float values
	      temperature = comp_data.temperature / 100.0f;
	      humidity    = comp_data.humidity    / 1024.0f;
	      pressure    = comp_data.pressure    / 10000.0f;

	      printf("BME280 Temp = %.2f °C, Fan Duty = %lu\r\n", temperature, duty);
	      sprintf(temp_string, "Temp: %.1f \xB0""C", temperature);
	      sprintf(hum_string,  "Hum:  %.1f %%", humidity);
	      sprintf(press_string,"Press:%.1f hPa", pressure);

	      // reaads temperature for BME280 this time
	      uint32_t duty = 0;

	      // Same fan control logic
	      if (temperature < 20) duty = 0;
	      else if (temperature < 24) duty = 500; // 50% duty cycle
	      else if (temperature < 35) duty = 800;
	      else duty = 1000;

	      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, duty);

	      // Draw to OLED
	      SSD1306_Clear();
	      SSD1306_GotoXY(0, 0);
	      SSD1306_Puts(temp_string, &Font_7x10, 1);
	      SSD1306_GotoXY(0, 20);
	      SSD1306_Puts(hum_string, &Font_7x10, 1);
	      SSD1306_GotoXY(0, 40);
	      SSD1306_Puts(press_string, &Font_7x10, 1);
	      SSD1306_UpdateScreen();
	      HAL_Delay(1000); // 1 sec update
	  }

	 // display on LED either way
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

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
