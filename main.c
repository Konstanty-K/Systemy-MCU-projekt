/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body (AUTO-RESTART UART)
  ******************************************************************************
  */
#define TASK 5
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "adc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#if TASK == 5
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "bmp2_config.h"
#include "heater_config.h"
#endif
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#if TASK == 5
extern ADC_HandleTypeDef hadc1;

// --- ZMIENNE PROCESU ---
float target_temp = 25.0f;
float current_temp = 0.0f;
float pwm_duty = 0.0f;

// --- PID ---
float Kp = 9.0f;
float Ki = 0.7f;
float error_integral = 0.0f;

// --- POTENCJOMETR ---
float last_pot_temp = 0.0f;
const float POT_THRESHOLD = 0.5f;

// --- KOMUNIKACJA ---
uint8_t rx_byte;
uint8_t rx_buffer[16];
int rx_index = 0;
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#if TASK == 5

int _write(int file, char *ptr, int len)
{
  return (HAL_UART_Transmit(&huart3, (uint8_t*)ptr, len, HAL_MAX_DELAY) == HAL_OK) ? len : -1;
}

// Obsługa błędów UART (restartuje nasłuch)
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart3)
    {
        // Wyczyszczenie flag błędów (Overrun, Noise, Frame)
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);

        // Restart
        HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
 if(huart == &huart3)
 {
   if (rx_byte == '\n' || rx_byte == '\r')
   {
       rx_buffer[rx_index] = 0;
       if(rx_index > 0)
       {
           float val = atof((char*)rx_buffer);
           if(val < 25.0f) val = 25.0f;
           if(val > 40.0f) val = 40.0f;
           target_temp = val;

           // DEBUG: Jeśli to zadziała, zobaczysz to w RealTerm
           printf("[UART OK] Set: %.2f\r\n", target_temp);
       }
       rx_index = 0;
   }
   else
   {
       if(rx_index < 15) rx_buffer[rx_index++] = rx_byte;
   }
   HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
 }
}
#endif
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_SPI4_Init();
  MX_TIM2_Init();
  MX_TIM7_Init();
  MX_ADC1_Init();

  /* USER CODE BEGIN 2 */
  #if TASK == 5
  printf("\r\n=== SYSTEM START ===\r\n");
  BMP2_Init(&bmp2dev);
  HEATER_PWM_Init(&hheater);
  HAL_TIM_Base_Start(&htim7);

  // Wymuszenie czystego startu UART (usunięcie śmieci z bufora)
  __HAL_UART_CLEAR_OREFLAG(&huart3);
  HAL_UART_Receive_IT(&huart3, &rx_byte, 1);

  // Kalibracja potencjometru
  HAL_ADC_Start(&hadc1);
  if(HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
      uint32_t raw = HAL_ADC_GetValue(&hadc1);
      last_pot_temp = 25.0f + ((float)raw / 4095.0f) * 15.0f;
  }
  HAL_ADC_Stop(&hadc1);

  int uart_check_counter = 0;
  #endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    #if TASK == 5
    // MECHANIZM SAMONAPRAWY UART (Sprawdzamy co 1000 pętli)
    uart_check_counter++;
    if(uart_check_counter > 100000) {
        // Jeśli UART nie jest w stanie BUSY_RX, to znaczy że przestał słuchać -> Restartujemy go
        if(HAL_UART_GetState(&huart3) == HAL_UART_STATE_READY) {
             HAL_UART_Receive_IT(&huart3, &rx_byte, 1);
        }
        uart_check_counter = 0;
    }

    if(__HAL_TIM_GET_FLAG(&htim7, TIM_FLAG_UPDATE))
    {
      __HAL_TIM_CLEAR_FLAG(&htim7, TIM_FLAG_UPDATE);

      // --- 1. POTENCJOMETR ---
      HAL_ADC_Start(&hadc1);
      if (HAL_ADC_PollForConversion(&hadc1, 2) == HAL_OK)
      {
          uint32_t adc_val = HAL_ADC_GetValue(&hadc1);
          float pot_temp = 25.0f + ((float)adc_val / 4095.0f) * 15.0f;

          if (fabs(pot_temp - last_pot_temp) > POT_THRESHOLD)
          {
              target_temp = pot_temp;
              last_pot_temp = pot_temp;
              printf("[POT] Change: %.2f\r\n", target_temp);
          }
      }
      HAL_ADC_Stop(&hadc1);

      // --- 2. PID ---
      current_temp = BMP2_ReadTemperature_degC(&bmp2dev);
      if(current_temp < -40.0f || current_temp > 125.0f) {
          HEATER_PWM_WriteDuty(&hheater, 0.0f);
          continue;
      }

      float error = target_temp - current_temp;
      if(pwm_duty > 1.0f && pwm_duty < 99.0f) error_integral += error;
      if(error_integral > 100.0f) error_integral = 100.0f;
      if(error_integral < -100.0f) error_integral = -100.0f;

      float output = (Kp * error) + (Ki * error_integral);
      if(output > 100.0f) output = 100.0f;
      if(output < 0.0f) output = 0.0f;
      pwm_duty = output;

      HEATER_PWM_WriteDuty(&hheater, pwm_duty);

      // --- 3. TELEMETRIA ---
      // Czyste CSV
      printf("%.2f,%.2f,%.2f\r\n", target_temp, current_temp, pwm_duty);
    }
    #endif
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */
