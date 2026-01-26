/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body for Heater Temperature Control (PI)
  ******************************************************************************
  */

/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdio.h>
#include <stdlib.h>
#include "bmp2_config.h"
#include "heater_config.h"

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
// --- ZMIENNE REGULATORA I PROCESU ---
float target_temp = 30.0f;     // Temperatura zadana [st. C] - startowa
float current_temp = 0.0f;     // Aktualna temperatura [st. C]
float pwm_duty = 0.0f;         // Wysterowanie grzałki (0.0 - 100.0 %)

// --- NASTAWY REGULATORA PI ---
// UWAGA: Te wartości mogą wymagać dostrojenia do Twojej konkretnej grzałki!
float Kp = 15.0f;              // Wzmocnienie Proporcjonalne (Reakcja na błąd teraz)
float Ki = 0.8f;               // Wzmocnienie Całkujące (Kasowanie uchybu ustalonego)

// --- PAMIĘĆ REGULATORA ---
float error_integral = 0.0f;   // Suma uchybów (Całka)

// --- KOMUNIKACJA UART ---
uint8_t tx_buffer[10];         // Bufor odbiorczy (np. na tekst "45.50")
const int tx_msg_len = 6;      // Ile znaków czytamy z terminala

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Przekierowanie printf na UART3 (dla telemetrii)
int _write(int file, char *ptr, int len)
{
  return (HAL_UART_Transmit(&huart3, (uint8_t*)ptr, len, HAL_MAX_DELAY) == HAL_OK) ? len : -1;
}

// Obsługa odbioru danych z UART (Zmiana temperatury zadanej)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
 if(huart == &huart3)
 {
  // 1. Konwersja odebranego tekstu na liczbę (float)
  float new_setpoint = atof((char*)tx_buffer);

  // 2. Walidacja danych (Safety First!)
  // Ograniczamy zakres, żeby nie wpisać przez pomyłkę 1000 stopni lub -200
  if(new_setpoint >= 20.0f && new_setpoint <= 40.0f)
  {
      target_temp = new_setpoint;

      // Opcjonalnie: Zerowanie całki przy dużej zmianie, żeby uniknąć przesterowania
      // error_integral = 0.0f;
  }

  // 3. Wyczyszczenie bufora (żeby stare śmieci nie zostały)
  for(int i=0; i<10; i++) tx_buffer[i] = 0;

  // 4. Ponowne włączenie nasłuchiwania
  HAL_UART_Receive_IT(&huart3, tx_buffer, tx_msg_len);
 }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART3_UART_Init();
  MX_SPI4_Init();
  MX_TIM2_Init();
  MX_TIM7_Init();

  /* USER CODE BEGIN 2 */

  // --- INICJALIZACJA SPRZĘTU ---
  printf("System Start...\r\n");

  // 1. Inicjalizacja czujnika BMP280
  int8_t bmp_status = BMP2_Init(&bmp2dev);
  if(bmp_status != 0) {
      printf("BMP280 Init Error: %d\r\n", bmp_status);
  }

  // 2. Inicjalizacja Grzałki (PWM)
  HEATER_PWM_Init(&hheater);

  // 3. Start Timera pętli sterowania (TIM7)
  HAL_TIM_Base_Start(&htim7);

  // 4. Start nasłuchu UART (Przerwania)
  HAL_UART_Receive_IT(&huart3, tx_buffer, tx_msg_len);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    // Pętla sterowania wyzwalana flagą Timera 7
    if(__HAL_TIM_GET_FLAG(&htim7, TIM_FLAG_UPDATE))
    {
      __HAL_TIM_CLEAR_FLAG(&htim7, TIM_FLAG_UPDATE);

      // --- KROK 1: POMIAR (Feedback) ---
      current_temp = BMP2_ReadTemperature_degC(&bmp2dev);

      // Zabezpieczenie awaryjne: jeśli czujnik zwróci błąd/zero absolutne
      if(current_temp < -40.0f || current_temp > 120.0f) {
          HEATER_PWM_WriteDuty(&hheater, 0.0f); // Wyłącz grzanie natychmiast
          printf("ERROR: Sensor Fail! Temp: %.2f\r\n", current_temp);
          continue; // Pomiń resztę pętli
      }

      // --- KROK 2: UCHYB (Error) ---
      float error = target_temp - current_temp;

      // --- KROK 3: CAŁKA (Integral) z Anti-Windup ---
      // Całkujemy tylko, jeśli sterowanie nie jest nasycone (nie jest 0% ani 100%)
      // To zapobiega "nakręcaniu się" całki, gdy grzałka i tak już grzeje na maksa.
      if(pwm_duty > 1.0f && pwm_duty < 99.0f) {
          error_integral += error;
      }

      // Dodatkowe, twarde ograniczenie całki (clamp)
      if(error_integral > 100.0f) error_integral = 100.0f;
      if(error_integral < -100.0f) error_integral = -100.0f;

      // --- KROK 4: WYLICZENIE WYJŚCIA (PI) ---
      float output = (Kp * error) + (Ki * error_integral);

      // --- KROK 5: NASYCENIE (Saturation) ---
      // PWM musi być w zakresie 0-100%
      if(output > 100.0f) output = 100.0f;
      if(output < 0.0f) output = 0.0f;

      pwm_duty = output;

      // --- KROK 6: STEROWANIE (Actuator) ---
      HEATER_PWM_WriteDuty(&hheater, pwm_duty);

      // --- KROK 7: TELEMETRIA (CSV) ---
      // Format: Target, Current, PWM
      // Idealny do podglądu w RealTerm lub SerialPlot
      printf("%.2f,%.2f,%.2f\r\n", target_temp, current_temp, pwm_duty);
    }

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
// Tutaj ewentualne dodatkowe funkcje
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
