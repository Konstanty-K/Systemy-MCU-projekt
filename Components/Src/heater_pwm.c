/**
  ******************************************************************************
  * @file     : heater_pwm.c
  * @author   : AW    Adrian.Wojcik@put.poznan.pl
  * @version  : 1.3.0
  * @date     : Nov 27, 2022
  * @brief    : Electric heater components driver
  *
  ******************************************************************************
  */

/* Private includes ----------------------------------------------------------*/
#include "heater_pwm.h"

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Public variables ----------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Public function prototypes ------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Public functions ----------------------------------------------------------*/
/**
  * @brief Initialize PWM heater control
  * @param[in] hhtr   : Heater PWM handle
  * @retval None
  */
void HEATER_PWM_Init(HEATER_PWM_Handle_TypeDef* hhtr)
{
  hhtr->Output.Duty = (hhtr->ActiveState == HEATER_ON_HIGH) ? (hhtr->Output.Duty) : (100.0f - hhtr->Output.Duty);
  PWM_Init(&(hhtr->Output));
}

/**
  * @brief Write PWM duty cycle
  * @param[in/out] hhtr   : Heater PWM handle
  * @param[in]     duty   : PWM duty cycle in percents (0. - 100.)
  * @retval None
  */
void HEATER_PWM_WriteDuty(HEATER_PWM_Handle_TypeDef* hhtr, float duty)
{
  hhtr->Output.Duty = (hhtr->ActiveState == HEATER_ON_HIGH) ? (duty) : (100.0f - duty);
  PWM_WriteDuty(&(hhtr->Output), hhtr->Output.Duty);
}

/**
  * @brief Set PWM duty cycle
  * @param[in] hhtr   : Heater PWM handle
  * @retval PWM duty cycle in percents (0. - 100.)
  */
float HEATER_PWM_ReadDuty(const HEATER_PWM_Handle_TypeDef* hhtr)
{
  return (hhtr->ActiveState == HEATER_ON_HIGH) ? (hhtr->Output.Duty) : (100.0f - hhtr->Output.Duty);
}
