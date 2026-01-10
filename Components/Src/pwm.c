/**
 ******************************************************************************
 * @file    pwm.c
 * @author  AW (Adrian.Wojcik@put.poznan.pl)
 * @version 1.3.0
 * @date    Nov 27, 2022
 * @brief   Pulse Width Modulation outputs components driver implementation
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "pwm.h"
#include "stm32f7xx_hal_tim.h"


/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
#define DUTY_TO_COMPARE(htim, duty) ((uint32_t)(((htim)->Instance->ARR + 1U) * (duty) / 100U))

/* Private variables ---------------------------------------------------------*/

/* Public variables ----------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/* Public functions ----------------------------------------------------------*/

/**
 * @brief  Initialize PWM output
 * @param  [in/out] hpwm: PWM output handle
 * @retval None
 */
void PWM_Init(PWM_Handle_TypeDef *hpwm)
{
    if (hpwm == NULL)
        return;

    HAL_TIM_PWM_Start(hpwm->Timer, hpwm->Channel);

    hpwm->Duty = 0.0f;

    __HAL_TIM_SET_COMPARE(hpwm->Timer, hpwm->Channel, 0);
}

/**
 * @brief  Write PWM duty cycle
 * @param  [in/out] hpwm: PWM output handle
 * @param  [in]     duty: PWM duty cycle in percents (0.0 - 100.0)
 * @retval None
 */
void PWM_WriteDuty(PWM_Handle_TypeDef *hpwm, float duty)
{
    if (hpwm == NULL)
        return;

    if (duty < 0.0f)
        duty = 0.0f;
    if (duty > 100.0f)
        duty = 100.0f;

    hpwm->Duty = duty;

    uint32_t compare_value = DUTY_TO_COMPARE(hpwm->Timer, duty);
    __HAL_TIM_SET_COMPARE(hpwm->Timer, hpwm->Channel, compare_value);
}


/**
 * @brief  Read PWM duty cycle
 * @param  [in] hpwm: PWM output handle
 * @retval PWM duty cycle in percents (0.0 - 100.0)
 */
float PWM_ReadDuty(const PWM_Handle_TypeDef *hpwm)
{
    if (hpwm == NULL)
        return 0.0f;

    return hpwm->Duty;
}
