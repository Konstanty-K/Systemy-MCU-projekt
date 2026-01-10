/**
  ******************************************************************************
  * @file     : pwm.h
  * @author   : AW    Adrian.Wojcik@put.poznan.pl
  * @version  : 1.3.0
  * @date     : Nov 27, 2022
  * @brief    : Pulse Width Modulation outputs components driver.
  *
  ******************************************************************************
  */

#ifndef INC_PWM_H_
#define INC_PWM_H_

/* Public includes -----------------------------------------------------------*/
#ifdef USE_HAL_DRIVER
#include "stm32f7xx_hal.h"
#endif

/* Public typedef ------------------------------------------------------------*/
/**
  * @brief PWM output handle structure definition
  */
#ifdef USE_HAL_DRIVER
typedef TIM_HandleTypeDef* PWM_TIM_TypeDef;
typedef uint32_t PWM_Channel_TypeDef;
#endif

typedef struct {
  PWM_TIM_TypeDef Timer;
  PWM_Channel_TypeDef Channel;
  float Duty;
} PWM_Handle_TypeDef;

/* Public define -------------------------------------------------------------*/

/* Public macro --------------------------------------------------------------*/
#ifdef USE_HAL_DRIVER

/**
 * @brief PWM output handle structure initialization with TIM handle
 *        and channel label
 */
#define PWM_INIT_HANDLE(TIMER_HANDLE, CHANNEL) \
  {                                            \
    .Timer = TIMER_HANDLE,                     \
    .Channel = CHANNEL,                        \
    .Duty = 0.0f                               \
  }

/**
 * @brief PWM duty cycle (in percent) to CCR register conversion
 * @param[in] htim : TIM handle
 * @param[in] duty : duty cycle (in percent)
 * @retval Capture/Compare register value
 */
#define __DUTY_TO_COMPARE(htim, duty) \
  ((duty*(__HAL_TIM_GET_AUTORELOAD((htim))+1))/100)

/**
 * @brief PWM CCR register to duty cycle (in percent) conversion
 * @param[in] htim : TIM handle
 * @param[in] ch   : TIM channel
 * @retval Duty cycle (in percent)
 */
#define __COMPARE_TO_DUTY(htim, ch) \
  ((100*__HAL_TIM_GET_COMPARE((htim), (ch)))/(__HAL_TIM_GET_AUTORELOAD((htim))+1))

#endif

/* Public variables ----------------------------------------------------------*/

/* Public function prototypes ------------------------------------------------*/
/**
  * @brief Initialize PWM output
  * @param[in/out] hpwm   : PWM output handle
  * @retval None
  */
void PWM_Init(PWM_Handle_TypeDef* hpwm);

/**
  * @brief Write PWM duty cycle
  * @param[in/out] hpwm   : PWM output handle
  * @param[in]     duty   : PWM duty cycle in percents (0. - 100.)
  * @retval None
  */
void PWM_WriteDuty(PWM_Handle_TypeDef* hpwm, float duty);

/**
  * @brief Set PWM duty cycle
  * @param[in]     hpwm   : PWM output handle
  * @retval PWM duty cycle in percents (0. - 100.)
  */
float PWM_ReadDuty(const PWM_Handle_TypeDef* hpwm);

#endif /* INC_PWM_H_ */
