/**
  ******************************************************************************
  * @file		  : heater_pwm.h
  * @author  	: AW		Adrian.Wojcik@put.poznan.pl
  * @version 	: 1.3.0
  * @date     : Nov 27, 2022
  * @brief    : Electric heater components driver
  *
  ******************************************************************************
  */
#ifndef INC_HEATER_PWM_H_
#define INC_HEATER_PWM_H_

/* Public includes -----------------------------------------------------------*/
#include "pwm.h"

/* Public typedef ------------------------------------------------------------*/
typedef enum { HEATER_ON_LOW, HEATER_ON_HIGH } HEATER_ActiveState_TypeDef;

/**
  * @brief PWM output (0.-100.%) HEATER handle structure definition
  */
typedef struct {
  PWM_Handle_TypeDef Output;
  HEATER_ActiveState_TypeDef ActiveState;
} HEATER_PWM_Handle_TypeDef;

/* Public define -------------------------------------------------------------*/

/* Public macro --------------------------------------------------------------*/

/**
  * @brief @brief PWM output (0.-100.%) heater handle structure initialization
  * @param[in] TIMER        : TIM handle
  *       This parameter can be one of the following values:
  *            @arg &htim1  : TIM1 (Advanced control timer, 6 channels)
  *            @arg &htim2  : TIM2 (General purpose timer, 4 channels)
  *            @arg &htim3  : TIM3 (General purpose timer, 4 channels)
  *            @arg &htim4  : TIM4 (General purpose timer, 4 channels)
  *            @arg &htim5  : TIM5 (General purpose timer, 4 channels)
  *            @arg &htim8  : TIM8 (Advanced control timer, 6 channels)
  *            @arg &htim9  : TIM9 (General purpose timer, 2 channels)
  *            @arg &htim10 : TIM10 (General purpose timer, 1 channel)
  *            @arg &htim11 : TIM11 (General purpose timer, 1 channel)
  *            @arg &htim12 : TIM12 (General purpose timer, 2 channels)
  *            @arg &htim13 : TIM13 (General purpose timer, 1 channel)
  *            @arg &htim14 : TIM14 (General purpose timer, 1 channel)
  * @param[in] CHANNEL      : TIM Channel
  *       This parameter can be one of the following values:
  *            @arg TIM_CHANNEL_1: TIM Channel 1 selected
  *            @arg TIM_CHANNEL_2: TIM Channel 2 selected
  *            @arg TIM_CHANNEL_3: TIM Channel 3 selected
  *            @arg TIM_CHANNEL_4: TIM Channel 4 selected
  *            @arg TIM_CHANNEL_5: TIM Channel 5 selected
  *            @arg TIM_CHANNEL_6: TIM Channel 6 selected
  * @param[in] ACTIVE_STATE : Active state (polarity) of HEATER
  *       This parameter can be one of the following values:
  *            @arg HEATER_ON_LOW  : Heater turns on if output state is 0
  *            @arg HEATER_ON_HIGH : Heater turns on if output state is 1
  */
#define HEATER_PWM_INIT_HANDLE(TIMER, CHANNEL, ACTIVE_STATE)  \
  {                                                        \
    .Output = PWM_INIT_HANDLE((TIMER), CHANNEL),           \
    .ActiveState = ACTIVE_STATE                            \
  }

/* Public variables ----------------------------------------------------------*/

/* Public function prototypes ------------------------------------------------*/
/**
  * @brief Initialize PWM heater control
  * @param[in/out] hhtr   : Heater PWM handle
  * @retval None
  */
void HEATER_PWM_Init(HEATER_PWM_Handle_TypeDef* hhtr);

/**
  * @brief Write PWM duty cycle
  * @param[in/out] hhtr   : Heater PWM handle
  * @param[in]     duty   : PWM duty cycle in percents (0. - 100.)
  * @retval None
  */
void HEATER_PWM_WriteDuty(HEATER_PWM_Handle_TypeDef* hhtr, float duty);

/**
  * @brief Set PWM duty cycle
  * @param[in] hhtr   : Heater PWM handle
  * @retval PWM duty cycle in percents (0. - 100.)
  */
float HEATER_PWM_ReadDuty(const HEATER_PWM_Handle_TypeDef* hhtr);

#endif /* INC_HEATER_PWM_H_ */
