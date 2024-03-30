/*
 * stm32f4xx_gp_timer.c
 *
 *  Created on: Mar 30, 2024
 *      Author: Leonardo Marquez
 */

#include "stm32f4xx_gp_timer.h"


/*********************************************************************
 * @fn      		  - GP_TIM_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given TIMx
 *
 * @param[in]         - base address of the TIMx peripheral
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            - none
 *
 * @Note              - none
 */
void GP_TIM_PeriClockControl(GP_TIM_RegDef_t* pTIMx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		// ENABLE
		if (pTIMx == TIM2)
		{
			TIM2_PCLK_EN();
		}
		else if (pTIMx == TIM3)
		{
			TIM3_PCLK_EN();
		}
		else if (pTIMx == TIM4)
		{
			TIM4_PCLK_EN();
		}
		else if (pTIMx == TIM5)
		{
			TIM5_PCLK_EN();
		}
	}
	else
	{
		// DISABLE
		if (pTIMx == TIM2)
		{
			TIM2_PCLK_DI();
		}
		else if (pTIMx == TIM3)
		{
			TIM3_PCLK_DI();
		}
		else if (pTIMx == TIM4)
		{
			TIM4_PCLK_DI();
		}
		else if (pTIMx == TIM5)
		{
			TIM5_PCLK_DI();
		}
	}
}


/*********************************************************************
 * @fn      		  - GP_TIM_Init
 *
 * @brief             - This function initializes the timer
 *
 * @param[in]         - Handle structure for TIMx
 *
 * @return            - none
 *
 * @Note              - none
 */
void GP_TIM_Init(GP_TIM_Handle_t* pGP_TIM_Handle)
{
	// Enable the peripheral clock
	GP_TIM_PeriClockControl(pGP_TIM_Handle->pTIMx, ENABLE);

	// Select clock source
	if (pGP_TIM_Handle->TIMConfig.ClockSource == GP_TIM_CK_INT)
	{
		// Internal Clock
		// Set SMS=000 in the TIMx_SMCR register
		pGP_TIM_Handle->pTIMx->SMCR &= ~(0x7 << GP_TIM_SMCR_SMS);
	}
	else
	{
		// TODO
	}

	// Set pre-scaler
	pGP_TIM_Handle->pTIMx->PSC = pGP_TIM_Handle->TIMConfig.Prescaler;

	// Set Period (ARR)
	pGP_TIM_Handle->pTIMx->ARR = pGP_TIM_Handle->TIMConfig.Autorealod;

	// Select timer mode
	if (pGP_TIM_Handle->TIMConfig.TimerMode == GP_TIM_MODE_PWM)
	{
		// PWM mode
	}
}


/*********************************************************************
 * @fn      		  - GP_TIM_Start
 *
 * @brief             - This function starts the timer
 *
 * @param[in]         - Handle structure for TIMx
 *
 * @return            - none
 *
 * @Note              - none
 */
void GP_TIM_Start(GP_TIM_Handle_t* pGP_TIM_Handle)
{
	// Set CR1 CEN bit
	pGP_TIM_Handle->pTIMx->CR1 |= (1 << GP_TIM_CR1_CEN);
}
