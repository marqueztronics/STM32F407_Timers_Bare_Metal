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

		// Select channel
		switch (pGP_TIM_Handle->TIMConfig.Channel)
		{
		case GP_TIM_CHAN_1:
			// Write Pulse value into CCR1 register Register
			pGP_TIM_Handle->pTIMx->CCR1 = pGP_TIM_Handle->TIMConfig.Pulse;

			// Select PWM mode by writing OC1M bits in the TIMx_CCMR1 register
			pGP_TIM_Handle->pTIMx->CCMR1 &= ~(0x7 << GP_TIM_CCMR1_OC1M);	// Clear bits

			if (pGP_TIM_Handle->TIMConfig.PWMMode == GP_TIM_PWM_MODE_1)
			{
				pGP_TIM_Handle->pTIMx->CCMR1 |= (0x6 << GP_TIM_CCMR1_OC1M);	// Write "110"
			}
			else if (pGP_TIM_Handle->TIMConfig.PWMMode == GP_TIM_PWM_MODE_2)
			{
				pGP_TIM_Handle->pTIMx->CCMR1 |= (0x7 << GP_TIM_CCMR1_OC1M);	// Write "111"
			}

			// Enable the capture/compare preload register by setting the OC1PE bit in the TIMx_CCMR1 register
			pGP_TIM_Handle->pTIMx->CCMR1 |= (1 << GP_TIM_CCMR1_OC1PE);

			// C1 output is enabled by the CC1E bit in the TIMx_CCER register
			pGP_TIM_Handle->pTIMx->CCER |= (1 << GP_TIM_CCER_CC1E);

			break;

		case GP_TIM_CHAN_2:
			// Write Pulse value into CCR2 register Register
			pGP_TIM_Handle->pTIMx->CCR2 = pGP_TIM_Handle->TIMConfig.Pulse;

			// Select PWM mode by writing OC2M bits in the TIMx_CCMR1 register
			pGP_TIM_Handle->pTIMx->CCMR1 &= ~(0x7 << GP_TIM_CCMR1_OC2M);	// Clear bits

			if (pGP_TIM_Handle->TIMConfig.PWMMode == GP_TIM_PWM_MODE_1)
			{
				pGP_TIM_Handle->pTIMx->CCMR1 |= (0x6 << GP_TIM_CCMR1_OC2M);	// Write "110"
			}
			else if (pGP_TIM_Handle->TIMConfig.PWMMode == GP_TIM_PWM_MODE_2)
			{
				pGP_TIM_Handle->pTIMx->CCMR1 |= (0x7 << GP_TIM_CCMR1_OC2M);	// Write "111"
			}

			// Enable the capture/compare preload register by setting the OC1PE bit in the TIMx_CCMR1 register
			pGP_TIM_Handle->pTIMx->CCMR1 |= (1 << GP_TIM_CCMR1_OC2PE);

			// C2 output is enabled by the CC2E bit in the TIMx_CCER register
			pGP_TIM_Handle->pTIMx->CCER |= (1 << GP_TIM_CCER_CC2E);

			break;

		case GP_TIM_CHAN_3:
			// Write Pulse value into CCR3 register Register
			pGP_TIM_Handle->pTIMx->CCR3 = pGP_TIM_Handle->TIMConfig.Pulse;

			// Select PWM mode by writing OC3M bits in the TIMx_CCMR2 register
			pGP_TIM_Handle->pTIMx->CCMR2 &= ~(0x7 << GP_TIM_CCMR2_OC3M);	// Clear bits

			if (pGP_TIM_Handle->TIMConfig.PWMMode == GP_TIM_PWM_MODE_1)
			{
				pGP_TIM_Handle->pTIMx->CCMR2 |= (0x6 << GP_TIM_CCMR2_OC3M);	// Write "110"
			}
			else if (pGP_TIM_Handle->TIMConfig.PWMMode == GP_TIM_PWM_MODE_2)
			{
				pGP_TIM_Handle->pTIMx->CCMR2 |= (0x7 << GP_TIM_CCMR2_OC3M);	// Write "111"
			}

			// Enable the capture/compare preload register by setting the OC3PE bit in the TIMx_CCMR1 register
			pGP_TIM_Handle->pTIMx->CCMR2 |= (1 << GP_TIM_CCMR2_OC3PE);

			// C3 output is enabled by the CC3E bit in the TIMx_CCER register
			pGP_TIM_Handle->pTIMx->CCER |= (1 << GP_TIM_CCER_CC3E);

			break;

		case GP_TIM_CHAN_4:
			// Write Pulse value into CCR4 register Register
			pGP_TIM_Handle->pTIMx->CCR4 = pGP_TIM_Handle->TIMConfig.Pulse;

			// Select PWM mode by writing OC4M bits in the TIMx_CCMR2 register
			pGP_TIM_Handle->pTIMx->CCMR2 &= ~(0x7 << GP_TIM_CCMR2_OC4M);	// Clear bits

			if (pGP_TIM_Handle->TIMConfig.PWMMode == GP_TIM_PWM_MODE_1)
			{
				pGP_TIM_Handle->pTIMx->CCMR2 |= (0x6 << GP_TIM_CCMR2_OC4M);	// Write "110"
			}
			else if (pGP_TIM_Handle->TIMConfig.PWMMode == GP_TIM_PWM_MODE_2)
			{
				pGP_TIM_Handle->pTIMx->CCMR2 |= (0x7 << GP_TIM_CCMR2_OC4M);	// Write "111"
			}

			// Enable the capture/compare preload register by setting the OC3PE bit in the TIMx_CCMR1 register
			pGP_TIM_Handle->pTIMx->CCMR2 |= (1 << GP_TIM_CCMR2_OC4PE);

			// C4 output is enabled by the CC4E bit in the TIMx_CCER register
			pGP_TIM_Handle->pTIMx->CCER |= (1 << GP_TIM_CCER_CC4E);
			break;
		}

		// Set the ARPE bit in the TIMx_CR1 register
		pGP_TIM_Handle->pTIMx->CR1 |= (1 << GP_TIM_CR1_ARPE);

		// Initialize all the registers by setting the UG bit in the TIMx_EGR register
		pGP_TIM_Handle->pTIMx->EGR |= (1 << GP_TIM_EGR_UG);
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
	// Set TIMx_CR1 CEN bit
	pGP_TIM_Handle->pTIMx->CR1 |= (1 << GP_TIM_CR1_CEN);
}


/*********************************************************************
 * @fn      		  - GP_TIM_Stop
 *
 * @brief             - This function stops the timer
 *
 * @param[in]         - Handle structure for TIMx
 *
 * @return            - none
 *
 * @Note              - none
 */
void GP_TIM_Stop(GP_TIM_Handle_t* pGP_TIM_Handle)
{
	// Reset TIMx_CR1 CEN bit
	pGP_TIM_Handle->pTIMx->CR1 &= ~(1 << GP_TIM_CR1_CEN);
}
