/*
 * stm32f4xx_gp_timer.h
 *
 *  Created on: Mar 29, 2024
 *      Author: Leonardo Marquez
 */

#ifndef INC_STM32F4XX_GP_TIMER_H_
#define INC_STM32F4XX_GP_TIMER_H_

#include "stm32f407xx.h"

typedef struct
{
	uint8_t ClockSource;				// Values from @GP_TIM_CLK_SOURCE
	uint16_t Prescaler;					// The counter clock frequency CK_CNT is equal to fCK_PSC / (PSC[15:0] + 1)
	uint32_t Autorealod;				// Auto-reload register. 32-bit value on TIM2 and TIM5. 16-bit value on TIM3 and TIM4
	uint8_t TimerMode;					// Values from @GP_TIM_MODE
	uint8_t CounterMode;				// Values from @GP_TIM_CNT_MODE
	uint8_t Channel;					// Values from @GP_TIM_CHANNEL
	uint8_t PWMMode;					// Values from @GP_TIM_PWM_MODE
	uint32_t Pulse;						// Output Compare value. 32-bit value on TIM2 and TIM5. 16-bit value on TIM3 and TIM4
}GP_TIM_Config_t;

/*
 * Handle structure for General Purpose Timer
 */
typedef struct
{
	GP_TIM_RegDef_t* pTIMx;				// Base address of the TIMx peripheral
	GP_TIM_Config_t TIMConfig;			// Configuration items
}GP_TIM_Handle_t;


/******************************************************************************************
 *								Configuration Options
 ******************************************************************************************/

/*
 * Clock Source options
 * @GP_TIM_CLK_SOURCE
 */
#define GP_TIM_CK_INT						0	// Internal clock
#define GP_TIM_CK_TI						1	// External clock mode1: external input pin
#define GP_TIM_CK_ETR						2	// External clock mode2: external trigger input
#define GP_TIM_CK_ITR						3	// Internal trigger inputs


/*
 * Clock Source options
 * @GP_TIM_MODE
 */
#define GP_TIM_MODE_COUNTER					0	// Counter mode
#define GP_TIM_MODE_PWM						1	// PWM mode

/*
 * Clock Source options
 * @GP_TIM_CNT_MODE
 */
#define GP_TIM_CNT_MODE_UP					0	// Up-counting mode
#define GP_TIM_CNT_MODE_DOWN				1	// Down-counting mode
#define GP_TIM_CNT_MODE_CENTER_ALIGNED		2	// Center-aligned mode (up/down counting)


/*
 * Clock Channel selection
 * @GP_TIM_CHANNEL
 */
#define GP_TIM_CHAN_1						0	// TIMx CH1
#define GP_TIM_CHAN_2						1	// TIMx CH2
#define GP_TIM_CHAN_3						2	// TIMx CH3
#define GP_TIM_CHAN_4						3	// TIMx CH4


/*
 * PWM Mode Selection
 * @GP_TIM_PWM_MODE
 */
#define GP_TIM_PWM_MODE_1					0	// PWM Mode 1
#define GP_TIM_PWM_MODE_2					1	// PWM Mode 1

/******************************************************************************************
 *								Registers' Bit-Fields
 ******************************************************************************************/

/*
 * TIMx_SMCR Bit-fields
 */
#define GP_TIM_SMCR_ETP						15	// External trigger polarity
#define GP_TIM_SMCR_ECE						14	// External clock enable
#define GP_TIM_SMCR_ETPS					12	// External trigger prescaler
#define GP_TIM_SMCR_ETF						8	// External trigger filter
#define GP_TIM_SMCR_MSM						7	// Master/Slave mode
#define GP_TIM_SMCR_TS						4	// Trigger selection
#define GP_TIM_SMCR_SMS						0	// Slave mode selection


/*
 * TIMx_CR1 Bit-fields
 */
#define GP_TIM_CR1_CKD						8	// Clock division
#define GP_TIM_CR1_ARPE						7	// Auto-reload preload enable
#define GP_TIM_CR1_CMS						5	// Center-aligned mode selection
#define GP_TIM_CR1_DIR						4	// Direction
#define GP_TIM_CR1_OPM						3	// One-pulse mode
#define GP_TIM_CR1_URS						2	// Update request source
#define GP_TIM_CR1_UDIS						1	// Update disable
#define GP_TIM_CR1_CEN						0	// Counter enable


/*
 * TIMx_EGR Bit-fields
 */
#define GP_TIM_EGR_TG						6	// Trigger generation
#define GP_TIM_EGR_CC4G						4	// Capture/compare 4 generation
#define GP_TIM_EGR_CC3G						3	// Capture/compare 3 generation
#define GP_TIM_EGR_CC2G						2	// Capture/compare 2 generation
#define GP_TIM_EGR_CC1G						1	// Capture/compare 1 generation
#define GP_TIM_EGR_UG						0	// Update generation


/*
 * TIMx_CCMR1 Bit-fields
 */
#define  GP_TIM_CCMR1_OC2CE					15	// Output compare 2 clear enable
#define  GP_TIM_CCMR1_OC2M					12	// Output compare 2 mode
#define  GP_TIM_CCMR1_OC2PE					11	// Output compare 2 preload enable
#define  GP_TIM_CCMR1_OC2FE					10	// Output compare 2 fast enable
#define  GP_TIM_CCMR1_CC2S					8	// Capture/Compare 2 selection
#define  GP_TIM_CCMR1_OC1CE					7	// Output compare 1 clear enable
#define  GP_TIM_CCMR1_OC1M					4	// Output compare 1 mode
#define  GP_TIM_CCMR1_OC1PE					3	// Output compare 1 preload enable
#define  GP_TIM_CCMR1_OC1FE					2	// Output compare 1 fast enable
#define  GP_TIM_CCMR1_CC1S					0	// Capture/Compare 1 selection


/*
 * TIMx_CCMR2 Bit-fields
 */
#define  GP_TIM_CCMR2_OC4CE					15	// Output compare 2 clear enable
#define  GP_TIM_CCMR2_OC4M					12	// Output compare 2 mode
#define  GP_TIM_CCMR2_OC4PE					11	// Output compare 2 preload enable
#define  GP_TIM_CCMR2_OC4FE					10	// Output compare 2 fast enable
#define  GP_TIM_CCMR2_CC4S					8	// Capture/Compare 2 selection
#define  GP_TIM_CCMR2_OC3CE					7	// Output compare 1 clear enable
#define  GP_TIM_CCMR2_OC3M					4	// Output compare 1 mode
#define  GP_TIM_CCMR2_OC3PE					3	// Output compare 1 preload enable
#define  GP_TIM_CCMR2_OC3FE					2	// Output compare 1 fast enable
#define  GP_TIM_CCMR2_CC3S					0	// Capture/Compare 1 selection


/*
 * TIMx_CCER Bit-fields
 */
#define GP_TIM_CCER_CC4NP					15	// Capture/Compare 4 output Polarity
#define GP_TIM_CCER_CC4P					13	// Capture/Compare 4 output Polarity
#define GP_TIM_CCER_CC4E					12	// Capture/Compare 4 output enable
#define GP_TIM_CCER_CC3NP					11	// Capture/Compare 3 output Polarity
#define GP_TIM_CCER_CC3P					9	// Capture/Compare 3 output Polarity
#define GP_TIM_CCER_CC3E					8	// Capture/Compare 3 output enable
#define GP_TIM_CCER_CC2NP					7	// Capture/Compare 2 output Polarity
#define GP_TIM_CCER_CC2P					5	// Capture/Compare 2 output Polarity
#define GP_TIM_CCER_CC2E					4	// Capture/Compare 2 output enable
#define GP_TIM_CCER_CC1NP					3	// Capture/Compare 1 output Polarity
#define GP_TIM_CCER_CC1P					1	// Capture/Compare 1 output Polarity
#define GP_TIM_CCER_CC1E					0	// Capture/Compare 1 output enable

/******************************************************************************************
 *								APIs supported by this driver
 *		 For more information about the APIs check the function definitions
 ******************************************************************************************/

/*
 * Peripheral Clock setup
 */
void GP_TIM_PeriClockControl(GP_TIM_RegDef_t* pTIMx, uint8_t EnorDi);

/*
 * Init and De-init
 */
void GP_TIM_Init(GP_TIM_Handle_t* pGP_TIM_Handle);
void GP_TIM_DeInit(GP_TIM_Handle_t* pGP_TIM_Handle);

/*
 * Start and stop counter
 */
void GP_TIM_Start(GP_TIM_Handle_t* pGP_TIM_Handle);
void GP_TIM_Stop(GP_TIM_Handle_t* pGP_TIM_Handle);

/*
 * Manipulate some timer settings
 */
void GP_TIM_SetPWMPulseWidth(GP_TIM_Handle_t* pGP_TIM_Handle, uint32_t pulse);


#endif /* INC_STM32F4XX_GP_TIMER_H_ */
