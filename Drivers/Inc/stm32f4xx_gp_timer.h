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
	uint32_t Period;					// Auto-reload register. 32-bit value on TIM2 and TIM5. 16-bit value on TIM3 and TIM4
	uint8_t TimerMode;					// Values from @GP_TIM_MODE
	uint8_t CounterMode;				// Values from @GP_TIM_CNT_MODE
}GP_TIM_Config_t;

/*
 * Handle structure for General Purpose Timer
 */
typedef struct
{
	GP_TIM_RegDef_t* pTIMx;				// Base address of the TIMx peripheral

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


#endif /* INC_STM32F4XX_GP_TIMER_H_ */
