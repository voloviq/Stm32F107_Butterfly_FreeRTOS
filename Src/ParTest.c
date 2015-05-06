/*
    FreeRTOS V8.2.1 - Copyright (C) 2015 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>!AND MODIFIED BY!<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/*-----------------------------------------------------------
 * Simple parallel port IO routines.
 *-----------------------------------------------------------*/

/* FreeRTOS.org includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "partest.h"

/* Standard includes. */
#include <string.h>

/* Library includes. */
#include "stm32f10x_conf.h"
#include "stm32f10x_lib.h"

//xxx#define partstNUM_LEDs		4

/* Holds the current output state for each of the LEDs. */
//xxxstatic unsigned char ucBitStates[ partstNUM_LEDs ];

/* Holds the port used by each of the LEDs. */
//xxxstatic GPIO_TypeDef * uxIO_Port[ partstNUM_LEDs ];

/* Holds the pin used by each of the LEDs. */
//xxxstatic const unsigned short uxIO_Pins[ partstNUM_LEDs ] = { GPIO_Pin_14, GPIO_Pin_13, GPIO_Pin_3, GPIO_Pin_4 };

/*-----------------------------------------------------------*/

void vParTestInitialise( void )
{
    // Connect perypherial GPIOE to global clocking system
    RCC->APB2ENR |= RCC_APB2Periph_GPIOE;

    // Connect perypherial GPIOE to global clocking system
    //xxxRCC->APB2ENR |= RCC_APB2Periph_GPIOA;

    // Connect perypherial GPIOE to global clocking system
    //xxxRCC->APB2ENR |= RCC_APB2Periph_GPIOC;


    // Configure choosed pin to choosed destiny

    // Warning:
    // To configure pin form range Px_0 to Px_7 You must
    // use GPIOx->CRL, for Px_8 to Px_15 You must
    // use GPIOx->CRH.

    // Two register is used to configure all gpio
    // GPIOx->CRL and GPIOx->CRH

    // Description of layout registers GPIOx->CRL and GPIOx->CRH
    // Register GPIOx->CRL have a MODE0 and CNFR0 to MODE7 and CNFR7
    // Register GPIOx->CRH have a MODE8 and CNFR8 to MODE15 and CNFR15

    // Reset value of this register is 0x44444444 both it's float input

    // Low value setup MODEx[1:0]
    //------------------------------------------------------------------
    // xx00 - Input mode (reset state)
    // xx01 - Output mode, max speed 10MHz
    // xx10 - Output mode, max speed 2MHz
    // xx11 - Output mode, max speed 50MHz
    //------------------------------------------------------------------

    // High value setup CNFx[1:0]
    //------------------------------------------------------------------
    // if You setup mode on 0x00
        // 00xx - Analog input mode
        // 01xx - Floating input (reset state)
        // 10xx - Input with pull-up / pull down
        // 11xx - Reserved
    // in other case
        // 00xx - General purpose output push-pull
        // 01xx - General purpose output Open-drain
        // 10xx - Alternate function with output push-pull
        // 11xx - Alternate function with output Open-drain
    //------------------------------------------------------------------


    // Warning: Gpio is more than 7 in bits You must use CRH reg.

    // GPIOE pin 14 na outpur push pullup Led colour red Kamami
    GPIOE->CRH |= 0x01<<(6*4);
    GPIOE->CRH &= ~(0x04<<(6*4));

    // GPIOE pin 15 na outpur push pullup Led colour red MCBSTM32
    GPIOE->CRH |= 0x01<<(7*4);
    GPIOE->CRH &= ~(0x04<<(7*4));

    // GPIOB pin 10 na outpur push pullup Led colour green MCBSTM32
	//xxxGPIOB->CRH |= 0x01<<(2*4);
	//xxxGPIOB->CRH &= ~(0x04<<(2*4));

	// GPIOB pin 11 na outpur push pullup Led colour green MCBSTM32
	//xxxGPIOB->CRH |= 0x01<<(3*4);
    //xxxGPIOB->CRH &= ~(0x04<<(3*4));

	// GPIOB pin 12 na outpur push pullup Led colour green MCBSTM32
    //xxxGPIOB->CRH |= 0x01<<(4*4);
    //xxxGPIOB->CRH &= ~(0x04<<(4*4));

	// GPIOB pin 13 na outpur push pullup Led colour green MCBSTM32
    //xxxGPIOB->CRH |= 0x01<<(5*4);
    //xxxGPIOB->CRH &= ~(0x04<<(5*4));

	// GPIOB pin 14 na outpur push pullup Led colour green MCBSTM32
    //xxxGPIOB->CRH |= 0x01<<(6*4);
    //xxxGPIOB->CRH &= ~(0x04<<(6*4));

	// GPIOB pin 15 na outpur push pullup Led colour green MCBSTM32
    //xxxGPIOB->CRH |= 0x01<<(7*4);
    //xxxGPIOB->CRH &= ~(0x04<<(7*4));

    // Setup GPIOA pin 0 is WKUP and GPIOC pin 13 TAMP as input on MCBSTM32
    //xxxGPIOA->CRL |= (0x04<<(0*4));
    //xxxGPIOC->CRH |= (0x04<<(5*4));
}
/*-----------------------------------------------------------*/
/*
void vParTestSetLED( unsigned portBASE_TYPE uxLED, signed portBASE_TYPE xValue )
{
	if( uxLED < partstNUM_LEDs )
	{
		portENTER_CRITICAL();
		{
			if( xValue != pdFALSE )
			{
				ucBitStates[ uxLED ] = pdTRUE;
			}
			else
			{
				ucBitStates[ uxLED ] = pdFALSE;
			}

            GPIO_WriteBit( uxIO_Port[ uxLED ], uxIO_Pins[ uxLED ], ucBitStates[ uxLED ] );
		}
		portEXIT_CRITICAL();
	}
}*/
/*-----------------------------------------------------------*/
/*
void vParTestToggleLED( unsigned portBASE_TYPE uxLED )
{
	if( uxLED < partstNUM_LEDs )
	{
		portENTER_CRITICAL();
		{
			ucBitStates[ uxLED ] = !ucBitStates[ uxLED ];
            GPIO_WriteBit( uxIO_Port[ uxLED ], uxIO_Pins[ uxLED ], ucBitStates[ uxLED ] );
		}
		portEXIT_CRITICAL();
	}
}*/
/*-----------------------------------------------------------*/
/*
portBASE_TYPE xGetLEDState( unsigned portBASE_TYPE uxLED )
{
	if( uxLED < partstNUM_LEDs )
	{
		return ( portBASE_TYPE ) ucBitStates[ uxLED ];
	}
	else
	{
		return 0;
	}
}*/
