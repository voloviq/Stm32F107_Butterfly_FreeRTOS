/*
	Description:
	Examples software which show work of Nokia 3310 work
	on Stm32 butterfly development board

	Compiler: Gnu Gcc

	Language: Ansi C

	Ide: Eclipse

	Author: Michal Wolowik

	Date: Warsaw april 2015
*/





// Include needed library
#include "Stm32l_level.h"
#include "Lcd3310.h"
#include "StringsConv.h"


/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Library includes. */
#include "stm32f10x_tim.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_nvic.h"
#include "stm32f10x_systick.h"

/* Demo app includes. */
#include "BlockQ.h"
#include "integer.h"
#include "flash.h"
#include "partest.h"
#include "semtest.h"
#include "PollQ.h"
#include "GenQTest.h"
#include "QPeek.h"
#include "recmutex.h"




/* The time between cycles of the 'check' functionality (defined within the
tick hook. */
#define mainCHECK_DELAY						( ( TickType_t ) 5000 / portTICK_PERIOD_MS )

/* Task priorities. */
#define mainQUEUE_POLL_PRIORITY				( tskIDLE_PRIORITY + 2 )
#define mainSEM_TEST_PRIORITY				( tskIDLE_PRIORITY + 1 )
#define mainBLOCK_Q_PRIORITY				( tskIDLE_PRIORITY + 2 )
#define mainUIP_TASK_PRIORITY				( tskIDLE_PRIORITY + 3 )
#define mainFLASH_TASK_PRIORITY				( tskIDLE_PRIORITY + 2 )
#define mainLCD_TASK_PRIORITY				( tskIDLE_PRIORITY + 3 )
#define mainINTEGER_TASK_PRIORITY           ( tskIDLE_PRIORITY )
#define mainGEN_QUEUE_TASK_PRIORITY			( tskIDLE_PRIORITY )

/* The WEB server has a larger stack as it utilises stack hungry string
handling library calls. */
#define mainBASIC_WEB_STACK_SIZE            ( configMINIMAL_STACK_SIZE * 4 )

/* The length of the queue used to send messages to the LCD task. */
#define mainQUEUE_SIZE						( 3 )

/* The period of the system clock in nano seconds.  This is used to calculate
the jitter time in nano seconds. */
#define mainNS_PER_CLOCK					( ( unsigned long ) ( ( 1.0 / ( double ) configCPU_CLOCK_HZ ) * 1000000000.0 ) )





/*
 * Very simple task that toggles an LED.
 */
static void LEDs_Blinky_1( void *pvParameters );

static void LEDs_Blinky_2( void *pvParameters );


static void prvSetupHardware( void );


int main(void)
{

	volatile unsigned int i = 0;
	char *temp_tab;
	unsigned char temp_a = 0;
	unsigned char change = 0;

	prvSetupHardware();

	/* Start the standard demo tasks.  These are just here to exercise the
	kernel port and provide examples of how the FreeRTOS API can be used. */
	//xxxvStartBlockingQueueTasks( mainBLOCK_Q_PRIORITY );
	//xxxvStartSemaphoreTasks( mainSEM_TEST_PRIORITY );
	//xxxvStartPolledQueueTasks( mainQUEUE_POLL_PRIORITY );
	//xxxvStartIntegerMathTasks( mainINTEGER_TASK_PRIORITY );
	//xxxvStartGenericQueueTasks( mainGEN_QUEUE_TASK_PRIORITY );
	//xxxvStartLEDFlashTasks( mainFLASH_TASK_PRIORITY );
	//xxxvStartQueuePeekTasks();
	//xxxvStartRecursiveMutexTasks();

	/* Start the LCD gatekeeper task - as described in the comments at the top
	of this file. */
	xTaskCreate( LEDs_Blinky_1, "Blinky_1", configMINIMAL_STACK_SIZE, NULL, mainLCD_TASK_PRIORITY, NULL );

	xTaskCreate( LEDs_Blinky_2, "Blinky_2", configMINIMAL_STACK_SIZE, NULL, mainLCD_TASK_PRIORITY, NULL );

	/* Configure the high frequency interrupt used to measure the interrupt
	jitter time.  When debugging it can be helpful to comment this line out
	to prevent the debugger repeatedly going into the interrupt service
	routine. */
	vSetupHighFrequencyTimer();


    /* Start the scheduler. */
	vTaskStartScheduler();

    /* Will only get here if there was insufficient memory to create the idle
    task.  The idle task is created within vTaskStartScheduler(). */
	for( ;; );

	while (1)
	{

		if(i > 0 && i< 0x4FFFF)
		{
			Led_1(1);
			Led_2(0);

			if(change == 0)
				temp_a++;

			change = 1;

			if(temp_a>99)
				temp_a = 0;
		}
		else if(i >= 0x4FFFF && i < 0x6FFFF)
		{
			Led_1(0);
			Led_2(1);

			if(change == 1)
			{
				temp_tab = Connect_2String_With_Value("",temp_a,"");

				LCD_Clear ();
				LCD_WriteChar('o', 53, 0); // Celsius degree character
				LCD_WriteChar(temp_tab[0], 5, 0);
				LCD_WriteChar(temp_tab[1], 29, 0);
			}
			change = 0;

		}
		else if(i >= 0x6FFFF)
			i = 0;

		i++;

/*
		if( !Read_Wkup_Button() )
			Led_Pb9_Light(1);
		else
			Led_Pb9_Light(0);

		if( !Read_Tamp_Button() )
			Led_Pb10_Light(1);
		else
			Led_Pb10_Light(0);
			*/
	}

	return 0;
}





static void LEDs_Blinky_1( void *pvParameters )
{
	( void ) pvParameters;

	unsigned char state = 0;

	/* Block for 500ms. */
	portTickType xLastFlashTime;

	for( ;; )
	{

		if(state != 0)
	        GPIOE->BSRR = GPIO_Pin_14;
		else
			GPIOE->BRR = GPIO_Pin_14;

		state ^= 1;

        xLastFlashTime = xTaskGetTickCount();
        vTaskDelayUntil( &xLastFlashTime, 500 );

		//xxxvTaskDelay( xDelay );
	}
}
/*-----------------------------------------------------------*/





static void LEDs_Blinky_2( void *pvParameters )
{
	( void ) pvParameters;

	unsigned char state = 0;

	/* Block for 500ms. */
	portTickType xLastFlashTime;

	for( ;; )
	{

		if(state != 0)
	        GPIOE->BSRR = GPIO_Pin_15;
		else
			GPIOE->BRR = GPIO_Pin_15;

		state ^= 1;

        xLastFlashTime = xTaskGetTickCount();
        vTaskDelayUntil( &xLastFlashTime, 50 );

		//xxxvTaskDelay( xDelay );
	}
}
/*-----------------------------------------------------------*/




/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
	/* Start with the clocks in their expected state. */
	RCC_DeInit();

	/* Enable HSE (high speed external clock). */
	RCC_HSEConfig( RCC_HSE_ON );

	/* Wait till HSE is ready. */
	while( RCC_GetFlagStatus( RCC_FLAG_HSERDY ) == RESET )
	{
	}

	/* 2 wait states required on the flash. */
	*( ( unsigned long * ) 0x40022000 ) = 0x02;

	/* HCLK = SYSCLK */
	RCC_HCLKConfig( RCC_SYSCLK_Div1 );

	/* PCLK2 = HCLK */
	RCC_PCLK2Config( RCC_HCLK_Div1 );

	/* PCLK1 = HCLK/2 */
	RCC_PCLK1Config( RCC_HCLK_Div2 );

	/* PLLCLK = (25MHz / 2 ) * 5 = 62.5 MHz. */
	RCC_PLLConfig( RCC_PLLSource_HSE_Div2, RCC_PLLMul_5 );

	/* Enable PLL. */
	RCC_PLLCmd( ENABLE );

	/* Wait till PLL is ready. */
	while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
	{
	}

	/* Select PLL as system clock source. */
	RCC_SYSCLKConfig( RCC_SYSCLKSource_PLLCLK );

	/* Wait till PLL is used as system clock source. */
	while( RCC_GetSYSCLKSource() != 0x08 )
	{
	}

	/* Enable GPIOA, GPIOB, GPIOC, GPIOD, GPIOE and AFIO clocks */
	RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |RCC_APB2Periph_GPIOC
							| RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOE | RCC_APB2Periph_AFIO, ENABLE );

	/* Set the Vector Table base address at 0x08000000 */
	NVIC_SetVectorTable( NVIC_VectTab_FLASH, 0x0 );

	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );

	/* Configure HCLK clock as SysTick clock source. */
	SysTick_CLKSourceConfig( SysTick_CLKSource_HCLK );

	/* Initialise the IO used for the LED outputs. */
	vParTestInitialise();
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	/* This function will get called if a task overflows its stack.   If the
	parameters are corrupt then inspect pxCurrentTCB to find which was the
	offending task. */

	( void ) pxTask;
	( void ) pcTaskName;

	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationTickHook( void )
{
char *pcMessage = "Status: PASS";
static unsigned long ulTicksSinceLastDisplay = 0;
portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	/* Called from every tick interrupt as described in the comments at the top
	of this file.

	Have enough ticks passed to make it	time to perform our health status
	check again? */
	ulTicksSinceLastDisplay++;
	if( ulTicksSinceLastDisplay >= mainCHECK_DELAY )
	{
		/* Reset the counter so these checks run again in mainCHECK_DELAY
		ticks time. */
		ulTicksSinceLastDisplay = 0;

		/* Has an error been found in any task? */
		if( xAreGenericQueueTasksStillRunning() != pdTRUE )
		{
			pcMessage = "ERROR: GEN Q";
		}
		else if( xAreQueuePeekTasksStillRunning() != pdTRUE )
		{
			pcMessage = "ERROR: PEEK Q";
		}
		else if( xAreBlockingQueuesStillRunning() != pdTRUE )
		{
			pcMessage = "ERROR: BLOCK Q";
		}
	    else if( xAreSemaphoreTasksStillRunning() != pdTRUE )
	    {
	        pcMessage = "ERROR: SEMAPHR";
	    }
	    else if( xArePollingQueuesStillRunning() != pdTRUE )
	    {
	        pcMessage = "ERROR: POLL Q";
	    }
	    else if( xAreIntegerMathsTaskStillRunning() != pdTRUE )
	    {
	        pcMessage = "ERROR: INT MATH";
	    }
	    else if( xAreRecursiveMutexTasksStillRunning() != pdTRUE )
	    {
	    	pcMessage = "ERROR: REC MUTEX";
	    }

		/* Send the message to the OLED gatekeeper for display.  The
		xHigherPriorityTaskWoken parameter is not actually used here
		as this function is running in the tick interrupt anyway - but
		it must still be supplied. */
		xHigherPriorityTaskWoken = pdFALSE;

	}
}
/*-----------------------------------------------------------*/

