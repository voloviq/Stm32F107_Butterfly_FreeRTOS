/*
    Description:

    This C module is used to configure
    all needed peryferial on MCBSTM32

	Compiler: Gnu Gcc

	Language: Ansi C

	Ide: Eclipse

	Author: Michal Wolowik

	Date: Rzeszow December 2008
*/






#include "Stm32l_level.h"






void Low_Level_Initialize(void)
{
	// Configure clocking system on MCBSTM32
	Rccu_Configure();

	// Initialize and configure all needed GPIO on MCBSTM32
	Gpio_Configure();

}






void Rccu_Configure(void)
{

    unsigned char hse_rdy = 0;






    // Clean all needed RCC register
    Clean_All_Rcc_Register();






    // Enable external high clock
    hse_rdy = Enable_External_Clock();







    // If HSE is enable configure PLL
	if(hse_rdy)
	{
        // FLASH ACR register bits 31-6 reserved

        // bits 0 and 1 and 2 is latency (000 - 0<SYSCLK<=24MHz,
        // 001 24MHz<SYSCLK<=48MHz, 010 47MHz<SYSCLK<=72MHz)

        // bit 3 is half cycles access enable 0 - disabled, 1 - enabled

        // bit 4 is preftech buffer 1 - enabled, 0 - disabled

        // bit 5 is read only, if this bit is 1 - preftech is enabled and if 0 is disabled

        // Latency is 010, half cycles is disabled, preftech buffer enabled
        FLASH->ACR = ((unsigned int)0xFFFFFFD2);


        // Description of RCC->CFGR register (PLL configure and other clocking)

        // 3 Bits MCO[26:24] 0xF b1xxx FFFFFF (all bit is rw - cleared by software)
        //-------------------------------------------------------------------------------------
        // 0xx (0x00000000) - No clocking
        // 100 (0x04000000) - System clock(SYSCLK) selected
        // 101 (0x05000000) - HSI clock selected
        // 110 (0x06000000) - HSE clock selected
        // 111 (0x07000000) - PLL clock divided by 2 selected
        //-------------------------------------------------------------------------------------

        // 1 Bit USBPRE[22] - set and clear by software to generate 48MHz USB clock
        // Warning: this bit must be set before enabling USB clock in RCC->APB1ENR.
        // This bit may be reset if USB clock is enabled.
        // Value 0 - PLL is divided by 1.5 if 1 is't divided
        //-------------------------------------------------------------------------------------
        // 0 0x00000000 - PLL clock is divided by 1.5
        // 1 0x00400000 - PLL clock is not divided
        //-------------------------------------------------------------------------------------

        // 4 Bits PLLMUL[21:18] - set and clear by software to setup PLL multiplier.
        // Warning: This bit should be written only if PLL is disabled. Don't exceed 72MHz.
        //-------------------------------------------------------------------------------------
        // 0000 (0x00000000) - PLL input clock x 2
        // 0001 (0x00040000) - PLL input clock x 3
        // 0010 (0x00080000) - PLL input clock x 4
        // 0011 (0x000C0000) - PLL input clock x 5
        // 0100 (0x00100000) - PLL input clock x 6
        // 0101 (0x00140000) - PLL input clock x 7
        // 0110 (0x00180000) - PLL input clock x 8
        // 0111 (0x001C0000) - PLL input clock x 9
        // 1000 (0x00200000) - PLL input clock x 10
        // 1001 (0x00240000) - PLL input clock x 11
        // 1010 (0x00280000) - PLL input clock x 12
        // 1011 (0x002C0000) - PLL input clock x 13
        // 1100 (0x00300000) - PLL input clock x 14
        // 1101 (0x00340000) - PLL input clock x 15
        // 1110 (0x00380000) - PLL input clock x 16
        // 1111 (0x003C0000) - PLL input clock x 17 in manual is 16
        RCC->CFGR |= 0x001C0000;
        //-------------------------------------------------------------------------------------

        // 1 Bit PLLXTPRE[17] - set and clear by software is use to divide HSE PLL entry.
        // Warning: This bit should be written only if PLL is disabled.
        //-------------------------------------------------------------------------------------
        // 0 (0x00000000) - HSE clock not divided
        // 1 (0x00020000) - HSE clock is divided by 2
        RCC->CFGR |= 0x00000000;
        //-------------------------------------------------------------------------------------

        // 1 Bit PLLSRC[16] - set and clear by software to select PLL clock source.
        // Warning: This bit should be written only if PLL is disabled
        //-------------------------------------------------------------------------------------
        // 0 (0x00000000) - HSI oscillator clock divided by 2 selected as PLL input clock
        // 1 (0x00010000) - HSE oscillator clock selected as PLL inut clock
        RCC->CFGR |= 0x00010000;
        //-------------------------------------------------------------------------------------

        // 2 Bits ADCPRE[15:14] - set and clear by software to select freq. clock for ADC.
        //-------------------------------------------------------------------------------------
        // 00 (0x00000000) - PLCK2 divided by 2
        // 01 (0x00004000) - PLCK2 divided by 4
        // 10 (0x00008000) - PLCK2 divided by 6
        // 11 (0x0000C000) - PLCK2 divided by 8
        //-------------------------------------------------------------------------------------

        // 3 Bits PPRE2[13:11] - set and clear by software to setup divisor for APB2 (PCLK2)
        // high speed prescaler
        //-------------------------------------------------------------------------------------
        // 0xx (0x00000000) - HCLK not divided
        // 100 (0x00002000) - HCLK is divided by 2
        // 101 (0x00002800) - HCLK is divided by 4
        // 110 (0x00003000) - HCLK is divided by 8
        // 111 (0x00003800) - HCLK is divided by 16
        RCC->CFGR |= 0x00000000;
        //-------------------------------------------------------------------------------------

        // 3 Bits PPRE1[10:8] - set and clear by software to setup divisor for APB1 (PCLK1)
        // low speed prescaler
        // Warning: don't exceed 36MHz it's low speed
        //-------------------------------------------------------------------------------------
        // 0xx (0x00000000) - HCLK not divided
        // 100 (0x00000400) - HCLK is divided by 2
        // 101 (0x00000500) - HCLK is divided by 4
        // 110 (0x00000600) - HCLK is divided by 8
        // 111 (0x00000700) - HCLK is divided by 16
        RCC->CFGR |= 0x00000400;
        //-------------------------------------------------------------------------------------

        // 4 Bits HPRE[7:4] - set and clear by software to setup divisor for AHB clock
        //-------------------------------------------------------------------------------------
        // 0xxx (0x00000000) - SYSCLK not divided
        // 1000 (0x00000080) - SYSCLK is divided by 2
        // 1001 (0x00000090) - SYSCLK is divided by 4
        // 1010 (0x000000A0) - SYSCLK is divided by 8
        // 1011 (0x000000B0) - SYSCLK is divided by 16
        // 1100 (0x000000C0) - SYSCLK is divided by 64
        // 1101 (0x000000D0) - SYSCLK is divided by 128
        // 1110 (0x000000E0) - SYSCLK is divided by 256
        // 1111 (0x000000F0) - SYSCLK is divided by 512
        RCC->CFGR |= 0x00000000;
        //-------------------------------------------------------------------------------------

        // 2 Bits SWS[3:2] - set and clear by software to indicate whitch source will be
        // used as system clock
        //-------------------------------------------------------------------------------------
        // 00 (0x00000000) - HSI oscilator is used to system clock
        // 01 (0x00000004) - HSE oscilator is used to system clock
        // 10 (0x00000008) - PLL oscilator is used to system clock
        // 11 (0x0000000C) - not applicable
        //-------------------------------------------------------------------------------------

        // 2 Bits SW[1:0] - set and clear by software to setup whitch source will be
        // used as system clock
        //-------------------------------------------------------------------------------------
        // 00 (0x00000000) - HSI oscilator is used to system clock
        // 01 (0x00000001) - HSE oscilator is used to system clock
        // 10 (0x00000002) - PLL oscilator is used to system clock
        // 11 (0x00000003) - not allowed
        //-------------------------------------------------------------------------------------


		// Enable PLL
        RCC->CR |= 1<<24;

        // Wait until PLL will be locked (check PLL_RDY flage)
		while( !(RCC->CR & (1<<25)) );

		// Choose PLL as main system clock source
        RCC->CFGR |= 0x00000002;

		// Wait until system switch to PLL clock
		while(!(RCC->CFGR&(0x00000008)));

	}

}






void Clean_All_Rcc_Register(void)
{

    // Set HSION bit
    RCC->CR |= (unsigned int)0x00000001;

    // Reset SW[1:0], HPRE[3:0], PPRE1[2:0], PPRE2[2:0], ADCPRE[1:0] and MCO[2:0] bits
    RCC->CFGR &= (unsigned int)0xF8FF0000;

    // Reset HSEON, CSSON and PLLON bits
    RCC->CR &= (unsigned int)0xFEF6FFFF;

    // Reset HSEBYP bit
    RCC->CR &= (unsigned int)0xFFFBFFFF;

    // Reset PLLSRC, PLLXTPRE, PLLMUL[3:0] and USBPRE bits
    RCC->CFGR &= (unsigned int)0xFF80FFFF;

    // Disable all interrupts
    RCC->CIR = 0x00000000;

}






unsigned char Enable_External_Clock(void)
{
    volatile unsigned int i = 0;

    // Reset HSEBYP bit set to 0 disable
    RCC->CR &= ((u32)0xFFFBFFFF);


    // Set HSEON bit in CR register
    RCC->CR |=  1<<16;


    // Wait here to stabilize external colck
    while( !(RCC->CR & (1<<17)) )
    {
        i++;

        if(i >= 0xFFFF)
        {
            return 0;
        }
    }

    return 1;

}







void Gpio_Configure(void)
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






void Led_1(unsigned char state)
{

	if(state != 0)
        GPIOE->BSRR = GPIO_Pin_14;
	else
		GPIOE->BRR = GPIO_Pin_14;

}






void Led_2(unsigned char state)
{

	if(state != 0)
        GPIOE->BSRR = GPIO_Pin_15;
	else
		GPIOE->BRR = GPIO_Pin_15;

}







unsigned char Read_Tamp_Button(void)
{
    if ((GPIOC->IDR & GPIO_Pin_13) != (unsigned int)Bit_RESET)
        return Bit_SET;
    else
        return Bit_RESET;
}






unsigned char Read_Wkup_Button(void)
{
    if ((GPIOA->IDR & GPIO_Pin_0) != (unsigned int)Bit_RESET)
        return Bit_SET;
    else
        return Bit_RESET;
}



