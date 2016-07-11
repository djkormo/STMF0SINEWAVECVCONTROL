/*
 * config.c
 *
 *  Created on: 11 lip 2016
 *      Author: kormo
 */
#include <stm32f0xx_gpio.h>
#include <stm32f0xx_rcc.h>
#include <stm32f0xx_tim.h>
#include <stm32f0xx_dac.h>
#include <stm32f0xx_dma.h>
#include <stm32f0xx_adc.h>


#include "config.h"

const uint8_t SAMPLES =2;
uint16_t RegularConvData [2] ={0,0} ;
#define GreenLED GPIO_Pin_9
#define BlueLED GPIO_Pin_8
#define LEDGPIO GPIOC

void InitClocks()
{
    // Set up 48 MHz Core Clock using HSI (8Mhz) with PLL x 6
    RCC_PLLConfig(RCC_PLLSource_HSI, RCC_PLLMul_6);
    RCC_PLLCmd(ENABLE);

    // Wait for PLLRDY after enabling PLL.
    while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) != SET)
    { }

    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);  // Select the PLL as clock source.
    SystemCoreClockUpdate();
}


// configure board
void  InitBoard(void)
{

		GPIO_InitTypeDef   LEDs;

        SystemInit();
        SystemCoreClockUpdate();

        //Enable GPIO Clock
        RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
        // enable Timer
        // TIM3 for DAC
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 ,ENABLE);
        // TIM1 for ADC
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

        //Initialize LEDs
        GPIO_StructInit(&LEDs);
        LEDs.GPIO_Pin = GreenLED | BlueLED;
        LEDs.GPIO_Mode = GPIO_Mode_OUT;
        LEDs.GPIO_OType = GPIO_OType_PP;
        LEDs.GPIO_PuPd = GPIO_PuPd_NOPULL;
        LEDs.GPIO_Speed = GPIO_Speed_Level_3; //50MHz
        GPIO_Init(LEDGPIO, &LEDs);
        // turn on green  and blue Leds
        GPIO_SetBits(LEDGPIO, GreenLED);
        GPIO_ResetBits(LEDGPIO, BlueLED);


}



// configure DAC
void InitDAC(void)
{

	GPIO_InitTypeDef	GPIO_InitStructure;
	DAC_InitTypeDef     DAC_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

  /* DAC Periph clock enable */

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_DAC, ENABLE);



  // Configure PA.04/05 (DAC) as output -------------------------
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);


  /* Fill DAC InitStructure */


  	  DAC_InitStructure.DAC_Trigger = DAC_Trigger_T3_TRGO;
 	  DAC_InitStructure.DAC_OutputBuffer = DAC_OutputBuffer_Disable;


 	  DAC_Init(DAC_Channel_1, &DAC_InitStructure);

 	  //(+) Enable the DAC channel using DAC_Cmd()
 	  DAC_Cmd(DAC_Channel_1, ENABLE);


  /* DAC channel1 Configuration */
 	  DAC_Init(DAC_Channel_2, &DAC_InitStructure);

  /* Enable DAC Channel1: Once the DAC channel1 is enabled, PA.04 is
     automatically connected to the DAC converter. */
 	  DAC_Cmd(DAC_Channel_2, ENABLE);



}


/*
 * Example of ADC with IRQ
 *
 * https://my.st.com/public/STe2ecommunities/mcu/Lists/STM32Discovery/Flat.aspx?RootFolder=https%3a%2f%2fmy%2est%2ecom%2fpublic%2fSTe2ecommunities%2fmcu%2fLists%2fSTM32Discovery%2fstm32f0%20interrupt&FolderCTID=0x01200200770978C69A1141439FE559EB459D75800084C20D8867EAD444A5987D47BE638E0F&currentviews=494
*/
/*
https://my.st.com/public/STe2ecommunities/mcu/Lists/cortex_mx_stm32/Flat.aspx?RootFolder=/public/STe2ecommunities/mcu/Lists/cortex_mx_stm32/STM32F0%20ADC-DMA%20touble&FolderCTID=0x01200200770978C69A1141439FE559EB459D7580009C4E14902C3CDE46A77F0FFD06506F5B&currentviews=35
*/


void InitDACTimer(void)
{
	TIM_TimeBaseInitTypeDef 	TTB;
	NVIC_InitTypeDef         	DACNVIC_InitStructure;


	 	 	SystemInit(); //Ensure CPU is running at correctly set clock speed
	     	SystemCoreClockUpdate(); //Update SystemCoreClock variable to current clock speed
	     	//SysTick_Config(SystemCoreClock/1000); //Set up a systick interrupt every millisecond
	     	//SysTick_Config(SystemCoreClock);
            TTB.TIM_CounterMode = TIM_CounterMode_Up;
            TTB.TIM_Prescaler = 300-1; //  4800 kHz // was 300-1
            TTB.TIM_Period = 10-1; //1Hz; // was 10
            TTB.TIM_RepetitionCounter = 0;
            TIM_TimeBaseInit(TIM3, &TTB);
            TIM_Cmd(TIM3, ENABLE);



            /* http://visualgdb.com/tutorials/arm/stm32/timers/ */
            TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);
            TIM_SelectOutputTrigger(TIM3, TIM_TRGOSource_Update);
            /* http://forbot.pl/blog/artykuly/programowanie/kurs-stm32-7-liczniki-timery-w-praktyce-pwm-id8459 */


            NVIC_SetPriority(TIM3_IRQn, 0);
            NVIC_EnableIRQ (TIM3_IRQn);
            DACNVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
            DACNVIC_InitStructure.NVIC_IRQChannelPriority = 0;
            DACNVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
            NVIC_Init(&DACNVIC_InitStructure);
            // Higher Priority
            NVIC_SetPriority(TIM3_IRQn, 0);

}



void InitADC(void)
{

 ADC_InitTypeDef 			ADC_InitStructure;
 TIM_TimeBaseInitTypeDef 	TIM_TimeBaseStructure;
 NVIC_InitTypeDef 			NVIC_InitStructure;
 GPIO_InitTypeDef 			GPIO_InitStructure;
 TIM_OCInitTypeDef 			TIM_OCInitStructure;
 DMA_InitTypeDef 			DMA_InitStructure;

  /* ADC1 DeInit */
  ADC_DeInit(ADC1);

  /* ADC1 Periph clock enable */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_TIM1, ENABLE);

  /* DMA1 clock enable */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  /* TIM1 Configuration */
  TIM_DeInit(TIM1);

  /* Time base configuration */
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock / 1000000) - 1; // 1 MHz, from 48 MHz
  TIM_TimeBaseStructure.TIM_Period = 100 - 1; // 100 Hz
  TIM_TimeBaseStructure.TIM_ClockDivision = 0x0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

  /* Output Compare PWM Mode configuration */
  TIM_OCStructInit(&TIM_OCInitStructure);
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; /* low edge by default */
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 0x01;
  TIM_OC4Init(TIM1, &TIM_OCInitStructure);

  /* TIM1 enable counter */
  TIM_Cmd(TIM1, ENABLE);

  /* Main Output Enable */
  TIM_CtrlPWMOutputs(TIM1, ENABLE);

  /* DMA1 Channel1 Config */
  DMA_DeInit(DMA1_Channel1);
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&RegularConvData[0];
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = 2;//SAMPLES;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);

  /* DMA1 Channel1 enable */
  DMA_Cmd(DMA1_Channel1, ENABLE);

  /* ADC DMA request in circular mode */
  ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_Circular);

  /* Enable DMA1 Channel1 Half Transfer and Transfer Complete interrupt */
  //DMA_ITConfig(DMA1_Channel1, DMA_IT_HT, ENABLE);
  DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);

  /* Enable ADC_DMA */
  ADC_DMACmd(ADC1, ENABLE);

  /* Initialize ADC structure */
  ADC_StructInit(&ADC_InitStructure);

  /* Configure the ADC1 in continous mode withe a resolutuion equal to 12 bits  */
  ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;

  ADC_InitStructure.ADC_ExternalTrigConv =  ADC_ExternalTrigConv_T1_CC4;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Upward;
  ADC_Init(ADC1, &ADC_InitStructure);

  /* Convert the  ADC_Channnel_0  with 7.5 Cycles as sampling time */
  ADC_ChannelConfig(ADC1, ADC_Channel_1 , ADC_SampleTime_239_5Cycles);

  /* Convert the  ADC_Channnel_1  with 7.5 Cycles as sampling time */
  ADC_ChannelConfig(ADC1, ADC_Channel_0 , ADC_SampleTime_239_5Cycles);

  /* Convert the  ADC_Channnel_2  with 7.5 Cycles as sampling time */
  //ADC_ChannelConfig(ADC1, ADC_Channel_2 , ADC_SampleTime_7_5Cycles);

  /* Convert the  ADC_Channnel_3  with 7.5 Cycles as sampling time */
  //ADC_ChannelConfig(ADC1, ADC_Channel_3 , ADC_SampleTime_7_5Cycles);

  /* ADC Calibration */
  ADC_GetCalibrationFactor(ADC1);

  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);

  /* Wait the ADCEN flag */
  while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADEN));

  /* ADC1 regular Software Start Conv */
  ADC_StartOfConversion(ADC1);


  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);


  /* Enable and set DMA1_Channel1 Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0x00;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);


}