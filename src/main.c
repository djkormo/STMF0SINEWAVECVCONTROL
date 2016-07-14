#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wmissing-declarations"
#pragma GCC diagnostic ignored "-Wreturn-type"

#include <stdio.h>
#include <stdlib.h>
#include "diag/Trace.h"
#include "config.h"
#include "algorithm.h"
#include "resources.h"

#include <math.h>
#include "stm32f0_discovery.h"
#include <stm32f0xx_gpio.h>
#include <stm32f0xx_rcc.h>
#include <stm32f0xx_tim.h>
#include <stm32f0xx_dac.h>
#include <stm32f0xx_dma.h>
#include <stm32f0xx_adc.h>



//Initialization structs


uint32_t lutIndex =0;
uint32_t lutStep =15;
uint8_t  lutStepADC =1;

int handleTIM =1;
int usingLeds =0;
int usingDAC=0; //* TODO not working
int usingADC=1;





//boundaries for ADC(12 bit resolution)

#define MININPUTADC 0.0
#define MAXINPUTADC 4095.0

float VoltValue=0;

uint32_t accumulator=0;
uint32_t accumulatorStep=0;
uint16_t CurrentTimerVal = 0;
uint16_t FMPhase =0;

uint16_t ADC1ConvertedPitchValue =0;
uint16_t ADC1ConvertedModulationValue =0;












/*
 * Example of ADC with IRQ
 *
 * https://my.st.com/public/STe2ecommunities/mcu/Lists/STM32Discovery/Flat.aspx?RootFolder=https%3a%2f%2fmy%2est%2ecom%2fpublic%2fSTe2ecommunities%2fmcu%2fLists%2fSTM32Discovery%2fstm32f0%20interrupt&FolderCTID=0x01200200770978C69A1141439FE559EB459D75800084C20D8867EAD444A5987D47BE638E0F&currentviews=494
*/
/*
https://my.st.com/public/STe2ecommunities/mcu/Lists/cortex_mx_stm32/Flat.aspx?RootFolder=/public/STe2ecommunities/mcu/Lists/cortex_mx_stm32/STM32F0%20ADC-DMA%20touble&FolderCTID=0x01200200770978C69A1141439FE559EB459D7580009C4E14902C3CDE46A77F0FFD06506F5B&currentviews=35
*/







void TIM3_IRQHandler()
{



    if (TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
    {

    	if (usingLeds)
    	{

        if (GPIO_ReadOutputDataBit(LEDGPIO, BlueLED))
                    GPIO_ResetBits(LEDGPIO, BlueLED);
                else
                    GPIO_SetBits(LEDGPIO, BlueLED);

        if (GPIO_ReadOutputDataBit(LEDGPIO, GreenLED))
                    GPIO_ResetBits(LEDGPIO, GreenLED);
                else
                    GPIO_SetBits(LEDGPIO, GreenLED);

    	}



    	if (usingDAC)
    	{
    	// index modulo number of samples

    	if (lutIndex+FMPhase>=1024)
    	{
    		//lutIndex= lutIndex%1024;
    		lutIndex-=1024; // instead of zero
    	}


			DAC_SetChannel1Data(DAC_Align_12b_R,
					(uint16_t) ((0.90)*(ADC1ConvertedModulationValue/4095.00)* Sine1024_12bit[lutIndex+FMPhase]));

			lutIndex+=lutStep;
    	}

    	/*
    	 http://stackoverflow.com/questions/16889426/fm-synthesis-using-phase-accumulator
    	 http://blog.thelonepole.com/2011/07/numerically-controlled-oscillators/
    	 */



       TIM_ClearITPendingBit(TIM3, TIM_IT_Update);

    }

}


void DMA1_Channel1_IRQHandler(void)
{
  /* Test on DMA1 Channel1 Transfer Complete interrupt */
  if (DMA_GetITStatus(DMA1_IT_HT1))
  {
    /* Clear DMA1 Channel1 Half Transfer interrupt pending bits */
    DMA_ClearITPendingBit(DMA1_IT_HT1);
    // do something

  }

  /* Test on DMA1 Channel1 Transfer Complete interrupt */
  if (DMA_GetITStatus(DMA1_IT_TC1))
  {
    /* Clear DMA1 Channel1 Transfer Complete interrupt pending bits */
    DMA_ClearITPendingBit(DMA1_IT_TC1);


    // read pitch
    ADC1ConvertedPitchValue=RegularConvData[0];
   // ADC1ConvertedPitchValue=3000;
    // read modulation value
    //ADC1ConvertedModulationValue=RegularConvData[1];

  }


  FMPhase=0; // for future use
}



int main(int argc, char* argv[])
{

	InitClocks();
    InitBoard();

    if (usingDAC)
    {
    	InitDAC();
    	InitDACTimer();
    }

    if (usingADC)
    {

    	InitADC();

    }




  // Infinite loop
  while (1)

  {



	  ADC1ConvertedModulationValue=4095;
	  VoltValue=(float)(10*ADC1ConvertedPitchValue/4095.0);

	  if (VoltValue < 1)
	  	     {

		  	  GPIO_ResetBits(GPIOC, BlueLED);
		  	  GPIO_ResetBits(GPIOC, GreenLED);

	  	     }
	  if (VoltValue < 2 && VoltValue>=1)
	  	     {

	  	       GPIO_ResetBits(GPIOC, BlueLED);
	  	  	   GPIO_SetBits(GPIOC, GreenLED);

	  	     }
	  if (VoltValue < 3 && VoltValue>=2)
	  	  	     {

	  	  	       GPIO_SetBits(GPIOC, BlueLED);
	  	  	       GPIO_ResetBits(GPIOC, GreenLED);
	  	  	  	  // lutStep=15;
	  	  	     }

	  if (VoltValue < 4 && VoltValue>=3)
	  	  	     {

	  	  	       GPIO_SetBits(GPIOC, BlueLED);
	  	  	       GPIO_SetBits(GPIOC, GreenLED);

	  	  	     }

	  if (VoltValue < 5 && VoltValue>=4)
	  	  	     {

		  	  	  GPIO_SetBits(GPIOC, BlueLED);
		  	  	  GPIO_SetBits(GPIOC, GreenLED);

	  	  	     }



	  // linear scale
	  lutStep=rangeScaleLinear(ADC1ConvertedPitchValue,0,4095,10,512);


  }

  return 0;
}




#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line) {
  while (1) {
  }
}
#endif

#pragma GCC diagnostic pop


