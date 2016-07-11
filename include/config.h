#ifndef CONFIG_H_
#define CONFIG_H_

#include <stdint.h>


// diodes
//#define GreenLED GPIO_Pin_9
//#define BlueLED GPIO_Pin_8
//#define LEDGPIO GPIOC

//Define Push button
#define PushButton_Pin GPIO_Pin_0
#define PushButton_GPIO GPIOA


//extern const uint8_t SAMPLES =2;
extern uint16_t RegularConvData[2];

extern void InitClocks(void);
extern void InitDAC(void);
void InitDACTimer(void);
void InitADC(void);
void  InitBoard(void);



#endif /* CONFIG_H_ */