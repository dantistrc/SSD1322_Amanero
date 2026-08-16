#include "gpio.h"
#include "Delay.h"
#include "DispPort.h"

//Display port setup-------------------
#define DAC_PORT     GPIOB

#define MUTE       1
#define DSD        6
#define F0         2  // 4
#define F1         5  // 3
#define F2         3  // 2
#define F3         4  // 1
#define CONNECT    0
#define MASTER_SL    7


void DAC_port_Init(void)
{
RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_JTAGDISABLE;	//JTAG disable to use PB3, PB4
	
GPIO_INIT_PIN( DAC_PORT, MUTE,    GPIO_MODE_INPUT_PULL_UP);	
GPIO_INIT_PIN( DAC_PORT, DSD,     GPIO_MODE_INPUT_PULL_DOWN);	
GPIO_INIT_PIN( DAC_PORT, F0,    	GPIO_MODE_INPUT_PULL_DOWN);
GPIO_INIT_PIN( DAC_PORT, F1,      GPIO_MODE_INPUT_PULL_DOWN);	
GPIO_INIT_PIN( DAC_PORT, F2,    	GPIO_MODE_INPUT_PULL_DOWN);
GPIO_INIT_PIN( DAC_PORT, F3,      GPIO_MODE_INPUT_PULL_DOWN);
	
GPIO_INIT_PIN( DAC_PORT, CONNECT,   GPIO_MODE_OUTPUT50_PUSH_PULL_DOWN);	
GPIO_INIT_PIN( DAC_PORT, MASTER_SL,   GPIO_MODE_OUTPUT50_PUSH_PULL_UP);
}

unsigned char MUTE_State(void)
{
	int mute_state;

	mute_state = ((DAC_PORT->IDR >> MUTE) & 1);	
	delay_ms(30);
return mute_state;
}

unsigned char DSD_State(void)
{
return ((DAC_PORT->IDR >> DSD) & 1);
}

unsigned char F0_State(void)
{
return ((DAC_PORT->IDR >> F0) & 1);
}

unsigned char F1_State(void)
{
return ((DAC_PORT->IDR >> F1) & 1);
}


unsigned char F2_State(void)
{
return ((DAC_PORT->IDR >> F2) & 1);	
}

unsigned char F3_State(void)
{
return ((DAC_PORT->IDR >> F3) & 1);
}

unsigned char Stream_ID(void) {
    extern volatile uint8_t last_signal;
    return last_signal;
}
/* unsigned char Stream_ID(void)
{
	unsigned char stream = 0;
	
	stream = (((DAC_PORT->IDR >> DSD) & 1) << 4) |
					 (((DAC_PORT->IDR >>  F3) & 1) << 3) | 
					 (((DAC_PORT->IDR >>  F2) & 1) << 2) |
					 (((DAC_PORT->IDR >>  F1) & 1) << 1) | 
					 (( DAC_PORT->IDR >>  F0) & 1);	

	delay_ms(50);
	
return stream;
}*/

void Connect_DAC(unsigned int state)
{
	  DAC_PORT->BSRR = 1 << (CONNECT + (!state * 0x01)); 		  //   CONNECT -> 0
}

void Master_Slave_Sel(unsigned int state)
{
	  DAC_PORT->BSRR = 1 << (MASTER_SL + (!state * 0x10)); 		  //   	MASTER -> 0
}





