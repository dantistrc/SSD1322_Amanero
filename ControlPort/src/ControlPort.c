#include "gpio.h"
#include "Delay.h"
#include "ControlPort.h"

//Control port setup-------------------
#define CONTROL_PORT     GPIOA

#define SLOW_BIT       8
#define SD_BIT         9
#define MASTER_SLAVE   10 
#define DSD_BIT        12 
#define MCLK_BIT       11  


void Control_port_Init(void)
{
RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

GPIO_INIT_PIN( CONTROL_PORT, SLOW_BIT,       GPIO_MODE_OUTPUT50_PUSH_PULL_DOWN);	
GPIO_INIT_PIN( CONTROL_PORT, SD_BIT,         GPIO_MODE_OUTPUT50_PUSH_PULL_DOWN);
GPIO_INIT_PIN( CONTROL_PORT, MASTER_SLAVE,   GPIO_MODE_OUTPUT50_PUSH_PULL_DOWN);	
GPIO_INIT_PIN( CONTROL_PORT, DSD_BIT,        GPIO_MODE_OUTPUT50_PUSH_PULL_DOWN);	
GPIO_INIT_PIN( CONTROL_PORT, MCLK_BIT,       GPIO_MODE_OUTPUT50_PUSH_PULL_DOWN);	
	
}

void MASTER_DAC(unsigned int state)
{
		CONTROL_PORT->BSRR = 1 << (MASTER_SLAVE + state*0x10); 			  
}

void DSD_DAC(unsigned int state)
{
		CONTROL_PORT->BSRR = 1 << (DSD_BIT + state*0x10); 			  
}

void MCLK_DAC(unsigned int state)
{
		CONTROL_PORT->BSRR = 1 << (MCLK_BIT + state*0x10); 			  	
}

void SLOW_BIT_DAC(unsigned int state)
{
		CONTROL_PORT->BSRR = 1 << (SLOW_BIT + state*0x10); 			  	
}

void SD_BIT_DAC(unsigned int state)
{
		CONTROL_PORT->BSRR = 1 << (SD_BIT + state*0x10); 			  	
}


