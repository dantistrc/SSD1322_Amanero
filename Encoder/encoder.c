#include "encoder.h"
#include "gpio.h"



//Encoder port setup-------------------
#define CONTROL_PORT     GPIOA
#define AE_PIN      0
#define BE_PIN      1
#define ButE_PIN    6

void Encoder_Init(void)
{
	RCC->APB2ENR|= RCC_APB2ENR_IOPAEN;
	
	GPIO_INIT_PIN(  CONTROL_PORT, AE_PIN,   GPIO_MODE_INPUT_PULL_UP);
	GPIO_INIT_PIN(  CONTROL_PORT, BE_PIN,   GPIO_MODE_INPUT_PULL_UP);
	GPIO_INIT_PIN(  CONTROL_PORT, ButE_PIN, GPIO_MODE_INPUT_PULL_UP);
	
	RCC->APB1ENR = RCC_APB1ENR_TIM2EN;

	TIM2->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
	
	TIM2->CCER = TIM_CCER_CC1P | TIM_CCER_CC2P;
	TIM2->SMCR = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
	
	// УСТАНАВЛИВАЕМ ПЕРИОД СЧЁТЧИКА (для чувствительности)
	// Для энкодера с 24 импульсами на оборот (самый распространённый):
	TIM2->ARR = 3;  // Если на 2 щелчка одно изменение — увеличь до 47

	// ВКЛЮЧАЕМ ФИЛЬТР
	TIM2->CCMR1 |= (0x0F << 4) | (0x0F << 12); // максимальная фильтрация (16 выборок)
	
	TIM2->CR1 |= ~TIM_CR1_ARPE;
	TIM2->CR1 = TIM_CR1_CEN;
	
	AFIO->EXTICR[0] 	|= AFIO_EXTICR2_EXTI6_PA;
	EXTI->IMR |=EXTI_IMR_MR6;
	EXTI->FTSR |= EXTI_FTSR_TR6;
	NVIC_EnableIRQ (EXTI9_5_IRQn);
	NVIC_SetPriority (EXTI9_5_IRQn, 15);	
	
	TIM2->DIER |= TIM_DIER_UIE;                  
	NVIC_EnableIRQ(TIM2_IRQn);                    
}

int Encoder_read(void)
{
    return TIM2->CNT;
}

char Encoder_direction(void)
{
    return ((TIM2->CR1 & TIM_CR1_DIR) ? 0 : 1);
}
/*
void Encoder_Init(void)
{
	RCC->APB2ENR|= RCC_APB2ENR_IOPAEN;
	
	GPIO_INIT_PIN(  CONTROL_PORT, AE_PIN,   GPIO_MODE_INPUT_PULL_UP);
	GPIO_INIT_PIN(  CONTROL_PORT, BE_PIN,   GPIO_MODE_INPUT_PULL_UP);
	GPIO_INIT_PIN(  CONTROL_PORT, ButE_PIN, GPIO_MODE_INPUT_PULL_UP);
	
	RCC->APB1ENR = RCC_APB1ENR_TIM2EN;

	TIM2->CCMR1 = TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0;
	
	TIM2->CCER = TIM_CCER_CC1P | TIM_CCER_CC2P;
	TIM2->SMCR = TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1;
	TIM2->ARR = 8;
	
	
	
	TIM2->CR1 |= ~TIM_CR1_ARPE;
	TIM2->CR1 = TIM_CR1_CEN;
	
	
	
	AFIO->EXTICR[0] 	|= AFIO_EXTICR2_EXTI6_PA;
	EXTI->IMR |=EXTI_IMR_MR6;
	EXTI->FTSR |= EXTI_FTSR_TR6;
	NVIC_EnableIRQ (EXTI9_5_IRQn);
	NVIC_SetPriority (EXTI9_5_IRQn, 15);	
	
	TIM2->DIER |= TIM_DIER_UIE;                  
	NVIC_EnableIRQ(TIM2_IRQn);                    
		
}

int Encoder_read(void)
{
int counter;
	
counter = TIM2->CNT; // ? Правильно! Энкодер на TIM2//counter = TIM3->CNT;	
	
return counter;		
}	

char Encoder_direction(void)
{
char captured_direction;
	
captured_direction = ((TIM2->CR1 & TIM_CR1_DIR) ? 0 : 1);
	
return captured_direction;	
}	
*/


