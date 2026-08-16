#include "UART_XMOS.h"
#include "stm32f10x.h"  // для регистров

// Глобальные переменные (их надо объявить в UART_XMOS.c или в заголовочном файле)
#define UART_BUFFER_SIZE 3
volatile uint8_t rx_buffer[UART_BUFFER_SIZE];
static uint8_t buffer_index = 0;

volatile uint8_t new_signal_received = 0;  // флаг, что пришёл новый пакет
volatile uint8_t last_signal = 0;



// Функция инициализации UART1 для приёма на PA10
void UART_Init(void) {
    // 1. Тактирование GPIOA и USART1
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_USART1EN;

    // 2. PA10 как RX (альтернативная функция, вход)
    GPIOA->CRH &= ~(GPIO_CRH_MODE10 | GPIO_CRH_CNF10);
    GPIOA->CRH |= GPIO_CRH_CNF10_1;   	// альтернативная функция входа

    // 3. Настройка USART1: 115200, 8 бит, 1 стоп, без чётности
    USART1->BRR = 625;   								// для 72 МГц и 115200
    USART1->CR1 &= ~USART_CR1_M;       	// 8 бит
    USART1->CR2 &= ~USART_CR2_STOP;    	// 1 стоп-бит
    USART1->CR1 &= ~USART_CR1_PCE;     	// без чётности
    USART1->CR1 |= USART_CR1_RE | USART_CR1_UE;   // приём и включение
    USART1->CR1 |= USART_CR1_RXNEIE;   	// прерывание по приёму

    // 4. Включить прерывание в NVIC
    NVIC_EnableIRQ(USART1_IRQn);
    NVIC_SetPriority(USART1_IRQn, 0);
}

// Функция чтения байта (неблокирующая, возвращает 1 если байт принят)
uint8_t UART_ReadByte(uint8_t *data) {
    if (USART1->SR & USART_SR_RXNE) {
        *data = (uint8_t)(USART1->DR & 0xFF);
        return 1;
    }
    return 0;
}
uint8_t UART_XMOS_GetSignal(void) {
    return last_signal;
}


// Обработчик прерывания USART1
void USART1_IRQHandler(void) {
    if (USART1->SR & USART_SR_RXNE) {
        uint8_t byte = USART1->DR & 0xFF;
        rx_buffer[buffer_index++] = byte;	
        if (buffer_index >= 3) {
            // Приняли 3 байта — мигаем светодиодом
     //       GPIOC->ODR ^= (1 << 13);   // переключаем PC13
            buffer_index = 0;
            new_signal_received = 1;
            // Сохраняем все три байта (для вывода на экран)
            last_signal = rx_buffer[1];  // средний байт

        }
    }
}


