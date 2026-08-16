void Encoder_Init(void) {
    // 1. Включить тактирование таймера и порта
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; 
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN; 

    // 2. Настроить пины PA0 (канал A) и PA1 (канал B) как входы
    GPIOA->CRL &= ~(GPIO_CRL_MODE0 | GPIO_CRL_CNF0 | GPIO_CRL_MODE1 | GPIO_CRL_CNF1);
    GPIOA->CRL |= (GPIO_CRL_CNF0_0 | GPIO_CRL_CNF1_0);

    // 3. Настроить таймер: период (например, 65535) и счётчик
    TIM2->ARR = 65535; 
    TIM2->PSC = 0; // Предделитель должен быть 0 для максимальной точности [citation:4]

    // 4. ВКЛЮЧИТЬ ЭНКОДЕРНЫЙ РЕЖИМ (самый важный шаг!)
    TIM2->CCMR1 |= (TIM_CCMR1_CC1S_0 | TIM_CCMR1_CC2S_0); // Подключить входы каналов к таймеру
    TIM2->SMCR |= (TIM_SMCR_SMS_0 | TIM_SMCR_SMS_1); // SMS = 011 (Режим 3)

    // 5. (Опционально) Можно настроить фильтр, чтобы убрать дребезг.
    TIM2->CCMR1 |= (TIM_CCMR1_IC1F_3); // Пример фильтра

    // 6. Запустить таймер!
    TIM2->CR1 |= TIM_CR1_CEN; 
}