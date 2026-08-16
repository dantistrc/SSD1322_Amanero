/*
 * ============================================================
 *  ГЛАВНЫЙ МОДУЛЬ УПРАВЛЕНИЯ УСТРОЙСТВОМ
 *  Платформа: STM32F103C8T6 (Blue Pill)
 *  Дисплей: SSD1322 256x64 (SPI)
 *  ЦАП: ES9039Q2M (управление через порты)
 *  Входы: XMOS (UART), IR (EXTI), Энкодер (TIM2), Кнопка (EXTI)
 * ============================================================
 */

#include "Delay.h"
#include "stm32f10x.h"
#include "stm32f10x_flash.h"
#include "DispPort.h"
#include "spi_ssd1322.h"
#include "encoder.h"
#include "ControlPort.h"
#include "stdio.h"
#include "UART_XMOS.h"
#include "ir_remote.h"
#include "aleks_font.h" // Подключаем наш гигантский шрифт
//#include "font_small.h"
extern const unsigned char SmallFont[][6];
extern volatile uint8_t rx_buffer[];      // Наш массив из UART_XMOS.c
extern volatile uint32_t ir_durations[];
extern volatile uint8_t  ir_duration_index;
extern volatile uint8_t  ir_packet_ready;
volatile uint8_t menu_need_update = 0; // Этот флаг крутит ТОЛЬКО нижнее меню (кнопки/энкодер)
volatile uint8_t ir_need_update = 0;   // А этот флаг крутит ТОЛЬКО верхний вывод пульта
volatile uint32_t ir_timeout_ms = 0; // Часы времени от последнего чиха пульта
volatile uint16_t ir_last_tick = 0; // Время последнего импульса пульта
// ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ УПРАВЛЕНИЯ ЦАПОМ
volatile float currentVolume = -40.0f; // Стартовая громкость в дБ (от -127.5 до 0.0)
volatile float currentBalance = 0.0f;  // Баланс (от -10.0 до +10.0)
volatile uint8_t currentInput = 1;     // Текущий вход (1 - USB, 2 - Coaxial, 3 - Optical)
volatile uint8_t currentFilter = 0;    // Текущий цифровой фильтр (0..6)


/* ============================================================
   1.  ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ (состояние устройства)
   ============================================================ */

/*
 *  ПЕРЕМЕННЫЕ УПРАВЛЕНИЯ:
 *  - enc_direction   : направление вращения энкодера (1 — вперёд, 0 — назад)
 *  - menu_level      : текущий уровень меню (0 — главный экран, 1..17 — пункты)
 *  - updated         : флаг, что нужно перерисовать экран (1 — обновить)
 *  - input_select    : выбор входа (0 — USB, 1 — S/PDIF)
 *  - dig_flt         : тип цифрового фильтра (0..3)
 *  - dsd_conf        : режим DSD (1 — авто, 0 — выкл)
 *  - mclk_conf       : режим MCLK (0 — авто, 1 — ручной)
 *  - mute_state      : состояние Mute (0 — звук включён, 1 — выключен)
 *  - mute_on         : состояние линии MUTE (0 — разомкнута, 1 — замкнута)
 */

unsigned char enc_direction;
unsigned char menu_level = 0;
unsigned char updated = 1;
unsigned char input_select;
unsigned char dig_flt;
unsigned char dsd_conf;
unsigned char mclk_conf;
unsigned char mute_state = 0;
unsigned char mute_on = 2;

/*
 *  ПЕРЕМЕННЫЕ IR-ПРИЁМНИКА:
 *  - ir_data_received : флаг, что принят новый IR-пакет
 *  - ir_code          : сохранённый код команды (пока не используется)
 */

volatile uint8_t  ir_data_received = 0;
volatile uint16_t ir_code = 0;

/*
 *  ТАЙМЕРЫ И СЧЁТЧИКИ:
 *  - halt_counter     : счётчик для автоматического отключения (Mute)
 *  - hall_timer_sec   : задержка до Mute (секунды)
 *  - menu_counter     : счётчик для возврата из меню
 *  - menu_timer_sec   : задержка до выхода из меню (секунды)
 *  - contrast         : яркость дисплея (0..255)
 */

unsigned int halt_counter = 0;
unsigned int hall_timer_sec = 300;
unsigned int menu_counter = 0;
unsigned int menu_timer_sec = 2100;
unsigned int contrast;
uint8_t remote_packet[4];
uint8_t ir_cmd = 0; 
/* ============================================================
   2.  КОНСТАНТЫ И МАКРОСЫ
   ============================================================ */

#define USB_SOURCE         0
#define SPDIF_SOURCE       1

#define SHARP_FLT         0
#define SLOW_FLT          1
#define SHARP_SD_FLT      2
#define SLOW_SD_FLT       3

#define DSD_AUTO_ON       1
#define DSD_AUTO_OFF      0

#define MCLK_AUTO_ON      0
#define MCLK_AUTO_OFF     1

// IR-приёмник на PB5
#define IR_PIN  GPIO_Pin_5
#define IR_PORT GPIOB
#define IR_CLK  RCC_APB2ENR_IOPBEN

/* ============================================================
		JSA
   ============================================================ */
// Состояния для нижней динамической строки меню
#define MODE_VOLUME  0
#define MODE_INPUT   1
#define MODE_FILTER  2
#define MODE_BALANCE 3

uint8_t current_mode = MODE_VOLUME; // Стартуем в режиме громкости
uint8_t volume_val = 40;  // Стартовая громкость
uint8_t input_val = 1;   // Стартовый вход (0-USB, 1-COA, 2-OPT)
uint8_t filter_val = 0;  // Стартовый фильтр ЦАПа
int8_t  balance_val = 0; // Стартовый баланс (ноль — центр)


/* ============================================================
   3.  ВСПОМОГАТЕЛЬНЫЕ БУФЕРЫ
   ============================================================ */

char buffer[16] = {'\0'};   // для форматирования строк (sprintf)

/* ============================================================
   4.  ОБРАБОТКА КНОПКИ (с антидребезгом через TIM4)
   ============================================================ */

volatile uint16_t button_tick = 0;     // время нажатия (в тиках TIM4)
volatile uint8_t button_pressed = 0;   // флаг нажатия
#define DEBOUNCE_TICKS 1               // задержка в 10 мс (при частоте TIM4 = 1 кГц)

/* ============================================================
   5.  РАБОТА С FLASH-ПАМЯТЬЮ (сохранение настроек)
   ============================================================ */

/*
 *  Структура для хранения настроек в Flash.
 *  Адрес: 0x8005000 (последняя страница Flash).
 */

typedef struct {
    int16_t input_select;   // 2 байта
    int16_t digital_filter; // 2 байта
    int16_t contrast;       // 2 байта
    int16_t volume_int;     // 2 байта (СЮДА ПИШЕМ ГРОМКОСТЬ * 10)
    int16_t balance_int;    // 2 байта (СЮДА ПИШЕМ БАЛАНС * 10)
} presets_def;

volatile presets_def preset;



#define PRESET_WORD_CNT 	sizeof(preset) / sizeof(uint32_t)

/**
  * @brief  Чтение настроек из Flash
  * @param  None
  * @retval None
  */
void FLASH_ReadSettings(void) {
    uint16_t p;
    uint32_t *source_adr = (uint32_t *)(0x8005000);
    uint32_t *dest_adr = (void *)&preset;

    for (p = 0; p < PRESET_WORD_CNT; ++p) {
        *(dest_adr + p) = *(__IO uint32_t*)(source_adr + p);
    }
}

/**
  * @brief  Запись настроек в Flash (стирает страницу и перезаписывает)
  * @param  None
  * @retval None
  */
void FLASH_WriteSettings(void) {
    uint8_t i;
    uint32_t pageAdr = 0x8005000;
    uint32_t *source_adr = (void *)&preset;

    FLASH_Unlock();
    FLASH_ErasePage(pageAdr);
    for (i = 0; i < PRESET_WORD_CNT; ++i) {
        FLASH_ProgramWord((uint32_t)(pageAdr + i * 4), *(source_adr + i));
    }
    FLASH_Lock();
}
// ТОЧЕЧНОЕ ЧТЕНИЕ ОДНОГО 32-БИТНОГО СЛОВА ИЗ ФЛЕШ
uint32_t FLASH_Read_Word(uint32_t address) {
    return *(volatile uint32_t*)address;
}

// ТОЧЕЧНАЯ ЗАПИСЬ ОДНОГО 32-БИТНОГО СЛОВА ВО ФЛЕШ
void FLASH_Write_Word(uint32_t address, uint32_t data) {
    // 1. Проверяем, нужно ли вообще писать? Если там уже лежат эти данные, выходим!
    // Это экономит ресурс флеша на 90%
    if (*(volatile uint32_t*)address == data) return; 
    
    // 2. Разблокируем флеш перед записью
    FLASH->KEYR = 0x45670123;
    FLASH->KEYR = 0xCDEF89AB;
    
    // 3. Перед записью в STM32 нужно стереть страницу (0x08005000 - начало страницы)
    // ВАЖНО: Страница стирается целиком, поэтому перед стиранием мы должны сохранить 
    // остальные параметры в оперативку, чтобы не стереть их насовсем!
    // Для этого мы временно считываем все 5 параметров в буфер:
    uint32_t buf[5];
    buf[0] = FLASH_Read_Word(0x08005000); // Вход
    buf[1] = FLASH_Read_Word(0x08005004); // Фильтр
    buf[2] = FLASH_Read_Word(0x08005008); // Контраст
    buf[3] = FLASH_Read_Word(0x0800500C); // Громкость
    buf[4] = FLASH_Read_Word(0x08005010); // Баланс
    
    // Подменяем в буфере точечно тот параметр, который хотим обновить
    if (address == 0x08005000) buf[0] = data;
    else if (address == 0x08005004) buf[1] = data;
    else if (address == 0x08005008) buf[2] = data;
    else if (address == 0x0800500C) buf[3] = data;
    else if (address == 0x08005010) buf[4] = data;

    // Стираем страницу 0x08005000
    while (FLASH->SR & FLASH_SR_BSY);
    FLASH->CR |= FLASH_CR_PER;
    FLASH->AR = 0x08005000;
    FLASH->CR |= FLASH_CR_STRT;
    while (FLASH->SR & FLASH_SR_BSY);
    FLASH->CR &= ~FLASH_CR_PER;
    
    // Прошиваем весь буфер обратно (уже с измененным одним параметром)
    FLASH->CR |= FLASH_CR_PG;
    for (uint8_t i = 0; i < 5; i++) {
        *(volatile uint16_t*)(0x08005000 + (i * 4)) = (uint16_t)(buf[i] & 0xFFFF);
        while (FLASH->SR & FLASH_SR_BSY);
        *(volatile uint16_t*)(0x08005002 + (i * 4)) = (uint16_t)((buf[i] >> 16) & 0xFFFF);
        while (FLASH->SR & FLASH_SR_BSY);
    }
    FLASH->CR &= ~FLASH_CR_PG;
    
    // Блокируем флеш обратно
    FLASH->CR |= FLASH_CR_LOCK;
}

/* ============================================================
   6.  I2C (для управления ES9039Q2M через I2C)
   ============================================================ */

/**
  * @brief  Инициализация I2C2 (PB10 — SCL, PB11 — SDA)
  * @param  None
  * @retval None
  */
void I2C2_Init(void) {
    RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

    GPIOB->CRH &= ~(GPIO_CRH_MODE10 | GPIO_CRH_CNF10);
    GPIOB->CRH |= GPIO_CRH_MODE10_0 | GPIO_CRH_CNF10_1;

    GPIOB->CRH &= ~(GPIO_CRH_MODE11 | GPIO_CRH_CNF11);
    GPIOB->CRH |= GPIO_CRH_MODE11_0 | GPIO_CRH_CNF11_1;

    I2C2->CR1 |= I2C_CR1_SWRST;
    I2C2->CR1 &= ~I2C_CR1_SWRST;
    I2C2->CCR = 180;
    I2C2->TRISE = 19;
    I2C2->CR2 |= I2C_CR2_FREQ_5 | I2C_CR2_FREQ_4 | I2C_CR2_FREQ_3;
    I2C2->CR1 |= I2C_CR1_PE;
}

/**
  * @brief  Отправка одного байта по I2C
  * @param  addr: 7-битный адрес устройства
  * @param  data: байт данных
  * @retval None
  */
void I2C2_WriteByte(uint8_t addr, uint8_t data) {
    //while (I2C2->SR2 & I2C_SR2_BUSY);
    I2C2->CR1 |= I2C_CR1_START;
    while (!(I2C2->SR1 & I2C_SR1_SB));
    I2C2->DR = (addr << 1);
    while (!(I2C2->SR1 & I2C_SR1_ADDR));
    (void)I2C2->SR2;
    I2C2->DR = data;
    while (!(I2C2->SR1 & I2C_SR1_TXE));
    I2C2->CR1 |= I2C_CR1_STOP;
}

/* ============================================================
   7.  ЛОГИКА ОБРАБОТКИ НАЖАТИЯ КНОПКИ
   ============================================================ */

/* ============================================================
   MENU
   ============================================================ */
void Update_Bottom_Line(void) {
    char buf[16];
    
        switch (current_mode) {
        case 0: // MODE_VOLUME
            // Передаем реальную громкость из volume_val вместо статической 24
            sprintf(buf, "VOL: -%d dB ", volume_val); 
            break;
            
        case 1: // MODE_INPUT
            // Выводим реальный вход в зависимости от input_val
            if (input_val == 0)      sprintf(buf, "IN: USB      ");
            else if (input_val == 1) sprintf(buf, "IN: COA      ");
            else                     sprintf(buf, "IN: OPT      ");
            break;
            
        case 2: // MODE_FILTER
            // Выводим номер текущего фильтра ЦАПа из filter_val
            sprintf(buf, "FLT: F-%d     ", filter_val); 
            break;
            
        case 3: // MODE_BALANCE
            // Выводим баланс из balance_val
            sprintf(buf, "BAL: %d      ", balance_val); 
            break;
    }

    
    // Выстреливаем собранную строку на координату Y = 44
    // Передаем аргумент '1', чтобы включить наш новый мелкий шрифт 16х11
    SSD1322_DrawString(0, 44, 1, (unsigned char*)buf);
		SSD1322_Update(0x00, 0x1F);
}
//======================== J S A ENCODER ========================================================================
void Process_Encoder_Rotation(uint8_t direction) {
    // Локальные переменные, которые мы завели в меню:
    // (Позже перенесем их на самый верх файла)
    static uint8_t vol = 40;     
    static uint8_t input = 1;    
    static uint8_t filter = 0;   

    if (direction) { // Крутим вправо (увеличение)
        switch (current_mode) {
            case 0: if (vol < 100) vol++; break;
            case 1: input = (input + 1) % 3; break; // USB -> COA -> OPT
            case 2: if (filter < 7) filter++; break; // Всего 8 фильтров у ESS9039
        }
    } else { // Крутим влево (уменьшение)
        switch (current_mode) {
            case 0: if (vol > 0) vol--; break;
            case 1: input = (input == 0) ? 2 : (input - 1); break;
            case 2: if (filter > 0) filter--; break;
        }
    }

    // Сразу же перерисовываем нижнюю строку с новыми цифрами!
		
    menu_need_update = 1; // Просто машем флажком, это занимает 1 такт процессора
//Update_Bottom_Line();																					//rrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrIRrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr
}

//===================================== J S A PRESS ENC =================================================================
void ProcessButtonPress(void) {
    // Если звук выключен (Mute) — включаем
    if (mute_state) {
        SSD1322_CommandWrite(0xAF);
        mute_state = 0;
        halt_counter = 0;
        return; // Выходим, первое нажатие просто снимает Mute
    }

uint16_t press_start = TIM4->CNT;
uint8_t is_long = 0;

// Крутимся, пока пин PA6 прижат к земле (6-й бит в IDR равен 0)
while ((GPIOA->IDR & (1 << 6)) == 0) {
    uint16_t now = TIM4->CNT;
    uint16_t diff = (now >= press_start) ? (now - press_start) : (now + 10000 - press_start);
    
    // Если удержание длится дольше 1500 тиков (полторы секунды)
    if (diff >= 1500) {
        is_long = 1;
        break;
    }
}

if (is_long) {
    // ВАРИАНТ 2: ДЛИННЫЙ ПРЕСС
    // Ждем, пока отпустишь кнопку
    while ((GPIOA->IDR & (1 << 6)) == 0) { __NOP(); }
}
else {
    // ВАРИАНТ 1: КОРОТКИЙ КЛИК
		menu_level = 1; // Активируем меню, чтобы TIM2 пустил нас к крутилке!
    current_mode++;
    if (current_mode > 3) {
        current_mode = 0; // Наша громкость MODE_VOLUME
    }
   menu_need_update = 1; // Просто машем флажком, это занимает 1 такт процессора
// Update_Bottom_Line();																					//rrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrIRrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr
		}

}
//==================================================== E N D = J S A ============================================================


/* ============================================================
   8.  ПРЕРЫВАНИЯ
   ============================================================ */

/**
  * @brief  Обработчик прерываний EXTI9_5 (кнопка PB6 и IR PB5)
  * @param  None
  * @retval None
  */
void EXTI9_5_IRQHandler(void) {
    // Кнопка на PB6 (антидребезг)
    if (EXTI->PR & EXTI_PR_PR6) {
        EXTI->PR = EXTI_PR_PR6;
        button_tick = TIM4->CNT;
        button_pressed = 1;
    }
//==============================================================
// IR-приёмник на PB5
//==============================================================		
    if (EXTI->PR & EXTI_PR_PR5) {
        EXTI->PR = EXTI_PR_PR5;
        //GPIOC->ODR ^= (1 << 13);   // мигаем светодиодом
        ir_data_received = 1;
        IR_Process_Bit();          // обработка битового потока IR
    }
}

/**
  * @brief  Обработчик TIM4 (1 кГц)
  * @param  None
  * @retval None
  */
void TIM4_IRQHandler(void) {
    if (TIM4->SR & TIM_SR_UIF) {
        TIM4->SR &= ~TIM_SR_UIF;

        if (halt_counter && halt_counter < hall_timer_sec) halt_counter++;
        if (menu_counter && menu_counter < menu_timer_sec) menu_counter++;

        if (button_pressed) {
            uint16_t now = TIM4->CNT;
            uint16_t diff = (now >= button_tick) ? (now - button_tick) : (now + 10000 - button_tick);
            if (diff >= DEBOUNCE_TICKS) {
                button_pressed = 0;
                ProcessButtonPress();
            }
        }
    }
}

/**
  * @brief  Обработчик TIM2 (энкодер)
  * @param  None
  * @retval None
  */
void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF;

        // Если Mute включён — выключаем при любом действии с энкодером
        if (mute_state) {
            SSD1322_CommandWrite(0xAF);
            mute_state = 0;
            halt_counter = 0;
        } else {
            halt_counter = 0;
            enc_direction = Encoder_direction();

            if (menu_level) {
//=====================================================				
 // Навигация по меню  J S A !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// Заменяем старый блок навигации по уровням
if (enc_direction) {
    // --- КРУТИМ ВПРАВО (ВЕЛИЧЕНИЕ) ---
    switch (current_mode) {
        case 0: // Громкость
            if (volume_val < 100) volume_val++;
            break;
        case 1: // Вход
            input_val = (input_val + 1) % 3; // Крутим по кругу: USB -> COA -> OPT
            break;
        case 2: // Фильтр
            if (filter_val < 4) filter_val++; // Например, всего 5 фильтров (0..4)
            break;
        case 3: // Баланс
            if (balance_val < 10) balance_val++; // Сдвиг в правый канал
            break;
    }
} else {
    // --- КРУТИМ ВЛЕВО (УМЕНЬШЕНИЕ) ---
    switch (current_mode) {
        case 0: // Громкость
            if (volume_val > 0) volume_val--;
            break;
        case 1: // Вход
            input_val = (input_val == 0) ? 2 : (input_val - 1);
            break;
        case 2: // Фильтр
            if (filter_val > 0) filter_val--;
            break;
        case 3: // Баланс
            if (balance_val > -10) balance_val--; // Сдвиг в левый канал
            break;
    }
}

// Мгновенно обновляем нижнюю строчку экрана с новыми значениями!
menu_need_update = 1; // Просто машем флажком, это занимает 1 такт процессора
//Update_Bottom_Line();																					//rrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrIRrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrrr
//=========================== J S A ++++++++++++++++++++++++++++++++++++================================
            } else {
                // Главный экран — переключение фильтров или входа
                if (!enc_direction) {
                    switch (dig_flt) {
                        case 0: dig_flt = SLOW_FLT; SLOW_BIT_DAC(0); SD_BIT_DAC(1); break;
                        case 1: dig_flt = SHARP_SD_FLT; SLOW_BIT_DAC(1); SD_BIT_DAC(0); break;
                        case 2: dig_flt = SLOW_SD_FLT; SLOW_BIT_DAC(0); SD_BIT_DAC(0); break;
                        case 3: dig_flt = SHARP_FLT; SLOW_BIT_DAC(1); SD_BIT_DAC(1); break;
                    }
                } else {
                    switch (input_select) {
                        case USB_SOURCE: input_select = SPDIF_SOURCE; MASTER_DAC(input_select); Master_Slave_Sel(input_select); break;
                        case SPDIF_SOURCE: input_select = USB_SOURCE; MASTER_DAC(input_select); Master_Slave_Sel(input_select); break;
                    }
                }
            }
        }
    }
}

/* ============================================================
   9.  ФУНКЦИИ ВЫВОДА НА ДИСПЛЕЙ (меню, статус, отладка)
   ============================================================ */

// ЖЕСТКАЯ ЗАПИСЬ В РЕГИСТР SABRE ESS9028 ПО I2C2
void ESS9028_WriteReg(uint8_t chip_addr, uint8_t reg, uint8_t data) {
    while (I2C2->SR2 & I2C_SR2_BUSY); // Убрали двойку из маски флага
    
    // 1. Генерируем СТАРТ
    I2C2->CR1 |= I2C_CR1_START; // Убрали двойку из маски флага
    while (!(I2C2->SR1 & I2C_SR1_SB)); // Убрали двойку из маски флага
    
    // 2. Отправляем адрес чипа ЦАП со сдвигом влево (режим записи)
    I2C2->DR = (chip_addr << 1);
    while (!(I2C2->SR1 & I2C_SR1_ADDR)); // Убрали двойку из маски флага
    (void)I2C2->SR2;
    
    // 3. Отправляем номер регистра
    I2C2->DR = reg;
    while (!(I2C2->SR1 & I2C_SR1_TXE)); // Убрали двойку из маски флага
    
    // 4. Отправляем сам байт данных громкости/баланса
    I2C2->DR = data;
    while (!(I2C2->SR1 & I2C_SR1_TXE)); // Убрали двойку из маски флага
    
    // 5. Генерируем СТОП
    I2C2->CR1 |= I2C_CR1_STOP; // Убрали двойку из маски флага
}

 // */=================================Громкость баланс===========================================
//*
#define ESS9028_I2C_ADDR   0x48 // Адрес ЦАПа на шине I2C

void Set_Volume_And_Balance(float currentVolume, float currentBalance) {
    float volumeLeft = currentVolume;
    float volumeRight = currentVolume;
    
    uint8_t regLeft;
    uint8_t regRight;

    //  РАСЧЕТ БАЛАНСА
    if (currentBalance < 0.0f) {
        // Баланс влево: душим ПРАВЫЙ канал
        volumeRight += (currentBalance * 0.5f); 
    } 
    else if (currentBalance > 0.0f) {
        // Баланс вправо: душим ЛЕВЫЙ канал
        volumeLeft -= (currentBalance * 0.5f);
    }

    //  ФОРМУЛА ПЕРЕВОДА В БАЙТЫ ЦАПА
    regLeft  = (uint8_t)(volumeLeft * -2.0f);
    regRight = (uint8_t)(volumeRight * -2.0f);

    //  В РЕГИСТРЫ ПО I2C2
    ESS9028_WriteReg(ESS9028_I2C_ADDR, 15, regLeft);  // 15 - Левый канал
    ESS9028_WriteReg(ESS9028_I2C_ADDR, 16, regRight); // 16 - Правый канал
}


//*/===========================Переключение фильтров=================================================
void ESS9028_SetFilter(uint8_t filter_num) {
    uint8_t reg_val = 0;
    
    // В Sabre регистр 7 отвечает за фильтры (биты 7:5 определяют тип FIR-фильтра)
    // Математически сдвигаем номер фильтра на 5 битов влево
    reg_val = (filter_num << 5) & 0xE0; 
    
    // Добавляем дефолтные настройки для остальных битов регистра 7 (например, оставляем DSD/OSF)
    reg_val |= 0x0C; 
    
    currentFilter = filter_num; // Запоминаем в систему
    
    // Выстрел в регистр 7 чипа по I2C
    ESS9028_WriteReg(ESS9028_I2C_ADDR, 7, reg_val);
}

//*/===========================Переключение логики входов (Регистр 1)=================================================

void ESS9028_SetInput(uint8_t input_num) {
    uint8_t reg_val = 0;
    
    switch (input_num) {
        case 1: 
            // Вход 1: USB / Amanero (Режим Serial/I2S)
            reg_val = 0x00; // Конфигурация под шину I2S/DSD
            break;
            
        case 2:
            // Вход 2: COAXIAL (Включаем встроенный S/PDIF демультиплексор, вход 1)
            reg_val = 0x40; // Биты переключения на S/PDIF Data Source
            break;
            
        case 3:
            // Вход 3: OPTICAL (Включаем S/PDIF демультиплексор, вход 2)
            reg_val = 0x50; 
            break;
            
        default:
            reg_val = 0x00;
            break;
    }
    
    currentInput = input_num; // Фиксируем в памяти
    
    // Выстрел в регистр 1 по I2C
    ESS9028_WriteReg(ESS9028_I2C_ADDR, 1, reg_val);
}















/**
  * @brief  Вывод информации о входе и частоте (главный экран)
  * @param  None
  * @retval None
  */
/*void STREAM_LCD(void) {
    if (!updated) { updated = 1; SSD1322_ClearRAM(); }

    if (!input_select) {
        switch (Stream_ID()) {
            case 0x01: SSD1322_DrawString(0, 5, 0, (unsigned char*)"PCM   44.1kHz"); break;
            case 0x02: SSD1322_DrawString(0, 5, 0, (unsigned char*)"PCM     48kHz"); break;
            case 0x03: SSD1322_DrawString(0, 5, 0, (unsigned char*)"PCM   88.2kHz"); break;
            case 0x04: SSD1322_DrawString(0, 5, 0, (unsigned char*)"PCM     96kHz"); break;
            case 0x05: SSD1322_DrawString(0, 5, 0, (unsigned char*)"PCM  176.4kHz"); break;
            case 0x06: SSD1322_DrawString(0, 5, 0, (unsigned char*)"PCM    192kHz"); break;
            case 0x07: SSD1322_DrawString(0, 5, 0, (unsigned char*)"PCM  352.8kHz"); break;
            case 0x08: SSD1322_DrawString(0, 5, 0, (unsigned char*)"PCM    384kHz"); break;
            case 0x09: SSD1322_DrawString(0, 5, 0, (unsigned char*)"PCM  705.6kHz"); break;
            case 0x0A: SSD1322_DrawString(0, 5, 0, (unsigned char*)"PCM    768kHz"); break;
            case 0x0B: SSD1322_DrawString(0, 5, 0, (unsigned char*)"PCM  1411.2kHz"); break;
            case 0x0C: SSD1322_DrawString(0, 5, 0, (unsigned char*)"PCM    1536kHz"); break;
            case 0x19: SSD1322_DrawString(0, 5, 0, (unsigned char*)"DSD64   2.822"); break;
            case 0x1A: SSD1322_DrawString(0, 5, 0, (unsigned char*)"DSD128  5.644"); break;
            case 0x1B: SSD1322_DrawString(0, 5, 0, (unsigned char*)"DSD256 11.289"); break;
            case 0x1C: SSD1322_DrawString(0, 5, 0, (unsigned char*)"DSD512 22.579"); break;
            case 0x1D: SSD1322_DrawString(0, 5, 0, (unsigned char*)"DSD1024 45.15"); break;
        }
    } else {
        SSD1322_DrawString(0, 5, 0, (unsigned char*)"S/PDIF  INPUT");
    }
}*/

/**
  * @brief  Вывод информации о фильтре и входе
  * @param  None
  * @retval None
  */
/*void INPUT_FLT_LCD(void) {
    if (!input_select) {
        switch (dig_flt) {
            case 0x00: SSD1322_DrawString(0, 40, 0,  (unsigned char*)"USB     SHARP"); break;
            case 0x01: SSD1322_DrawString(0, 40, 0, (unsigned char*)"USB      SLOW"); break;
            case 0x02: SSD1322_DrawString(0, 40, 0, (unsigned char*)"USB  SHARP SD"); break;
            case 0x03: SSD1322_DrawString(0, 40, 0, (unsigned char*)"USB   SLOW SD"); break;
        }
    } else {
        switch (dig_flt) {
            case 0x00: SSD1322_DrawString(0, 40, 0, (unsigned char*)"SHARP    DGFL"); break;
            case 0x01: SSD1322_DrawString(0, 40, 0, (unsigned char*)"SLOW     DGFL"); break;
            case 0x02: SSD1322_DrawString(0, 40, 0, (unsigned char*)"SHARP SD DGFL"); break;
            case 0x03: SSD1322_DrawString(0, 40, 0, (unsigned char*)"SLOW SD  DGFL"); break;
        }
    }
}
*/
/**
  * @brief  Вывод сообщения о Mute
  * @param  None
  * @retval None
  */
void MUTED_LCD(void) {
    if (!updated) { updated = 1; SSD1322_ClearRAM(); }
    delay_ms(50);
    SSD1322_DrawString(0, 5, 0, (unsigned char*)"DAC IS  MUTED");
}

/**
  * @brief  Настройка IR-приёмника на PB5 (EXTI, подтяжка, прерывание)
  * @param  None
  * @retval None
  */

// =====================================================================================================================================================================
//   УМНАЯ ФУНКЦИЯ ОТРИСОВКИ ДЛЯ ПЯТИ БУКВ СЛОВА "ALEKS"
//

// ============================================================
//   ВЫВОД ВСЕЙ НАДПИСИ "ALEKS" ПОСТРОЧНО С ВЫРАВНИВАНИЕМ
// ============================================================
void SSD1322_DrawAleksFull(uint8_t start_col_addr, uint8_t start_row, uint16_t time) {	// X,Y,time
			SSD1322_CommandWrite(0x15);
			SSD1322_DataWrite(start_col_addr);           // Начало (например, 0x1C)
			SSD1322_DataWrite(start_col_addr + 50 - 1);  // Конец окна
			SSD1322_CommandWrite(0x75);
			SSD1322_DataWrite(start_row);
			SSD1322_DataWrite(start_row + 24 - 1);       // Высота шрифта
			SSD1322_CommandWrite(0x5C); // Начинаем лить данные в RAM
			for (uint8_t row = 0; row < 24; row++) {											// Проходим по всем 5 буквам слова "ALEKS"200 пикселей / 4 = 50 единиц адресации столбцов.
			for (uint8_t ch = 0; ch < 5; ch++) {													// Указатель на начало текущей буквы (шаг 38 колонок) 5 букв по 40 пикселей (38 + 2 пустых) = 200 пикселей.
			const uint32_t *char_ptr = &Font_Aleks38x24[(ch * 39) + 1];		//const uint32_t *char_ptr = &Font_Aleks38x24[ch * 38];// Выводим 19 байт (38 пикселей) текущей буквы
			for (uint8_t b = 0; b < 19; b++) {
			uint8_t out_byte = 0x00;		// Проверяем бит в вертикальной колонке массива	
				if (char_ptr[b * 2]     & (1UL << row)) out_byte |= 0xF0;		// Если буква смещена на 1 пиксель, используй (1UL << (row + 1))
				if (char_ptr[b * 2 + 1] & (1UL << row)) out_byte |= 0x0F;
				SSD1322_DataWrite(out_byte);																// ДОБИВКА: 20-й пустой байт (2 пикселя), чтобы буква стала кратна 4
}
				SSD1322_DataWrite(0x00);																		// Это гарантирует, что контроллер ровно закроет "шаг" адресации
		}
	}

			delay_ms(time);																								//Заставка держится 1 секунду
			SSD1322_ClearRAM();
}


// ====================================================================================================================================================================



void IR_Init(void) {
	// ============================================================
// ИСПРАВЛЕННАЯ ИНИЦИАЛИЗАЦИЯ ИК-ПОРТА И ТАЙМЕРА TIM3
// ============================================================

    // 1. Настройка пина PB5 как входа с подтяжкой
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

    GPIOB->CRL &= ~(GPIO_CRL_CNF5 | GPIO_CRL_MODE5);
    GPIOB->CRL |= GPIO_CRL_CNF5_1;          // вход с подтяжкой
    GPIOB->ODR |= (1 << 5);                 // подтяжка к питанию

    // 2. Настройка EXTI на PB5 (СТРОГО ПО СПАДУ)
    RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
    AFIO->EXTICR[1] &= ~AFIO_EXTICR2_EXTI5;
    AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI5_PB;

    EXTI->IMR |= EXTI_IMR_MR5;
    EXTI->FTSR |= EXTI_FTSR_TR5;   // по спаду (HIGH > LOW)
    EXTI->RTSR &= ~EXTI_RTSR_TR5;  // по подъёму — ОТКЛЮЧЕНО!

    NVIC_EnableIRQ(EXTI9_5_IRQn);
    NVIC_SetPriority(EXTI9_5_IRQn, 1);

    // 3. НАСТРОЙКА ТАЙМЕРА TIM3 ДЛЯ ЗАМЕРА ВРЕМЕНИ
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;      // включаем тактирование TIM3
    TIM3->PSC = 719;                           // предделитель 719 (шаг 10 мкс)
    TIM3->ARR = 0xFFFF;                  // максимум (16 бит)
    TIM3->CR1 |= TIM_CR1_CEN;                // запускаем счёт
}



/* ============================================================
										M A I N
   ============================================================ */

/**
  * @brief  Главная функция (точка входа)
  * @param  None
  * @retval int (не используется)
  */
int main(void) {
	//==========================================================================================
	     // ====================================================================
    // 1. СНАЧАЛА ЧИТАЕМ ДАННЫЕ ИЗ ФЛЕШ-ПАМЯТИ И РАСКИДЫВАЕМ ИХ
    // ====================================================================
        FLASH_ReadSettings();
    
    // ВРЕМЕННО НА ОДИН РАЗ КОММЕНТИРУЕМ IF, ЧТОБЫ ПЕРЕЗАПИСАТЬ ФОРМАТ ПАМЯТИ
    // if (preset.input_select == 0xFF) {
        preset.input_select = 1;
        preset.digital_filter = 0;
        preset.contrast = 0x8F;
        preset.volume_int = -400; // Это наши -40.0 дБ, умноженные на 10
        preset.balance_int = 0;   // Баланс 0
        
      //  FLASH_WriteSettings();
    // }
    
    // РАСКИДЫВАЕМ ИЗ СТРУКТУРЫ ПО РАБОЧИМ ПЕРЕМЕННЫМ
    currentInput   = (uint8_t)preset.input_select;
    currentFilter  = (uint8_t)preset.digital_filter;
    
    // Переводим обратно во float! -400 / 10.0f даст честные -40.0f
    currentVolume  = (float)preset.volume_int / 10.0f;
    currentBalance = (float)preset.balance_int / 10.0f;


    // ====================================================================
    // 2. И ТОЛЬКО ТЕПЕРЬ ЗАПУСКАЕМ ИНИЦИАЛИЗАЦИЮ ПЕРИФЕРИИ И ДИСПЛЕЯ
    // ====================================================================
    DWT_Init();               // Инициализация DWT для задержек
    DAC_port_Init();          // Настройка пинов для ЦАП
    Encoder_Init();           // Инициализация энкодера
    Control_port_Init();      // Настройка управляющих пинов
    UART_Init();              // Инициализация UART для приёма от XMOS
    I2C2_Init();              // Инициализация I2C
    IR_Init();                // Инициализация IR-приёмника
    
    // Дисплей инициализируем ПОСЛЕДНИМ в этой пачке, когда I2C и SPI уже готовы,
    // и когда данные контраста (preset.contrast) уже лежат в оперативной памяти!
    SSD1322_Init();           // Инициализация дисплея
    
    // ----- Настройка TIM4 (1 кГц) -----
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    TIM4->PSC = SystemCoreClock / 10000 - 1;
    TIM4->ARR = 10000;
    TIM4->DIER |= TIM_DIER_UIE;
    TIM4->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM4_IRQn);


    // Настройка режимов работы ЦАП
  /*  MASTER_DAC(input_select);
    Master_Slave_Sel(input_select);
    DSD_DAC(dsd_conf);
    MCLK_DAC(mclk_conf);

    delay_ms(50);
*/
  /*  // Установка фильтра
    switch (dig_flt) {
        case SHARP_FLT:   SLOW_BIT_DAC(1); SD_BIT_DAC(1); break;
        case SLOW_FLT:    SLOW_BIT_DAC(0); SD_BIT_DAC(1); break;
        case SHARP_SD_FLT: SLOW_BIT_DAC(1); SD_BIT_DAC(0); break;
        case SLOW_SD_FLT:  SLOW_BIT_DAC(0); SD_BIT_DAC(0); break;
    }*/

    // Настройка светодиода PC13
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
    GPIOC->CRH &= ~(GPIO_CRH_MODE13 | GPIO_CRH_CNF13);
    GPIOC->CRH |= GPIO_CRH_MODE13_0;
    GPIOC->ODR &= ~(1 << 13);

    Connect_DAC(1);
// ----- ПРИВЕТСТВЕННЫЙ ЭКРАН -----
//SSD1322_DrawString(0, 19,0, (unsigned char*)"  ALEKS  ");
//delay_ms(100);
/*SSD1322_ClearRAM();
SSD1322_DrawString(0, 0, 0,(unsigned char*)"USB DSD 44.1");
//delay_ms(1000);
SSD1322_DrawString(0, 34, 0,(unsigned char*)"VOLUME");	//X,Y,TEXT
SSD1322_DrawString(100, 34,0, (unsigned char*)"120dB");	//X,Y,TEXT		
delay_ms(10000);
SSD1322_DrawString(0, 0, 1,(unsigned char*)"USB PCM 768 ");		
//SSD1322_ClearRAM();
*/


SSD1322_DrawAleksFull(35, 20, 1000);/// Пример вызова: X=30 (байт), Y=20 (строка)

SSD1322_DrawString(0, 0, 0,(unsigned char*)"USB DSD 44.1");
// ============================================================

uint16_t last_encoder_value = 0; // Наш эталон для сравнения ручки

//===========================================================================================================================================================================
    while (1) {
				if (menu_need_update) {
        menu_need_update = 0; // Сбрасываем флаг
        Update_Bottom_Line(); // Спокойно и не спеша шлём данные в SPI в фоне!

               // ====================================================================
        // ОБРАБОТКА ЭНКОДЕРА ГРОМКОСТИ ПРЯМЫМ ХОДОМ
        // ====================================================================
        if (TIM3->CNT > last_encoder_value) {
            // Крутанули ВПРАВО — прибавляем звук на 0.5 дБ
            currentVolume += 0.5f;
            if (currentVolume > 0.0f) currentVolume = 0.0f; // Ограничение максимума
            
            Set_Volume_And_Balance(currentVolume, currentBalance); // Пуляем в ЦАП!
            menu_need_update = 1; // Флаг на отрисовку экрана
            last_encoder_value = TIM3->CNT;
        }
        else if (TIM3->CNT < last_encoder_value) {
            // Крутанули ВЛЕВО — убавляем звук на 0.5 дБ
            currentVolume -= 0.5f;
            if (currentVolume < -127.5f) currentVolume = -127.5f; // Полная тишина
            
            Set_Volume_And_Balance(currentVolume, currentBalance); // Пуляем в ЦАП!
            menu_need_update = 1;
            last_encoder_value = TIM3->CNT;
        }

					// ====================================================================
        // 1. АППАРАТНЫЙ ТАЙМАУТ ОТПУСКАНИЯ КНОПКИ ПУЛЬТА (0.2 сек = 2000 тиков)
        // ====================================================================
        uint16_t current_tick = TIM4->CNT;
        uint16_t ir_diff = (current_tick >= ir_last_tick) ? (current_tick - ir_last_tick) : (current_tick + 10000 - ir_last_tick);

        if (ir_diff > 2000) {
            if (hold_counter > 0) {
                hold_counter = 0; // Кнопку точно отпустили!
            }
        }

        // ====================================================================
        // 2. ОБРАБОТКА ИК-ПУЛЬТА (Привязка к кнопкам и плавному удержанию)
        // ====================================================================
        if (IR_GetPacket(remote_packet)) {
                ir_cmd = remote_packet[2]; // Достаем 3-й байт команды из прилетевшего пакета!

            // Если прилетел пакет (или удерживается старый) - вычисляем действие
            // В Watch-окне Keil мы ловим 3-й байт команды: remote_packet[3]
           // uint8_t ir_cmd = remote_packet[3];
            
            switch (ir_cmd) {
                case 0x1E: // ПРИМЕР: Кнопка "Громкость +"
                    // Если кнопку зажали - прибавляем быстрее, если клик - по 0.5 дБ
                    if (hold_counter > 5) currentVolume += 1.0f; 
                    else currentVolume += 0.5f;
                    
                    if (currentVolume > 0.0f) currentVolume = 0.0f; // Ограничение максимума
                    
                    Set_Volume_And_Balance(currentVolume, currentBalance);
                    menu_need_update = 1;
                    break;

                case 0x1F: // ПРИМЕР: Кнопка "Громкость -"
                    if (hold_counter > 5) currentVolume -= 1.0f;
                    else currentVolume -= 0.5f;
                    
                    if (currentVolume < -127.5f) currentVolume = -127.5f; // Полная тишина
                    
                    Set_Volume_And_Balance(currentVolume, currentBalance);
                    menu_need_update = 1;
                    break;
                    
                case 0x20: // ПРИМЕР: Кнопка переключения входов
                    if (hold_counter == 0) { // Только на одиночный клик, чтобы не прыгало!
                        currentInput++;
                        if (currentInput > 3) currentInput = 1;
                        ESS9028_SetInput(currentInput);
                        menu_need_update = 1;
                    }
                    break;
            }
        }

        // ====================================================================
        // 3. ОБРАБОТКА ФИЗИЧЕСКИХ ЭНКОДЕРОВ (Ручное управление)
        // ====================================================================
        // Твой готовый код антидребезга и опроса фаз А и В
        // Пример интеграции в каркас:
        /*
        if (Encoder_Get_Turn() == ENCODER_RIGHT) {
            if (current_mode == MODE_VOLUME) {
                currentVolume += 0.5f;
                if (currentVolume > 0.0f) currentVolume = 0.0f;
                Set_Volume_And_Balance(currentVolume, currentBalance);
            }
            menu_need_update = 1;
        }
        */

        // ====================================================================
        // 4. ОБНОВЛЕНИЕ СТРОКИ OLED-ДИСПЛЕЯ ПО ФЛАГУ
        // ====================================================================
        if (menu_need_update) {
            menu_need_update = 0;
            
            // Вызываем твою отрисовку. Внутри нее sprintf будет красиво выводить
            // наши новые вещественные переменные типа: sprintf(buf, "Vol: %.1f dB", currentVolume);
            Update_Bottom_Line(); 
        }
    }

// ============================================================


		// АППАРАТНЫЙ ТАЙМАУТ ПУЛЬТА 0.2 СЕКУНДЫ (2000 ТИКОВ)
        uint16_t current_tick = TIM4->CNT;
        uint16_t ir_diff = (current_tick >= ir_last_tick) ? (current_tick - ir_last_tick) : (current_tick + 10000 - ir_last_tick);

        if (ir_diff > 2000) { // Если тишина длится дольше 2000 тиков (0.2 сек)
            if (hold_counter > 0) {
                hold_counter = 0; //СБРОС: Палец с пульта убрали!
            }
        }
		

				
				Set_Volume_And_Balance(currentVolume, currentBalance);			//  ОТПРАВКА НАСТРОЕК ЗВУКА В РЕГИСТРЫ ЦАП       
    }
		
		
}
void Process_XMOS_Signal(void) {												// ----- ОБРАБОТКА СИГНАЛА ОТ XMOS-XU316 pin12 -----
         uint8_t signal = UART_XMOS_GetSignal();
        if (signal != 0) {
            switch (signal) {
                case 0x01: SSD1322_DrawString(0, 5, 0, (unsigned char*)"PCM   44.1kHz"); break;
                case 0x02: SSD1322_DrawString(0, 5, 0, (unsigned char*)"PCM     48kHz"); break;
                case 0x03: SSD1322_DrawString(0, 5, 0, (unsigned char*)"PCM   88.2kHz"); break;
                case 0x04: SSD1322_DrawString(0, 5, 0, (unsigned char*)"PCM     96kHz"); break;
                case 0x05: SSD1322_DrawString(0, 5, 0, (unsigned char*)"PCM  176.4kHz"); break;
                case 0x06: SSD1322_DrawString(0, 5, 0, (unsigned char*)"PCM    192kHz"); break;
                case 0x07: SSD1322_DrawString(0, 5, 0, (unsigned char*)"PCM  352.8kHz"); break;
                case 0x08: SSD1322_DrawString(0, 5, 0, (unsigned char*)"PCM    384kHz"); break;
                case 0x09: SSD1322_DrawString(0, 5, 0, (unsigned char*)"PCM  705.6kHz"); break;
                case 0x0A: SSD1322_DrawString(0, 5, 0, (unsigned char*)"PCM    768kHz"); break;
                case 0x0B: SSD1322_DrawString(0, 5, 0, (unsigned char*)"PCM  1411.2kHz"); break;
                case 0x0C: SSD1322_DrawString(0, 5, 0, (unsigned char*)"PCM    1536kHz"); break;
                case 0x19: SSD1322_DrawString(0, 5, 0, (unsigned char*)"DSD64   2.822"); break;
                case 0x1A: SSD1322_DrawString(0, 5, 0, (unsigned char*)"DSD128  5.644"); break;
                case 0x1B: SSD1322_DrawString(0, 5, 0, (unsigned char*)"DSD256 11.289"); break;
                case 0x1C: SSD1322_DrawString(0, 5, 0, (unsigned char*)"DSD512 22.579"); break;
                case 0x1D: SSD1322_DrawString(0, 5, 0, (unsigned char*)"DSD1024 45.15"); break;
            }
        }
		}