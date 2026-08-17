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
    uint8_t input_select;      // 0 – USB, 1 – S/PDIF
    uint8_t digital_filter;    // 0..3
    uint8_t dsd_config;        // 0/1
    uint8_t mclk_config;       // 0/1
    uint16_t contrast;         // яркость
    uint16_t grayscale;        // (не используется)
} presets_def;

static presets_def preset;

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
    while (I2C2->SR2 & I2C_SR2_BUSY);
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

/**
  * @brief  Основная логика нажатия кнопки
  * @param  None
  * @retval None
  */
void ProcessButtonPress(void) {
    // Если звук выключен (Mute) — включаем
    if (mute_state) {
        SSD1322_CommandWrite(0xAF);
        mute_state = 0;
        halt_counter = 0;
    }

    menu_counter = 1;  // Запускаем таймаут меню

    // Навигация по меню
    switch (menu_level) {
        case 0:  // Главный экран > вход в меню
            menu_level = 1;
            updated = 0;
            break;

        case 1:  // Выбор входа > переход к USB или S/PDIF
            menu_level = (input_select == USB_SOURCE) ? 6 : 7;
            break;

        case 2:  // Выбор фильтра > переход к конкретному типу
            switch (dig_flt) {
                case SHARP_FLT:   menu_level = 8;  break;
                case SLOW_FLT:    menu_level = 9;  break;
                case SHARP_SD_FLT: menu_level = 10; break;
                case SLOW_SD_FLT:  menu_level = 11; break;
            }
            break;

        case 3:  // DSD конфиг > вкл/выкл
            menu_level = (dsd_conf == DSD_AUTO_ON) ? 12 : 13;
            break;

        case 4:  // MCLK конфиг > вкл/выкл
            menu_level = (mclk_conf == MCLK_AUTO_ON) ? 14 : 15;
            break;

        case 5:  // Сохранение настроек и выход
            preset.input_select   = input_select;
            preset.digital_filter = dig_flt;
            preset.dsd_config     = dsd_conf;
            preset.mclk_config    = mclk_conf;
            preset.contrast       = contrast;
            FLASH_WriteSettings();
            updated = 0;
            menu_level = 0;
            menu_counter = 0;
            break;

        // ----- Установка значений -----
        case 6:  menu_level = 1; input_select = USB_SOURCE; MASTER_DAC(input_select); Master_Slave_Sel(input_select); break;
        case 7:  menu_level = 1; input_select = SPDIF_SOURCE; MASTER_DAC(input_select); Master_Slave_Sel(input_select); break;
        case 8:  menu_level = 2; dig_flt = SHARP_FLT; SLOW_BIT_DAC(1); SD_BIT_DAC(1); break;
        case 9:  menu_level = 2; dig_flt = SLOW_FLT; SLOW_BIT_DAC(0); SD_BIT_DAC(1); break;
        case 10: menu_level = 2; dig_flt = SHARP_SD_FLT; SLOW_BIT_DAC(1); SD_BIT_DAC(0); break;
        case 11: menu_level = 2; dig_flt = SLOW_SD_FLT; SLOW_BIT_DAC(0); SD_BIT_DAC(0); break;
        case 12: menu_level = 3; dsd_conf = DSD_AUTO_ON; DSD_DAC(dsd_conf); break;
        case 13: menu_level = 3; dsd_conf = DSD_AUTO_OFF; DSD_DAC(dsd_conf); break;
        case 14: menu_level = 4; mclk_conf = MCLK_AUTO_ON; MCLK_DAC(mclk_conf); break;
        case 15: menu_level = 4; mclk_conf = MCLK_AUTO_OFF; MCLK_DAC(mclk_conf); break;
        case 16: menu_level = 17; break;  // Переход к яркости
        case 17: menu_level = 16; break;  // Обратно
    }
}

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

    // IR-приёмник на PB5
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
                // Навигация по меню
                if (enc_direction) {
                    menu_counter = 1;
                    switch (menu_level) {
                        case 1:  menu_level = 2; break;
                        case 2:  menu_level = 3; break;
                        case 3:  menu_level = 4; break;
                        case 4:  menu_level = 16; break;
                        case 5:  menu_level = 1; break;
                        case 16: menu_level = 5; break;
                        case 6:  menu_level = 7; break;
                        case 7:  menu_level = 6; break;
                        case 8:  menu_level = 9; break;
                        case 9:  menu_level = 10; break;
                        case 10: menu_level = 11; break;
                        case 11: menu_level = 8; break;
                        case 12: menu_level = 13; break;
                        case 13: menu_level = 12; break;
                        case 14: menu_level = 15; break;
                        case 15: menu_level = 14; break;
                        case 17: if (contrast < 0xff) contrast += 0x10; break;
                    }
                } else {
                    switch (menu_level) {
                        case 1:  menu_level = 5; break;
                        case 2:  menu_level = 1; break;
                        case 3:  menu_level = 2; break;
                        case 4:  menu_level = 3; break;
                        case 16: menu_level = 4; break;
                        case 5:  menu_level = 16; break;
                        case 6:  menu_level = 7; break;
                        case 7:  menu_level = 6; break;
                        case 8:  menu_level = 11; break;
                        case 9:  menu_level = 8; break;
                        case 10: menu_level = 9; break;
                        case 11: menu_level = 10; break;
                        case 12: menu_level = 13; break;
                        case 13: menu_level = 12; break;
                        case 14: menu_level = 15; break;
                        case 15: menu_level = 14; break;
                        case 17: if (contrast > 0x10) contrast -= 0x10; break;
                    }
                }
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

/**
  * @brief  Вывод меню на экран
  * @param  None
  * @retval None
  */
void MENU_LCD(void) {
    switch (menu_level) {
        case 1: if (!updated) { updated = 1; SSD1322_ClearRAM(); } SSD1322_DrawString(0, 19, (unsigned char*)"INPUT  SELECT"); break;
        case 2: SSD1322_DrawString(0, 19, (unsigned char*)" DIGITAL FLT "); break;
        case 3: SSD1322_DrawString(0, 19, (unsigned char*)"  DSD CONFIG "); break;
        case 4: SSD1322_DrawString(0, 19, (unsigned char*)" SAMPLE RATE "); break;
        case 5: SSD1322_DrawString(0, 19, (unsigned char*)"    EXIT     "); break;
        case 6: SSD1322_DrawString(0, 19, (unsigned char*)" USB SELECT  "); break;
        case 7: SSD1322_DrawString(0, 19, (unsigned char*)"S/PDIF SELECT"); break;
        case 8: SSD1322_DrawString(0, 19, (unsigned char*)"SHARP    DGFL"); break;
        case 9: SSD1322_DrawString(0, 19, (unsigned char*)"SLOW     DGFL"); break;
        case 10: SSD1322_DrawString(0, 19, (unsigned char*)"SHARP SD DGFL"); break;
        case 11: SSD1322_DrawString(0, 19, (unsigned char*)"SLOW SD  DGFL"); break;
        case 12: SSD1322_DrawString(0, 19, (unsigned char*)" DSD AUTO ON "); break;
        case 13: SSD1322_DrawString(0, 19, (unsigned char*)" DSD AUTO OFF"); break;
        case 14: SSD1322_DrawString(0, 19, (unsigned char*)" AUTOSAMPLE  "); break;
        case 15: SSD1322_DrawString(0, 19, (unsigned char*)"NO AUTOSAMPLE"); break;
        case 16: SSD1322_DrawString(0, 19, (unsigned char*)" BRIGHTNESS  "); break;
        case 17:
            SSD1322_CmdDataWrite(0xc1, contrast);
            sprintf(buffer, "BRIGHTNESS:%02i", (contrast / 0x10));
            SSD1322_DrawString(0, 19, (unsigned char*)buffer);
            break;
    }
}

/**
  * @brief  Вывод информации о входе и частоте (главный экран)
  * @param  None
  * @retval None
  */
void STREAM_LCD(void) {
    if (!updated) { updated = 1; SSD1322_ClearRAM(); }

    if (!input_select) {
        switch (Stream_ID()) {
            case 0x01: SSD1322_DrawString(0, 5, (unsigned char*)"PCM   44.1kHz"); break;
            case 0x02: SSD1322_DrawString(0, 5, (unsigned char*)"PCM     48kHz"); break;
            case 0x03: SSD1322_DrawString(0, 5, (unsigned char*)"PCM   88.2kHz"); break;
            case 0x04: SSD1322_DrawString(0, 5, (unsigned char*)"PCM     96kHz"); break;
            case 0x05: SSD1322_DrawString(0, 5, (unsigned char*)"PCM  176.4kHz"); break;
            case 0x06: SSD1322_DrawString(0, 5, (unsigned char*)"PCM    192kHz"); break;
            case 0x07: SSD1322_DrawString(0, 5, (unsigned char*)"PCM  352.8kHz"); break;
            case 0x08: SSD1322_DrawString(0, 5, (unsigned char*)"PCM    384kHz"); break;
            case 0x09: SSD1322_DrawString(0, 5, (unsigned char*)"PCM  705.6kHz"); break;
            case 0x0A: SSD1322_DrawString(0, 5, (unsigned char*)"PCM    768kHz"); break;
            case 0x0B: SSD1322_DrawString(0, 5, (unsigned char*)"PCM  1411.2kHz"); break;
            case 0x0C: SSD1322_DrawString(0, 5, (unsigned char*)"PCM    1536kHz"); break;
            case 0x19: SSD1322_DrawString(0, 5, (unsigned char*)"DSD64   2.822"); break;
            case 0x1A: SSD1322_DrawString(0, 5, (unsigned char*)"DSD128  5.644"); break;
            case 0x1B: SSD1322_DrawString(0, 5, (unsigned char*)"DSD256 11.289"); break;
            case 0x1C: SSD1322_DrawString(0, 5, (unsigned char*)"DSD512 22.579"); break;
            case 0x1D: SSD1322_DrawString(0, 5, (unsigned char*)"DSD1024 45.15"); break;
        }
    } else {
        SSD1322_DrawString(0, 5, (unsigned char*)"S/PDIF  INPUT");
    }
}

/**
  * @brief  Вывод информации о фильтре и входе
  * @param  None
  * @retval None
  */
void INPUT_FLT_LCD(void) {
    if (!input_select) {
        switch (dig_flt) {
            case 0x00: SSD1322_DrawString(0, 40, (unsigned char*)"USB     SHARP"); break;
            case 0x01: SSD1322_DrawString(0, 40, (unsigned char*)"USB      SLOW"); break;
            case 0x02: SSD1322_DrawString(0, 40, (unsigned char*)"USB  SHARP SD"); break;
            case 0x03: SSD1322_DrawString(0, 40, (unsigned char*)"USB   SLOW SD"); break;
        }
    } else {
        switch (dig_flt) {
            case 0x00: SSD1322_DrawString(0, 40, (unsigned char*)"SHARP    DGFL"); break;
            case 0x01: SSD1322_DrawString(0, 40, (unsigned char*)"SLOW     DGFL"); break;
            case 0x02: SSD1322_DrawString(0, 40, (unsigned char*)"SHARP SD DGFL"); break;
            case 0x03: SSD1322_DrawString(0, 40, (unsigned char*)"SLOW SD  DGFL"); break;
        }
    }
}

/**
  * @brief  Вывод сообщения о Mute
  * @param  None
  * @retval None
  */
void MUTED_LCD(void) {
    if (!updated) { updated = 1; SSD1322_ClearRAM(); }
    delay_ms(50);
    SSD1322_DrawString(0, 5, (unsigned char*)"DAC IS  MUTED");
}

/**
  * @brief  Настройка IR-приёмника на PB5 (EXTI, подтяжка, прерывание)
  * @param  None
  * @retval None
  */

// ============================================================
// ЧИСТАЯ ФУНКЦИЯ В MAIN.C ДЛЯ ОГРОМНОГО ШРИФТА (БЕЗ ПРОВЕРОК)
// ============================================================

void SSD1322_DrawAleksChar(uint16_t x, uint16_t y, char c) {
    // Вычисляем точный индекс буквы в массиве.
    // Если твой массив начинается с пробела (код 32), пишем: c - 32
    // Если твой массив начинается с буквы 'A' (код 65), пишем: c - 65
    uint16_t char_index = c - 32; 
    
    // Берем указатель на начало нужной буквы. Шаг строго 38 колонок!
    const uint32_t *char_data = &Font_Aleks38x24[char_index * 38];

    // Пробегаем по ширине символа (38 вертикальных колонок)
    for (uint8_t col = 0; col < 38; col++) {
        uint32_t column_pixels = char_data[col];

        // Пробегаем по высоте символа (24 пикселя)
        for (uint8_t row = 0; row < 24; row++) {
            // Проверяем биты в колонке
            if (column_pixels & (1UL << row)) {
                SSD1322_DrawPixel(x + col, y + row, 0x0F); // Горит на полную холодным белым!
            } else {
                SSD1322_DrawPixel(x + col, y + row, 0x00); // Черный фон
            }
        }
    }
}

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

// ============================================================


   /* // 1. Настройка пина PB5 как входа с подтяжкой
    RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;

    GPIOB->CRL &= ~(GPIO_CRL_CNF5 | GPIO_CRL_MODE5);
    GPIOB->CRL |= GPIO_CRL_CNF5_0;          // вход с подтяжкой
    GPIOB->ODR |= (1 << 5);                 // подтяжка к питанию

    // 2. Настройка EXTI на PB5 (прерывание по СПАДУ и ПОДЪЁМУ)
		RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;
		AFIO->EXTICR[1] &= ~AFIO_EXTICR2_EXTI5;
		AFIO->EXTICR[1] |= AFIO_EXTICR2_EXTI5_PB;

		EXTI->IMR |= EXTI_IMR_MR5;
		EXTI->FTSR |= EXTI_FTSR_TR5;   // по спаду (HIGH > LOW)
		EXTI->RTSR |= EXTI_RTSR_TR5;   // по подъёму (LOW > HIGH)  <-- ДОБАВИТЬ!

    NVIC_EnableIRQ(EXTI9_5_IRQn);
    NVIC_SetPriority(EXTI9_5_IRQn, 1);

    // 3. НАСТРОЙКА ТАЙМЕРА TIM3 ДЛЯ ЗАМЕРА ВРЕМЕНИ (добавлено!)
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;      // включаем тактирование TIM3
    TIM3->PSC = 719;                           // предделитель 1 (считаем такты)
    TIM3->ARR = 0xFFFF;                  // максимум (16 бит)
    TIM3->CR1 |= TIM_CR1_CEN;                // запускаем счёт*/

// ============================================================




// ============================================================
//  ВИЗУАЛЬНЫЙ ОТЛАДЧИК НА ЭКРАНЕ (показывает число)
// ============================================================

void DebugNum(uint16_t value) {
    char buf[16];
    sprintf(buf, "DBG %d", value);
    SSD1322_ClearBuffer();
    SSD1322_DrawStringBuffer(0, 0, buf);
    SSD1322_Update(0x00, 0x1F);
}
// ============================================================
//  ВИЗУАЛЬНЫЙ ОТЛАДЧИК НА ЭКРАНЕ
//  Выводит номер шага или символ на дисплей
// ============================================================
		void DebugShow(uint8_t step) {
    char buf[16];
    sprintf(buf, "DBG %d", step);
    SSD1322_ClearBuffer();
    SSD1322_DrawStringBuffer(25, 0, buf);//(x, y, *str)
    SSD1322_Update(0x20, 0x3F);//0x1F);
}
// ============================================================
//  ФУНКЦИЯ ДЛЯ ВЫВОДА ДЛИТЕЛЬНОСТИ НА ЭКРАН (отладка)
// ============================================================

void DebugDuration(uint32_t duration) {
    char buf[16];
		//sprintf(buf, "0x%04X", (uint16_t)duration);
    sprintf(buf, "DUR %lu", duration);
    SSD1322_ClearBuffer();
    SSD1322_DrawStringBuffer(0, 0, buf);
    SSD1322_Update(0x00, 0x1F);
}

/* ============================================================
   11. ОСНОВНАЯ ПРОГРАММА
   ============================================================ */

/**
  * @brief  Главная функция (точка входа)
  * @param  None
  * @retval int (не используется)
  */
int main(void) {
    DWT_Init();                // Инициализация DWT для задержек
    SSD1322_Init();            // Инициализация дисплея
    DAC_port_Init();           // Настройка пинов для ЦАП
    Encoder_Init();            // Инициализация энкодера
    Control_port_Init();       // Настройка управляющих пинов
    UART_Init();               // Инициализация UART для приёма от XMOS
    I2C2_Init();               // Инициализация I2C
    IR_Init();                 // Инициализация IR-приёмника

    // ----- Настройка TIM4 (1 кГц) -----
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    TIM4->PSC = SystemCoreClock / 10000 - 1;
    TIM4->ARR = 10000;
    TIM4->DIER |= TIM_DIER_UIE;
    TIM4->CR1 |= TIM_CR1_CEN;
    NVIC_EnableIRQ(TIM4_IRQn);

    // ----- Загрузка настроек из Flash -----
    FLASH_ReadSettings();
    if (preset.input_select == 0xff) {
        preset.input_select   = 0x00;
        preset.digital_filter = 0x00;
        preset.dsd_config     = 0x01;
        preset.mclk_config    = 0x00;
        preset.contrast       = 0x7f;
        preset.grayscale      = 0x88;
        FLASH_WriteSettings();
    }

    // Применяем загруженные настройки
    input_select = preset.input_select;
    dig_flt      = preset.digital_filter;
    dsd_conf     = preset.dsd_config;
    mclk_conf    = preset.mclk_config;
    contrast     = preset.contrast;

    SSD1322_CmdDataWrite(0xc1, contrast);

    // Настройка режимов работы ЦАП
    MASTER_DAC(input_select);
    Master_Slave_Sel(input_select);
    DSD_DAC(dsd_conf);
    MCLK_DAC(mclk_conf);

    delay_ms(50);

    // Установка фильтра
    switch (dig_flt) {
        case SHARP_FLT:   SLOW_BIT_DAC(1); SD_BIT_DAC(1); break;
        case SLOW_FLT:    SLOW_BIT_DAC(0); SD_BIT_DAC(1); break;
        case SHARP_SD_FLT: SLOW_BIT_DAC(1); SD_BIT_DAC(0); break;
        case SLOW_SD_FLT:  SLOW_BIT_DAC(0); SD_BIT_DAC(0); break;
    }

    // Настройка светодиода PC13
    RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
    GPIOC->CRH &= ~(GPIO_CRH_MODE13 | GPIO_CRH_CNF13);
    GPIOC->CRH |= GPIO_CRH_MODE13_0;
    GPIOC->ODR &= ~(1 << 13);

    Connect_DAC(1);
// ----- ПРИВЕТСТВЕННЫЙ ЭКРАН -----
SSD1322_DrawString(0, 19, (unsigned char*)"  ALEKS  ");
delay_ms(1000);
SSD1322_ClearRAM();
// ============================================================
//  ТЕСТ ШРИФТА
// ============================================================
/*
SSD1322_ClearBuffer();
SSD1322_DrawStringBuffer(0, 0, "123456789ABCDEF");//"ABCDEFGHIJKLMNOPQRSTUVWXYZ");
SSD1322_Update(0x10, 0x2F);
		*/
		
/*SSD1322_ClearBuffer();                // очищаем буфер  
SSD1322_DrawCharBuffer(10, 5, 'f');   // рисуем букву 'A' в координатах (10,10)  
SSD1322_Update(0x00, 0x1F);            // отправляем на экран
		
// ----- 1. Рисуем сырые байты буквы 'A' -----
uint8_t raw[8];
for (int i = 0; i < 8; i++) {
    raw[i] = SmallFont['2' - 0x20][i];
}

for (int row = 0; row < 8; row++) {
    for (int col = 0; col < 6; col++) {
        if (raw[row] & (0x80 >> col)) {
            SSD1322_DrawPixel(10 + col, 10 + row, 0x0F);
        }
    }
}*/
/*
// ----- 2. Рисуем заведомо правильную букву 'А' (контроль) -----

uint8_t test_A[8][6] = {
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00},
    {0x00, 0x0F, 0x0F, 0x0F, 0x00, 0x00},
    {0x00, 0x0F, 0x00, 0x0F, 0x00, 0x00},
    {0x00, 0x0F, 0x0F, 0x0F, 0x00, 0x00},
    {0x00, 0x0F, 0x00, 0x0F, 0x00, 0x00},
    {0x00, 0x0F, 0x00, 0x0F, 0x00, 0x00},
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
};

for (int row = 0; row < 8; row++) {
    for (int col = 0; col < 6; col++) {
        if (test_A[row][col]) {
            SSD1322_DrawPixel(30 + col, 10 + row, 0x0F);
        }
    }
}

SSD1322_Update(0x00, 0x1F); // отправляем верхнюю половину*/



//------DEBUG DISPLAY------
//DebugShow(1);   // шаг 1

/*// Симулируем приём пакета: [0xAA, 0x55, 0xFF]
rx_buffer[0] = 0xAA;
rx_buffer[1] = 0x55; // Наш last_signal (85 в десятичном виде)
rx_buffer[2] = 0xFF;

last_signal = rx_buffer[1]; // Записываем 55 в переменную
new_signal_received = 1;     // Взводим флаг вручную

// Сразу вызываем твой отладчик для проверки
DebugShow(last_signal); */

    while (1) {

/*// ============================================================
// ОБРАБОТКА УДЕРЖАНИЯ КНОПКИ В MAIN.C
// ============================================================

uint8_t remote_packet[4];

if (IR_GetPacket(remote_packet)) {
    
    // Если прилетел код повтора удержания кнопки
    if (remote_packet[0] == 0xEE) {
        // Выводим на экран надпись или число, символизирующее повтор
        DebugShow(99); // Например, число 99 будет означать REPEAT
    } 
    // Если прилетел обычный новый пакет из 4-х байт
    else {
        // Показываем твой значащий байт команды (обычно это 3-й байт, индекс)
        DebugShow(remote_packet[2]); 
    }
}

// ============================================================*/

			
// ============================================================
// ТВОЙ ОТЛАДОЧНЫЙ ВЫВОД ВСЕХ 4 БАЙТ ПУЛЬТА В MAIN.C
// ============================================================
uint8_t remote_packet[4];

if (IR_GetPacket(remote_packet)) {
    // По очереди показываем все 4 байта с паузой, чтобы успеть записать
    DebugShow(remote_packet[0]); delay_ms(1000); // Показывает 1-й байт
    DebugShow(remote_packet[1]); delay_ms(1000); // Показывает 2-й байт
    DebugShow(remote_packet[2]); delay_ms(1000); // Показывает 3-й байт (КОМАНДА)
    DebugShow(remote_packet[3]); delay_ms(1000); // Показывает 4-й байт
}
// ============================================================

		/*	if (ir_packet_ready) {
    char buf[64];
    char *ptr = buf;
    for (int i = 0; i < ir_duration_index && i < 10; i++) {
        ptr += sprintf(ptr, "%lu ", ir_durations[i]);
    }
    SSD1322_ClearBuffer();
    SSD1322_DrawStringBuffer(0, 0, buf);
    SSD1322_Update(0x00, 0x1F);
    ir_packet_ready = 0;
}*/
			
		
	// ----- ПРОВЕРКА I2C (ES9039) -----
//uint8_t test_addr = 0x48;   // адрес ES9039 (или 0x4A, если ADDR подтянут)
//I2C2_WriteByte(test_addr, 0x00);  // просто отправляем байт
// Если чип ответит — он не зависнет, если нет — уйдёт в BUSY

// Вывод результата на экран
//char i2c_buf[16];
//sprintf(i2c_buf, "I2C: 0x%02X", test_addr);
//SSD1322_ClearBuffer();
//SSD1322_DrawStringBuffer(0, 0, i2c_buf);
//SSD1322_Update(0x00, 0x1F);

}		//Убрать для включения вывода на дисплей всё что ниже

        // ----- ЧТЕНИЕ СОСТОЯНИЯ MUTE -----
        mute_on = MUTE_State();

        // ----- МЕНЮ -----
        if (menu_level) {
            MENU_LCD();
            if (menu_counter == menu_timer_sec) {
                menu_counter = 0;
                menu_level = 0;
                SSD1322_ClearRAM();
            }
        } else {
            // ----- ГЛАВНЫЙ ЭКРАН -----
            switch (mute_on) {
                case 0:  // Звук включён
                    if (mute_state) {
                        SSD1322_CommandWrite(0xAF);
                        mute_state = 0;
                        halt_counter = 0;
                    }
                     STREAM_LCD();   // временно отключено
                    INPUT_FLT_LCD();  // вывод фильтра
                    break;

                case 1:  // Звук выключен (Mute)
                    if (!halt_counter && !mute_state) halt_counter = 1;
                    if (halt_counter == hall_timer_sec) {
                        SSD1322_CommandWrite(0xAE);
                        mute_state = 1;
                    }
                     MUTED_LCD();   // временно отключено
                     INPUT_FLT_LCD();
                    break;
            }
        }

        // ----- ОБРАБОТКА СИГНАЛА ОТ XMOS -----
        uint8_t signal = UART_XMOS_GetSignal();
        if (signal != 0) {
            switch (signal) {
                case 0x01: SSD1322_DrawString(0, 5, (unsigned char*)"PCM   44.1kHz"); break;
                case 0x02: SSD1322_DrawString(0, 5, (unsigned char*)"PCM     48kHz"); break;
                case 0x03: SSD1322_DrawString(0, 5, (unsigned char*)"PCM   88.2kHz"); break;
                case 0x04: SSD1322_DrawString(0, 5, (unsigned char*)"PCM     96kHz"); break;
                case 0x05: SSD1322_DrawString(0, 5, (unsigned char*)"PCM  176.4kHz"); break;
                case 0x06: SSD1322_DrawString(0, 5, (unsigned char*)"PCM    192kHz"); break;
                case 0x07: SSD1322_DrawString(0, 5, (unsigned char*)"PCM  352.8kHz"); break;
                case 0x08: SSD1322_DrawString(0, 5, (unsigned char*)"PCM    384kHz"); break;
                case 0x09: SSD1322_DrawString(0, 5, (unsigned char*)"PCM  705.6kHz"); break;
                case 0x0A: SSD1322_DrawString(0, 5, (unsigned char*)"PCM    768kHz"); break;
                case 0x0B: SSD1322_DrawString(0, 5, (unsigned char*)"PCM  1411.2kHz"); break;
                case 0x0C: SSD1322_DrawString(0, 5, (unsigned char*)"PCM    1536kHz"); break;
                case 0x19: SSD1322_DrawString(0, 5, (unsigned char*)"DSD64   2.822"); break;
                case 0x1A: SSD1322_DrawString(0, 5, (unsigned char*)"DSD128  5.644"); break;
                case 0x1B: SSD1322_DrawString(0, 5, (unsigned char*)"DSD256 11.289"); break;
                case 0x1C: SSD1322_DrawString(0, 5, (unsigned char*)"DSD512 22.579"); break;
                case 0x1D: SSD1322_DrawString(0, 5, (unsigned char*)"DSD1024 45.15"); break;
            }
        }
    }
