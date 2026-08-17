/*
 * ============================================================
 *  ФАЙЛ ДРАЙВЕРА ДЛЯ SSD1322 (256x64, SPI)
 *  Платформа: STM32F103C8T6 (Blue Pill)
 *  Содержит все функции для работы с дисплеем
 * ============================================================
 */

#include "gpio.h"
#include "Delay.h"
#include "spi_ssd1322.h"
#include "font.h"
#include "font_small.h"
#include "small_dot_font.h"
typedef struct {
    uint8_t input_select;
    uint8_t digital_filter;
    uint8_t contrast;
    uint8_t reserved;
    float   volume;
    float   balance;
} presets_def;

extern volatile presets_def preset;

/* ============================================================
   1.  ПИНЫ И РАЗВОДКА (аппаратная часть)
   ============================================================ */

#define SSD1322_PORT     GPIOA   // порт для всех пинов дисплея

#define SSD1322_CS       2       // CS (Chip Select)   — PA2
#define SSD1322_RST      3       // RESET              — PA3
#define SSD1322_DC       4       // DC (Data/Command)  — PA4
#define SSD1322_SCLK     5       // SCK (Clock)        — PA5
#define SSD1322_SDIO     7       // MOSI (Data)        — PA7

/* ============================================================
   2.  НИЗКОУРОВНЕВЫЕ ФУНКЦИИ SPI
   ============================================================ */

/**
  * @brief  Инициализация SPI1 (4-проводной режим)
  * @param  None
  * @retval None
  */
void SPI4W_Init(void) {
    // Включаем тактирование GPIOA и AFIO
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN | RCC_APB2ENR_AFIOEN;

    // Настройка пинов: SCK, MOSI — альтернативная функция, CS, DC, RST — выходы
    GPIO_INIT_PIN( SSD1322_PORT, SSD1322_SCLK,    GPIO_MODE_OUTPUT50_ALT_PUSH_PULL);
    GPIO_INIT_PIN( SSD1322_PORT, SSD1322_SDIO,    GPIO_MODE_OUTPUT50_ALT_PUSH_PULL);
    GPIO_INIT_PIN( SSD1322_PORT, SSD1322_CS,      GPIO_MODE_OUTPUT50_PUSH_PULL_UP);
    GPIO_INIT_PIN( SSD1322_PORT, SSD1322_DC,      GPIO_MODE_OUTPUT50_PUSH_PULL_UP);
    GPIO_INIT_PIN( SSD1322_PORT, SSD1322_RST,     GPIO_MODE_OUTPUT50_PUSH_PULL_UP);

    // Включаем тактирование SPI1
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    // CS по умолчанию HIGH
    SSD1322_PORT->BSRR = 1 << (SSD1322_CS);

    // Настройка SPI1: Master, CPOL=1, CPHA=1, частота Fpclk/4, MSB first
    SPI1->CR1 |= SPI_CR1_BR_1;
    SPI1->CR1 |= SPI_CR1_CPOL;
    SPI1->CR1 |= SPI_CR1_CPHA;
    SPI1->CR1 &= ~SPI_CR1_DFF;
    SPI1->CR1 &= ~SPI_CR1_LSBFIRST;
    SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;
    SPI1->CR1 |= SPI_CR1_MSTR;
    SPI1->CR1 |= SPI_CR1_SPE;
}

/**
  * @brief  Отправка байта по SPI с управлением CS
  * @param  data: передаваемый байт
  * @retval None
  */
void Spi_Write_Data(uint16_t data) {
    SSD1322_PORT->BSRR = 1 << (SSD1322_CS + 16);  // CS LOW
    SPI1->DR = data;
    delay_us(1);
    SSD1322_PORT->BSRR = 1 << (SSD1322_CS);       // CS HIGH
    while (!(SPI1->SR & SPI_SR_TXE));             // ждём окончания передачи
}

/**
  * @brief  Запись данных (DC = HIGH)
  */
void SSD1322_DataWrite(uint8_t data) {
    Spi_Write_Data(data);
}

/**
  * @brief  Запись команды (DC = LOW)
  */
void SSD1322_CommandWrite(uint8_t command) {
    SSD1322_PORT->BSRR = 1 << (SSD1322_DC + 16);  // DC LOW
    Spi_Write_Data(command);
    SSD1322_PORT->BSRR = 1 << (SSD1322_DC);       // DC HIGH
}

/**
  * @brief  Запись команды + данных
  */
void SSD1322_CmdDataWrite(uint8_t command, uint8_t data) {
    SSD1322_CommandWrite(command);
    SSD1322_DataWrite(data);
}

/**
  * @brief  Аппаратный сброс
  */
void SSD1322_Reset(void) {
    SSD1322_PORT->BSRR = 1 << (SSD1322_RST + 16); // RESET LOW
    delay_us(150);
    SSD1322_PORT->BSRR = 1 << (SSD1322_RST);      // RESET HIGH
}

/* ============================================================
   3.  ИНИЦИАЛИЗАЦИЯ И ОЧИСТКА
   ============================================================ */

/**
  * @brief  Полная инициализация SSD1322
  */
void SSD1322_Init(void) {
    SPI4W_Init();
    SSD1322_Reset();

    // Последовательность команд из даташита
    SSD1322_CmdDataWrite(0xfd, 0x12);       // unlock
    SSD1322_CommandWrite(0xae);             // display off
    SSD1322_CmdDataWrite(0xb3, 0x91);       // clock divide
    SSD1322_CmdDataWrite(0xca, 0x3f);       // multiplex
    SSD1322_CmdDataWrite(0xa2, 0x00);       // display offset
    SSD1322_CmdDataWrite(0xa1, 0x00);       // display start line
    SSD1322_CmdDataWrite(0xa0, 0x14);       // remap
    SSD1322_DataWrite(0x11);
    SSD1322_CmdDataWrite(0xab, 0x01);       // VDD regulator
    SSD1322_CmdDataWrite(0xb4, 0xa0);       // enhancement A
    SSD1322_DataWrite(0x05 | 0xfd);
    SSD1322_CmdDataWrite(0xc1, preset.contrast); // contrast из структуры   //SSD1322_CmdDataWrite(0xc1, 0x8f);       // contrast
		
    SSD1322_CmdDataWrite(0xc7, 0x0f);       // scale factor
    SSD1322_CommandWrite(0xb9);             // linear gray
    SSD1322_CmdDataWrite(0xb1, 0xe2);       // phase
    SSD1322_CmdDataWrite(0xd1, 0x082 | 0x020); // enhancement B
    SSD1322_DataWrite(0x20);
    SSD1322_CmdDataWrite(0xbb, 0x0f);       // precharge voltage
    SSD1322_CmdDataWrite(0xb6, 0x08);       // precharge period
    SSD1322_CmdDataWrite(0xbe, 0x07);       // vcomh
    SSD1322_CommandWrite(0xa6);             // normal display
    SSD1322_CommandWrite(0xa9);             // exit partial

    SSD1322_ClearRAM();                     // очистка экрана
    SSD1322_CommandWrite(0xaf);             // display on
}

/**
  * @brief  Очистка экрана (заливка чёрным)
  */
void SSD1322_ClearRAM(void) {
    SSD1322_CommandWrite(0x15);
    SSD1322_DataWrite(0x00);
    SSD1322_DataWrite(0x77);
    SSD1322_CommandWrite(0x75);
    SSD1322_DataWrite(0x00);
    SSD1322_DataWrite(0x7f);
    SSD1322_CommandWrite(0x5C);

    for (int y = 0; y < 128; y++) {
        for (int x = 0; x < 120; x++) {
            SSD1322_DataWrite(0x00);
        }
    }
}

/* ============================================================
   4.  УПРАВЛЕНИЕ ОБЛАСТЯМИ
   ============================================================ */

/**
  * @brief  Установка диапазона строк
  */
void SSD1322_SetRowAddress(uint8_t add) {
    add &= 0x3f;
    SSD1322_CommandWrite(0x75);
    SSD1322_DataWrite(add);
    SSD1322_DataWrite(0x3f);
}

/**
  * @brief  Установка диапазона столбцов
  */
void SSD1322_SetColumnAddress(uint8_t add) {
    add &= 0x3f;
    SSD1322_CommandWrite(0x15);
    SSD1322_DataWrite(0x1c + add);
    SSD1322_DataWrite(0x5b);
}

/* ============================================================
   5.  ПРЕОБРАЗОВАНИЕ ДАННЫХ (4-битные градации)
   ============================================================ */

/**
  * @brief  Преобразование 1-битных данных в 4-битные
  * @param  temp: байт с 8 битами (0/1)
  * @retval None
  */
void SSD1322_Data_processing(uint8_t temp) {
    uint8_t t1 = temp & 0x80;
    uint8_t t2 = (temp & 0x40) >> 3;
    uint8_t t3 = (temp & 0x20) << 2;
    uint8_t t4 = (temp & 0x10) >> 1;
    uint8_t t5 = (temp & 0x08) << 4;
    uint8_t t6 = (temp & 0x04) << 1;
    uint8_t t7 = (temp & 0x02) << 6;
    uint8_t t8 = (temp & 0x01) << 3;

    uint8_t h1 = t1 | (t1 >> 1) | (t1 >> 2) | (t1 >> 3);
    uint8_t h2 = t2 | (t2 >> 1) | (t2 >> 2) | (t2 >> 3);
    uint8_t h3 = t3 | (t3 >> 1) | (t3 >> 2) | (t3 >> 3);
    uint8_t h4 = t4 | (t4 >> 1) | (t4 >> 2) | (t4 >> 3);
    uint8_t h5 = t5 | (t5 >> 1) | (t5 >> 2) | (t5 >> 3);
    uint8_t h6 = t6 | (t6 >> 1) | (t6 >> 2) | (t6 >> 3);
    uint8_t h7 = t7 | (t7 >> 1) | (t7 >> 2) | (t7 >> 3);
    uint8_t h8 = t8 | (t8 >> 1) | (t8 >> 2) | (t8 >> 3);

    uint8_t d1 = h1 | h2;
    uint8_t d2 = h3 | h4;
    uint8_t d3 = h5 | h6;
    uint8_t d4 = h7 | h8;

    SSD1322_DataWrite(d1);
    SSD1322_DataWrite(d2);
    SSD1322_DataWrite(d3);
    SSD1322_DataWrite(d4);
}

/* ============================================================
   6.  ВЫВОД БОЛЬШОГО ШРИФТА (26 байт на символ)
   ============================================================ */

/**
  * @brief  Вывод одного ASCII-символа (большой шрифт)
  */

void SSD1322_DrawSingleAscii(uint16_t x, uint16_t y, uint8_t font_type, uint8_t *pAscii) {
    uint16_t offset;
    int max_rows;

    if (font_type == 1) {
        // --- НАШ НАСТРОЕННЫЙ МЕЛКИЙ ШРИФТ 16х11 (шаг 16 элементов) ---
        offset = (*pAscii - 32) * 16;
        max_rows = 16;
    } else {
        // --- ТВОЙ СТАНДАРТНЫЙ БОЛЬШОЙ ШРИФТ 26x16 (шаг 26 элементов) ---
        offset = (*pAscii - 32) * 26;
        max_rows = 26;
    }

    for (int i = 0; i < max_rows; i++) {
        uint8_t str_l, str_p;

        if (font_type == 1) {
            // Читаем из нового мелкого массива
            str_l = (AsciiLib_Small[offset + i] >> 8) & 0xFF;
            str_p = AsciiLib_Small[offset + i] & 0xFF;
        } else {
            // Читаем из твоего старого большого массива (укажи его точное имя)
            str_l = (AsciiLib_Big[offset + i] >> 8) & 0xFF;
            str_p = AsciiLib_Big[offset + i] & 0xFF;
        }

        SSD1322_SetRowAddress(y + i);
        SSD1322_SetColumnAddress(x);
        SSD1322_CommandWrite(0x5c);
        SSD1322_Data_processing(str_l);
        SSD1322_Data_processing(str_p);
    }
}



/*void SSD1322_DrawSingleAscii(uint16_t x, uint16_t y, uint8_t *pAscii) {
    uint16_t offset = (*pAscii - 32) * 26;

    for (int i = 0; i < 26; i++) {
        uint8_t str_l = (AsciiLib[offset + i] >> 8) & 0xFF;
        uint8_t str_p = AsciiLib[offset + i] & 0xFF;

        SSD1322_SetRowAddress(y + i);
        SSD1322_SetColumnAddress(x);
        SSD1322_CommandWrite(0x5c);
        SSD1322_Data_processing(str_l);
        SSD1322_Data_processing(str_p);
    }
}


  * @brief  Вывод строки (большой шрифт)
  */
void SSD1322_DrawString(uint16_t x, uint16_t y, uint8_t font_type, uint8_t *pStr) {              //void SSD1322_DrawString(uint16_t x, uint16_t y, uint8_t *pStr)
    while (*pStr) {
        if (*pStr > 0x80) {   // кириллица
            SSD1322_DrawSingleAscii(x, y, font_type, pStr); //SSD1322_DrawSingleAscii(x, y, pStr);
            x += 4;
            pStr += 2;
        } else {              // латиница
            SSD1322_DrawSingleAscii(x, y, font_type, pStr); //SSD1322_DrawSingleAscii(x, y, pStr);
            x += 5;
            pStr += 1;
        }
    }
}

/* ============================================================
   7.  ВЫВОД МАЛЕНЬКОГО ШРИФТА (6x8)
   ============================================================ */

/*
  * @brief  Вывод одного маленького символа
  */
void SSD1322_DrawSmallChar(uint8_t x, uint8_t y, uint8_t c) {
    if (c < 0x20 || c > 0x7E) c = 0x20;

    SSD1322_CommandWrite(0x15);
    SSD1322_DataWrite(x);
    SSD1322_DataWrite(x + 5);

    SSD1322_CommandWrite(0x75);
    SSD1322_DataWrite(y);
    SSD1322_DataWrite(y + 7);

    SSD1322_CommandWrite(0x5C);

    for (int row = 0; row < 8; row++) {
        SSD1322_DataWrite(SmallFont[c - 0x20][row]);
    }
}

/**
  * @brief  Вывод строки маленьким шрифтом
  */
void SSD1322_DrawSmallString(uint8_t x, uint8_t y, const char *str) {
    while (*str) {
        SSD1322_DrawSmallChar(x, y, *str);
        x += 6;
        str++;
    }
}

	/**
  * @brief  Рисует символ в буфере (6x8) из SmallFont с инверсией битов
  * @param  x: координата X (0..255)
  * @param  y: координата Y (0..31) — только верхняя половина!
  * @param  c: ASCII-код символа
  */


void SSD1322_DrawCharBuffer(uint16_t x, uint16_t y, uint8_t c) {
    if (c < 0x20 || c > 0x7E) c = 0x20;

    // Проходим по битам (строкам) — 0..7
    for (int row = 0; row < 8; row++) {
        // Проходим по байтам (столбцам) — 0..5
        for (int col = 0; col < 6; col++) {
            // Берём байт для данного столбца
            uint8_t byte = SmallFont[c - 0x20][col];	//[c - 0x20-7][col];
            // Проверяем бит на позиции row (младший бит — это row = 0)
            if (byte & (1 << row)) {
                SSD1322_DrawPixel(x + col, y + row , 0x0F);
            }
        }
    }
}
		/***********************************************************/
// ============================================================
//  ВЫВОД СТРОКИ (НОВАЯ ФУНКЦИЯ)
// ============================================================

/**
  * @brief  Вывод строки символов в буфер
  * @param  x: координата X (0..255)
  * @param  y: координата Y (0..31) — только верхняя половина
  * @param  str: указатель на строку
  * @retval None
  */
void SSD1322_DrawStringBuffer(uint16_t x, uint16_t y, const char *str) {
    while (*str) {
        SSD1322_DrawCharBuffer(x, y, *str);
        x += 6;  // ширина символа (6 пикселей)
        str++;
    }
}
/* ============================================================
   8.  БУФЕР (Framebuffer)
   ============================================================ */

#define SCREEN_WIDTH  256
#define SCREEN_HEIGHT 64
#define BUFFER_SIZE   ((SCREEN_WIDTH * SCREEN_HEIGHT) / 4) // 4096 байт

uint8_t oled_buffer[4096];

/**
  * @brief  Очистка буфера
  */
void SSD1322_ClearBuffer(void) {
    for (int i = 0; i < BUFFER_SIZE; i++) {
        oled_buffer[i] = 0x00;
    }
}

/**
  * @brief  Отправка буфера на экран
  */
void SSD1322_Update(uint8_t start_row, uint8_t end_row) {
			SSD1322_CommandWrite(0x15);
			SSD1322_DataWrite(0x25);//(0x1C);
			SSD1322_DataWrite(0x5B);

			SSD1322_CommandWrite(0x75);
			SSD1322_DataWrite(start_row);   // теперь можно менять!
			SSD1322_DataWrite(end_row);
			SSD1322_CommandWrite(0x5C);
			for (int i = 0; i < BUFFER_SIZE; i++) {
					SSD1322_DataWrite(oled_buffer[i]);
			}
}

/**
  * @brief  Заполнение буфера шаблоном
  */
void SSD1322_FillPattern(uint8_t pattern) {
    for (int i = 0; i < BUFFER_SIZE; i++) {
        oled_buffer[i] = pattern;
    }
}
/**
  * @brief  Рисует точку в буфере (попиксельно)
  * @param  x: координата X (0..255)
  * @param  y: координата Y (0..63)
  * @param  color: 0x00 — чёрный, 0x0F — белый
  */
void SSD1322_DrawPixel(uint16_t x, uint16_t y, uint8_t color) {
    if (x >= 256 || y >= 64) return;  // защита от выхода за границы

    uint16_t index = (y * 128) + (x / 2);  // каждый байт = 2 пикселя

    if (x % 2 == 0) {
        // Старший полубайт (левый пиксель)
        oled_buffer[index] = (oled_buffer[index] & 0x0F) | (color << 4);
    } else {
        // Младший полубайт (правый пиксель)
        oled_buffer[index] = (oled_buffer[index] & 0xF0) | color;
    }
}
/* ============================================================
   9.  ОТЛАДОЧНЫЕ ФУНКЦИИ
   ============================================================ */

/**
  * @brief  Вывод картинки из массива
  */
void SSD1322_Display_Picture(uint8_t pic[]) {
    SSD1322_SetRowAddress(0);
    SSD1322_SetColumnAddress(0);
    SSD1322_CommandWrite(0x5c);

    for (int i = 0; i < 64; i++) {
        for (int j = 0; j < 32; j++) {
            SSD1322_Data_processing(pic[i * 32 + j]);
        }
    }
}

/**
  * @brief  Тест градаций серого
  */
void Gray_test(void) {
    SSD1322_SetRowAddress(0);
    SSD1322_SetColumnAddress(0);
    SSD1322_CommandWrite(0x5c);

    uint8_t j = 0;
    for (int m = 0; m < 32; m++) {
        for (int k = 0; k < 16; k++) {
            for (int i = 0; i < 8; i++) {
                SSD1322_DataWrite(j);
            }
            j += 0x11;
        }
        j = 0;
    }

    j = 255;
    for (int m = 0; m < 32; m++) {
        for (int k = 0; k < 16; k++) {
            for (int i = 0; i < 8; i++) {
                SSD1322_DataWrite(j);
            }
            j -= 0x11;
        }
        j = 255;
    }
}
//=============================================================
// Таблица символов для быстрого перевода в HEX
static const char hex_chars[] = "0123456789ABCDEF";

// Функция перевода байта в строку HEX (формат: "XX ")
static void byte_to_hex(uint8_t byte, char *buf) {
    buf[0] = hex_chars[(byte >> 4) & 0x0F]; // Старшая половина байта
    buf[1] = hex_chars[byte & 0x0F];        // Младшая половина байта
    buf[2] = ' ';                           // Добавляем пробел для разделения
}
//=============================================================
// Сообщаем компилятору, что эти переменные живут в файле UART_XMOS.c
extern volatile uint8_t rx_buffer[3]; 
extern volatile uint8_t buffer_index;

void DebugUartHex(void) {
    char buf[16]; // Буфер под итоговую строку (хватит на "UART: XX XX XX")
    
    // Пишем префикс
    buf[0] = 'U'; buf[1] = 'A'; buf[2] = 'R'; buf[3] = 'T'; buf[4] = ':'; buf[5] = ' ';
    
    // Переводим первый байт пакета
    byte_to_hex(rx_buffer[0], &buf[6]);
    
    // Переводим второй байт пакета (тот самый last_signal)
    byte_to_hex(rx_buffer[1], &buf[9]);
    
    // Переводим третий байт пакета
    byte_to_hex(rx_buffer[2], &buf[12]);
    
    // Закрываем строку нулем
    buf[15] = '\0';

    // Выводим на экран (например, в координаты x=0, y=0)
    // Внимание: ClearBuffer убираем, чтобы не стирать весь экран!
    SSD1322_DrawStringBuffer(0, 0, buf);
    SSD1322_Update(0x00, 0x1F); // Обновляем область экрана
}
//==================================================================
// Быстрый перевод числа в строку без использования тяжелой sprintf
//==================================================================

static void fast_itoa(uint32_t val, char *buf, const char *prefix) {
    char tmp[11];
    char *p = tmp;
    
    // Копируем префикс (например, "DBG " или "DUR ")
    while (*prefix) {
        *buf++ = *prefix++;
    }
    
    // Переводим число в обратном порядке
    if (val == 0) {
        *p++ = '0';
    } else {
        while (val > 0) {
            *p++ = (val % 10) + '0';
            val /= 10;
        }
    }
    
    // Разворачиваем строку в итоговый буфер
    while (p > tmp) {
        *buf++ = *--p;
    }
    *buf = '\0'; // Закрывающий ноль
}

/*
// ============================================================
//  ВИЗУАЛЬНЫЙ ОТЛАДЧИК НА ЭКРАНЕ (показывает число)
// ============================================================
void DebugNum(uint16_t value) {
    char buf[16];
    fast_itoa(value, buf, "DBG ");
    
    // Убираем SSD1322_ClearBuffer(); — чтобы не тереть остальной экран!
    // Если нужно стереть только старое значение, можно сначала вывести туда пробелы:
    // SSD1322_DrawStringBuffer(0, 0, "        "); 
    
    SSD1322_DrawStringBuffer(0, 0, buf);
    SSD1322_Update(0x00, 0x1F); // Обновляем только нужную область колонок
}

// ============================================================
//  ВИЗУАЛЬНЫЙ ОТЛАДЧИК НА ЭКРАНЕ (номер шага)
// ============================================================
void DebugShow(uint8_t step) {
    char buf[16];
    fast_itoa(step, buf, "DBG ");
    
    // Выводим по координатам (25, 0)
    SSD1322_DrawStringBuffer(25, 0, buf);
    SSD1322_Update(0x20, 0x3F); // Обновляем только эту зону экрана
}

// ============================================================
//  ФУНКЦИЯ ДЛЯ ВЫВОДА ДЛИТЕЛЬНОСТИ НА ЭКРАН (отладка)
// ============================================================
void DebugDuration(uint32_t duration) {
    char buf[16];
    fast_itoa(duration, buf, "DUR ");
    
    SSD1322_DrawStringBuffer(0, 0, buf);
    SSD1322_Update(0x00, 0x1F);
}*/