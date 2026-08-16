/*
 * ============================================================
 *  ЗАГОЛОВОЧНЫЙ ФАЙЛ ДЛЯ SSD1322 (256x64, SPI)
 *  Платформа: STM32F103C8T6 (Blue Pill)
 *  Содержит прототипы всех функций для работы с дисплеем
 * ============================================================
 */

#ifndef SPI_SSD1322_H
#define SPI_SSD1322_H

#include <stdint.h>
#include <string.h>

/* ============================================================
   1.  ИНИЦИАЛИЗАЦИЯ
   ============================================================ */

/**
  * @brief  Инициализация SPI для дисплея (4-проводной режим)
  * @param  None
  * @retval None
  */
void SPI4W_Init(void);

/**
  * @brief  Аппаратный сброс дисплея (RESET)
  * @param  None
  * @retval None
  */
void SSD1322_Reset(void);

/**
  * @brief  Полная инициализация SSD1322 (команды + включение)
  * @param  None
  * @retval None
  */
void SSD1322_Init(void);

/* ============================================================
   2.  НИЗКОУРОВНЕВАЯ РАБОТА С SPI
   ============================================================ */

/**
  * @brief  Отправка байта по SPI с управлением CS
  * @param  data: передаваемый байт
  * @retval None
  */
void Spi_Write_Data(uint16_t data);

/**
  * @brief  Запись данных в дисплей (DC = HIGH)
  * @param  data: байт данных
  * @retval None
  */
void SSD1322_DataWrite(uint8_t data);

/**
  * @brief  Запись команды в дисплей (DC = LOW)
  * @param  command: код команды
  * @retval None
  */
void SSD1322_CommandWrite(uint8_t command);

/**
  * @brief  Запись команды + одного байта данных
  * @param  command: код команды
  * @param  data: байт данных
  * @retval None
  */
void SSD1322_CmdDataWrite(uint8_t command, uint8_t data);

/* ============================================================
   3.  УПРАВЛЕНИЕ ОБЛАСТЯМИ ПАМЯТИ
   ============================================================ */

/**
  * @brief  Установка диапазона строк (вертикаль)
  * @param  add: начальная строка (0..63)
  * @retval None
  */
void SSD1322_SetRowAddress(uint8_t add);

/**
  * @brief  Установка диапазона столбцов (горизонталь)
  * @param  add: начальный столбец (0..63)
  * @retval None
  */
void SSD1322_SetColumnAddress(uint8_t add);

/**
  * @brief  Очистка экрана (заливка чёрным)
  * @param  None
  * @retval None
  */
void SSD1322_ClearRAM(void);

/* ============================================================
   4.  РИСОВАНИЕ ТЕКСТА (БОЛЬШОЙ ШРИФТ)
   ============================================================ */

/**
  * @brief  Вывод одного ASCII-символа (большой шрифт)
  * @param  x: координата X (0..255)
  * @param  y: координата Y (0..63)
  * @param  pAscii: указатель на символ
  * @retval None
  */
//void SSD1322_DrawSingleAscii(uint16_t x, uint16_t y, uint8_t *pAscii);

void SSD1322_DrawSingleAscii(uint16_t x, uint16_t y, uint8_t font_type, uint8_t *pAscii);
/**
  * @brief  Вывод строки (большой шрифт)
  * @param  x: координата X (0..255)
  * @param  y: координата Y (0..63)
  * @param  pStr: указатель на строку
  * @retval None
  */
	void SSD1322_DrawString(uint16_t x, uint16_t y, uint8_t font_type, uint8_t *pStr);

//void SSD1322_DrawString(uint16_t x, uint16_t y, uint8_t *pStr);

/* ============================================================
   5.  РИСОВАНИЕ ТЕКСТА (МАЛЕНЬКИЙ ШРИФТ 6x8)
   ============================================================ */

/**
  * @brief  Вывод одного маленького символа
  * @param  x: координата X (0..255)
  * @param  y: координата Y (0..63)
  * @param  c: ASCII-код символа
  * @retval None
  */
void SSD1322_DrawSmallChar(uint8_t x, uint8_t y, uint8_t c);

/**
  * @brief  Вывод строки маленьким шрифтом
  * @param  x: координата X (0..255)
  * @param  y: координата Y (0..63)
  * @param  str: указатель на строку
  * @retval None
  */
void SSD1322_DrawSmallString(uint8_t x, uint8_t y, const char *str);

/* ============================================================
   6.  РАБОТА С БУФЕРОМ (Framebuffer)
   ============================================================ */

/**
  * @brief  Очистка буфера (заливка чёрным)
  * @param  None
  * @retval None
  */
void SSD1322_ClearBuffer(void);

/**
  * @brief  Отправка буфера на экран (команды + данные)
  * @param  None
  * @retval None
  */
void SSD1322_Update(uint8_t start_row, uint8_t end_row);
/**
  * @brief  Отправка буфера на экран (команды L/R+ данные)
  * @param  None
  * @retval None
  */
void SSD1322_UpdateV(uint8_t start_row, uint8_t end_row,uint8_t coloumn);
/**
  * @brief  Заполнение буфера заданным шаблоном (для теста)
  * @param  pattern: байт для заливки
  * @retval None
  */
void SSD1322_FillPattern(uint8_t pattern);

/**
  * @brief  Внешний буфер (4096 байт — половина экрана)
  */
extern uint8_t oled_buffer[4096];
/**
*
*
*
*
*/
void SSD1322_DrawCharBuffer(uint16_t x, uint16_t y, uint8_t c);
/**
  * @brief  Рисует точку в буфере (попиксельно)
  * @param  x: координата X (0..255)
  * @param  y: координата Y (0..63)
  * @param  color: 0x00 — чёрный, 0x0F — белый
  */
void SSD1322_DrawPixel(uint16_t x, uint16_t y, uint8_t color);

/* ============================================================
   7.  ОТЛАДОЧНЫЕ И ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ
   ============================================================ */

/**
  * @brief  Преобразование 1-битных данных в 4-битные (градации серого)
  * @param  temp: исходный байт
  * @retval None
  */
void SSD1322_Data_processing(uint8_t temp);

/**
  * @brief  Вывод картинки из массива
  * @param  pic: массив данных
  * @retval None
  */
void SSD1322_Display_Picture(uint8_t pic[]);

/**
  * @brief  Тест градаций серого
  * @param  None
  * @retval None
  */
void Gray_test(void);

// ============================================================
//  ОБЪЯВЛЕНИЯ ФУНКЦИЙ (ДОБАВИТЬ В КОНЕЦ)
// ============================================================

void SSD1322_DrawStringBuffer(uint16_t x, uint16_t y, const char *str);

#endif /* SPI_SSD1322_H */