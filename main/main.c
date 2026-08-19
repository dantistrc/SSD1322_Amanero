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
void Set_Volume_And_Balance(uint8_t volume_val, uint8_t balance_val);
void Process_XMOS_Signal(void);
static uint8_t last_mute = 255; // Черновик: помнит состояние Mute в прошлом круге
uint8_t signal =0;

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
uint16_t menu_counter = 0;
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
#define FLASH_PRESET_ADDR   0x0800FC00
#define PRESET_WORD_CNT  2
#define ESS9028_I2C_ADDR   0x48 // Адрес ЦАПа на шине I2C
/* ============================================================
		JSA
   ============================================================ */
uint8_t menu_mode_val = 0; // Стартуем в режиме громкости
uint8_t volume_val = 40;  // Стартовая громкость
uint8_t input_val = 0;   // Стартовый вход (0-USB, 1-COA, 2-OPT)
uint8_t filter_val = 0;  // Стартовый фильтр ЦАПа
uint8_t  balance_val = 0; // Стартовый баланс (ноль — центр)
uint8_t  contrast_val = 100; //яркость
static uint8_t mute_flag = 0; // Помнит состояние кнопки Муте
uint32_t screen_return_timer = 0;

/* ============================================================
   3.  ВСПОМОГАТЕЛЬНЫЕ БУФЕРЫ
   ============================================================ */

char buffer[16] = {'\0'};   // для форматирования строк (sprintf)

/* ============================================================
   4.  ОБРАБОТКА КНОПКИ (с антидребезгом через TIM4)
   ============================================================ */

volatile uint16_t button_tick = 0;     // время нажатия (в тиках TIM4)
volatile uint8_t button_pressed = 0;   // флаг нажатия
#define DEBOUNCE_TICKS 10               // 1 задержка в 10 мс (при частоте TIM4 = 1 кГц)

/* ============================================================
   5.  РАБОТА С FLASH-ПАМЯТЬЮ (сохранение настроек)
   ============================================================ */
/*
 *  Структура для хранения настроек в Flash.
 *  Адрес:  #define FLASH_PRESET_ADDR   0x0800FC00.
 */


    typedef struct {
    uint8_t input_select;   			// Выбор входа (1 байт)
    uint8_t digital_filter; 			// Цифровой фильтр (1 байт)
    uint8_t contrast;       			// Яркость OLED (1 байт)
    uint8_t volume_int;     			// Громкость в шагах 0..255 (1 байт)
    uint8_t balance_int;    			// Баланс в шагах 0..255 (1 байт)
    uint8_t reserve2;       			// Пустышка для ровного счёта
    uint8_t reserve3;       			// Пустышка для ровного счёта
} presets_def;
	presets_def preset;





/**
  * @brief  Чтение настроек из Flash
  * @param  None
  * @retval None
  */
void FLASH_ReadSettings(void) {																								//JSA=PASSED
    uint16_t p;
    uint32_t *source_adr = (uint32_t *)(FLASH_PRESET_ADDR);
    uint32_t *dest_adr = (void *)&preset;

    for (p = 0; p < PRESET_WORD_CNT; ++p) {
        *(dest_adr + p) = *(__IO uint32_t*)(source_adr + p);
    }
}


  //=========================================================================================  Запись настроек в Flash (стирает страницу и перезаписывает)

void FLASH_WriteSettings(void) {																														//JSA=PASSED
    uint8_t i;
		uint32_t pageAdr = FLASH_PRESET_ADDR;
    uint32_t *source_adr = (void *)&preset;

    FLASH_Unlock();
    FLASH_ErasePage(pageAdr);
    for (i = 0; i < PRESET_WORD_CNT; ++i) {
        FLASH_ProgramWord((uint32_t)(pageAdr + i * 4), *(source_adr + i));
    }
    FLASH_Lock();
}
void Save_Active_Menu_Setting(void) {																												//JSA=TESTING    
    preset.input_select   = input_val;																											// Шаг 1. Сначала переносим ВСЕ текущие крутилки с экрана в боевой preset
    preset.digital_filter = filter_val;
    preset.volume_int     = volume_val;
    preset.balance_int    = balance_val;
		preset.contrast    = contrast_val;    																							// (Контраст и остальное тоже можно приписать сюда, если они есть на экране)    
    presets_def backup_preset;																															// Шаг 2. Создаем временный черновик-буфер в ОЗУ для старой памяти    
    uint32_t *source_adr = (uint32_t *)(FLASH_PRESET_ADDR);																	// Выкачиваем старые настройки из Flash строго в черновик backup_preset
    uint32_t *dest_adr = (void *)&backup_preset;
    uint16_t p;
    for (p = 0; p < PRESET_WORD_CNT; ++p) {
        *(dest_adr + p) = *(__IO uint32_t*)(source_adr + p);
    }    
    switch (menu_mode_val) {																																// Шаг 3. ТВОЙ АЛГОРИТМ: склеиваем данные в черновике, чтобы НЕ ПЕРЕТЕРЕТЬ лишнее!
        case 0: 																																						// МЫ В ГЛАВНОМ МЕНЮ (Громкость по умолчанию)
            backup_preset.volume_int = preset.volume_int; 																	// Заменяем во Flash ТОЛЬКО громкость!
            break;            
        case 1: 																																						// МЫ В МЕНЮ ВХОДОВ
            backup_preset.input_select = preset.input_select; 															// Заменяем во Flash ТОЛЬКО вход
            break;            
        case 2: 																																						// МЫ В МЕНЮ ФИЛЬТРОВ
            backup_preset.digital_filter = preset.digital_filter; 													// Заменяем во Flash ТОЛЬКО фильтр
            break;            
        case 3: 																																						// МЫ В МЕНЮ БАЛАНСА
            backup_preset.balance_int = preset.balance_int; 																// Заменяем во Flash ТОЛЬКО баланс
            break; 
				case 4: 																																						// МЫ В МЕНЮ БАЛАНСА
            backup_preset.contrast = preset.contrast; 																// Заменяем во Flash ТОЛЬКО баланс
            break;  				
        default:
																																														// =================== Сюда Дописать яркость
            return; 
    }    
    preset = backup_preset;																																	// Шаг 4. Возвращаем склеенный пирог обратно в боевой preset    
    FLASH_WriteSettings();																																	// Шаг 5. Намертво прошиваем этот склеенный пирог во Flash!
}

/* ============================================================
   6.  I2C (для управления ES9039Q2M через I2C)
   ============================================================ */


	


void I2C2_Init(void) {																																			//  Инициализация I2C2 (PB10 — SCL, PB11 — SDA)
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


  //   Отправка одного байта по I2C
  //  addr: 7-битный адрес устройства
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
   MENU  ЛОГИКА ОБРАБОТКИ НАЖАТИЯ КНОПКИ
   ============================================================ */
void Update_Bottom_Line(void) {
						char buf[16];						    
						static uint8_t last_mode = 255; 																// Локальный черновик: помнит, какой экран мы рисовали прошлым
				if (menu_mode_val != last_mode || mute_flag != last_mute) {					// Если изменился режим ИЛИ изменилось состояние Mute — только тогда чистим экран один раз!
        SSD1322_ClearRAM();																									// Очистка дисплея
        last_mode = menu_mode_val;   																				// Запоминаем новый режим
        last_mute = mute_flag;       																				// Запоминаем новое состояние Mute
    }
        switch (menu_mode_val) {
				case 0: 																																													// MODE_VOLUME
						if (mute_flag == 1) {
								
								SSD1322_DrawString(0, 20, 0, (unsigned char*)"  MUTE"); 		// Если звук выключен — пишем MUTE в те же координаты, перекрывая громкость!
						} else {							
								SSD1322_DrawString(0, 20, 0, (unsigned char*)"VOLUME");			// Если всё штатно — пишем стандартный VOLUME
						}            
            unsigned char vol_str[4];                                     	// 2. Быстро разбиваем байт громкости на три символа-цифры
						if (volume_val / 100 == 0) {																		// Сотни: если сотен нет, пишем пробел, иначе — цифру
								vol_str[0] = ' ';
						} else {
								vol_str[0] = (volume_val / 100) + '0';
						}
						if ((volume_val / 100 == 0) && (((volume_val % 100) / 10) == 0)) {    // Десятки: если сотен нет И десятков нет — пишем пробел, иначе — цифру
								vol_str[1] = ' ';
						} else {
								vol_str[1] = ((volume_val % 100) / 10) + '0';
						}						
						vol_str[2] = (volume_val % 10) + '0';														// Единицы выводим всегда, даже если это чистый ноль
						vol_str[3] = '\0'; 																							// Конец строки                        
            SSD1322_DrawString(30, 20, 0, vol_str);  												// 3. Выводим получившиеся цифры (сдвигаемся по X на 40 пикселей вправо)                      
            SSD1322_DrawString(45, 20, 0, (unsigned char*)"dB");						// 4. Дописываем единицы измерения (сдвигаемся по X на 64 пикселя вправо)																					//            
            if (input_val == 0)      SSD1322_DrawSmallString(1, 56,"INPUT USB     ");// mini Выводим реальный вход в зависимости от input_val
						else if (input_val == 1) SSD1322_DrawSmallString(1, 56,"INPUT S/PDIF  ");
						else                     SSD1322_DrawSmallString(1, 56,"INPUT TOSLINK ");
																																						//  mini Выводим красивое название фильтра ESS9039 // SSD1322_DrawSmallString(37, 0,
								 if (filter_val == 0) SSD1322_DrawSmallString(1, 0,"FIR1 MinPhase"); // Minimum phase (default)
            else if (filter_val == 1) SSD1322_DrawSmallString(1, 0,"FIR2 LinApod"); // Linear phase apodizing fast roll-off
            else if (filter_val == 2) SSD1322_DrawSmallString(1, 0,"FIR3 LinFast"); // Linear phase fast roll-off
            else if (filter_val == 3) SSD1322_DrawSmallString(1, 0,"FIR4 LinLowR"); // Linear phase fast roll-off low ripple
            else if (filter_val == 4) SSD1322_DrawSmallString(1, 0,"FIR5 LinSlow"); // Linear phase slow roll-off
            else if (filter_val == 5) SSD1322_DrawSmallString(1, 0,"FIR6 MinFast"); // Minimum phase fast roll-off
            else if (filter_val == 6) SSD1322_DrawSmallString(1, 0,"FIR7 MinSlow"); // Minimum phase slow roll-off
            else                      SSD1322_DrawSmallString(1, 0,"FIR8 MinLowD"); // Minimum phase slow roll-off low dispersion
						Process_XMOS_Signal();
						break;
        case 1: 																																																							// MODE_INPUT
																			SSD1322_DrawString(19, 0, 0, (unsigned char*)"INPUT");							// "NAME"
            if (input_val == 0)      SSD1322_DrawString(80, 32, 0, (unsigned char*)"  USB  ");            // Выводим реальный вход в зависимости от input_val
						else if (input_val == 1) SSD1322_DrawString(80, 33, 0, (unsigned char*)"S/PDIF");
						else                     SSD1322_DrawString(80, 32, 0, (unsigned char*)"TOSLINK");
            break;            
        case 2: 																																		// MODE_FILTER Выводим красивое название фильтра ESS9039 "БОЛЬШОЕ МЕНЮ"
																			SSD1322_DrawString(25, 0, 0, (unsigned char*)"FIR");
            if (filter_val == 0)      SSD1322_DrawString(5, 35, 0, (unsigned char*)"1 MIN PHASE"); // Minimum phase (default)
            else if (filter_val == 1) SSD1322_DrawString(5, 35, 0, (unsigned char*)"2 LIN APOD "); // Linear phase apodizing fast roll-off
            else if (filter_val == 2) SSD1322_DrawString(5, 35, 0, (unsigned char*)"3 LIN FAST "); // Linear phase fast roll-off
            else if (filter_val == 3) SSD1322_DrawString(5, 35, 0, (unsigned char*)"4 LIN LOWR "); // Linear phase fast roll-off low ripple
            else if (filter_val == 4) SSD1322_DrawString(5, 35, 0, (unsigned char*)"5 LIN SLOW "); // Linear phase slow roll-off
            else if (filter_val == 5) SSD1322_DrawString(5, 35, 0, (unsigned char*)"6 MIN FAST "); // Minimum phase fast roll-off
            else if (filter_val == 6) SSD1322_DrawString(5, 35, 0, (unsigned char*)"7 MIN SLOW "); // Minimum phase slow roll-off
            else                      SSD1322_DrawString(5, 35, 0, (unsigned char*)"8 MIN LOWD "); // Minimum phase slow roll-off low dispersion
            break;            
        case 3: 																																																						// MODE_BALANCE
            SSD1322_DrawString(15, 0, 0, (unsigned char*)"BALANCE");            													// Выводим текст "BALANCE" "БОЛЬШОЕ МЕНЮ"
            unsigned char bal_str[4];																								// Быстро бьем байт на сотни, десятки и единицы
                    
						if (balance_val / 100 == 0) {																						// Сотни: если сотен нет, пишем пробел, иначе — цифру
								bal_str[0] = ' ';
						} else {
								bal_str[0] = (balance_val / 100) + '0';
						}
						if ((balance_val / 100 == 0) && (((balance_val % 100) / 10) == 0)) {		// Десятки: если сотен нет И десятков нет — пишем пробел, иначе — цифру
								bal_str[1] = ' ';
						} else {
								bal_str[1] = ((balance_val % 100) / 10) + '0';
						}						
						bal_str[2] = (balance_val % 10) + '0';																	// Единицы выводим всегда, даже если это чистый ноль
						bal_str[3] = '\0'; 																											// Конец строки            
            SSD1322_DrawString(20, 35, 0, bal_str);																	// Печатаем получившиеся три цифры сразу за текстом (сдвиг по X на 40 пикселей)
						SSD1322_DrawString(35, 35, 0, (unsigned char*)"dB");
            break;
				case 4: 																																																						// MODE_BALANCE
            SSD1322_DrawString(12, 0, 0, (unsigned char*)"BRIGHTNESS");            													// Выводим текст "BALANCE" "БОЛЬШОЕ МЕНЮ"
            unsigned char bra_str[4];																								// Быстро бьем байт на сотни, десятки и единицы
                    
						if (contrast_val / 100 == 0) {																						// Сотни: если сотен нет, пишем пробел, иначе — цифру
								bra_str[0] = ' ';
						} else {
								bra_str[0] = (contrast_val / 100) + '0';
						}
						if ((contrast_val / 100 == 0) && (((contrast_val % 100) / 10) == 0)) {		// Десятки: если сотен нет И десятков нет — пишем пробел, иначе — цифру
								bal_str[1] = ' ';
						} else {
								bra_str[1] = ((contrast_val % 100) / 10) + '0';
						}						
						bra_str[2] = (contrast_val % 10) + '0';																	// Единицы выводим всегда, даже если это чистый ноль
						bra_str[3] = '\0'; 																											// Конец строки            
            SSD1322_DrawString(20, 35, 0, bra_str);																	// Печатаем получившиеся три цифры сразу за текстом (сдвиг по X на 40 пикселей)
						//SSD1322_DrawString(35, 35, 0, (unsigned char*)"dB");
            break;		
    }
}

//==========================UART-DISPLAY=========================================================================
void Process_XMOS_Signal(void) {												// ----- ОБРАБОТКА СИГНАЛА ОТ XMOS-XU316 pin12 -----

            switch (last_signal) {

							  case 0x01: SSD1322_DrawSmallString(37, 0, "PCM 44.1kHz  "); break;
                case 0x02: SSD1322_DrawSmallString(37, 0, "PCM 48kHz    "); break;
                case 0x03: SSD1322_DrawSmallString(37, 0, "PCM 88.2kHz  "); break;
                case 0x04: SSD1322_DrawSmallString(37, 0, "PCM 96kHz    "); break;
                case 0x05: SSD1322_DrawSmallString(37, 0, "PCM 176.4kHz "); break;
                case 0x06: SSD1322_DrawSmallString(37, 0, "PCM 192kHz   "); break;
                case 0x07: SSD1322_DrawSmallString(37, 0, "PCM 352.8kHz "); break;
                case 0x08: SSD1322_DrawSmallString(37, 0, "PCM 384kHz   "); break;
                case 0x09: SSD1322_DrawSmallString(37, 0, "PCM 705.6kHz "); break;
                case 0x0A: SSD1322_DrawSmallString(37, 0, "PCM 768kHz   "); break;
                case 0x0B: SSD1322_DrawSmallString(37, 0, "PCM 1411.2kHz"); break;
                case 0x0C: SSD1322_DrawSmallString(37, 0, "PCM 1536kHz  "); break;
                case 0x19: SSD1322_DrawSmallString(37, 0, "DSD64 2.822  "); break;
                case 0x1A: SSD1322_DrawSmallString(37, 0, "DSD128 5.644 "); break;
                case 0x1B: SSD1322_DrawSmallString(37, 0, "DSD256 11.289"); break;
                case 0x1C: SSD1322_DrawSmallString(37, 0, "DSD512 22.579"); break;
                case 0x1D: SSD1322_DrawSmallString(37, 0, "DSD1024 45.15"); break;
            }
        }


//===================================== J S A PRESS BUTTON (ENC) =================================================================
void ProcessButtonPress(void)
{
    menu_counter = 0;																								// Обнуляем твой системный счетчик в самом начале удержания кнопки   
    while ((GPIOA->IDR & (1 << 6)) == 0) 														// Ждем, пока кнопка нажата (строка 603 с твоего скриншота)
    {
																																		// Пока мы держим кнопку, прерывание таймера висит,         
        if (TIM4->SR & TIM_SR_UIF) 																	// но мы можем принудительно пнуть инкремент счетчика прямо тут!
        {
            TIM4->SR &= ~TIM_SR_UIF;
            menu_counter++;
					 if (menu_counter == 2000) 																// Если держали 2 секунды и больше — сохраняем настройки
						{
						SSD1322_ClearRAM();																									// Очистка дисплея	
						SSD1322_DrawString(13, 0, 0, (unsigned char*)"SAVE SET");
						FLASH_ReadSettings(); 																	//После того как считаем нужно подменить соответствующий параметр и вызвать запись, что уже приготовлено ниже	
						Save_Active_Menu_Setting();           									// FLASH_WriteSettings Уже внутри есть!!!	
						menu_mode_val =0;																				//Выходим в главное меню
						delay_ms(1000);																					// 1sec
						SSD1322_ClearRAM();																			// Очистка дисплея		
						menu_need_update = 1;	
						}
				}
    }
																																		// Кнопку отпустили!    
    if (menu_counter >= 2000)																				// Если удержали 2 секунды и больше — выходим, короткий клик пропускается
    {
        return; 
    }
    menu_level = 1; 																								// Сюда попадем, только если отпустили кнопку РАНЬШЕ 2 секунд (короткий клик)
    menu_mode_val++;
    if (menu_mode_val > 4) menu_mode_val = 0;					//    !!!!  5/1 ?????
    menu_need_update = 1;
}


//=============================================M U T E======================================================
void Set_DAC_Mute(uint8_t state)
{
    if (state == 1) 
    {
        // Включаем глобальный Soft Mute: пишем 1 в бит 0 Регистра 7
        // Звук плавно и красиво затихнет в ноль без щелчков в колонках
     //   ESS9028_WriteReg(ESS9028_I2C_ADDR, 7, 0x01); 
    }
    else 
    {
        // Выключаем Soft Mute: возвращаем регистр 7 в исходный ноль
        // Звук плавно вернётся на ту громкость, которая сейчас выставлена
    //    ESS9028_WriteReg(ESS9028_I2C_ADDR, 7, 0x00); 
    }
}

//==================================================== E N D = J S A ============================================================


/* ==============================================================================================================================
																											8.  ПРЕРЫВАНИЯ
   ============================================================================================================================== */

																																			

  
void EXTI9_5_IRQHandler(void) {																				//  Обработчик прерываний EXTI9_5 (кнопка PB6 и IR PB5)    
    if (EXTI->PR & EXTI_PR_PR6) {																			// Кнопка на PB6 (антидребезг)
        EXTI->PR = EXTI_PR_PR6;
        button_tick = TIM4->CNT;																			// button_tick
        button_pressed = 1;																						//PRESS BUTTON		Кнопка нажата, установили флаг		
    }
//==============================================================
// IR-приёмник на PB5
//==============================================================		
    if (EXTI->PR & EXTI_PR_PR5) {
        EXTI->PR = EXTI_PR_PR5;
        //GPIOC->ODR ^= (1 << 13);   // мигаем светодиодом
        ir_data_received = 1;
        IR_Process_Bit();          																							// обработка битового потока IR
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
					if (ir_delay_counter > 0) {																						//	Ниже четыре строки для включения прерывания (маскировка второй посылки IR)
					ir_delay_counter = 0;
					EXTI->PR = EXTI_PR_PR5;       																				// Сжигаем застрявший в очереди флаг дубля пульта
					NVIC_EnableIRQ(EXTI9_5_IRQn);  																				// IRQ ON Пульт снова готов
}
					if (button_pressed) {
					uint16_t now = TIM4->CNT;
					uint16_t diff = (now >= button_tick) ? (now - button_tick) : (now + 10000 - button_tick);
					if (diff >= DEBOUNCE_TICKS) {
					button_pressed = 0;
					ProcessButtonPress();																									//=====================================================CALL_ProcessButtonPress================================================
            }
        }
    }
}

extern volatile uint16_t ir_delay_counter; 

// =====================================================================================Обработчик TIM2 (энкодер)======================================
void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF;        
        if (mute_state) {																														// Если Mute включён — выключаем при любом действии с энкодером
            SSD1322_CommandWrite(0xAF);
            mute_state = 0;
            halt_counter = 0;
        } else {
            halt_counter = 0;
            enc_direction = Encoder_direction();

            if (menu_level) {
//=====================================================				
 // Навигация по меню  J S A !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

if (enc_direction) {
    // --- КРУТИМ ВПРАВО (ВЕЛИЧЕНИЕ) ---
    switch (menu_mode_val) {
        case 0: 																																	// =============================Громкость=============================
            if (volume_val < 255) volume_val++;
            break;
        case 1: 																																	// ===============================Вход================================
            input_val = (input_val + 1) % 3; 																			// Крутим по кругу: USB -> COA -> OPT -> BRIGHT
            break;
        case 2: 																																	// ==============================Фильтр===============================
            if (filter_val < 7) filter_val++; 																		// Например, всего 5 фильтров (0..4)
            break;
        case 3: 																																	// ==============================Баланс===============================
            if (balance_val < 177) balance_val++;																	// Сдвиг в правый канал
            break;
				case 4: 																																	// ==============================Яркость==============================
            if (contrast_val < 255) contrast_val++;														// Сдвиг в правый канал
						SSD1322_CmdDataWrite(0xc1, contrast_val);
            break;
    }
} else {
    // --- КРУТИМ ВЛЕВО (УМЕНЬШЕНИЕ) ---
    switch (menu_mode_val) {
        case 0: 																																	// =============================Громкость=============================
            if (volume_val > 0) volume_val--;
            break;
        case 1: 																																	// ===============================Вход================================
            input_val = (input_val == 0) ? 2 : (input_val - 1);										// Крутим по кругу: USB -> COA -> OPT -> BRIGHT
            break;
        case 2: 																																	// ==============================Фильтр===============================
            if (filter_val > 0) filter_val--;																			// Например, всего 8 фильтров (0..7)
            break;
        case 3: 																																	// ==============================Баланс===============================
            if (balance_val > 77) balance_val--;																	// Сдвиг в левый канал
            break;
				case 4: 																																	// ==============================Яркость==============================
            if (contrast_val > 0) contrast_val--;															// Сдвиг в правый канал
						SSD1322_CmdDataWrite(0xc1, contrast_val);
            break;
    }
}
menu_need_update = 1; 																														// Устанавливаем флажок обновление Дисплея запустится через While(1)
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



//*/===========================Переключение фильтров=================================================
void ESS9028_SetFilter(uint8_t filter_num) {
    uint8_t reg_val = 0;
    
    // В Sabre регистр 7 отвечает за фильтры (биты 7:5 определяют тип FIR-фильтра)
    // Математически сдвигаем номер фильтра на 5 битов влево
    reg_val = (filter_num << 5) & 0xE0; 
    
    // Добавляем дефолтные настройки для остальных битов регистра 7 (например, оставляем DSD/OSF)
    reg_val |= 0x0C; 
    
    filter_val = filter_num; // Запоминаем в систему
    
    // Выстрел в регистр 7 чипа по I2C
    ESS9028_WriteReg(ESS9028_I2C_ADDR, 7, reg_val);
}

//*/===========================Переключение логики входов (Регистр 1)=================================================

void ESS9039_SetInput(uint8_t input_num) {
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
    
    input_val = input_num; // Фиксируем в памяти
    
    // Выстрел в регистр 1 по I2C
    ESS9028_WriteReg(ESS9028_I2C_ADDR, 1, reg_val);
}


  // */=================================Громкость баланс===========================================
/*void Set_Volume_And_Balance(uint8_t volume_val, uint8_t balance_val) {
    // 1. ПЕРЕВОДИМ НАШУ ГРОМКОСТЬ В БАЗОВЫЕ БАЙТЫ ЦАПА (РАЗВОРАЧИВАЕМ ШКАЛУ)
    int16_t calcLeft  = 255 - volume_val;
    int16_t calcRight = 255 - volume_val;
    
    // 2. РАСЧЕТ БАЛАНСА НА ЧИСТЫХ ЦЕЛЫХ ЧИСЛАХ (127 - ЦЕНТР)
    if (balance_val < 127) {
        // Баланс влево: душим ПРАВЫЙ канал
        calcRight += (127 - balance_val);
    }
    else if (balance_val > 127) {
        // Баланс вправо: душим ЛЕВЫЙ канал
        calcLeft += (balance_val - 127);
    }
    
    // 3. ПРОВЕРЯЕМ ГРАНИЦЫ И ЗАПИСЫВАЕМ В БЕЗЗНАКОВЫЕ БАЙТЫ РЕГИСТРОВ
    if (calcLeft > 255)  calcLeft = 255;  // Полная тишина, если пережали балансом
    if (calcLeft < 0)    calcLeft = 0;    // Максимальный звук
    if (calcRight > 255) calcRight = 255;
    if (calcRight < 0)   calcRight = 0;
    
    uint8_t regLeft  = (uint8_t)calcLeft;
    uint8_t regRight = (uint8_t)calcRight;
    
    // 4. ОТПРАВЛЯЕМ В РЕГИСТРЫ ЦАПА ПО I2C
    //ESS9028_WriteReg(ESS9028_I2C_ADDR, 15, regLeft);  // Левый канал
    //ESS9028_WriteReg(ESS9028_I2C_ADDR, 16, regRight); // Правый канал
}*/

/**
  * @brief  Вывод информации о входе и частоте (главный экран)
  * @param  None
  * @retval None
  */

/**
  * @brief  Вывод сообщения о Mute
  * @param  None
  * @retval None
  */
/*void MUTED_LCD(void) {
    if (!updated) { updated = 1; SSD1322_ClearRAM(); }
    delay_ms(50);
    SSD1322_DrawString(0, 5, 0, (unsigned char*)"DAC IS  MUTED");
}*//**
  * @brief  Настройка IR-приёмника на PB5 (EXTI, подтяжка, прерывание)
  * @param  None
  * @retval None
  */





// =================IR-REMOTE====================================================================================================================================================
//-------------------------------------------
//-------------------0X0B--------------------
//-------------------------------------------
//-------------------------------------------
//-------------------------------------------
//-----0X08----------0X5D-----------0X07-----
//-------------------------------------------
//-------------------------------------------
//-------------------------------------------
//-------------------0X0D--------------------
//-------------------------------------------
//-------------------------------------------
//-------------------------------------------
//-----0X02-------------------------0X5E-----
//-------------------------------------------

void Process_IR_Command(uint32_t ir_code) {
				switch (ir_code) {
        case 0x0B: // VOLUME++
            if (volume_val < 254) volume_val = volume_val + 2 ;
            menu_need_update = 1; // Взводим наш родной флаг экрана!
            break;
        case 0x0D: // VOLUME--
            if (volume_val > 0) volume_val = volume_val - 2;
            menu_need_update = 1;
            break;
        case 0x5D: // MUTE
            mute_flag = !mute_flag; // Переключаем флаг при каждом нажатии (0 превратится в 1, а 1 в 0)
						Set_DAC_Mute(mute_flag); // Дергаем нашу функцию --------MUTE--------- + DISP
            menu_need_update = 1;
            break;
				case 0x07: // BALANCE>R
            if (balance_val < 177) balance_val++;
            menu_mode_val = 3;     // Принудительно включаем экран БАЛАНСА
            menu_need_update = 1;
						menu_counter = 1000;      // Включаем встроенные часы!
            break;
            
        case 0x08: // BALANCE<L
            if (balance_val > 77) balance_val--;
            menu_mode_val = 3;     // Принудительно включаем экран БАЛАНСА
            menu_need_update = 1;
            menu_counter = 1000;      // Включаем встроенные часы!
            break;

				case 0x02: // INPUT 
            input_val = (input_val + 1) % 3; // Крутим по кругу: USB -> COA -> OPT
            menu_need_update = 1;
            break;
				case 0x5E: // FILTRE++
						filter_val = (filter_val + 1) % 8; // Крутим фильтры: 0 -> 7 -> 0
            menu_need_update = 1;
            break;
    }
}  
//
    


// ============================================================
//   ВЫВОД ВСЕЙ НАДПИСИ "ALEKS" ПОСТРОЧНО С ВЫРАВНИВАНИЕМ
// ============================================================
void 	SSD1322_DrawAleksFull(uint8_t start_col_addr, uint8_t start_row, uint16_t time) {	// X,Y,time
			SSD1322_CommandWrite(0x15);
			SSD1322_DataWrite(start_col_addr);           									// Начало (например, 0x1C)
			SSD1322_DataWrite(start_col_addr + 50 - 1);  									// Конец окна
			SSD1322_CommandWrite(0x75);
			SSD1322_DataWrite(start_row);
			SSD1322_DataWrite(start_row + 24 - 1);       									// Высота шрифта
			SSD1322_CommandWrite(0x5C); 																	// Начинаем лить данные в RAM
			for (uint8_t row = 0; row < 24; row++) {											// Проходим по всем 5 буквам слова "ALEKS"200 пикселей / 4 = 50 единиц адресации столбцов.
			for (uint8_t ch = 0; ch < 5; ch++) {													// Указатель на начало текущей буквы (шаг 38 колонок) 5 букв по 40 пикселей (38 + 2 пустых) = 200 пикселей.
			const uint32_t *char_ptr = &Font_Aleks38x24[(ch * 39) + 1];		//const uint32_t *char_ptr = &Font_Aleks38x24[ch * 38];// Выводим 19 байт (38 пикселей) текущей буквы
			for (uint8_t b = 0; b < 19; b++) {
			uint8_t out_byte = 0x00;		                          				// Проверяем бит в вертикальной колонке массива	
			if (char_ptr[b * 2]     & (1UL << row)) out_byte |= 0xF0;			// Если буква смещена на 1 пиксель, используй (1UL << (row + 1))
			if (char_ptr[b * 2 + 1] & (1UL << row)) out_byte |= 0x0F;
			SSD1322_DataWrite(out_byte);																	// ДОБИВКА: 20-й пустой байт (2 пикселя), чтобы буква стала кратна 4
}
			SSD1322_DataWrite(0x00);																			// Это гарантирует, что контроллер ровно закроет "шаг" адресации
		}
	}
			//SSD1322_DrawSmallString(28, 35, "SERGEY888");						//        TEST FONT
			delay_ms(time);																								//Заставка держится 1 секунду
			SSD1322_ClearRAM();
}


// ====================================================================================================================================================================
// ============================================================
//   
// ============================================================
void spice(uint16_t time){
		SSD1322_DrawSmallString(1, 1,  " HIGH RESOLUTION DSD512 & PCM768");
    SSD1322_DrawSmallString(1, 14, "  HIGH-END DAC ESS9039Q2M v3.7  ");
    SSD1322_DrawSmallString(1, 27, "   CIRCUIT & DESIGN & SOFT BY   ");
    SSD1322_DrawSmallString(1, 43, "    YARIGIN SERGEY (JSA) 2026   ");
    SSD1322_DrawSmallString(1, 56, "    MADE ON THE PLANET EARTH    ");
		delay_ms(time);																								//Заставка держится 1 секунду
		SSD1322_ClearRAM();
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
    RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;        // включаем тактирование TIM3
    TIM3->PSC = 719;                           // предделитель 719 (шаг 10 мкс)
    TIM3->ARR = 0xFFFF;                        // максимум (16 бит)
    TIM3->CR1 |= TIM_CR1_CEN;                  // запускаем счёт
}





//******************************************************************************************************** M A I N ************************************************************************
int main(void) {		

// =====================================================================//
// ИНИЦИАЛИЗАЦИЯ ПАМЯТИ: ПРОВЕРЯЕМ СТАТУС, ЧИТАЕМ И РАЗДАЕМ ПАРАМЕТРЫ		//
// =====================================================================//
    if (*(__IO uint32_t*)FLASH_PRESET_ADDR == 0xFFFFFFFF) {								// Если чистая — прописываем её дефолтными значениями в шагах крутилки
																																					
        preset.input_select = 0;   																				// USB вход при первом старте
        preset.digital_filter = 0; 																				// 0-й фильтр
        preset.contrast = 0x2F;    																				// Яркость OLED экрана
        preset.volume_int = 80;    																				// Стартовые 80 шагов громкости
        preset.balance_int = 127;  																				// Баланс ровно по центру (127 из 255)
        //preset.contrast = 30;   																					// Яркость дисплея
        FLASH_WriteSettings();     																				// Сохраняем эту базу в подвал
    }


    FLASH_ReadSettings();   																			 				//  считываем рабочие данные из флешки на полку структуры

    
    input_val   		= preset.input_select;
    filter_val  		= preset.digital_filter;															// Присваиваем значения из структуры в рабочие переменные  крутилки
    volume_val  		= preset.volume_int;
    balance_val 		= preset.balance_int;
	  contrast_val 		= preset.contrast;   


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
    
		
		// Переменная счетчика задержки ИК-пульта (мы ее объявили volatile в другом файле)


    // ----- Настройка TIM4 (1 кГц) -----
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
    TIM4->PSC = SystemCoreClock / 7200 - 1;						//10000
    TIM4->ARR = 10;																	//10000
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
				//spice(30000);
/* USER CODE BEGIN 2 */

/* USER CODE BEGIN 2 */

// Задаем свой счетчик секунд для старта
uint8_t startup_seconds = 0;


				if ((GPIOA->IDR & (1 << 6)) == 0) {																	// Опрашиваем вход PA6 напрямую через регистр IDR (при нажатии — 0)    
				while ((GPIOA->IDR & (1 << 6)) == 0) {													// Пока кнопка зажата, сами считаем секунды       
        delay_ms(1000); 																						// Ждем ровно 1 секунду
        startup_seconds++; 																					// Увеличиваем наш стартовый счетчик        																																		// Как только зажали на 3 секунды
        if (startup_seconds >= 10) {            
            spice(30000); 																					// Врубаем визитку! Буквы сразу полетят на стекло по SPI
            
            
 
        }
    }
}




				SSD1322_DrawAleksFull(35, 20, 1000);												// Пример вызова: X=30 (байт), Y=20 (строка)
				uint16_t last_encoder_value = 0; 														// Наш эталон для сравнения ручки
				menu_need_update = 1; 																			// Принудительный запуск экрана при старте прибора
				menu_level = 1;       																			// Принудительно открываем шлагбаум для крутилки со старта!

//===========================================================================================================================================================================
    while (1) {
				if (menu_need_update) {
        menu_need_update = 0; 																			// Сбрасываем флаг
        Update_Bottom_Line(); 																			// Спокойно и не спеша шлём данные в SPI в фоне!
				}	
				if 	(ir_packet_ready) {
            ir_packet_ready = 0; // Сразу сбрасываем флаг прерывания

						Process_IR_Command(ir_rx_buffer[2]); 
			
				}

	        // Сторож автовозврата экрана на главный
        if (menu_counter >= 3000 && menu_mode_val == 3) 
        {
            menu_mode_val = 0;    // Сбрасываем экран на ГРОМКОСТЬ
            menu_need_update = 1; // Машем флажком перерисовки
            menu_counter = 0;     // Останавливаем и обнуляем часы
        }
if 	(new_signal_received) {
						new_signal_received = 0;
            Process_XMOS_Signal();
				}
	}
}
