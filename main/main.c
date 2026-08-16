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
 *  Адрес:  #define FLASH_PRESET_ADDR   0x0800FC00.
 */


    typedef struct {
    uint8_t input_select;   // Выбор входа (1 байт)
    uint8_t digital_filter; // Цифровой фильтр (1 байт)
    uint8_t contrast;       // Яркость OLED (1 байт)
    uint8_t volume_int;     // Громкость в шагах 0..255 (1 байт)
    uint8_t balance_int;    // Баланс в шагах 0..255 (1 байт)
    uint8_t reserve1;       // Пустышка для ровного счёта
    uint8_t reserve2;       // Пустышка для ровного счёта
    uint8_t reserve3;       // Пустышка для ровного счёта
} presets_def;
	presets_def preset;





/**
  * @brief  Чтение настроек из Flash
  * @param  None
  * @retval None
  */
void FLASH_ReadSettings(void) {
    uint16_t p;
    uint32_t *source_adr = (uint32_t *)(FLASH_PRESET_ADDR);
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
		uint32_t pageAdr = FLASH_PRESET_ADDR;
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
    
        switch (menu_mode_val) {
        case 0: // MODE_VOLUME
            // Передаем реальную громкость из volume_val вместо статической 24
            SSD1322_DrawString(0, 20, 1,(unsigned char*)"USB DSD 44.1");//sprintf(buf, "VOL: -%d dB ", volume_val); 
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
        switch (menu_mode_val) {
            case 0: if (vol < 100) vol++; break;
            case 1: input = (input + 1) % 3; break; // USB -> COA -> OPT
            case 2: if (filter < 7) filter++; break; // Всего 8 фильтров у ESS9039
        }
    } else { // Крутим влево (уменьшение)
        switch (menu_mode_val) {
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
	
	if (is_long) {
    // ВАРИАНТ 2: ДЛИННЫЙ ПРЕСС
    // Ждем, пока отпустишь кнопку
		//************************************************************************************************
    while ((GPIOA->IDR & (1 << 6)) == 0) { __NOP(); }
    
    // Смотрим, в каком пункте меню мы зажали кнопку, и обновляем только его в структуре:
    if (menu_mode_val == 0) {
        preset.volume_int = volume_val;   // Запоминаем только громкость
    }
    else if (menu_mode_val == 1) {
        preset.input_select = input_val;  // Запоминаем только вход
    }
    else if (menu_mode_val == 2) {
        preset.digital_filter = filter_val; // Запоминаем только фильтр
    }
    else if (menu_mode_val == 3) {
        preset.balance_int = balance_val;  // Запоминаем только баланс
    }
    
    // Пуляем обновлённую структуру в наш безопасный подвал флеша!
    FLASH_WriteSettings(); 
}

//*************************************************************************************************
    while ((GPIOA->IDR & (1 << 6)) == 0) { __NOP(); }
}
else {
    // ВАРИАНТ 1: КОРОТКИЙ КЛИК
		menu_level = 1; // Активируем меню, чтобы TIM2 пустил нас к крутилке!
    menu_mode_val++;
    if (menu_mode_val > 3) {
        menu_mode_val = 0; // Наша громкость MODE_VOLUME
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
    switch (menu_mode_val) {
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
    switch (menu_mode_val) {
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
    
    input_val = input_num; // Фиксируем в памяти
    
    // Выстрел в регистр 1 по I2C
    ESS9028_WriteReg(ESS9028_I2C_ADDR, 1, reg_val);
}


 // */=================================Громкость баланс===========================================
void Set_Volume_And_Balance(uint8_t volume_val, uint8_t balance_val) {
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
    ESS9028_WriteReg(ESS9028_I2C_ADDR, 15, regLeft);  // Левый канал
    ESS9028_WriteReg(ESS9028_I2C_ADDR, 16, regRight); // Правый канал
}

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
int main(void) {			//******************************************************************************************************** M A I N *************************
	//==========================================================================================
	// ====================================================================
    // ИНИЦИАЛИЗАЦИЯ ПАМЯТИ: ПРОВЕРЯЕМ СТАТУС, ЧИТАЕМ И РАЗДАЕМ ПАРАМЕТРЫ
    // ====================================================================
    
    // Шаг 1: Проверяем напрямую первое слово во флешке — чистая она или уже Рабочая?
    if (*(__IO uint32_t*)FLASH_PRESET_ADDR == 0xFFFFFFFF) {
        // Если чистая — прописываем её дефолтными значениями в шагах крутилки
        preset.input_select = 0;   // USB вход при первом старте
        preset.digital_filter = 0; // 0-й фильтр
        preset.contrast = 0x1F;    // Яркость OLED экрана
        preset.volume_int = 80;    // Стартовые 80 шагов громкости
        preset.balance_int = 127;  // Баланс ровно по центру (127 из 255)
        
        FLASH_WriteSettings();     // Сохраняем эту базу в подвал
    }

    // Шаг 2: Теперь гарантированно считываем рабочие данные из флешки на полку структуры
    FLASH_ReadSettings();

    // Шаг 3: Присваиваем значения из структуры в рабочие переменные твоей крутилки
    input_val   = preset.input_select;
    filter_val  = preset.digital_filter;
    volume_val  = preset.volume_int;
    balance_val = preset.balance_int;
	     


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

        ProcessButtonPress();
				// ====================================================================
        // ОБРАБОТКА ЭНКОДЕРА ГРОМКОСТИ ПРЯМЫМ ХОДОМ
        // ====================================================================
        if (TIM3->CNT > last_encoder_value) {
            // Крутанули ВПРАВО — прибавляем звук на 0.5 дБ
            volume_val += 1;
            if (volume_val > 255) volume_val = 255; // Наш честный максимум байта
            Set_Volume_And_Balance(volume_val, balance_val); // Пуляем в ЦАП!
            menu_need_update = 1; // Флаг на отрисовку экрана
            last_encoder_value = TIM3->CNT;
        }
        else if (TIM3->CNT < last_encoder_value) {
            // Крутанули ВЛЕВО — убавляем звук на 0.5 дБ
            volume_val -= 1;
            if (volume_val == 0 || volume_val > 255) volume_val = 0; // Безопасный стоп на полной тишине   
            Set_Volume_And_Balance(volume_val, balance_val); // Пуляем в ЦАП!
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
             case 0x1E: // Кнопка "Громкость +"
								// Если кнопку зажали — прибавляем быстрее (например, по 4 шага), если клик — по 2 шага
								if (hold_counter > 5) volume_val += 4;
								else volume_val += 2;

								// Ограничение максимума: выше 255 байт подняться физически не может!
								if (volume_val > 255) volume_val = 255; 

								Set_Volume_And_Balance(volume_val, balance_val);
								menu_need_update = 1;
								break;

						case 0x1F: // Кнопка "Громкость -"
								// Если зажали — убавляем быстрее (по 4 шага), если клик — по 2 шага
								if (hold_counter > 5) volume_val -= 4;
								else volume_val -= 2;

								// Защита снизу: если громкость ушла в ноль или попыталась улететь ниже нуля (переполниться)
								if (volume_val > 255 || volume_val == 0) volume_val = 0; // Полная тишина

								Set_Volume_And_Balance(volume_val, balance_val);
								menu_need_update = 1;
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
            if (menu_mode_val == MODE_VOLUME) {
                volume_val += 0.5f;
                if (volume_val > 0.0f) volume_val = 0.0f;
                Set_Volume_And_Balance(volume_val, balance_val);
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
            // наши новые вещественные переменные типа: sprintf(buf, "Vol: %.1f dB", volume_val);
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
		

				
				Set_Volume_And_Balance(volume_val, balance_val);			//  ОТПРАВКА НАСТРОЕК ЗВУКА В РЕГИСТРЫ ЦАП       
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