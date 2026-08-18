#include "ir_remote.h"
#include "stm32f10x.h"

// ============================================================
//  АВТОМАТ ДЛЯ ПРИЁМА И РАСПАКОВКИ ВСЕХ 4 БАЙТ ПАКЕТА NEC
//  Настройка EXTI: строго по СПАДУ (Falling Edge)
//  Таймер TIM3: PSC = 719 (шаг счета 10 мкс)
// ============================================================
// ============================================================
volatile uint32_t ir_durations[32];
volatile uint8_t  ir_duration_index = 0;
// ============================================================

#define IR_STATE_IDLE     0  // Ожидание стартового спада
#define IR_STATE_DATA     1  // Приём 32 битов данных

volatile uint8_t  ir_state = IR_STATE_IDLE;
volatile uint8_t  ir_bit_count = 0;
volatile uint32_t ir_current_packet = 0; // Сюда собираем все 32 бита
volatile uint8_t  ir_packet_ready = 0;
volatile uint8_t  ir_rx_buffer[4];       // Буфер под все 4 байта пульта
volatile uint16_t hold_counter = 0; // Глобальный счётчик удержания пульта
extern volatile uint16_t ir_last_tick;
extern uint32_t screen_return_timer;

// ============================================================
// ОБНОВЛЕННЫЙ АВТОМАТ С ПОДДЕРЖКОЙ ПОВТОРА КНОПКИ (REPEAT)
// ============================================================

void IR_Process_Bit(void) {
    uint32_t duration = TIM3->CNT;
    TIM3->CNT = 0; // Мгновенно обнуляем таймер
// ====================================================================
// ЖЕСТКИЙ КАПКАН ДЛЯ ПОВТОРА (Поиск 98.86 мс)
// ====================================================================
if ((duration > 9500 && duration < 10300) && (ir_state == 0)) {
    hold_counter++;           // Считаем удержания
    ir_last_tick = TIM4->CNT; // Запоминаем текущий тик таймера 4
}
    switch (ir_state) {
        
        case IR_STATE_IDLE:
            // 1. ПРОВЕРКА НА ОБЫЧНЫЙ СТАРТ (13.5 мс -> ~1350 ед.)
            if (duration > 1200 && duration < 1500) {
                ir_state = IR_STATE_DATA; 
                ir_bit_count = 0;
                ir_current_packet = 0;
                break;
            }

            // 2. ПРОВЕРКА НА ИМПУЛЬС ПОВТОРА (11.25 мс -> ~1125 ед.)
            // Если удерживаем кнопку, ворота ставим: 1050 - 1180
            if (duration > 1050 && duration < 1180) {
                // Маяк для main.c: выставляем специальный флаг повтора!
                // Для этого запишем в ir_rx_buffer[0] особый маркер, например, 0xEE
                ir_rx_buffer[0] = 0xEE; 
                ir_packet_ready = 1;      // Говорим мейну: "Кнопку всё ещё держат!"
                ir_state = IR_STATE_IDLE; // Сразу уходим в ожидание следующего повтора
            }
            break;

        case IR_STATE_DATA:
            if (duration < 80 || duration > 280) {
                ir_state = IR_STATE_IDLE; 
                break;
            }

            ir_current_packet >>= 1;

            if (duration > 160) {
                ir_current_packet |= 0x80000000; 
            }

            ir_bit_count++;

            if (ir_bit_count >= 32) {
                ir_rx_buffer[0] = (uint8_t)(ir_current_packet & 0xFF);         
                ir_rx_buffer[1] = (uint8_t)((ir_current_packet >> 8) & 0xFF);  
                ir_rx_buffer[2] = (uint8_t)((ir_current_packet >> 16) & 0xFF); 
                ir_rx_buffer[3] = (uint8_t)((ir_current_packet >> 24) & 0xFF); 
                
                ir_packet_ready = 1;      
                ir_state = IR_STATE_IDLE; 
            }
            break;

        default:
            ir_state = IR_STATE_IDLE;
            break;
    }
}


// ============================================================
//  ФУНКЦИЯ ДЛЯ ЗАБОРA ВСЕГО ПАКЕТА В MAIN.C
// ============================================================
uint8_t IR_GetPacket(uint8_t *buffer) {
    if (ir_packet_ready) {
        ir_packet_ready = 0;
        for (int i = 0; i < 4; i++) {
            buffer[i] = ir_rx_buffer[i]; // Копируем все 4 байта в твой буфер
        }
        return 1; // Успех
    }
    return 0; // Данных нет
}
// ============================================================
