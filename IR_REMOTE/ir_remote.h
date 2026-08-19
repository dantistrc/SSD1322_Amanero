#ifndef IR_REMOTE_H
#define IR_REMOTE_H

#include <stdint.h>
#define MAX_DURATIONS 32

typedef struct {
    uint16_t command;
    uint8_t address;
    uint8_t key_code;
} ir_data_t;

void IR_Init(void);
uint8_t IR_GetCommand(ir_data_t *ir_data);
void IR_Process_Bit(void);
uint8_t IR_GetPacket(uint8_t *buffer);   // <-- ÄÎÁÀÂÜ ÝÒÓ ÑÒÐÎÊÓ
void DebugShow(uint8_t step);
void DebugDuration(uint32_t duration);   // â íà÷àëå main.c
extern volatile uint32_t ir_durations[MAX_DURATIONS];
extern volatile uint8_t  ir_duration_index;
extern volatile uint8_t  ir_packet_ready;
extern volatile uint16_t hold_counter;
extern volatile uint8_t ir_rx_buffer[];       // Áóôåð ïîä âñå 4 áàéòà ïóëüòà
extern volatile uint8_t ir_state;
extern volatile uint16_t ir_delay_counter;
#define IR_KEY_POWER           0x61A2
#define IR_KEY_VOLUMEUP        0x40A2
#define IR_KEY_VOLUMEDOWN      0x42A2
#define IR_KEY_PLAY            0x6112
#define IR_KEY_STOP            0x6392
#define IR_KEY_MUTE            0x63A2



#endif