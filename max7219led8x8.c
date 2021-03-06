
#define F_CPU 16500000L
#include <stdlib.h>
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/pgmspace.h>
#include "max7219led8x8.h"
#include "character_bank.h"
#include "usbdrv.h"


void MAX7219_byte(uint8_t data)
{
    PORTB &= ~(1 << MAX7219_CS);    // Set to LOW
    for(uint8_t i = 8; i >= 1; i--)
    {
        PORTB &= ~(1 << MAX7219_CLK);   // Set to LOW
        if ((data & 0x80) != 0)         // Mask the MSB of the data
            PORTB |= (1 << MAX7219_DIN);    // Set to HIGH
        else
            PORTB &= ~(1 << MAX7219_DIN);   // Set to LOW
        data = data<<1;
        PORTB |= (1 << MAX7219_CLK);        // Set to HIGH
    }
}
 
void MAX7219_word(uint8_t address, uint8_t data)
{
    PORTB &= ~(1 << MAX7219_CS);    // Set to LOW
    MAX7219_byte(address);          //
    MAX7219_byte(data);             //
    PORTB |= (1 << MAX7219_CS);     // Set to HIGH
}
 
void MAX7219_init(void)
{
    DDRB |= (1 << MAX7219_CLK); // Set port as output
    DDRB |= (1 << MAX7219_CS);  // Set port as output
    DDRB |= (1 << MAX7219_DIN); // Set port as output
    _delay_ms(50);  // TODO: Q: Is this necessary?
    MAX7219_word(0x09, 0x00);   // Decode: BCD
    MAX7219_word(0x0a, 0x03);   // Brightness
    MAX7219_word(0x0b, 0x07);   //
    MAX7219_word(0x0c, 0x01);   //
    MAX7219_word(0x0f, 0x00);   //
}
 
void MAX7219_row(uint8_t address, uint8_t data) {
    if (address >= 1 && address <= 8) MAX7219_word(address, data);
}
 
uint8_t MAX7219_buffer[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
 
void MAX7219_buffer_out(void) {
    // Output the buffer
    for (uint8_t row = 1; row <= 8; row++)
        MAX7219_row(row, MAX7219_buffer[row - 1]);
}
 
void MAX7219_pixel_set(uint8_t x, uint8_t y) {
    uint8_t sx = 7 - (x & 0b0111);
    uint8_t sy = (y & 0b0111);
    MAX7219_buffer[sy] |= (1 << sx);
}
 
void MAX7219_pixel_clr(uint8_t x, uint8_t y) {
    uint8_t sx = 7 - (x & 0b0111);
    uint8_t sy = (y & 0b0111);
    MAX7219_buffer[sy] &= ~(1 << sx);
}

void MAX7219_buffer_clear_all(void){
    for (uint8_t i = 1; i <= 16; i++)
        MAX7219_buffer[i - 1] = 0x00;
    MAX7219_buffer_out();
}

void MAX7219_buffer_wc(uint8_t x){
    x = x * 8;
    for(uint8_t row = 1; row <= 8; row++){
        MAX7219_buffer[row-1] = pgm_read_byte(&characters[x + (row -1)]);
    }
    MAX7219_buffer_out();
}

void MAX7219_buffer_next_load(uint8_t x){
    x = x * 8;
    for(uint8_t row = 1; row <= 8; row++){
        MAX7219_buffer[row-1 + 8] = pgm_read_byte(&characters[x + (row -1)]);
    }
}

void MAX7219_buffer_shift(void){
    uint8_t i;
    for(i = 1; i < 16; i++){
        MAX7219_buffer[i -1] = MAX7219_buffer[i];
    }
    MAX7219_buffer[i-1] = 0;
}
