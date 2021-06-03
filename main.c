#define F_CPU 16500000L
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <stdbool.h> 
#include "usbdrv.h"
#include "max7219led8x8.h"

#define USB_DATA_IN 1
#define USB_DRAW_MATRIX 2
#define USB_STRING_SCROLL 3
#define USB_DATA_OUT 5

#define MAX7219_DIN     PB0
#define MAX7219_CS      PB1
#define MAX7219_CLK     PB2


static uchar replyBuf[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static uchar dataReceived = 0, dataLength = 0; // for USB_DATA_IN

// this gets called when custom control message is received
USB_PUBLIC uchar usbFunctionSetup(uchar data[8]) {
	usbRequest_t *rq = (void *)data; // cast data to correct type
	
	switch(rq->bRequest) { // custom command is in the bRequest field

    case USB_DRAW_MATRIX:
        for(uint8_t i = 0; i< 8 ; i++){         //Set pixels
            uchar row = replyBuf[i];
            for(uint8_t j = 0; j< 8; j++){
                if(row & (0b00000001)){
                    MAX7219_pixel_set(7-i, j);   // Set pixel
                }
                else
                {
                    MAX7219_pixel_clr(7-i, j);   // Clear pixel
                }
                row = row >> 1;
            }
        }
        MAX7219_buffer_out();   // Output the buffer
        return 0;

    case USB_STRING_SCROLL:
        if(dataLength == 1){                    //if only one character is passed
            uint8_t letter = replyBuf[0];
            if(letter == 32){
                MAX7219_buffer_clear_all();
                return 0;
            }
            letter -= 65;
            MAX7219_buffer_wc(letter);
            return 0;
        }
        
        else{                                  //more than one character
            uint8_t letter = replyBuf[0];
            if(letter == 32)
                MAX7219_buffer_next_load(26);               //space (empty character)
            else{
                letter -= 65;
                MAX7219_buffer_wc(letter);          //write first char to display
            }
        
            for(uint8_t i = 0; i < dataLength * 8; i++){            //loop is run 8*num of chars (each loop shifts 1 col. (there are 8 cols in each char))
                if((i % 8 == 0) && (i/8 != (dataLength -1))){       //if 1st column of char and not last char
                    uint8_t letter = replyBuf[(i/8) + 1];
                    if(letter == 32){
                        MAX7219_buffer_next_load(26);               //space (empty character)
                    }
                    else{
                    letter -= 65;
                    MAX7219_buffer_next_load(letter);               //load next char to buffer
                    }
                }
                MAX7219_buffer_out();                               //display buffer
                _delay_ms(70);
                MAX7219_buffer_shift();                             //shift buffer
                wdt_reset();                                        // reset watchdog timer
            }
        }

        return 0;

    case USB_DATA_OUT: // send data to PC
        usbMsgPtr = replyBuf;
        return sizeof(replyBuf);

    case USB_DATA_IN: // receive data from PC
		dataLength  = (uchar)rq->wLength.word;
        dataReceived = 0;
		
		if(dataLength  > sizeof(replyBuf)) // limit to buffer size
			dataLength  = sizeof(replyBuf);
			
		return USB_NO_MSG; // usbFunctionWrite will be called now
    }

    return 0; // should not get here
}

// This gets called when data is sent from PC to the device
USB_PUBLIC uchar usbFunctionWrite(uchar *data, uchar len) {
	uchar i;
			
	for(i = 0; dataReceived < dataLength && i < len; i++, dataReceived++)
		replyBuf[dataReceived] = data[i];
		
    return (dataReceived == dataLength); // 1 if we received it all, 0 if not
}

#define abs(x) ((x) > 0 ? (x) : (-x))

static void oscCal(){
    int frameLength, targetLength = (unsigned)(1499 * (double)F_CPU / 10.5e6 + 0.5);
    int bestDeviation = 9999;
    uchar trialCal, bestCal, step, region;

    // do a binary search in regions 0-127 and 128-255 to get optimum OSCCAL
    for(region = 0; region <= 1; region++) {
        frameLength = 0;
        trialCal = (region == 0) ? 0 : 128;
        
        for(step = 64; step > 0; step >>= 1) { 
            if(frameLength < targetLength) // true for initial iteration
                trialCal += step; // frequency too low
            else
                trialCal -= step; // frequency too high
                
            OSCCAL = trialCal;
            frameLength = usbMeasureFrameLength();
            
            if(abs(frameLength-targetLength) < bestDeviation) {
                bestCal = trialCal; // new optimum found
                bestDeviation = abs(frameLength -targetLength);
            }
        }
    }

    OSCCAL = bestCal;
}
// Called by V-USB after device reset
void hadUsbReset() {            //performing OSC callibration
    oscCal();
}

int main() {
	uchar i;
    cli();
    oscCal();
    sei();
    //OSCCAL = eeprom_read_byte(0);
    DDRB &= ~(0b00011000);

    MAX7219_init();
    MAX7219_buffer_clear_all();

    /*while (1) {       //cycle through every led
        for (uint8_t y = 0; y < 5; y++) {
            for (uint8_t x = 0; x < 5; x++) {
                MAX7219_pixel_set(x, y);   // Set pixel
                _delay_ms(10);
            }
        }
        MAX7219_buffer_out();   // Output the buffer

        for (uint8_t y = 0; y < 7; y++) {
            for (uint8_t x = 0; x < 7; x++) {
                MAX7219_pixel_clr(x, y);   // Clear pixel
                _delay_ms(10);
            }
        }
        MAX7219_buffer_out();   // Output the buffer
    } */
	
    wdt_enable(WDTO_1S); // enable 1s watchdog timer

    cli();
    usbDeviceDisconnect(); // enforce re-enumeration
    for(i = 0; i<250; i++) { // wait 500 ms
        wdt_reset(); // keep the watchdog happy
        _delay_ms(2);
    }
    usbDeviceConnect();
	usbInit();
    sei(); // Enable interrupts after re-enumeration
	
    while(1) {
        wdt_reset(); // keep the watchdog happy
        usbPoll();
    };
	
    return 0;
}
