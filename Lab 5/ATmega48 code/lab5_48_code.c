// Makenzie Brian
// December 3, 2017
// ECE473
// lab5_48_code.c
// Control for atmega48 to send temp to 128 via uart.

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include "uart_functions_m48.h"
#include "twi_master.h"
#include "lm73_functions.h"

#define LM73_ADDRESS 0x90

uint8_t i;
uint16_t lm73_temp;
char str[2] = {'0','0'};
char lcd_string[3];

extern uint8_t lm73_wr_buf[2];
extern uint8_t lm73_rd_buf[2];

int main(){
	uint16_t lm73_temp;
	uint8_t temperature[1] = {0};

	DDRB = 0xFF;	//portb to output
	DDRD = 0xFF;	//portd to output

	uart_init();	//init uart
	init_twi();		//init twi

	sei();			//turn on interrupts

	lm73_wr_buf[0] = 0b00000000;		//sensor to read temp
	twi_start_wr(LM73_ADDRESS,lm73_wr_buf,2);
	_delay_ms(2);
	
	while(1){
		temperature[0] = 0;		//init to zero

		_delay_ms(100);
		twi_start_rd(LM73_ADDRESS,lm73_rd_buf,2); //get temp
		_delay_ms(2);

		lm73_temp = lm73_rd_buf[0];		//read temp
		lm73_temp = lm73_temp << 8;
		lm73_temp |= lm73_rd_buf[1];

		lm73_temp_convert(temperature,lm73_temp,1);//convert to temp
		temperature[0] = ((temperature[0]*9)/5+32);//to farenheit
		itoa(temperature[0],lcd_string,10);
		
		while(uart_getc()!='t'){}		//wait for t
		_delay_us(100);
		uart_putc(lcd_string[0]);		//send temp
		uart_putc(lcd_string[1]);
	}//while
}//main
