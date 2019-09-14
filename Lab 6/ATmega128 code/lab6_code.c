//	File: lab6_code.c
//	Author: Makenzie Brian
//	Date: 12.5.17
//	Description: Alarm Clock; Keeps time once set with encoders in button mode 1;
//	can set alarm to time with encoders in button mode 2; can turn alarm on/off 
//	with button 3; can snooze alarm with button 4 for 10 seconds; display changes brightness 
//	with ambient light; changes volume with encoder, change radio with encoder;
//	button 6 is radio tune; button 7 is radio alarm on/off; button 8 is radio on/off

//  HARDWARE SETUP:
//  PORTA is connected to the segments of the LED display and to the pushbuttons
//  PORTA.0 corresponds to segment A and pushbutton 0, 
//	PORTA.1 corresponds to segment B and pushbutton 1, etc.
//	The pushbuttons and the 7-segment are connected via pins A_2, B_2, etc.
//	on the 7-segmeant board and controlled via tri-state buffer.
//  PORTB bits 4-6 go to A,B,C inputs of the 74HC138 Decoder.
//	PORTB bits 2-3 are connected to the Bargraph Board using SPI.
//	Encoders are connected to PORTB, PORTC, and PORTE, see beloww for 
//	details. Temp sensor connected via I2C. ATmega48 connected via RS232.

//	ATmega128 	to 	7-Segment
//	PORTA.0		->	A
//	PORTA.1		->	B
//	PORTA.2		->	C
//	PORTA.3		->	D
//	PORTA.4		->	E
//	PORTA.5		->	F
//	PORTA.6		->	G
//	PORTA.7		->	DP
//	PORTB.4		->	SEL0
//	PORTB.5		->	SEL1
//	PORTB.6		->	SEL2
//	PORTB.7		->	PWM
//	GND			->	EN_N
//	GND			->	GND
//	VCC			->	EN
//	VCC			->	VDD

//	7-Segment 	to 	Pushbuttons
//	A_2			->	J1
//	B_2			->	J2
//	C_2			->	J3
//	D_2			->	J4
//	E_2			->	J5
//	F_2			->	J6
//	G_2			->	J7
//	DP_2		->	J8
//	DEC7		->	COM_EN

//	ATmega128	to	Bargraph Board
//	PORTD.2		->	regclk
//	PORTB.1		->	srclk
//	PORTB.2		->	sdin
//	PORTC.3		->	oe_n
//	GND			->	GND
//	VCC			->	VDD
//	NC			->	sd_out

//	ATmega128	to	Encoders
//	PORTB.3		->	SOUT
//	GND			->	SIN			//PC4
//	PORTC.2		->	CKINH
//	PORTB.1		->	SCK
//	PORTE.6		->	SH/LD
//	GND			->	GND
//	VCC			->	VDD

//	ATmega128	to	Pushbuttons
//	GND			->	COM_LVL		//PC1
//	GND			->	GND
//	VCC			->	VDD

//	ATmega128	to	Photocell
//	VCC			->	TopVoltageDivider
//	PORTF.0		->	MidVoltageDivider
//	GND			->	BotVoltageDivider

//	ATmega		to	TempSensor
//	VCC			->	VDD
//	GND			->	GND
//	PD0			->	CLK
//	PD1			->	DAT

//	ATmega		to	ATmega48
//	RX			->	TX_232		all via RS232 PORT
//	TX			->	RX_232
//	GND			->	GND

//	ATmega128 	to Radio
//	VCC			->	VIN
//	GND			->	GND
//	PORTD.0		->	SCLK	through transistor
//	PORTD.1		->	SDIO	through transistor
//	PORTE.7		->	GP02	through transistor
//	PORTE.2		->	RST_N
//	VCC			->	ENBL

//	ATmega128	to	Audio
//	VCC			->	VDD
//	GND			->	GND
//	See schematic for others: uses PE3, PD1



#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include "hd44780.h"
#include "lm73_functions.h"
#include "uart_functions.h"
#include "si4734.h"

//holds data to be sent to the segments. logic zero turns segment on
uint8_t segment_data[5];

//decimal to 7-segment LED display encodings, logic "0" turns on segment
//binary values for 7 seg into array
uint8_t dec_to_7seg[12] = {0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x90, 0xFF, 0xFF}; 

int8_t time[4] = {0};	//hour and minute for time and alarm

volatile uint8_t mode = 0;		//holds mode
volatile uint8_t alarm = 0;		//state of alarm (ON or OFF)
volatile uint8_t enc1[2];		//value of first encoder
volatile uint8_t enc2[2];		//value of second encoder
volatile uint8_t prev_enc1[2] = {0};	//previous value of enc 1
volatile uint8_t prev_enc2[2] = {0};	//previous value of enc 2volatile uint8_t button = 0;	//variable to hold button
volatile uint16_t second = 0;		//hold sec
volatile uint8_t min = 0;		//hold min
volatile uint8_t snooze = 0;			//snooze on/off
volatile uint8_t snooze_cnt = 0;	//counts snooze cycle
uint16_t adc_val = 0;				//collected value from adc
uint8_t alm_snd = 0;		//whether to sound the alarm or not

char* lcd_arr = "Alarm off       "			//starts off
						 "In:00F Out:11F  ";

uint16_t eeprom_fm_freq;		//si4734 variables
uint16_t eeprom_am_freq;
uint16_t eeprom_sw_freq;
uint8_t eeprom_volume;
volatile uint16_t freq_buf;
uint16_t current_fm_freq;
uint16_t current_am_freq;
uint16_t current_sw_freq;
uint8_t current_volume;
extern enum radio_band{FM, AM, SW};
volatile enum radio_band current_radio_band;
extern volatile uint8_t STC_interrupt;

//life lesson: there are not a lot of good radio stations in coravllis
volatile uint16_t radio_presets[8] = {8870,9990,10630,9330,10230,10470,10530,10790};

extern uint8_t lm73_wr_buf[2];	//array, writing to temp
extern uint8_t lm73_rd_buf[2];	//array, reading from temp

volatile uint8_t rad_on = 0;
volatile uint8_t turn_rad_on = 0;
volatile uint8_t turn_rad_off = 0;
volatile uint8_t change_freq = 0;
volatile int8_t f = 2;
volatile uint8_t tune_freq = 0;
volatile uint8_t rad_alm = 0;
volatile uint8_t rad_alm_on = 0;
volatile uint8_t rad_alm_off = 0;


//*****************************************************************************
//                            chk_buttons                                      
//	Checks the state of the button number passed to it. It shifts in ones till   
//	the button is pushed. Function returns a 1 only once per debounced button    
//	push so a debounce and toggle function can be implemented at the same time.  
//	Adapted to check all buttons from Ganssel's "Guide to Debouncing"            
//	Expects active low pushbuttons on PINA port.  Debounce time is determined by 
//	external loop delay times 12. 
//****************************************************************************
uint8_t chk_buttons(uint8_t button){
	static uint16_t state[8] = {0}; //holds present state
	//updates state of button
	state[button] = (state[button] << 1) | (! bit_is_clear(PINA, button)) | 0xE000;
	//buttons pressed if held for 12 cycles, not bounce
	if (state[button] == 0xF000){
		return 1;
	}//if
	return 0;
}//chk_buttons



//************************************************************************
//                              segsum                            
//	Takes a 16-bit binary input value and places the appropriate 
//	equivalent 4 digit BCD segment code in the array segment_data 
//	for display. 
//     
//	Array is loaded at exit as:  |digit3|digit2|colon|digit1|digit0|
//************************************************************************
void segsum(uint8_t hour,uint8_t min,uint8_t blink){
	static uint8_t blink_cnt = 0;	//counter for lbinking segs

	segment_data[0] = dec_to_7seg[min%10];	//data to segs
	segment_data[1] = dec_to_7seg[min/10];	
	segment_data[3] = dec_to_7seg[hour%10];	
	segment_data[4] = dec_to_7seg[hour/10];	

	if(blink == 1){	//blink segs for setting
		if(blink_cnt>175 && blink_cnt<200){	//LEDs off
			segment_data[0] = 0xFF;
			segment_data[1] = 0xFF;
			segment_data[3] = 0xFF;
			segment_data[4] = 0xFF;
		}//if
	}//if

	blink_cnt++;	//inc count
	if(blink_cnt>200)	//reset count
		blink_cnt = 0;

	if(hour / 10 == 0){		//blank leading zero
		segment_data[4] = 0xFF;
	}//if

	//indicates if there is an alarm on the LED display
	switch(alarm){
		case 0x00:
			segment_data[2] |= 0x04;
			break;

		case 0x01:
			segment_data[2] &= ~0x04;
			break;

		default:
			break;
	}//switch
}//segsum



//*****************************************************************************
//                              display_bbg  
//	Displays mode on bar graph via SPI.
//*****************************************************************************

void display_bg(){
	switch(mode){
		case 0x00:			//no mode
			SPDR = 0x00;
		case 0x01:			//clock
			SPDR = mode;
			break;
		case 0x02:			//alarm
			SPDR = mode;
			break;
		default:
			break;
	}//switch

	SPDR = mode; 	//mode out
		
	while(bit_is_clear(SPSR,SPIF)){} 	//wait until transf complete

	PORTD |= 0x04;		//data goes out
	PORTD &= ~0x04;
	
}//display_bg



//***********************************************************************************
//									enc_read
//	Reads spi to see encoder state and changes. Saves to changes to volatiles 
//	based on mode.
//***********************************************************************************
int8_t enc_read(){
	uint8_t tem = 0;	//tem variable to get digits

	//save previous value of encoders for comparison
	prev_enc1[0] = enc1[0];
	prev_enc1[1] = enc1[1];
	prev_enc2[0] = enc2[0];
	prev_enc2[1] = enc2[1];


	//shift data in from encoders
	PORTE &= ~(1<<PE6);		//parallel input to encoder (SH/LD low)
	PORTC |= (1<<PC2);		//ignore serial output(clk inh high)
	SPDR = 0x00;		//send junk data, gets clk running for SPI
	PORTC &= ~(1<<PC2);	//allows for encoder to output data(clk inh low)
	PORTE |= (1<<PE6);		//shift data out of encoder (SH/LD high)
	while(!(SPSR & (1<<SPIF))){}	//wait for transfer complete
	tem = SPDR;		//store encoder data

	enc1[0] = tem & 0x01;			//first encoder bit 1
	enc1[1] = tem & 0x02;			//first encoder bit 2
	enc2[0] = (tem & 0x04)>>2;	//second encoder bit 1
	enc2[1] = (tem & 0x08)>>2;	//second encoder bit 2
	
	//check enc 1 dir to set hour/volume
	if(prev_enc1[0]==enc1[0]){
		if(enc1[0]==1 && (prev_enc1[1]<enc1[1])){
			return 1;
		}//if
		if(enc1[0]==1 && (prev_enc1[1]>enc1[1])){
			return -1;
		}//if
	}//if
		
	//check enc 2 dir to set minute/frequency
	if(prev_enc2[0]==enc2[0]){
		if(enc2[0]==1 && (prev_enc2[1]<enc2[1])){
			return 2;
		}//if
		if(enc2[0]==1 && (prev_enc2[1]>enc2[1])){
			return -2;
		}//if
	}//if
	return 0;
}//enc_read



//***************************************************************************
//									check_adc()
//	Checks the brightness vis photoresistor. Controls LED brightness.
//***************************************************************************
void check_adc(){
	ADCSRA |= (1<<ADSC);				//start transfer
	while(bit_is_clear(ADCSRA,ADIF));	//wait for transfer complete
	ADCSRA |= (1<<ADIF);				//clear flag
	adc_val = ADCH;						//take in value
	
	if(adc_val>100){					//compare value to range
		OCR2 = 50;						//change brightness
	}//else if
	else if(adc_val>90){
		OCR2 = 65;
	}//else if
	else if(adc_val>80){
		OCR2 = 80;
	}//else if
	else if(adc_val>70){
		OCR2 = 105;
	}//else if
	else if(adc_val>60){
		OCR2 = 135;
	}//else if
	else if(adc_val>50){
		OCR2 = 175;
	}//else if
	else if(adc_val>40){
		OCR2 = 205;
	}//else if
	else if(adc_val>30){
		OCR2 = 230;
	}//else if
	else{
		OCR2 = 250;
	}//else
}//check_adc


//*************************************************************************
//							check_mode
//	Checks buttons to see what mode has been selected
//*************************************************************************
void check_mode(){
	uint8_t i;

	//PORTA all inputs
	PORTA = 0xFF;
	DDRA = 0x00;
	
	//enable tristate buffer
	PORTB |= 0x70;
	
	//check buttons for mode/actions
	for(i = 0; i < 8; i++){
		if(chk_buttons(i)){
			switch(i){
				case 0:	//first button, set time mode
					if(bit_is_clear(mode,1) && bit_is_clear(mode,5)){
						mode ^= 0x01;
					}//if
					second = 0;
					break;
				case 1:	//second button, set alarm mode
					if(bit_is_clear(mode,0) && bit_is_clear(mode,5)){
						mode ^= 0x02;
					}//if
					break;
				case 2:	//third button, alarm on/off
					if(bit_is_clear(mode,1)){
						alarm ^= 0x01;
					}//if
					break;
				case 3:	//fourth button, snooze alarm
					if(alm_snd == 1){
						snooze = 0x01;
					}//if
					break;
				case 4:	//fifth button, lower speaker volume
					break;
				case 5:	//sixth button, tunr radio
					if(bit_is_clear(mode,0) && bit_is_clear(mode,1)){
						tune_freq ^= 0x01;
					}//if
					break;
				case 6:	//seventh button, radio alarm on/off
					rad_alm ^= 0x01;
					if(rad_alm == 0x01){
						rad_alm_on = 1;
					}//if
					else if(rad_alm == 0x00){
						rad_alm_off = 1;
					}//else if
					break;
				case 7: //eight button, radio on/off
					if((alm_snd == 0) && (snooze == 0)){
						rad_on ^= 0x01;
						if(rad_on == 0x01){
							turn_rad_on = 1;
						}//if
						else if(rad_on == 0x00){
							turn_rad_off = 1;
						}//else if
						}//if
					break;
				default:	//other buttons, do nothing
					break;
			}//switch
		}//if
	}//for
	
	PORTB &= ~0x70;
	DDRA = 0xFF;
}//check_mode



//***************************************************************************
//									get_temp
//	Get temps from sensor via I2ca nd ATmega48. Gives in Farenheit.
//***************************************************************************
void get_temp(){
	uint16_t lm73_temp;
	uint8_t temperature[1] = {0};
	char temp[2];

	//twi_start_rd(0x90, lm73_rd_buf, 2); //send a temp from sensor
	//_delay_ms(2);

	lm73_temp = lm73_rd_buf[0];	//read temp data
	lm73_temp = lm73_temp << 8;
	lm73_temp |= lm73_rd_buf[1];

	lm73_temp_convert(temperature, lm73_temp, 1); //convert to temp
	temperature[0] = ((temperature[0] * 9) / 5) + 32;	//to Farenheit
	itoa(temperature[0], temp, 10);

	uart_putc('t');	//get 48 to send temp
	lcd_arr[27] = uart_getc();	//outside temp
	lcd_arr[28] = uart_getc();	//for LCD
	
	lcd_arr[19] = temp[0];	//to LCD
	lcd_arr[20] = temp[1];	//to LCD
}//get_temp



//*****************************************************************************
//                           	   display_seg   
//	Diplays digits on 7 seg based on mode and time.
//*****************************************************************************
void display_seg(uint8_t mode){
	uint8_t shft = 0;	//shifts based on mode to know which time to use
	uint8_t bnk = 0;	//blinks in alarms set mode
	
	switch(mode){
		case 0x01:		//clock
			bnk = 1;
			break;
		case 0x02:		//alarm
			shft = 2;
			break;
		default:
			break;
	}//switch

	PORTB = 0x00;		//init PORTB display first digit

	//get display values
	if((tune_freq == 0x00) || (rad_on == 0x00)){
		segsum(time[shft+1],time[shft],bnk);
	}//if
}//display_seg


//*****************************************************************************
//								chk_alarm
//	Checks if alarm is or should be going off. Checks and resets snooze
//	values if over 10 seconds. Says what to send to LCD.
//****************************************************************************
void chk_alarm(){
	if((alm_snd == 0) && (alarm == 0x01)){	//alarm on
		lcd_arr[6] = 'O';
		lcd_arr[7] = 'n';
		lcd_arr[8] = ' ';
		lcd_arr[9] = ' ';
		lcd_arr[10] = ' ';
		lcd_arr[11] = ' ';

		if((time[0]==time[2]) && (time[1]==time[3])){	//if now
			alm_snd = 1;
			turn_rad_off = 1;
			rad_on = 0;
			rad_alm_on = 1;
		}//if
	}//if
	
	else if(alarm == 0x00){	//alarm off
		alm_snd = 0;
		if(rad_alm == 1){
			rad_alm_off = 1;
		}//if

		lcd_arr[6] = 'O';
		lcd_arr[7] = 'f';
		lcd_arr[8] = 'f';
		lcd_arr[9] = ' ';
		lcd_arr[10] = ' ';
		lcd_arr[11] = ' ';
	}//else if
	
	if(snooze == 0x01){		//if snooze
		alm_snd = 0;		//no sound

		if(rad_alm == 1){
			rad_alm_off = 1;
		}//if

		lcd_arr[6] = 'S';
		lcd_arr[7] = 'n';
		lcd_arr[8] = 'o';
		lcd_arr[9] = 'o';
		lcd_arr[10] = 'z';
		lcd_arr[11] = 'e';
	}//if
	
	if(snooze_cnt >= 10){	//reset snooze and snooze counter
		snooze_cnt = 0;
		snooze = 0x00;		
		alm_snd = 1;
		
		if(rad_alm == 1){
			rad_alm_on = 1;
		}//if
	}//if

	if(alm_snd == 1){			//alarm should be beeping
		if(rad_alm == 0){
			turn_rad_off = 1;
			TIMSK |= (1<<OCIE1A);	//TCNT1 int
		}//if
		else if(rad_alm == 1){
			if(rad_alm_on == 1){
				TIMSK &= ~(1<<OCIE1A);	//TCNT1 int off
				turn_rad_on = 1;
				rad_alm_on = 0;
			}//if
		}//else if

		lcd_arr[6] = 'B';
		lcd_arr[7] = 'E';
		lcd_arr[8] = 'E';
		lcd_arr[9] = 'P';
		lcd_arr[10] = 'S';
		lcd_arr[11] = ' ';
	}//if
	else if(alm_snd == 0){	//turns off the alarm tone
		if(rad_alm == 0){
			TIMSK &= ~(1<<OCIE1A);	//TCNT1 int off
		}//if
		else if(rad_alm == 1){
			if((rad_alm_off == 1) && (rad_on == 0)){
				turn_rad_off = 1;
				rad_alm_off = 0;
			}//if
		}//else if
	}//else if

	refresh_lcd(lcd_arr);	//update LCD message
}//chk_alarm



//***************************************************************************
//								set_time
//	PWM for alarm sound if alarm on.
//***************************************************************************
void set_time(uint8_t mode){
	uint8_t shft = 0;		//time/alarm

	switch(mode){
		case 0x00:			//clock
			return;
		case 0x01:			//setting time
			break;
		case 0x02:
			shft = 2;		//setting alarm
			break;
		default:
			break;
	}//switch

	switch(enc_read()){
		case 1:
			time[shft+1]++;
			
			//limits to 24 hour clock
			if(time[shft+1]>23){
				time[shft+1] -= 24;
			}//if
			break;

		case -1:
			//limits to 24 hour clock
			time[shft+1]--;

			if(time[shft+1]<0){
				time[shft+1] += 24;
			}//if
			break;

		case 2:
			time[shft]++;
			
			//limits to 60 minutes
			if(time[shft]>59){
				time[shft] -= 60;
			}//if
			break;

		case -2:
			time[shft]--;
			
			//limits to 60 minutes
			if(time[shft]<0){
				time[shft] += 60;
			}//if
			break;

		default:
			break;
	}//switch
}//set_time



//*************************************************************************
// 								Timer Interrupt 0
//	Keeps time of clock by incrementing seconds, and following values if 
//	necessary.
//*************************************************************************
ISR(TIMER0_OVF_vect){
	second++;

		if(second == 128){				//if 60 add to minute
			get_temp();
			min++;
			segment_data[2] ^= 0x03;	//blink colon

			if(snooze == 0x01){			//add to snooze
				snooze_cnt++;
			}//if
			if(min == 60){				//add 1 minute to held time
				time[0]++;
				min = 0;				//seconds to 0
			}//if
			if(time[0] == 60){			//add 1 hour to held time
				time[1]++;
				if(time[1] == 25){		//limits to 24 hour clock
					time[1] = 0;
				}//if
			
				time[0] = 0;			//minutes to 0
			}//if

			second = 0;
		}//if
}//ISR



//***************************************************************************
//							Timer Interrupt 1
//	PWM for alarm sound if alarm on.
//***************************************************************************
ISR(TIMER1_COMPA_vect){
	PORTD ^= (1<<PD3);
}//ISR



//************************************************************************
//							Timer Interrupt 2
// Updates based on peripherals and status.
//************************************************************************
ISR(TIMER2_OVF_vect){
	static uint8_t i = 0;
	int8_t enc = 0;
	
	check_mode();			//check butt and enc status
	set_time(mode);

	chk_alarm();			//check mode
	check_adc();				//check adc
	display_bg();	//display mode on bargraph
	display_seg(mode);			//display 7 seg things

	enc = enc_read();

	if((bit_is_clear(mode,0)) && (bit_is_clear(mode,1))){
		switch(enc){
			case 1:
				if(OCR3A>100){
					OCR3A = OCR3A - 10;
				}//if
				break;

			case -1:
				if(OCR3A<190){
					OCR3A = OCR3A + 10;
				}//if
				break;

			default:
				break;
		}//switch
	}//if

	if((rad_on == 1) && (tune_freq == 0x01)){
		switch(enc){
			case 2:
				change_freq = 1;
				f++;
				if(f>7){
					f -= 8;
				}//if
				current_fm_freq = radio_presets[f];
				break;

			case -2:
				change_freq = 1;
				f--;
				if(f<0){
					f += 8;
				}//if
				current_fm_freq = radio_presets[f];
				break;

			default:
				break;
		}//switch
		
		if(current_fm_freq<10000){
			segment_data[0] = dec_to_7seg[current_fm_freq%10];
			segment_data[1] = dec_to_7seg[current_fm_freq/10%10];
			segment_data[2] = 0xFF;
			segment_data[3] = dec_to_7seg[current_fm_freq/100%10];
			segment_data[4] = dec_to_7seg[current_fm_freq/1000%10];
			segment_data[3] &= ~(1<<7);
		}//if
		else{
			segment_data[0] = dec_to_7seg[current_fm_freq/10%10];
			segment_data[1] = dec_to_7seg[current_fm_freq/100%10];
			segment_data[2] = 0xFF;
			segment_data[3] = dec_to_7seg[current_fm_freq/1000%10];
			segment_data[4] = dec_to_7seg[current_fm_freq/10000%10];
			segment_data[1] &= ~(1<<7);
		}//else
	}//if

	DDRA = 0xFF;
	PORTA = segment_data[i];
	PORTB = (i<<4);
	
	i++;		//cycles through bit
	if(i>5){
		i = 0;
	}//if

	refresh_lcd(lcd_arr);	//update message
}//ISR



//***************************************************************************
//								Interrupt 7
// Interrupt for le stuff
//***************************************************************************
ISR(INT7_vect){
	STC_interrupt = TRUE;
}//ISR



//*************************************************************************
//							init_all_the_things
//	Init the ports. Init timers. Init ADC. Init SPI. Set up LCD. Start 
//	interrupts. 
//*************************************************************************
void init_all_the_things(){
	DDRC = 0xFF;	//PORTC low for pushbuttons
	PORTC = 0x00;
	DDRB = 0xFF;	//PORTB output, low
	PORTB = 0x00;
	DDRD = (1<<PD2);	//PD2 as output
	DDRD |= 0x03;
	DDRE = 0xFF;	//PORTE output, low
	PORTE |= 0x00;
	DDRF |= 0x08;	//LCD strobe bit

	segment_data[2] = 0xFF;		//init colon off

	time[1] = 1;			//init time to 1:00pm
	
	//timer 0 init
	ASSR |= (1<<AS0);
	TIMSK |= (1<<TOIE0);
	TCCR0 |= (1<<CS00);

	//timer 1 init
	DDRD |= (1<<PD3);
	TCCR1B |= (1<<WGM12) | (1<<CS11) | (1<<CS10);
	OCR1A = 0X00FF;

	//timer 2 init
	DDRB |= (1<<PB7);
	TCCR2 |= (1<<WGM21) | (1<<WGM20) | (1<<COM21) | (1<<COM20) | (1<<CS21) | (1<<CS20);
	TIMSK |= (1<<TOIE2);

	//timer 3 init
	TCCR3A |= (1<<WGM30) | (1<<COM3A1);
	TCCR3B |= (1<<WGM32) | (1<<CS32);
	OCR3A = 130;

	//SPI init
	SPCR = (1<<SPE) | (1<<MSTR) | (1<<SPR0);
	SPSR = (1<<SPI2X);

	//ADC init
	DDRF &= ~(1<<PF0);
	ADCSRA |= (1<<ADPS2) | (1<<ADPS1) | (1<<ADPS0);
	ADMUX |= (1<<REFS0) | (1<<ADLAR);
	ADCSRA |= (1<<ADEN);

	lcd_init();		//set up LCD

	init_twi();		//set up I2C

	uart_init();	//set up uart

	PORTE &= ~(1<<PE7);
	DDRE |= 0x80;
	PORTE |= (1<<PE2);
	_delay_us(200);
	PORTE &= ~(1<<PE2);
	_delay_us(30);
	DDRE &= ~(0x80);
	EICRB |= (1<<ISC71) | (1<<ISC70);
	EIMSK |= (1<<INT7);

	lm73_wr_buf[0] = 0b00000000;	//sensor to read temp
	twi_start_wr(0x90, lm73_wr_buf, 2);

	sei();	//enables interrupts
}//init_all_the_things



//***************************************************************************
//									Main
//***************************************************************************
int main(){
	init_all_the_things();		//init everything

	fm_pwr_up();
	current_fm_freq = radio_presets[f];

	while(1){						//radio checking
		if(turn_rad_on ==1){
			set_property(0x4001,0x0000);
			while(twi_busy()){}
			fm_tune_freq();
			while(twi_busy()){}
			turn_rad_on = 0;
		}//if

		if(turn_rad_off == 1){
			set_property(0x4001,0x0003);
			while(twi_busy()){}
			turn_rad_off = 0;
		}//if

		if(change_freq == 1){
			freq_buf = current_fm_freq;
			fm_tune_freq();
			while(twi_busy()){}
			change_freq = 0;
		}//if

		twi_start_rd(LM73_ADDRESS,lm73_rd_buf,2);
		while(twi_busy()){}
	}//while
}//main
