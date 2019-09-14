//	File: lab3_code.c
//	Author: Makenzie Brian
//	Date: 10.16.17
//	Description: Counts up on 7-segment display using encoders to
//	increment, buttons change mult, button 1 is increment mult by 1, 
//	button 2 is increment mult by 2, etc. mult is shown on the Bargraph 
//	Board.

//  HARDWARE SETUP:
//  PORTA is connected to the segments of the LED display and to the pushbuttons
//  PORTA.0 corresponds to segment A and pushbutton 0, 
//	PORTA.1 corresponds to segment B and pushbutton 1, etc.
//	The pushbuttons and the 7-segment are connected via pins A_2, B_2, etc.
//	on the 7-segmeant board and controlled via tri-state buffer.
//  PORTB bits 4-6 go to A,B,C inputs of the 74HC138 Decoder.
//	PORTB bits 2-3 are connected to the Bargraph Board using SPI.
//	Encoders are connected to PORTB, PORTC, and PORTE, see beloww for 
//	details.

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
//	GND			->	PWM
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
//	PORTB.7		->	oe_n
//	GND			->	GND
//	VCC			->	VDD
//	NC			->	sd_out

//	ATmega128	to	Encoders
//	PORTB.3		->	SOUT
//	GND			->	SIN
//	PORTC.2		->	CKINH
//	PORTB.2		->	SCK
//	PORTE.6		->	SH/LD
//	GND			->	GND
//	VCC			->	VDD

//	ATmega128	to	Pushbuttons
//	GND			->	COM_LVL
//	GND			->	GND
//	VCC			->	VDD

#define F_CPU 16000000 // cpu speed in hertz 
#define TRUE 1
#define FALSE 0

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

//holds data to be sent to the segments. logic zero turns segment on
uint8_t segment_data[5];

//decimal to 7-segment LED display encodings, logic "0" turns on segment
//binary values for 7 seg into array
uint8_t dec_to_7seg[12] = {0xC0, 0xF9, 0xA4, 0xB0, 0x99, 0x92, 0x82, 0xF8, 0x80, 0x90, 0xFF, 0xFF}; 

//stores number for counting
volatile int16_t count = 1;

//store which mode for multiplication 		
volatile uint8_t mult = 0;	

//stores the current data for encoder 1
volatile uint8_t cur1a, cur1b;

//stores the current data for encoder 2
volatile uint8_t cur2a, cur2b;

//stores the previous data for encoder 1
volatile uint8_t prev_cur1a = 0, prev_cur1b = 0;

//stores the previous data for encoder 2
volatile uint8_t prev_cur2a = 0, prev_cur2b = 0;

//****************************************************************************
//                            chk_buttons                                      
//Checks the state of the button number passed to it. It shifts in ones till   
//the button is pushed. Function returns a 1 only once per debounced button    
//push so a debounce and toggle function can be implemented at the same time.  
//Adapted to check all buttons from Ganssel's "Guide to Debouncing"            
//Expects active low pushbuttons on PINA port.  Debounce time is determined by 
//external loop delay times 12. 
//****************************************************************************
uint8_t chk_buttons(uint8_t button){
	static uint16_t state[8] = {0}; 	//holds present state

	//updates state of button
	state[button] = (state[button] <<1)|(!bit_is_clear(PINA,button))|0xE000;
 
	//buttons pressed if held for 12 cycles, not bounce
	if (state[button] == 0xF000){
		return 1;
	}//if
	return 0;
}//chk_buttons
//******************************************************************************


//******************************************************************************
//                     		encoders
//Check encoders for change of state and determines direction based on
//which bits are set from the return values.
//******************************************************************************
void encoders(){

	uint8_t encVal = 0;

	//sends mode to bargraph
	if (mult == 0){
		SPDR = 0x01;
	}//if
	else{
	SPDR = mult<<1;
	}//else
		
	//send out data
	while(!(SPSR & (1<<SPIF))){} 	//wait till data is sent out

	//load next val
	PORTD |= (1 << PD2);
	PORTD &= ~(1 << PD2);

	//reads in the encVal
	PORTE &= ~(1<<PE6);				//enable load data
	PORTC |= (1 << PC2);			//inhibit clocking of new data
	SPDR = 0xFF;					//send junk data to start SPI clock
	PORTC &= ~(1 << PC2);			//clock new data
	PORTE |= (1<<PE6);				//load data via SPI 
	while(!(SPSR & (1<<SPIF))){}	//wait for data sending
	
	encVal = SPDR; 					//get data from encoders
		
	
	//extracts out the encoder values for comparison
	cur1a = encVal & 0x01;			//encoder 1 is on bits 0 and 1
	cur1b = encVal & 0x02;
	cur2a = (encVal & 0x04) >> 2;	//shift in LSB positions
	cur2b = (encVal & 0x08) >> 2;	//encoder 2 is on bits 2 and 3
	
	if(mult !=3){				//does nothing when both modes are on
		//encoder1 is same
		if(prev_cur1a == cur1a){
			//if the encoder moves CW
			if(cur1a == 1 && (prev_cur1b < cur1b)){
				count += (1<<mult);	//increment by multiplier
				//bound 0-1023
				if(count > 1023){
					count -= 1024;
				}//if
			}//if
			//encoder1 CCW
			if(cur1a == 1 && (prev_cur1b > cur1b)){
				count -= (1 << mult);//decrement by multiplier
				//bound 0-1023
				if(count < 0){
					count += 1024;
				}//if
			}//if
		}//if
		
		//encoder2 is same
		if(prev_cur2a == cur2a){
			//encoder1 CW
			if(cur2a == 1 && (prev_cur2b < cur2b)){
				count += (1 << mult);//increment by multiplier
				//boun 0-1023
				if(count > 1023){
					count -= 1023;
				}//if			
			}//if
			
			//encoder2 CCW
			if(cur2a == 1 && (prev_cur2b > cur2b)){
				count -= (1 << mult);//decrement by multipler
				//bound 0-1023
				if(count < 0){
					count += 1024;
				}//if			
			}//if
		}//if
	}//if
	//previous encoder values for next change	
	prev_cur1a = cur1a;
	prev_cur1b = cur1b;
	prev_cur2a = cur2a;
	prev_cur2b = cur2b;
}//encoders

//************************************************************************
//                              segment_sum                            
//Takes a 16-bit binary input value and places the appropriate 
//equivalent 4 digit BCD segment code in the array segment_data 
//for display. 
//                
//Array is loaded at exit as:  |digit3|digit2|colon|digit1|digit0|
//************************************************************************
void segsum(uint16_t sum)
{
	//holds the count value
	uint16_t holdsum = sum;

	//sets the number of digits displayed to zero
	dec_to_7seg[10] = 0;
	
	//count is 0 blank out all other digits to display
	uint8_t i = 0;
	if(sum == 0){

		//display 0 in digit 1
		segment_data[0] = 0;
		dec_to_7seg[10] = 1;

		//blank out other digits
		for(i = 1; i < 5; i++){
			segment_data[i] = 11;
		}//for
	}//if

	//count is not 0
	else{
		//get values for digits	
		while(holdsum != 0){
			segment_data[dec_to_7seg[10]] = holdsum % 10;//1's val
			holdsum /= 10;				//get 10's val, etc.
			dec_to_7seg[10]++;
		}//while
	}//else
	
	//if 4 or less digits displayed
	if(dec_to_7seg[10] < 4){
		//blank out leading zeros	
		for(i = 4; i >= dec_to_7seg[10]; i--){
			segment_data[i] = 11;
		}//for
	}//if

	//adjusts data for colon position
	if(dec_to_7seg[10]>3){				  //if 4 digits
		segment_data[4] = segment_data[3];//shifts 4th for colon
	}//if

	if(dec_to_7seg[10]>=2){				  //if 3 or more digits
		segment_data[3] = segment_data[2];//shifts 3rd for colon
	}//if

	segment_data[2] = 11;				  //colon is blank

}//segsum


//**********************************************************************
void buttons(){
	
	uint8_t i = 0;

	//only check the first two buttons because only 2 modes
	for(i=0;i<2;i++){
		if(chk_buttons(i)){
			mult ^= (1 << i);	//change the mode based on button
		}//if
	}//for
	
}//buttons

//***********************************************************************

//***********************************************************************
//                     ISR for timer counter zero
//***********************************************************************
ISR(TIMER0_OVF_vect){

	//PORTA input; pullups
	PORTA = 0xFF;
	DDRA = 0x00;

	PORTB |= (1<<PB4)|(1<<PB5)|(1<<PB6);	//enable pushbuttons; tristate
	
	buttons();	//loops through buttons to check mode change

	PORTB &= (0<<PB4)|(0<<PB5)|(0<<PB6);	//disable buttons; tristate
	
	DDRA = 0xFF;	//PORTA outputs; enable 7Seg
	
	encoders();		//checks encoders and changes count
}//ISR

//*************************************************************************
//								init
//Initializes ports, SPi, and TC0. See comments below for detail.
//************************************************************************
void init(){
	//PORTB output for 7Seg Selects
	DDRB = 0xFF;
	PORTB = 0x00;
	
	//PORTC output for GND and pushbuttons
	//COM_LVL to GND pushbutton board, CHKIN to GND for encoders
	DDRC = 0xFF;	
	PORTC = 0x00;

	DDRD = (1<<PD2);	//enable output to bargraph

	//Output for decoder enable
	DDRE = 0xFF;
	PORTE = 0x00;

	//Timer 0 init
	TIMSK |= (1<<TOIE0);	//enable overflow interrupts flag
	TCCR0 |= (1<<CS02)|(1<<CS00);	//normal mode, prescale by 128

	//SPI communication init
	//master mode, clk low on idle, leading edge sample
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);
	SPSR = (1<<SPI2X);				//choose double speed operation

}//init


//***************************************************************************
int main(){
	dec_to_7seg[10] = 1;		//init to display the one digit

	dec_to_7seg[11] = 0xFF;		//use to initially turn off segs

	init();		//initialize timer/counter, spi, and ports
	
	sei();		//enable interrupts

	//loop forever
	while(1){
		
		PORTB = 0x00;	//encoders start on digit 1 for selects
		
		segsum(count);	//break up disp_value to 4, BCD digits in array

		DDRA = 0xFF;	//makes PORTA outputs

		cli();			//turns off interrupts

		//loops through each digit
		uint8_t i;
		for(i = 0; i < 5; i++){
			//outputs number to 7seg from segment data
			PORTA = dec_to_7seg[segment_data[i]];
			
			PORTB = (i << 4);	//update digit to display
			
			_delay_ms(.7);		//delay so visable
		}//for
		
		sei();	//re-enables interrupts after count displayed

	}//while
	return 0;
}//main
