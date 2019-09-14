//	File: lab2_skel.c
//	Author: Makenzie Brian
//	Date: 9.30.17
//	Description: Counts up on 7-segment display using pushbutton board to
//	increment, button 1 increments by 1, button 2 increments by 2, etc.

//  HARDWARE SETUP:
//  PORTA is connected to the segments of the LED display and to the pushbuttons
//  PORTA.0 corresponds to segment A and pushbutton 0, 
//	PORTA.1 corresponds to segment B and pushbutton 1, etc.
//	The pushbuttons and the 7-segment are connected via pins A_2, B_2, etc.
//	on the 7-segmeant board and controlled via tri-state buffer.
//  PORTB bits 4-6 go to A,B,C inputs of the 74HC138 Decoder.
//  PORTB bit 7 goes to the PWM transistor base, pulled low here.

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
//	VCC			->	EN

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

//	ATmega128	to	Pushbuttons
//	GND			->	COM_LVL

#define F_CPU 16000000 // cpu speed in hertz 
#define TRUE 1
#define FALSE 0
#include <avr/io.h>
#include <util/delay.h>

//holds data to be sent to the segments, logic zero turns segment on
uint8_t segment_data[5];

//decimal to 7-segment LED display encodings, logic "0" turns on segment
uint8_t dec_to_7seg[12]; 


//******************************************************************************
//                            chk_buttons                                      
//Checks the state of the button number passed to it. It shifts in ones till   
//the button is pushed. Function returns a 1 only once per debounced button    
//push so a debounce and toggle function can be implemented at the same time.  
//Adapted to check all buttons from Ganssel's "Guide to Debouncing"            
//Expects active low pushbuttons on PINA port. Debounce time is determined by 
//external loop delay times 12. 

uint8_t chk_buttons(uint8_t button) 
{
	static uint16_t state[8] = {0}; 	//holds present state
	state[button] = (state[button] << 1) | (! bit_is_clear(PINA, button)) | 0xE000;
	if (state[button] == 0xF000){
		return 1;
	}//if
	return 0;
}//chk_buttons
//******************************************************************************

//******************************************************************************
//                                   segment_sum                                    
//Takes a 16-bit binary input value and places the appropriate equivalent 4 
//digit BCD segment code in the array segment_data for display.                       
//Array is loaded at exit as:  |digit3|digit2|colon|digit1|digit0|

void segsum(uint16_t sum)
{
  //determine how many digits there are 
  //break up decimal sum into 4 digit-segments
  //blank out leading zero digits 
  //now move data to right place for misplaced colon position


	segment_data[0] = sum%10;
	segment_data[1] = sum/10%10;
	segment_data[2] = 0xFF;			//colon always off
	segment_data[3] = sum/100%10;
	segment_data[4] = sum/1000%10;
	
	if(dec_to_7seg[10]<5 && segment_data[dec_to_7seg[10]]!=0){
		dec_to_7seg[10]++;
	}//if

}//segment_sum
//******************************************************************************


//******************************************************************************
int main()
{
	//initialize variables
	uint8_t i;
	uint16_t count = 0;

	//set PORTC to low to use as additional GND
	DDRC = 0xFF;
	PORTC = 0x00;

	//set port bits 4-7 B as outputs
	DDRB = 0xF8; 
	PORTB = 0x00;

	//binary values for 7 seg into array
	dec_to_7seg[0] = 0b11000000;
	dec_to_7seg[1] = 0b11111001;
	dec_to_7seg[2] = 0b10100100;
	dec_to_7seg[3] = 0b10110000;
	dec_to_7seg[4] = 0b10011001;
	dec_to_7seg[5] = 0b10010010;
	dec_to_7seg[6] = 0b10000010;
	dec_to_7seg[7] = 0b11111000;
	dec_to_7seg[8] = 0b10000000;
	dec_to_7seg[9] = 0b10010000;
	dec_to_7seg[10] = 1;			//blank out leading zeros

	while(1)
	{
		//insert loop delay for debounce
      
		//make PORTA an input port with pullups
		PORTA = 0xFF;
		DDRA = 0x00;
		//enable tristate buffer for pushbutton switche
		PORTB = 0x70;
		
		//now check each button and increment the count as needed
		for(i=0;i<8;i++){
			if(chk_buttons(i)){
				count = count + (1<<i);

				//bound the count to 0 - 1023
				if(count>1023){
					count -= 1023;
					dec_to_7seg[10]=1;
				}//if
			}//if

		}//for

		//break up the disp_value to 4, BCD digits in the array
		segsum(count);
		
		//disable tristate buffer for pushbutton switches
		//make PORTA an output
		DDRA = 0xFF;
		//send PORTB the digit to display
		PORTB = 0x00;

		//send 7 segment code to LED segments
		//bound a counter (0-4) to keep track of digit to display
		for(i=0;i<dec_to_7seg[10];i++)
		{
			if(i==2){
				PORTA = 0xFF;
			}
			else{
				PORTA = dec_to_7seg[segment_data[i]];
			}

			_delay_ms(1);
			
			//update digit to display
			PORTB += 0x10;
		}//for
	}//while
	return 0;
}//main
