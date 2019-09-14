#include <util/twi.h>
#include "lm73_functions.h"
#include <util/delay.h>

uint8_t lm73_wr_buf[2];
uint8_t lm73_rd_buf[2];

//********************************************************************************

//******************************************************************************
void lm73_temp_convert(uint8_t *temp_digits, uint16_t lm73_temp, uint8_t f_not_c){

	if ((lm73_temp & 0x8000) == 0x0000) {

		if(lm73_temp & 1 << 14) {
				temp_digits[0] += 128;
		}

		if(lm73_temp & 1 << 13) {
				temp_digits[0] += 64;
		}
	
		if(lm73_temp & 1 << 12) {
				temp_digits[0] += 32;
		}

		if(lm73_temp & 1 << 11) {
				temp_digits[0] += 16;
		}

		if(lm73_temp & 1 << 10) {
				temp_digits[0] += 8;
		}

		if(lm73_temp & 1 << 9) {
				temp_digits[0] += 4;
		}

		if(lm73_temp & 1 << 8) {
				temp_digits[0] += 2;
		}

		if(lm73_temp & 1 << 7) {
				temp_digits[0] += 1;
		}


		if(lm73_temp & 1 << 6) {
				temp_digits[1] = 50;
		}

		if(lm73_temp & 1 << 5) {
				temp_digits[1] = 25;
		}

		if(lm73_temp & 1 << 4) {
				temp_digits[1] = 13;
		}

		if(lm73_temp & 1 << 3) {
				temp_digits[1] = 6;
		}

		if(lm73_temp & 1 << 2) {
				temp_digits[1] = 3;
		}

		
	}


}
//******************************************************************************
