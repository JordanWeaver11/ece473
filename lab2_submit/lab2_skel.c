// lab2_skel.c 
// R. Traylor
// 9.12.08

//  HARDWARE SETUP:
//  PORTA is connected to the segments of the LED display. and to the pushbuttons.
//  PORTA.0 corresponds to segment a, PORTA.1 corresponds to segement b, etc.
//  PORTB bits 4-6 go to a,b,c inputs of the 74HC138.
//  PORTB bit 7 goes to the PWM transistor base.

#define F_CPU 16000000 // cpu speed in hertz 
#define TRUE 1
#define FALSE 0
#define MAX_CHECKS 7
#include <avr/io.h>
#include <util/delay.h>

//debouncing values
uint8_t Debounced_State = 0;	//debounced state of the switches
uint8_t State[MAX_CHECKS];		//array that maintains bounce status
uint8_t Index = 0;				//pointer into State

//holds data to be sent to the segments. logic zero turns segment on
uint8_t segment_data[5] = {0xc0, 0xff, 0x07, 0xff, 0xff};

//decimal to 7-segment LED display encodings, logic "0" turns on segment
//uint8_t dec_to_7seg[10] = {0x03, 0x9f, 0x25, 0x0d, 0x99, 0x49, 0x41, 0x1f, 0x01, 0x19};
uint8_t dec_to_7seg[10] = {0xc0, 0xf9, 0xa4, 0xb0, 0x99, 0x92, 0x82, 0xf8, 0x80, 0x98};


//store portb values for each digit
uint8_t portb_digit[5] = {0, (1<<PB4), (1<<PB5), (1<<PB4) | (1<<PB5), (1<<PB6)};
//uint8_t portb_digit[4] = {0, 1, 3, 4};

uint8_t disp_num = 0;

//*****************************************************************************
//							bin_to_bcd
//Converts binary number to bdc by modding by 10 and shifting by 4 bits per digit.

uint16_t bin_to_bcd(uint16_t i) {
    uint16_t binaryShift = 0;
    uint16_t digit;
    uint16_t bcd = 0;
    while (i > 0) {
        //mod by 10 each time to move from 1s to 10s, so on
        digit = i % 10;
        //extract each decimal digit encoded with 4 bits
        bcd += (digit << binaryShift);
        //shift another 4 bits to get the next digit
        binaryShift += 4;
        i /= 10;
    }
    return bcd;
}


/*
uint16_t bin_to_bcd(uint16_t i) {
	uint16_t bcd = 0;
	uint16_t shift = 0;
	while(i > 0) {
		bcd |= (i % 10) << (shift++ << 2);
		i /= 10;
	}
	return bcd;
}
*/


/*
uint16_t bin_to_bcd(uint8_t i) {
	uint16_t bcd = 0;
	uint16_t shift = 0;
	
}
*/


//*******************************************************************************

//******************************************************************************
//								debounce
//Debounces pushbuttons so that they are only detected once per button push.
//Taken from lecture slides
/*
uint8_t debounce_switch() {
	static uint16_t state = 0; //holds present state
	state = (state << 1) | (! bit_is_clear(PINA, 5)) | 0xE000;
	if(state = 0xf000) {
		return TRUE;
	}
	return FALSE;
}
*/
//*******************************************************************************



//******************************************************************************
//                            chk_buttons                                      
//Checks the state of the button number passed to it. It shifts in ones till   
//the button is pushed. Function returns a 1 only once per debounced button    
//push so a debounce and toggle function can be implemented at the same time.  
//Adapted to check all buttons from Ganssel's "Guide to Debouncing"            
//Expects active low pushbuttons on PINA port.  Debounce time is determined by 
//external loop delay times 12. 
//
/*
uint8_t chk_buttons(uint8_t button) {
	
}
*/
void chk_buttons(uint8_t button) {
	uint8_t i, j;
	State[Index] = ~(PINA) & (1<<button);
	++Index;
	j = 0xff;
	for(i = 0; i < MAX_CHECKS - 1; i++) {
		j = j & State[i];
		Debounced_State = Debounced_State ^ j;
		if(Index >= MAX_CHECKS) {
			Index = 0;
		}
	}
}
//******************************************************************************

//***********************************************************************************
//                                   segment_sum                                    
//takes a 16-bit binary input value and places the appropriate equivalent 4 digit 
//BCD segment code in the array segment_data for display.                       
//array is loaded at exit as:  digit3|digit2||colon|digit1|digit0|
/*
void segsum(uint16_t sum) {
  //determine how many digits there are 
  uint8_t digits = 4;
  //start far left, decrement digits every time I find a leading zero
  while( (sum>>((digits-1) * 4)) == 0 ) {
	digits--;  //CAUTION: FIXME bug if sum == 0
  }
  
  //break up decimal sum into 4 digit-segments
  uint8_t i = 0;
  while (i < (digits)){
	  if(i < 2) {
		  //extract segments by shifting sum left, then right
		  segment_data[i] = (sum<<(digits-1-i))>>12;
	  }
	  else {
		//extract segments by shifting sum left, then right
		segment_data[i] = (sum<<(digits-1-i))>>12;
	  }
	  i++;
  }
  
  //blank out leading zero digits 
  i = 3;
  while(i > (digits-1)) {
	  //0xff creates a blank
	  segment_data[i] = 0xff;
	  i--;
  }
  for(i = 0; i < 5; i++) {
	  segment_data[i] = bin_to_bcd(7) & 0xff;
  }
}//segment_sum
*/


void segsum(uint16_t bcd) {
    uint8_t digit;
    uint8_t i = 0;
    //bcd = 0b0001001000110100;
	for(i = 0; i < 5; i++) {
		//ignore colon on display
		if(i == 2) {
			i++;
		}
		//bcd >>= 4;
		//extract the rightmost 4 bits
		digit = bcd & 0xF;
		//put the extracted digit into segment_data array
		segment_data[i] = digit;
		//shift the bcd value to move to the next digit
		bcd >>= 4;
	}
	/*
	bcd = 0b0001001000110100;
	segment_data[0] = bcd & 0xf;
	segment_data[1] = (bcd>>4) & 0xf;
	segment_data[2] = 7;
	segment_data[3] = (bcd>>8) & 0xf;
	segment_data[4] = (bcd>>12) & 0xf;
	*/
} 

//***********************************************************************************

//***********************************************************************************
uint8_t main()
{
//set port bits 4-7 B as outputs
DDRB = (1<<DDB4) | (1<<DDB5) | (1<<DDB6) | (1<<DDB7);
PORTB = 0;

int count = 0;
while(1){
	int i = 0;
  //insert loop delay for debounce
  for(i=0;i<3;i++){_delay_ms(1);} //0.01 second wait
  //make PORTA an input port with pullups 
  DDRA = 0;
  PORTA = 0xff;
  //enable tristate buffer for pushbutton switches
  PORTB = (1<<PB4) | (1<<PB5) | (1<<PB6);
  
  //now check each button and increment the count as needed
  for(i = 0; i < 8; i++) {
	  chk_buttons(i);
  }
  if(Debounced_State & (1<<PA0)) {
	  disp_num += 1;
  }
  if(Debounced_State & (1<<PA1)) {
	  disp_num += 2;
  }
  if(Debounced_State & (1<<PA2)) {
	  disp_num += 4;
  }
  if(Debounced_State & (1<<PA3)) {
	  disp_num += 8;
  }
  if(Debounced_State & (1<<PA4)) {
	  disp_num += 16;
  }
  if(Debounced_State & (1<<PA5)) {
	  disp_num += 32;
  }
  if(Debounced_State & (1<<PA6)) {
	  disp_num += 64;
  }
  if(Debounced_State & (1<<PA7)) {
	  disp_num += 128;
  }
  
  /*
  if(PINA & PA0) {
	  _delay_ms(100);
	  if(PINA & PA0) disp_num += 1;
  }
  if(PINA & PA1) {
	  _delay_ms(100);
	  if(PINA & PA1) disp_num += 2;
  }
  if(PINA & PA2) {
	  _delay_ms(100);
	  if(PINA & PA2) disp_num += 4;
  }
  if(PINA & PA3) {
	  _delay_ms(100);
	  if(PINA & PA3) disp_num += 8;
  }
  if(PINA & PA4) {
	  _delay_ms(100);
	  if(PINA & PA4) disp_num += 16;
  }
  if(PINA & PA5) {
	  _delay_ms(100);
	  if(PINA & PA5) disp_num += 32;
  }
  if(PINA & PA6) {
	  _delay_ms(100);
	  if(PINA & PA6) disp_num += 64;
  }
  if(PINA & PA7) {
	  _delay_ms(100);
	  if(PINA & PA7) disp_num += 128;
  }
  */
  
  
  
//  disp_num += chk_buttons(1);
  
//  disp_num = 3;                                        //DEBUG
  //disable tristate buffer for pushbutton switches
  PORTB = (1<<PB5) | (1<<PB6);  //enables unused Y6 output
  //bound the count to 0 - 1023
  if(disp_num > 1023) {
	  disp_num = 1;
  }
  /*
  //break up the disp_value to 4, BCD digits in the array: call (segsum)
  if( disp_num != 0 ) {
	segsum(bin_to_bcd(3));
  }
  */
  //make PORTA an output
  DDRA = 0xff;
  
  if(count == 2) {
	count++;
  }
  
  
  
  segsum(bin_to_bcd(disp_num));
  PORTA = dec_to_7seg[segment_data[count]];
  PORTB = portb_digit[count];
  
  
  /*
  PORTA = dec_to_7seg[7];
  PORTB = portb_digit[count];
  */
  
  /*
  PORTA = bin_to_bcd(3) & 0xff;
  PORTB = portb_digit[count];
  */
  count++;
  if(count > 4) {
	  count = 0;
  }

/*
  //bound a counter (0-4) to keep track of digit to display 
  for(i = 0; i < 4; i++) {
	  //send 7 segment code to LED segments
		//PORTA = dec_to_7seg[segment_data[i]];
	  //send PORTB the digit to display
	  
	  PORTA = dec_to_7seg[1];
	  PORTB = portb_digit[count];
  
	  count++;
	  if(count > 4) {
		  count = 0;
	  }
	  if(count == 2) {
		  count++;
	  }
	  //update digit to display
  }
  */


  }//while
}//main
