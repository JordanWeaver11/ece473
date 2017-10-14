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
#include <avr/io.h>
#include <util/delay.h>

//holds data to be sent to the segments. logic zero turns segment on
uint8_t segment_data[5] = {0xc0, 0xff, 0x07, 0xff, 0xff};

//decimal to 7-segment LED display encodings, logic "0" turns on segment
uint8_t dec_to_7seg[10] = {0xc0, 0xf9, 0xa4, 0xb0, 0x99, 0x92, 0x82, 0xf8, 0x80, 0x98};


//store portb values for each digit
uint8_t portb_digit[5] = {0, (1<<PB4), (1<<PB5), (1<<PB4) | (1<<PB5), (1<<PB6)};
//uint8_t portb_digit[4] = {0, 1, 3, 4};

uint16_t disp_num = 0;

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
uint8_t chk_buttons(uint8_t button) {
	static uint16_t state[8]; //holds present state
	state[button] = (state[button] << 1) | (!bit_is_clear(PINA, button)) | 0xE000; //update state
	if(state[button] == 0xf000) {
		return TRUE;  //return true after 12 clears
	}
	return FALSE;
}
//******************************************************************************

//***********************************************************************************
//                                   segment_sum                                    
//takes a 16-bit binary input value and places the appropriate equivalent 4 digit 
//BCD segment code in the array segment_data for display.                       
//array is loaded at exit as:  digit3|digit2||colon|digit1|digit0|
void segsum(uint16_t bcd) {
    uint8_t digit;
    uint8_t i = 0;
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
} 
//***********************************************************************************

//***********************************************************************************
uint8_t main()
{
//set port bits 4-7 B as outputs
DDRB = (1<<DDB4) | (1<<DDB5) | (1<<DDB6) | (1<<DDB7);
PORTB = 0;

int count = 0;
int i = 0;
while(1){
  //insert loop delay for debounce
  for(i=0;i<2;i++){_delay_ms(1);}
  //make PORTA an input port with pullups 
  DDRA = 0;
  PORTA = 0xff;
  //enable tristate buffer for pushbutton switches
  PORTB = (1<<PB4) | (1<<PB5) | (1<<PB6);
  
  //now check each button and increment the count as needed
  for(i = 0; i < 8; i++) {
	  if(chk_buttons(i)) {
		  disp_num += (1 << i);
	  }
  }
  
  //disable tristate buffer for pushbutton switches
  PORTB = (1<<PB5) | (1<<PB6);  //enables unused Y6 output
  //bound the count to 0 - 1023
  if(disp_num > 1023) {
	  disp_num = 1;
  }

  //make PORTA an output
  DDRA = 0xff;
  
  //avoid the colon
  if(count == 2) {
	count++;
  }
  //break up the number to display into 4 separate bcd digits
  segsum(bin_to_bcd(disp_num));
  //write the digits to the 7-seg display
  PORTA = dec_to_7seg[segment_data[count]];
  //select the correct digit
  PORTB = portb_digit[count];
  //update digit to display
  count++;
  //loop back to first digit when needed
  if(count > 4) {
	  count = 0;
  }

  }//while
}//main
