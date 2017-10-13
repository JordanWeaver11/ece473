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
uint8_t segment_data[5] = {0xff, 0xff, 0xff, 0xff, 0xff};

//decimal to 7-segment LED display encodings, logic "0" turns on segment
uint8_t dec_to_7seg[10] = {0x03, 0x9f, 0x25, 0x0d, 0x99, 0x49, 0x41, 0x1f, 0x01, 0x19};


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
	
}
//******************************************************************************

//***********************************************************************************
//                                   segment_sum                                    
//takes a 16-bit binary input value and places the appropriate equivalent 4 digit 
//BCD segment code in the array segment_data for display.                       
//array is loaded at exit as:  |digit3|digit2|colon|digit1|digit0|
void segsum(uint16_t sum) {
  //determine how many digits there are 
  uint8_t digits = 0;
  while((sum>>digits * 4)) {
	  digits++;
  }
  //break up decimal sum into 4 digit-segments
  
  //blank out leading zero digits 
  //now move data to right place for misplaced colon position
}//segment_sum
//***********************************************************************************


//***********************************************************************************
uint8_t main()
{
//set port bits 4-7 B as outputs
DDRB = (1<<DDB4) | (1<<DDB5) | (1<<DDB6) | (1<<DDB7);
PORTB = 0;
/*
//Initialize count at 0
PORTB = ;  //enable Y0 output
DDRA = 0;
PORTA = 
*/
while(1){
  //insert loop delay for debounce
  //make PORTA an input port with pullups 
  DDRA = 0;
  PORTA = 0xff;
  //enable tristate buffer for pushbutton switches
  PORTB = (1<<PB4) | (1<<PB5) | (1<<PB6);
  //now check each button and increment the count as needed
  //disable tristate buffer for pushbutton switches
  PORTB = (1<<PB5) | (1<<PB6);  //enables unused Y6 output
  //bound the count to 0 - 1023
  //break up the disp_value to 4, BCD digits in the array: call (segsum)
  //bound a counter (0-4) to keep track of digit to display 
	  //make PORTA an output
	  //send 7 segment code to LED segments
	  //send PORTB the digit to display
	  //update digit to display
  }//while
  return 0;
}//main
