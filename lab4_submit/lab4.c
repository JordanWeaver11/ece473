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
#include <avr/interrupt.h>
#include <util/delay.h>
#include <string.h>
#include <stdlib.h>
#include "hd44780.h"
#include <string.h>

//-------------------------------
//LCD STUFF
char    lcd_string_array[32] = {' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ',
								' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' ',};  //holds a string to refresh the LCD
//------------------------------

//holds data to be sent to the segments. logic zero turns segment on
uint8_t segment_data[5] = {0xc0, 0xff, 0xff, 0xff, 0xff};

//decimal to 7-segment LED display encodings, logic "0" turns on segment
uint8_t dec_to_7seg[10] = {0xc0, 0xf9, 0xa4, 0xb0, 0x99, 0x92, 0x82, 0xf8, 0x80, 0x98};


//store portb values for each digit
uint8_t portb_digit_or[5] = {0, (1<<PB4), (1<<PB5), (1<<PB4) | (1<<PB5), (1<<PB6)};
uint8_t portb_digit_and[5] = {~(1<<PB4) & ~(1<<PB5) & ~(1<<PB6), ~(1<<PB5) & ~(1<<PB6), ~(1<<PB4) & ~(1<<PB6), ~(1<<PB6), ~(1<<PB4) & ~(1<<PB5)};

int16_t disp_num = 0;

uint8_t to_ovf_count = 0;
uint8_t sec = 0;
uint8_t update_LED = 0;
uint8_t minutes = 0;
uint8_t hours = 0;
uint8_t alarm_set = 0;
uint8_t alarm_hours = 0;
uint8_t alarm_minutes = 0;
char lcd_str_minutes[16];  //holds string to send to lcd  
char lcd_str_hour[16];  //holds string to send to lcd 

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
	state[button] = (state[button] << 1) | (!bit_is_clear(PINA, button)) | 0xFE00; //update state
	if(state[button] == 0xFF00) {
		return TRUE;  //return true after 8 clears
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
		//extract the rightmost 4 bits
		digit = bcd & 0xF;
		//put the extracted digit into segment_data array
		segment_data[i] = digit;
		//shift the bcd value to move to the next digit
		bcd >>= 4;
	}
} 
//***********************************************************************************

//***********************************************************************
//                            spi_init                               
//**********************************************************************
void spi_init(void){
  DDRB  |= 0x07; //output mode for SS, MOSI, SCLK
  PORTB |= _BV(PB1);  //port B initalization for SPI, SS_n off
  //set ss low
  //PORTB &= ~(1<<PB0);

  SPCR   = (1<<MSTR) | (1<<SPE); //master mode, clk low on idle, leading edge sample

  SPSR   = (1<<SPI2X); //choose double speed operation
  
  /* Run this code before attempting to write to the LCD.*/
  DDRF  |= 0x08;  //port F bit 3 is enable for LCD
  PORTF &= 0xF7;  //port F bit 3 is initially low
 }//spi_init
//**********************************************************************

//***********************************************************************
//                              tcnt0_init                             
//
//
void tcnt0_init(void){
 TIMSK |= (1<<TOIE0);	//enable overflow interrupt
 ASSR |= (1<<AS0);		//use external 32kHz clock
 TCCR0 |= (1<<CS00);	//no prescale
}
//*************************************************************************

//***********************************************************************
//                              tcnt0_init                             
//
//
void tcnt2_init(void){
 //fast PWM mode, non-inverting, clck/8 prescale
 TCCR2 |= (1<<CS21) | (1<<COM21) | (1<<COM20) | (1<<WGM20) | (1<<WGM21);
 //start on brightests
 OCR2 = 1;
}
//*************************************************************************

//**********************************************************************
//							spi_read
//Writes to the SPI port and reads from spi port.
//Taken from lecture slides
//**********************************************************************
uint8_t spi_write_read(uint8_t send_byte) {
	//write data to slave
	SPDR = send_byte;
	//read data from slave
	while (bit_is_clear(SPSR, SPIF)) {}
	//return the read data
	return ~(SPDR);
}//read_spi
//***********************************************************************************


//*************************************************************************
//                           timer/counter0 ISR                          
//When the TCNT0 overflow interrupt occurs, the sec variable is    
//incremented.
//TCNT0 interrupts come at 1s internals.
// 32768 / (2^8 * 128 * 2)
//*************************************************************************
ISR(TIMER0_OVF_vect){
  if(!(TIFR & (1<<TOV0))) { //wait until overflow
	TIFR |= (1<<TOV0);		//clear by writing 1 to TOV0
	to_ovf_count++;			//extend counter
	if((to_ovf_count % 128) == 0) {
		sec++;				//track seconds
		//toggle colon
		segment_data[2] = ~(segment_data[2] ^ 0x02);
	}
  }

}
//*******************************************************************************

//clears the lcd_string_array
void clear_lcd_array(void) {
	int i = 0;
	for(i = 0; i < 32; i++) {
		lcd_string_array[i] = ' ';
	}
}

uint8_t main()
{
//set port B bits 4-7 as outputs
DDRB |= (1<<DDB4) | (1<<DDB5) | (1<<DDB6) | (1<<DDB7);
//drive PWM low
PORTB &= ~(1<<PB7);
//set portC to output SH!LD
DDRC = (1<<DDC0) | (1<<DDC1);
PORTC = 0;

tcnt0_init();  //initalize counter timer zero
tcnt2_init();  //initalize counter timer two
spi_init();    //initalize SPI port
lcd_init();    //initalize LCD (lcd_functions.h)
clear_display();
sei();         //enable interrupts before entering loop

int digit_count = 0;
int i = 0;

//Initalize ADC and its ports DDRF  &= ~(_BV(DDF7)); //make port F bit 7 is ADC input  
PORTF &= ~(_BV(PF7));  //port F bit 7 pullups must be off
                       
ADMUX = (1<<REFS0) | (1<<MUX2) | (1<<MUX1) | (1<<MUX0); //single-ended, input PORTF bit 7, right adjusted, 10 bits
ADCSRA = (1<<ADEN) | (1<<ADPS0) | (1<<ADPS1) | (1<<ADPS2); //ADC enabled, don't start et, single shot mode
															//division factor is 128 (125khz)
while(1){
	
//ADC--------------------------------------------------------------------------------
	ADCSRA |= (1<<ADSC);    // poke ADSC and start conversion
	while (bit_is_clear(ADCSRA,ADIF)){}; // spin while interrupt flag not set

	ADCSRA |= (1<<ADIF); //its done, clear flag by writing a one

	OCR2 = (ADC & 0xff);                      //read the ADC output as 16 bits
//END ADC---------------------------------------------------------------------------

//Buttons---------------------------------------------------------------------------
	//make PORTA an input port with pullups 
	DDRA = 0;
	PORTA = 0xff;
	//enable tristate buffer for pushbutton switches
	PORTB |= (1<<PB4) | (1<<PB5) | (1<<PB6);
	//now check each button and increment the count as needed
	//uint8_t button_press = 0;
	for(i = 0; i < 8; i++) {
	  if(chk_buttons(i)) {
		  //turn the mode on at button if its off
		  //turn mode off at button if its on
		switch(i) {
			case 0:
				minutes = 0;
				hours = 0;
				alarm_minutes = 0;
				alarm_hours = 0;
				alarm_set = 0;
				break;
			case 1:
				minutes++;
				break;
			case 2:
				hours++;
				break;
			case 3:
				//toggle alarm_set flag to turn alarm on/off
				alarm_set ^= 0x01;
				break;
			case 4:
				alarm_minutes++;
				break;
			case 5:
				alarm_hours++;
				break;
			case 6:
				break;
			case 7:
				break;
		}
	  }

	}
	//disable tristate buffer for pushbutton switches
	PORTB |= (1<<PB5) | (1<<PB6);  //enables unused Y6 output
	PORTB &= ~(1<<PB4);
//END BUTTONS-----------------------------------------------------------------------------

//LED DISPLAY-----------------------------------------------------------------------------
  //adjust time for overflow
  if(sec > 59) {
	  minutes++;
	  sec = 0;
  }
  if(minutes > 59) {
	  hours++;
	  minutes = 0;
  }
  if(hours > 24) {
	  hours = 0;
  }
  
  //update the time to display on the LED
  disp_num = (hours * 100) + minutes;
  //break up the number to display into 4 separate bcd digits
  segsum(bin_to_bcd(disp_num));
  //make PORTA an output
  DDRA = 0xff;
  //write the digits to the 7-seg display
  PORTA = dec_to_7seg[segment_data[digit_count]];
  //select the correct digit
  PORTB |= portb_digit_or[digit_count];
  PORTB &= portb_digit_and[digit_count];
  //update digit to display
  digit_count++;
  //remove leading zero from clock
  if((digit_count == 4) & (hours < 10)) {
	  digit_count++;
  }
  //loop back to first digit when needed
  if(digit_count > 4) {
	  digit_count = 0;
  }
//END LED DISPLAY-----------------------------------------------------------------------------

//LCD DISPLAY---------------------------------------------------------------------------------
  //check for minutes or hour overflow
  if(alarm_minutes > 59) {
	  alarm_hours++;
	  alarm_minutes = 0;
  }
  if(alarm_hours > 24) {
	  alarm_hours = 0;
  }
  //clean out the lcd_string_array
  clear_lcd_array();
  //convert minutes from int to string
  itoa(alarm_minutes, lcd_str_minutes, 10);
  //convert hours from int to string
  itoa(alarm_hours, lcd_str_hour, 10);
  //copy string minutes and hours strings into master string
  strcpy(lcd_string_array, lcd_str_hour);
  strcat(lcd_string_array, ":");
  //add a leading 0 if minutes < 10
  if(alarm_minutes < 10) {
	  strcat(lcd_string_array, "0");
  }
  strcat(lcd_string_array, lcd_str_minutes);
  //indicate whether the alarm is set
  if(alarm_set) {
	  strcat(lcd_string_array, " ALARM");
  }
  
  //delete null terminator from string functions
  lcd_string_array[strlen(lcd_string_array)] = ' ';
  //write the master string to the lcd
  refresh_lcd(lcd_string_array);
//END LCD DISPLAY---------------------------------------------------------------------------------  

  _delay_ms(1);
  }//while
}//main
