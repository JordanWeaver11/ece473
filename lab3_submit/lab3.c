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
#define LEFT_ENC 0
#define RIGHT_ENC 1

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>

//holds data to be sent to the segments. logic zero turns segment on
uint8_t segment_data[5] = {0xc0, 0xff, 0x07, 0xff, 0xff};

//decimal to 7-segment LED display encodings, logic "0" turns on segment
uint8_t dec_to_7seg[10] = {0xc0, 0xf9, 0xa4, 0xb0, 0x99, 0x92, 0x82, 0xf8, 0x80, 0x98};


//store portb values for each digit
uint8_t portb_digit_or[5] = {0, (1<<PB4), (1<<PB5), (1<<PB4) | (1<<PB5), (1<<PB6)};
uint8_t portb_digit_and[5] = {~(1<<PB4) & ~(1<<PB5) & ~(1<<PB6), ~(1<<PB5) & ~(1<<PB6), ~(1<<PB4) & ~(1<<PB6), ~(1<<PB6), ~(1<<PB4) & ~(1<<PB5)};

uint16_t disp_num = 0;
uint8_t write_ready = 0;

uint8_t history[2] = {0, 0};
uint8_t inc_dec_flag = 0;
uint8_t the_mode = 0;

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
	if(state[button] == 0xF000) {
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
  //set ss low
  //PORTB &= ~(1<<PB0);

  SPCR   = (1<<MSTR) | (1<<SPE); //master mode, clk low on idle, leading edge sample

  SPSR   = (1<<SPI2X); //choose double speed operation
  //SPCR |= (1<<SPR1) | (1<<SPR0);  //clock slowest
  //SPCR |= (1<<CPOL) | (1<<CPHA);  // clock idle high
 }//spi_init
//**********************************************************************

//***********************************************************************
//                              tcnt0_init                             
//
//
void tcnt0_init(void){
  TIMSK |= (1<<TOIE0);             //enable interrupts
  TCCR0 |= (1<<CS02) | (1<<CS00);  //normal mode, prescale by 128
}
//*************************************************************************

//**********************************************************************
//							spi_read
//Reads the SPI port.
//Taken from lecture slides
//**********************************************************************
uint8_t spi_write_read(uint8_t send_byte) {
	SPDR = send_byte;
	while (bit_is_clear(SPSR, SPIF)) {}
	return ~(SPDR);
}//read_spi
//***********************************************************************************


//*************************************************************************
//                           timer/counter0 ISR                          
//When the TCNT0 overflow interrupt occurs, the count_ms variable is    
//incremented.
//TCNT0 interrupts come at 8ms internals.
// 1/16000000				= 62.5nS
// 1/(16000000/128)			= 8us
// (1/(16000000/128)*256	= 2ms
//*************************************************************************
ISR(TIMER0_OVF_vect){
  write_ready = 1;
}
//*******************************************************************************

uint8_t main()
{
//set port B bits 4-7 as outputs
DDRB = (1<<DDB4) | (1<<DDB5) | (1<<DDB6) | (1<<DDB7);
//set portC to output SH!LD
DDRC = (1<<DDC0) | (1<<DDC1);
PORTC = 0;

tcnt0_init();  //initalize counter timer zero
spi_init();    //initalize SPI port
sei();         //enable interrupts before entering loop

int count = 0;
int i = 0;
while(1){
  //insert loop delay for display
  _delay_ms(1);
/*
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
*/

  //disable tristate buffer for pushbutton switches
  PORTB |= (1<<PB5) | (1<<PB6);  //enables unused Y6 output
  PORTB &= ~(1<<PB4);
  //bound the count to 0 - 1023
  if(disp_num > 1023) {
	  disp_num = 1;
  }
  
  //avoid the colon
  if(count == 2) {
	count++;
  }
  
  if(write_ready) {
	  PORTC |= 0x01;
	  
	  uint8_t spi_in = spi_write_read(the_mode);
	  for(i = 1; i >= 0; i--) {
		  if((spi_in == 0x01) & (history[i] == 0x03)) {
			  inc_dec_flag = 1;
			  history[i] = 0;
		  }
		  else if((spi_in == 0x02) & (history[i] == 0x03)) {
			  inc_dec_flag = -1;
			  history[i] = 0;
		  }
		  else if(spi_in == 0x03) {
			  history[i] = 3;
		  }
		  spi_in >>= 2;
	  }
	  
	  PORTC &= ~(1<<PC0);
	  PORTC |=  (1<<PC1);                   //send rising edge to regclk on HC595 
	  PORTC &= ~(1<<PC1);                   //send falling edge to regclk on HC595
	  
	  //make PORTA an input port with pullups 
	  DDRA = 0;
	  PORTA = 0xff;
	  //enable tristate buffer for pushbutton switches
	  PORTB = (1<<PB4) | (1<<PB5) | (1<<PB6);
	  //now check each button and increment the count as needed
	  //uint8_t button_press = 0;
	  for(i = 0; i < 3; i++) {
		  if(chk_buttons(i)) {
			  the_mode = (1<<i);
		  }
	  }
	  //disable tristate buffer for pushbutton switches
	  PORTB |= (1<<PB5) | (1<<PB6);  //enables unused Y6 output
	  PORTB &= ~(1<<PB4);
	  
	  switch(the_mode) {
		  case 0x00:
			disp_num += inc_dec_flag;
		  case 0x01:  //reset to +1 mode if you push button 0
			disp_num += inc_dec_flag;
			the_mode = 0x00;
			break;
		  case (1<<1):
			disp_num += inc_dec_flag * 2;
			break;
		  case (1<<2):
			disp_num += inc_dec_flag * 4;
			break;
	  }
	  inc_dec_flag = 0;
	  
	  write_ready = 0;
  }
  
  //break up the number to display into 4 separate bcd digits
  segsum(bin_to_bcd(disp_num));
  //make PORTA an output
  DDRA = 0xff;
  //write the digits to the 7-seg display
  PORTA = dec_to_7seg[segment_data[count]];
  //select the correct digit
  PORTB |= portb_digit_or[count];
  PORTB &= portb_digit_and[count];
  //update digit to display
  count++;
  //loop back to first digit when needed
  if(count > 4) {
	  count = 0;
  }

  }//while
}//main
