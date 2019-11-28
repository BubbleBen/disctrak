//NOTE THIS CODE DOES NOT YET WORK, AS OF NOW THIS IS JUST USED TO DEBUG THE RECEIVER 
//AND UTILISES THE MCHR3K LIBRARY.
#ifndef F_CPU
#define F_CPU 1000000UL
#endif
#include <avr/io.h>
#include <stdio.h>
#include "Manchester_r.h"
#include "Manchester_r.cpp"
#include "LCD.c"

#define DATA PORTB
#define DIR_DATA DDRB
#define CRTL PORTD
#define DIR_CRTL DDRD
#define EN 5
#define RW 7
#define RS 2

int main(void)
{
	DDRB |= (1<<PB2);
	DDRB |= (1<<PB0); // debugging LED
	//PORTB |= (1<<PB0);
	DIR_CRTL |= 1<<EN | 1<<RW | 1<<RS;
	_delay_ms(15);

	send_command(0x01); //Clear Screen 0x01 = 00000001
	_delay_ms(2);
	send_command(0x38);
	_delay_us(50);
	send_command(0b00001100);
	_delay_us(50);
	send_str("DiscTrak");
	_delay_us(50);
	send_command(0b11000000);
	_delay_us(50);
	send_str("Statistics");
	_delay_us(50);
	send_command(0b00010100);
	_delay_us(50);
	send_str("Feedback");
	
	man.setupReceive(MAN_1200);
	man.beginReceive();
	uint16_t msg;
	while(1) {
		
		if(man.receiveComplete()) {
			msg = man.getMessage(); // do smthing if recieved
			man.beginReceive(); //start listening for next message right after you retrieve the message
			//do something with your message here
		
			
		}
		
		  if (rx_mode < RX_MODE_MSG) {//receiving something
			  //DDRB |= (1<<PB0);
			  // Increment counter
			  rx_count += 8;
			  // Check for value change
			  // sample twice, only the same means a change.
			  static uint8_t rx_sample_0=0;
			  static uint8_t rx_sample_1=0;
			  rx_sample_1 = (PINC && (1<<PC1));
			  
			  if ( rx_sample_1 == rx_sample_0 ) {
				  //PORTB |= (1<<PB0);
				  rx_sample = rx_sample_1;
			  }
			  rx_sample_0 = rx_sample_1;
				
				if (rx_sample == 1) {
					PORTB |= (1<<PB2);
					//PORTB &= (1<<PB0);

				} else {
					//PORTB &= (1<<PB0);
					PORTB &= ~(1<<PB2);
				}

			  //check sample transition
			  uint8_t transition = (rx_sample != rx_last_sample);
			  
			  if (rx_mode == RX_MODE_PRE) {
				  // Wait for first transition to HIGH
				  //PORTB |= (1<<PB0);
				  if (transition && (rx_sample == 1)) {
					  rx_count = 0;
					  rx_sync_count = 0;
					  rx_mode = RX_MODE_SYNC;
					  //PORTB |= (1<<PB0);
				  }
			  }
			  else if (rx_mode == RX_MODE_SYNC) {
				  // Initial sync block
				  if (transition) {
					  if( ( (rx_sync_count < (SYNC_PULSE_MIN * 2) )  || (rx_last_sample == 1)  ) &&
					  ( (rx_count < MinCount) || (rx_count > MaxCount))) {
						  // First 20 bits and all 1 bits are expected to be regular
						  // Transition was too slow/fast
						  //PORTB |= (1<<PB0);
						  rx_mode = RX_MODE_PRE;
					  } else if((rx_last_sample == 0) && ((rx_count < MinCount) || (rx_count > MaxLongCount))) {
						  // 0 bits after the 20th bit are allowed to be a double bit
						  // Transition was too slow/fast
						  rx_mode = RX_MODE_PRE;
						  //PORTB |= (1<<PB2);
					  } else {
						  PORTB |= (1<<PB0);
						  rx_sync_count++;
						  if((rx_last_sample == 0) &&
						  (rx_sync_count >= (SYNC_PULSE_MIN * 2) ) &&
						  (rx_count >= MinLongCount)) {
							  // We have seen at least 10 regular transitions
							  // Lock sequence ends with unencoded bits 01
							  // This is encoded and TX as HI,LO,LO,HI
							  // We have seen a long low - we are now locked!
							  rx_mode    = RX_MODE_DATA;
							  rx_manBits = 0;
							  rx_numMB   = 0;
							  rx_curByte = 0;
						  } else if (rx_sync_count >= (SYNC_PULSE_MAX * 2) ) {
							  rx_mode = RX_MODE_PRE;
						  }
						  rx_count = 0;
					  }
				  }
			  } else if (rx_mode == RX_MODE_DATA) {
				  // Receive data
				  if (transition) {
					  if((rx_count < MinCount) ||
					  (rx_count > MaxLongCount)) {
						  // wrong signal lenght, discard the message
						  rx_mode = RX_MODE_PRE;
					  } else {
						  if(rx_count >= MinLongCount) {// was the previous bit a double bit?
							  AddManBit(&rx_manBits, &rx_numMB, &rx_curByte, rx_data, rx_last_sample);
						  } if ((rx_sample == 1) && (rx_curByte >= rx_maxBytes)) {
							  rx_mode = RX_MODE_MSG;
						  } else {
							  // Add the current bit
							  AddManBit(&rx_manBits, &rx_numMB, &rx_curByte, rx_data, rx_sample);
							  rx_count = 0;
						  }
					  }
				  }
			  }
			  
			  // Get ready for next loop
			  rx_last_sample = rx_sample;
		  }
		}
}

