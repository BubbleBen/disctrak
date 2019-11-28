/* Tiny I2C
   David Johnson-Davies - www.technoblogy.com - 14th April 2018
   
   CC BY 4.0
   Licensed under a Creative Commons Attribution 4.0 International license: 
   http://creativecommons.org/licenses/by/4.0/
*/

#include "i2c_master.h"

TinyI2CMaster::TinyI2CMaster() 
{
}

// Minimal Tiny I2C Routines **********************************************

uint8_t TinyI2CMaster::transfer (uint8_t data) {
  USISR = data;                               // Set USISR according to data.
                                              // Prepare clocking.
  data = 0<<USISIE | 0<<USIOIE |              // Interrupts disabled
         1<<USIWM1 | 0<<USIWM0 |              // Set USI in Two-wire mode.
         1<<USICS1 | 0<<USICS0 | 1<<USICLK |  // Software clock strobe as source.
         1<<USITC;                            // Toggle Clock Port.
  do {
    DELAY_T2TWI;
    USICR = data;                             // Generate positive SCL edge.
    while (!(PORTB & 1<<PB2));   // Wait for SCL to go high.
    DELAY_T4TWI;
    USICR = data;                             // Generate negative SCL edge.
  } while (!(USISR & 1<<USIOIF));             // Check for transfer complete.

  DELAY_T2TWI;
  data = USIDR;                               // Read out data.
  USIDR = 0xFF;                               // Release SDA.
  DDRB |= (1<<PB0);                // Enable SDA as output.

  return data;                                // Return the data from the USIDR
}

void TinyI2CMaster::init () {
  PORTB |= 1<<PB0;                 // Enable pullup on SDA.
  PORTB |= 1<<PB2;              // Enable pullup on SCL.

  DDRB |= 1<<PB2;               // Enable SCL as output.
  DDRB |= 1<<PB0;                  // Enable SDA as output.

  USIDR = 0xFF;                               // Preload data register with "released level" data.
  USICR = 0<<USISIE | 0<<USIOIE |             // Disable Interrupts.
          1<<USIWM1 | 0<<USIWM0 |             // Set USI in Two-wire mode.
          1<<USICS1 | 0<<USICS0 | 1<<USICLK | // Software strobe as counter clock source
          0<<USITC;
  USISR = 1<<USISIF | 1<<USIOIF | 1<<USIPF | 1<<USIDC | // Clear flags,
          0x0<<USICNT0;                       // and reset counter.
}

uint8_t TinyI2CMaster::read (void) {
  if ((I2Ccount != 0) && (I2Ccount != -1)) I2Ccount--;
  
  /* Read a byte */
  DDRB &= ~(1<<PB0);               // Enable SDA as input.
  uint8_t data = TinyI2CMaster::transfer(USISR_8bit);

  /* Prepare to generate ACK (or NACK in case of End Of Transmission) */
  if (I2Ccount == 0) USIDR = 0xFF; else USIDR = 0x00;
  TinyI2CMaster::transfer(USISR_1bit);                 // Generate ACK/NACK.

  return data;                                // Read successfully completed
}

uint8_t TinyI2CMaster::readLast (void) {
  I2Ccount = 0;
  return TinyI2CMaster::read();
}

bool TinyI2CMaster::write (uint8_t data) {
  /* Write a byte */
  PORTB &= ~(1<<PB2);           // Pull SCL LOW.
  USIDR = data;                               // Setup data.
  TinyI2CMaster::transfer(USISR_8bit);                 // Send 8 bits on bus.

  /* Clock and verify (N)ACK from slave */
  DDRB &= ~(1<<PB0);               // Enable SDA as input.
  if (TinyI2CMaster::transfer(USISR_1bit) & 1<<TWI_NACK_BIT) return false;

  return true;                                // Write successfully completed
}

// Start transmission by sending address
bool TinyI2CMaster::start (uint8_t address, int readcount) {
  if (readcount != 0) { I2Ccount = readcount; readcount = 1; }
  uint8_t addressRW = address<<1 | readcount;

  /* Release SCL to ensure that (repeated) Start can be performed */
  PORTB |= 1<<PB2;              // Release SCL.
  while (!(PORTB & 1<<PB2));     // Verify that SCL becomes high.
#ifdef TWI_FAST_MODE
  DELAY_T4TWI;
#else
  DELAY_T2TWI;
#endif

  // Generate Start Condition */
  PORTB &= ~(1<<PB0);              // Force SDA LOW.
  while ((PORTB & 1<<PB0));			// Verify that SDA becomes low.
  DELAY_T4TWI;
  PORTB &= ~(1<<PB2);           // Pull SCL LOW.
  PORTB |= 1<<PB0;                 // Release SDA.

  if (!(USISR & 1<<USISIF)) return false;
  /*if (!(USISR & 1<<USISIF)) {
	  return false;
  } else {
	  return true;
  }*/

  // Write address 
  PORTB &= ~(1<<PB2);           // Pull SCL LOW.
  USIDR = addressRW;                          // Setup data.
  TinyI2CMaster::transfer(USISR_8bit);                 // Send 8 bits on bus.

  // Clock and verify (N)ACK from slave 
  DDRB &= ~(1<<PB0);               // Enable SDA as input.
  if (TinyI2CMaster::transfer(USISR_1bit) & 1<<TWI_NACK_BIT) return false; // No ACK */

  return true;                                // Start successfully completed
}

bool TinyI2CMaster::restart(uint8_t address, int readcount) { // TODO: screws up somewhere in here when trying to read
  return TinyI2CMaster::start(address, readcount);
}

void TinyI2CMaster::stop (void) {
  PORTB &= ~(1<<PB0);              // Pull SDA low.
  PORTB |= 1<<PB2;              // Release SCL.
  while (!(PORTB & 1<<PB2));     // Wait for SCL to go high.
  DELAY_T4TWI;
  PORTB |= 1<<PB0;                 // Release SDA.
  DELAY_T2TWI;
}

TinyI2CMaster TinyI2C = TinyI2CMaster();      // Instantiate a TinyI2C object