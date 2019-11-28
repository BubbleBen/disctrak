/*
This code is based on the Atmel Corporation Manchester
Coding Basics Application Note.
http://www.atmel.com/dyn/resources/prod_documents/doc9164.pdf
Quotes from the application note:
"Manchester coding states that there will always be a transition of the message signal
at the mid-point of the data bit frame.
What occurs at the bit edges depends on the state of the previous bit frame and
does not always produce a transition. A logical '1' is defined as a mid-point transition
from low to high and a '0' is a mid-point transition from high to low.
We use Timing Based Manchester Decode.
In this approach we will capture the time between each transition coming from the demodulation
circuit."
Timer 2 is used with a ATMega328. Timer 1 is used for a ATtiny85.
This code gives a basic data rate as 1200 bauds. In manchester encoding we send 1 0 for a data bit 0.
We send 0 1 for a data bit 1. This ensures an average over time of a fixed DC level in the TX/RX.
This is required by the ASK RF link system to ensure its correct operation.
The data rate is then 600 bits/s.
*/

#include <avr/interrupt.h>
#include "Manchester_r.h"

//static int8_t RxPin = 255;

static int16_t rx_sample = 0;
static int16_t rx_last_sample = 0;
static uint8_t rx_count = 0;
static uint8_t rx_sync_count = 0;
static uint8_t rx_mode = RX_MODE_IDLE;

static uint16_t rx_manBits = 0; //the received manchester 32 bits
static uint8_t rx_numMB = 0; //the number of received manchester bits
static uint8_t rx_curByte = 0;

static uint8_t rx_maxBytes = 2;
static uint8_t rx_default_data[2];
static uint8_t* rx_data = rx_default_data;

Manchester::Manchester() //constructor
{
  applyWorkAround1Mhz = 0;
}

void Manchester::workAround1MhzTinyCore(uint8_t a)
{
  applyWorkAround1Mhz = a;
}


void Manchester::setupReceive(uint8_t SF)
{
  //setRxPin(pin);
  DDRC |= ~(1<<PC1);
  ::MANRX_SetupReceive(SF);
}

//decode 8 bit payload and 4 bit ID from the message, return true if checksum is correct, otherwise false
uint8_t Manchester::decodeMessage(uint16_t m, uint8_t &id, uint8_t &data)
{
  //extract components
  data = (m & 0xFF);
  id = (m >> 12);
  uint8_t ch = (m >> 8) & 0b1111; //checksum received
  //calculate checksum
  uint8_t ech = (id ^ data ^ (data >> 4) ^ 0b0011) & 0b1111; //checksum expected
  return ch == ech;
}

//encode 8 bit payload, 4 bit ID and 4 bit checksum into 16 bit
uint16_t Manchester::encodeMessage(uint8_t id, uint8_t data)
{
  uint8_t chsum = (id ^ data ^ (data >> 4) ^ 0b0011) & 0b1111;
  uint16_t m = ((id) << 12) | (chsum << 8) | (data);
  return m;
}

void Manchester::beginReceiveArray(uint8_t maxBytes, uint8_t *data)
{
  ::MANRX_BeginReceiveBytes(maxBytes, data);
}

void Manchester::beginReceive(void)
{
  ::MANRX_BeginReceive();
}


uint8_t Manchester::receiveComplete(void)
{
  return ::MANRX_ReceiveComplete();
}


uint8_t Manchester::getMessage(void)
{
  return ::MANRX_GetMessage();
}


void Manchester::stopReceive(void)
{
  ::MANRX_StopReceive();
}

//global functions

#if defined( ESP8266 )
   volatile uint16_t ESPtimer = 0;
   void timer0_ISR (void);
#endif

void MANRX_SetupReceive(uint8_t speedFactor)
{
 
    /*
    Timer 2 is used with a ATMega328.
    http://www.atmel.com/dyn/resources/prod_documents/doc8161.pdf page 162
    How to find the correct value: (OCRxA +1) = F_CPU / prescaler / 1953.125
    OCR2A is only 8 bit register
    */

    TCCR2A = _BV(WGM21); // reset counter on match
    #if F_CPU == 1000000UL
      TCCR2B = _BV(CS21); // 1/8 prescaler
      OCR2A = (64 >> speedFactor) - 1;
    #elif F_CPU == 8000000UL
      TCCR2B = _BV(CS21) | _BV(CS20); // 1/32 prescaler
      OCR2A = (128 >> speedFactor) - 1; 
    #elif F_CPU == 16000000UL
      TCCR2B = _BV(CS22); // 1/64 prescaler
      OCR2A = (128 >> speedFactor) - 1; 
    #else
    #error "Manchester library only supports 8mhz, 16mhz on ATMega328"
    #endif
    TIMSK2 = _BV(OCIE2A); // Turn on interrupt
    TCNT2 = 0; // Set counter to 0

} //end of setupReceive

void MANRX_BeginReceive(void)
{
  rx_maxBytes = 2;
  rx_data = rx_default_data;
  rx_mode = RX_MODE_PRE;
}

void MANRX_BeginReceiveBytes(uint8_t maxBytes, uint8_t *data)
{
  rx_maxBytes = maxBytes;
  rx_data = data;
  rx_mode = RX_MODE_PRE;
}

void MANRX_StopReceive(void)
{
  rx_mode = RX_MODE_IDLE;
}

uint8_t MANRX_ReceiveComplete(void)
{
  return (rx_mode == RX_MODE_MSG);
}

uint8_t MANRX_GetMessage(void)
{
  return (((int16_t)rx_data[0]) << 8) | (int16_t)rx_data[1];
}

void AddManBit(uint16_t *manBits, uint8_t *numMB,
               uint8_t *curByte, uint8_t *data,
               uint8_t bit)
{
  *manBits <<= 1;
  *manBits |= bit;
  (*numMB)++;
  if (*numMB == 16)
  {
    uint8_t newData = 0;
    for (int8_t i = 0; i < 8; i++)
    {
      // ManBits holds 16 bits of manchester data
      // 1 = LO,HI
      // 0 = HI,LO
      // We can decode each bit by looking at the bottom bit of each pair.
      newData <<= 1;
      newData |= (*manBits & 1); // store the one
      *manBits = *manBits >> 2; //get next data bit
    }
    data[*curByte] = newData ^ DECOUPLING_MASK;
    (*curByte)++;

    // added by caoxp @ https://github.com/caoxp
    // compatible with unfixed-length data, with the data length defined by the first byte.
	// at a maximum of 255 total data length.
    if( (*curByte) == 1)
    {
      rx_maxBytes = data[0];
    }
    
    *numMB = 0;
  }
}


Manchester man;