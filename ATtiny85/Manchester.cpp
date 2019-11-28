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

#include "Manchester.h"

/*static int8_t RxPin = 255;

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
static uint8_t* rx_data = rx_default_data;*/

void delay_us(int delay);

Manchester::Manchester() //constructor
{
  applyWorkAround1Mhz = 0;
}


/*void Manchester::setTxPin(uint8_t pin)
{
  TxPin = pin; // user sets the digital pin as output
  pinMode(TxPin, OUTPUT); 
}*/

void Manchester::workAround1MhzTinyCore(uint8_t a)
{
  applyWorkAround1Mhz = a;
}

void Manchester::setupTransmit(uint8_t SF)
{
	//setTxPin(pin);
	DDRB |= (1<<PB1);
	speedFactor = SF; // MAN_1200 which equals 2
	//we don't use exact calculation of passed time spent outside of transmitter
	//because of high ovehead associated with it, instead we use this 
	//emprirically determined values to compensate for the time loss
  
    uint16_t compensationFactor = 88; //must be divisible by 8 for workaround

	delay1 = (HALF_BIT_INTERVAL >> speedFactor) - compensationFactor;
	delay2 = (HALF_BIT_INTERVAL >> speedFactor) - 2;
  
    delay2 -= 22; //22+2 = 24 is divisible by 8
    if (applyWorkAround1Mhz) { //definition of micro delay is broken for 1MHz speed in tiny cores as of now (May 2013)
      //this is a workaround that will allow us to transmit on 1Mhz
      //divide the wait time by 8
      delay1 >>= 3;
      delay2 >>= 3;
    }
}


/*void Manchester::setup(uint8_t Tpin, uint8_t Rpin, uint8_t SF)
{
  setupTransmit(Tpin, SF);
  //setupReceive(Rpin, SF);
}*/


void Manchester::transmit(uint8_t data)
{
  uint8_t byteData[2] = {2, data};
  transmitArray(2, byteData);
}

/*
The 433.92 Mhz receivers have AGC, if no signal is present the gain will be set
to its highest level.
In this condition it will switch high to low at random intervals due to input noise.
A CRO connected to the data line looks like 433.92 is full of transmissions.
Any ASK transmission method must first sent a capture signal of 101010........
When the receiver has adjusted its AGC to the required level for the transmisssion
the actual data transmission can occur.
We send 14 0's 1010... It takes 1 to 3 10's for the receiver to adjust to
the transmit level.
The receiver waits until we have at least 10 10's and then a start pulse 01.
The receiver is then operating correctly and we have locked onto the transmission.
*/
void Manchester::transmitArray(uint8_t numBytes, uint8_t *data)
{

#if SYNC_BIT_VALUE
  for( int8_t i = 0; i < SYNC_PULSE_DEF; i++) //send capture pulses
  {
    sendOne(); //end of capture pulses
  }
  sendZero(); //start data pulse
#else
  for( int8_t i = 0; i < SYNC_PULSE_DEF; i++) //send capture pulses
  {
    sendZero(); //end of capture pulses
  }
  sendOne(); //start data pulse
#endif
 
  // Send the user data
  for (uint8_t i = 0; i < numBytes; i++)
  {
    uint16_t mask = 0x01; //mask to send bits
    uint8_t d = data[i];  // ^ DECOUPLING_MASK;
    for (uint8_t j = 0; j < 8; j++)
    {
      if ((d & mask) == 0){
        sendZero();
      } else {
        sendOne();
	  }
      mask <<= 1; //get next bit
    }//end of byte
  }//end of data

  // Send 3 terminatings 0's to correctly terminate the previous bit and to turn the transmitter off
#if SYNC_BIT_VALUE
  sendOne();
  sendOne();
  sendOne();
#else
  sendZero();
  sendZero();
  sendZero();
#endif
}//end of send the data

// TODO: fix the arduino stuff
void Manchester::sendZero(void)
{
  delay_us(delay1);
  PORTB |= (1<<PB1);

  delay_us(delay2);
  PORTB &= ~(1<<PB1);
}//end of send a zero

// TODO: fix the arduino stuff
void Manchester::sendOne(void)
{
  delay_us(delay1);
  PORTB &= ~(1<<PB1);

  delay_us(delay2);
  PORTB |= (1<<PB1);
}//end of send one

//TODO use repairing codes perhabs?
//http://en.wikipedia.org/wiki/Hamming_code

/*
    format of the message including checksum and ID
    
    [0][1][2][3][4][5][6][7][8][9][a][b][c][d][e][f]
    [    ID    ][ checksum ][         data         ]      
                  checksum = ID xor data[7:4] xor data[3:0] xor 0b0011
                  
*/

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

Manchester man;

void delay_us(int delay) {
	int i;
	for (i = 0; i < delay; i++) {
		_delay_us(1);
	}
}
