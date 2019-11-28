#include <avr/io.h>

#define DATA PORTB
#define DIR_DATA DDRB
#define CRTL PORTD
#define DIR_CRTL DDRD
#define EN 5
#define RW 7
#define RS 2

void busy_check(void);
void enable_on(void);
void send_command(unsigned char command);
void send_char(unsigned char character);
void send_str(char *string);

/*DIR_CRTL |= 1<<EN | 1<<RW | 1<<RS;
_delay_ms(15);

send_command(0x01); //Clear Screen 0x01 = 00000001
_delay_ms(2);
send_command(0x38);
_delay_us(50);
send_command(0b00001110);
_delay_us(50);

send_char(0x4E); //N
send_char(0x65); //e
send_char(0x77); //w
send_char(0x62); //b
send_char(0x69); //i
send_char(0x65); //e
send_char(0x48); //H
send_char(0x61); //a
send_char(0x63); //c
send_char(0x6B); //k
send_char(0x2E); //.
send_char(0x63); //c
send_char(0x6F); //o
send_char(0x6D); //m
send_char(0x6D);

send_str("Patrick");*/

void busy_check()
{
	DIR_DATA = 0;
	CRTL |= 1<<RW;
	CRTL &= ~1<<RS;

	while (DATA >= 0x80)
	{
		enable_on();
	}
	//whyy
	DIR_DATA = 0xFF; //0xFF means 0b11111111
}

void enable_on()
{
	CRTL |= 1<<EN;
	asm volatile ("nop");
	asm volatile ("nop");
	CRTL &= ~1<<EN;
}

void send_command(unsigned char command)
{
	busy_check();
	DATA = command;
	CRTL &= ~ ((1<<RW)|(1<<RS));
	enable_on();
	DATA = 0;
}

void send_char(unsigned char character)
{
	busy_check();
	DATA = character;
	CRTL &= ~ (1<<RW);
	CRTL |= 1<<RS;
	enable_on();
	DATA = 0;
}

void send_str(char *StringOfCharacters)
{
	while(*StringOfCharacters > 0)
	{
		send_char(*StringOfCharacters++);
	}
}