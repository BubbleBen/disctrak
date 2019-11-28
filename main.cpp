/*
 * Frisbee Project ATtiny85.cpp
 *
 * Created: 5/11/2019 10:55:51 AM
 * Author : benja
 */ 

#ifndef F_CPU
#define F_CPU 1000000UL
#endif
#include <avr/io.h>
#include "i2c_master.h"
#include "i2c_master.cpp"
#include "Manchester.h"
#include "Manchester.cpp"

#define MPU 0x68 // can be set to 0x69 if AD0 has 5V
#define SIZE_OF_DATA_ARR 40
#define ACCEL_SENS 2048
#define GYRO_SENS 16.4

bool MPU_init();
uint8_t read_single(int adr, int reg, int read_num);
bool write_single(int adr, int reg, uint8_t data);
bool write_config(int adr);
void getAccel(int adr);
void getGyro(int adr);
void insert_accel_array(uint8_t data, uint8_t arr[]);
void insert_gyro_array(uint16_t data, uint16_t arr[]);
uint16_t findAvg(uint16_t arr[]);
uint8_t convert_accel(uint8_t data);
uint16_t convert_gyro(uint16_t data);
void transmit_data();
uint8_t findAccelMax(uint8_t arr[]);
uint16_t findGyroMax(uint16_t arr[]);
void transmit_max();
bool check_end();

int counter = 0;
uint16_t accel_X = 0, accel_Y = 0, accel_Z = 0;
uint16_t gyro_X = 0, gyro_Y = 0, gyro_Z = 0;
uint8_t accel_X_arr[SIZE_OF_DATA_ARR] = {}, accel_Y_arr[SIZE_OF_DATA_ARR] = {}, accel_Z_arr[SIZE_OF_DATA_ARR] = {};
uint16_t gyro_X_arr[SIZE_OF_DATA_ARR] = {}, gyro_Y_arr[SIZE_OF_DATA_ARR] = {}, gyro_Z_arr[SIZE_OF_DATA_ARR] = {};

// =======================================================
/*				SEMINAR IDEAS
	- video of three things:
		1. Throwing frisbee without spin - i.e. don't rotate breadboard much
		2. Throwing frisbee with wrong angle - i.e. move breadboard up and to the right for a backhand
		3. Throwing frisbee well, flat and straight - i.e. spin breadboard a lot while moving it forwards
		
	TODO: need to implement these in the program! */
// ======================================================
	

int main(void)
{
	// Debugging pins and variables
	DDRB |= (1<<PB4);
	DDRB &= ~(1<<PB3);		// push button input
	//bool LED = false;
	
	// Test read and write
	TinyI2CMaster I2C;
	I2C.init();
	MPU_init();
	bool proper_config = write_config(MPU);
	if (!proper_config) {
		while(1) {
			PORTB ^= (1<<PB4);
			_delay_ms(100);
		}
	}

	Manchester man;
	man.workAround1MhzTinyCore();
	man.setupTransmit(MAN_1200);
	
    while (1) 
    {
		
		// while frisbee is a go
		if ((PINB & (1<<PB3)) != 0) {
			_delay_ms(20);								// delay for debouncing
			if ((PINB & (1<<PB3)) != 0) {
				counter = 0;
				while (1) {								// test this while loop - should hold program until the breadboard moves
					PORTB |= (1<<PB4);
					uint8_t data = ~read_single(MPU, 0x3B, 1) + 1;
					if (data > 0xF) {break;}
				}
				PORTB &= ~(1<<PB4);
				//_delay_ms(100);
				
				while (counter < SIZE_OF_DATA_ARR) {
					if (check_end()) {break;}
					getAccel(MPU);						// accel sends 8 bit ints
					getGyro(MPU);						// gyro sends 16 bit ints, TODO: PERHAPS DISABLE FOR TESTING?
					transmit_data();
					_delay_ms(25);
					counter++;
				}
				if (counter != SIZE_OF_DATA_ARR) {
					man.transmit(0b10101010);			// lets the receiver know the full array is not filled
					_delay_ms(20);
				}
				transmit_max();
				
			}
		}
    }
	
	return 0;
}

uint8_t read_single(int adr, int reg, int read_num) {
	TinyI2CMaster I2C;
	int reg_value;
	
	I2C.start(adr, 0);
	I2C.write(reg);
	I2C.restart(adr, read_num);
	reg_value = I2C.read();
	I2C.stop();
	
	//reg_value = ~reg_value + 1;
	return reg_value;
}

bool write_single(int adr, int reg, uint8_t data) {
	TinyI2CMaster I2C;
	bool write_success = false;
	
	I2C.start(adr, 0);
	I2C.write(reg);
	write_success = I2C.write(data);
	I2C.stop();
	
	return write_success;
}

bool write_config(int adr) {
	TinyI2CMaster I2C;
	bool success = true;
	uint8_t gyro_config = read_single(MPU, 0x1B, 1);
	uint8_t accel_config = read_single(MPU, 0x1C, 1);
	gyro_config = gyro_config | 0x18;				// set sensitivity to 2000dps
	accel_config = accel_config | 0x18;				// set sensitivity to 8gs
	
	I2C.start(adr, 0);
	I2C.write(0x1B);
	success = I2C.write(gyro_config);
	if (!success) {
		I2C.stop();
		return success;								// return if gyro was not successfully written to
	}
	
	success = I2C.write(accel_config);
	I2C.stop();
	
	return success;
}

bool MPU_init() {
	uint8_t pwr_mgmt = read_single(MPU, 0x6B, 1);	// read current values in MPU
	pwr_mgmt = pwr_mgmt & 0xBF;					// wake up MPU from sleep mode
	write_single(MPU, 0x6B, pwr_mgmt);
	uint8_t power = read_single(MPU, 0x6B, 1);
	if (power == 0x0) {			
		return true;							// if MPU is woken up from sleep mode, return true
	} else {
		return false;
	}
}

void getAccel(int adr) { 
	TinyI2CMaster I2C;
	
	I2C.start(adr, 0);
	I2C.write(0x3B);
	I2C.restart(adr, 6);
	accel_X = I2C.read() << 8;				// get upper byte
	accel_X |= I2C.read();					// get lower byte
	accel_Y = I2C.read() << 8;
	accel_Y |= I2C.read();
	accel_Z = I2C.read() << 8;				// check type comparison - I2C.read() returns an int, whereas accel is a uint16_t
	accel_Z |= I2C.read();
	I2C.stop();
	
	accel_X = ~accel_X + 1;
	accel_Y = ~accel_Y + 1;
	accel_Z = ~accel_Z + 1;
	uint8_t x = convert_accel(accel_X);			// convert to real life values
	uint8_t y = convert_accel(accel_Y);
	uint8_t z = convert_accel(accel_Z);
	insert_accel_array(x, accel_X_arr);		// store values
	insert_accel_array(y, accel_Y_arr);
	insert_accel_array(z, accel_Z_arr);
}

void getGyro(int adr) {
	TinyI2CMaster I2C;
	
	I2C.start(adr, 0);
	I2C.write(0x43);
	I2C.restart(adr, 6);
	gyro_X = I2C.read() << 8;				// get upper byte
	gyro_X |= I2C.read();					// get lower byte
	gyro_Y = I2C.read() << 8;
	gyro_Y |= I2C.read();
	gyro_Z = I2C.read() << 8;
	gyro_Z |= I2C.read();
	I2C.stop();
	
	gyro_X = ~gyro_X + 1;
	gyro_Y = ~gyro_Y + 1;
	gyro_Z = ~gyro_Z + 1;
	uint16_t x = convert_gyro(gyro_X);			// convert to real life values
	uint16_t y = convert_gyro(gyro_Y);
	uint16_t z = convert_gyro(gyro_Z);
	insert_gyro_array(x, gyro_X_arr);	// store values
	insert_gyro_array(y, gyro_Y_arr);
	insert_gyro_array(z, gyro_Z_arr);
}

void insert_accel_array(uint8_t data, uint8_t arr[]) {
	if (counter < SIZE_OF_DATA_ARR) {
		arr[counter] = data;
		counter++;
	}
}

void insert_gyro_array(uint16_t data, uint16_t arr[]) {
	if (counter < SIZE_OF_DATA_ARR) {
		arr[counter] = data;
		counter++;
	}
}

uint8_t convert_accel(uint8_t data) {	
	return data / ACCEL_SENS;
}

uint16_t convert_gyro(uint16_t data) {
	return data / GYRO_SENS;
}

void transmit_data() {
	man.transmit(accel_X_arr[counter]);
	man.transmit(accel_Y_arr[counter]);
	man.transmit(accel_Z_arr[counter]);
	
	uint8_t sent_byte;									// send upper and lower bits separately
	sent_byte = (gyro_X_arr[counter] & 0xFF00) >> 8;
	man.transmit(sent_byte);
	sent_byte = (gyro_X_arr[counter] & 0xFF);
	man.transmit(sent_byte);
	sent_byte = (gyro_Y_arr[counter] & 0xFF00) >> 8;
	man.transmit(sent_byte);
	sent_byte = (gyro_Y_arr[counter] & 0xFF);
	man.transmit(sent_byte);
	sent_byte = (gyro_Z_arr[counter] & 0xFF00) >> 8;
	man.transmit(sent_byte);
	sent_byte = (gyro_Z_arr[counter] & 0xFF);
	man.transmit(sent_byte);
}

uint8_t findAccelMax(uint8_t arr[]) {
	int max = 0, i = 0;
	while (i < SIZE_OF_DATA_ARR) {
		if (max < arr[i]) {
			max = arr[i];
		}
		i++;
	}
	
	return max;
}

uint16_t findGyroMax(uint16_t arr[]) {
	uint8_t max = 0, i = 0;
	while (i < SIZE_OF_DATA_ARR) {
		if (max < arr[i]) {
			max = arr[i];
		}
		i++;
	}
	
	return max;
}

void transmit_max() {
	uint8_t max;
	max = findAccelMax(accel_X_arr);
	man.transmit(max);
	max = findAccelMax(accel_Y_arr);
	man.transmit(max);
	max = findAccelMax(accel_Z_arr);
	man.transmit(max);
	max = findGyroMax(gyro_X_arr);
	man.transmit(max);
	max = findGyroMax(gyro_Y_arr);
	man.transmit(max);
	max = findGyroMax(gyro_Z_arr);
	man.transmit(max);
}

bool check_end() {
	bool end = false;
	if (read_single(MPU, 0x3B, 1) > 0xF0) {
		end = true;
	} else if (read_single(MPU, 0x3D, 1) > 0xF0) {
		end = true;
	} else if (read_single(MPU, 0x3F, 1) > 0xF0) {
		end = true;
	} else if (read_single(MPU, 0x48, 1) < 0xF) {
		end = true;
	}
	
	return end;
}

/*float findAvg(uint16_t arr[]) {
	float sum = 0;
	int i = 0;
	while (arr[i] != 0 || i < SIZE_OF_DATA_ARR) {
		sum += arr[i];
		i++;
	}
	float avg = sum / i;
	
	return avg;
}*/