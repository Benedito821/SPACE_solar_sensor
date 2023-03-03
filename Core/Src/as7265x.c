/**
 * AS7265x triad spectroscopic sensor I2C library.
 * 
 * See https://ams.com/as7265x for sensor datasheet.

 * Most recent code from:
 * https://github.com/jdesbonnet/as7265x 
 */
/**
 * Read all 18 channels and ouput in ascending wavelength order
 * (channels A, B, C, D, E, F, G, H, R, I, S, J, T, U, V, W, K, L)
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include "main.h"
#include "i2c.h"
#include "as7265x.h"

#define AS7265X_I2C_ADDR (0x49)<<1

as7265x_channels_t channels;
as7265x_raw_channels_t raw_channels;

/**
 * Config AS7265x
 */
void as7265x_config(void){

	uint8_t get_ver;
	get_ver = as7265x_vreg_read(0x02);
	as7265x_set_gain ( AS7265X_GAIN_64X);
	as7265x_set_integration_time ( 200);
}


void as7265x_read (void) {

	as7265x_set_measurement_mode( AS7265X_MEASUREMENT_MODE_6CHAN_ONE_SHOT);



	while ( ! as7265x_is_data_available()){}

	//as7265x_get_all_calibrated_values( &channels);
	//as7265x_order_channels(i2c_fd, &channels);
	as7265x_get_all_raw_values( &raw_channels);
//	as7265x_order_raw_channels( &raw_channels);
}


/**
 * Read a I2C (real) register from AS7265x
 */
uint8_t i2cm_read(int addr) {
	uint8_t result;
	HAL_I2C_Mem_Read(&hi2c1, AS7265X_I2C_ADDR, addr, 1, &result,1,1000);
	return result;
} 

/**
 * Write a I2C (real) register to AS7265x.
 */
void i2cm_write(uint8_t addr, uint8_t value) {
	HAL_I2C_Mem_Write(&hi2c1,AS7265X_I2C_ADDR ,addr,1, &value, 1,1000);
}

/**
 * Write to AS7265x virtual register. Based on code in the AS7265x datasheet.
 */
void as7265x_vreg_write(uint8_t virtualReg, uint8_t d)
{
	volatile uint8_t status;
	while (1)
	{
		// Read slave I²C status to see if the write buffer is ready.
		status = i2cm_read(I2C_AS72XX_SLAVE_STATUS_REG);
		if ((status & I2C_AS72XX_SLAVE_TX_VALID) == 0) 
			// No inbound TX pending at slave. Okay to write now.
			break ;
	}
	// Send the virtual register address (enabling bit 7 to indicate a write).
	i2cm_write( I2C_AS72XX_SLAVE_WRITE_REG, (virtualReg | 0x80)) ;
	while (1)
	{
		// Read the slave I²C status to see if the write buffer is ready.
		status = i2cm_read(I2C_AS72XX_SLAVE_STATUS_REG) ;
		if ((status & I2C_AS72XX_SLAVE_TX_VALID) == 0)
		// No inbound TX pending at slave. Okay to write data now.
		break ;
	}
	// Send the data to complete the operation.
	i2cm_write(I2C_AS72XX_SLAVE_WRITE_REG, d) ;
}

/**
 * Read from AS7265x virtual register. Based on code in the AS7265x datasheet. 
 */
uint8_t as7265x_vreg_read( uint8_t virtualReg)
{
	volatile uint8_t status, d;

	status = i2cm_read( I2C_AS72XX_SLAVE_STATUS_REG);
	if ( (status & I2C_AS72XX_SLAVE_RX_VALID) != 0)  {
		// data to be read
		d = i2cm_read( I2C_AS72XX_SLAVE_READ_REG);
	}

	// Wait for WRITE flag to clear
	while (1)
	{
		// Read slave I²C status to see if the read buffer is ready.
		status = i2cm_read(I2C_AS72XX_SLAVE_STATUS_REG) ;
		if ((status & I2C_AS72XX_SLAVE_TX_VALID) == 0)
		// No inbound TX pending at slave. Okay to write now.
		break;
	}


	// Send the virtual register address (disabling bit 7 to indicate a read).
	i2cm_write(I2C_AS72XX_SLAVE_WRITE_REG, virtualReg);


	while (1)
	{
		// Read the slave I²C status to see if our read data is available.
		status = i2cm_read( I2C_AS72XX_SLAVE_STATUS_REG);
		if ((status & I2C_AS72XX_SLAVE_RX_VALID)!= 0)
		// Read data is ready.
		break;
	}

	// Read the data to complete the operation.
	d = i2cm_read( I2C_AS72XX_SLAVE_READ_REG) ;
	return d;
}

/**
 * Test DATA_RDY flag of configuration virtual register (add
 *
 * @return 0 if not set, non-zero if set.
 */
int as7265x_is_data_available (void)
{
	int status = as7265x_vreg_read( AS7265X_CONFIG);
	return (status & (1<<1) );
}

/**
 * Select device
 *
 * @param device 0=master; 1=first slave; 2=second slave
 */
void as7265x_device_select( uint8_t device) {
	as7265x_vreg_write( AS7265X_DEV_SELECT_CONTROL, device);
}


/**
 * Set amplifier gain

 * @param gain 0= 1x (default), 1=3.7x, 2=16x, 3=64x
 */
void as7265x_set_gain ( int gain)
{

	int value = as7265x_vreg_read( AS7265X_CONFIG);
	value &= 0b11001111; // clear gain bits
	value |= (gain&0b11) << 4;
	as7265x_vreg_write( AS7265X_CONFIG,value);
}

/**
 * Set ADC integration time. 
 *
 * @param time from 1..255. 2.8ms units.
 */
void as7265x_set_integration_time( uint8_t time)
{
	as7265x_vreg_write( AS7265X_INTERGRATION_TIME, time);
}

/**
 * Set bulb current.
 *
 * @param device 0, 1, or 2
 * @param current 
 */
void as7265x_set_bulb_current( uint8_t device, uint8_t current)
{
	as7265x_device_select(device);

	current &= 0b11;

	uint8_t value = as7265x_vreg_read(AS7265X_LED_CONFIG);
	value &= 0b11001111; //Clear ICL_DRV bits
	value |= (current << 4); //Set ICL_DRV bits with user's choice
	as7265x_vreg_write(AS7265X_LED_CONFIG, value);
}

/**
 * Bulb enable / disable
 */
void as7265x_bulb_enable ( uint8_t device)
{
	as7265x_device_select( device);

	uint8_t value = as7265x_vreg_read( AS7265X_LED_CONFIG);
	// bit 3: bulb en/disable
	value |= (1 << 3);
	as7265x_vreg_write( AS7265X_LED_CONFIG, value);
}

void as7265x_bulb_disable ( uint8_t device)
{
        as7265x_device_select( device);
	uint8_t value = as7265x_vreg_read( AS7265X_LED_CONFIG);
	// bit 3: bulb en/disable
	value &= ~(1 << 3);
	as7265x_vreg_write( AS7265X_LED_CONFIG, value);
}

void as7265x_indicator_enable (void)
{
	as7265x_device_select(0);
	uint8_t value = as7265x_vreg_read( AS7265X_LED_CONFIG);
	value |= (1<<0);
	as7265x_vreg_write(AS7265X_LED_CONFIG, value);
}

void as7265x_indicator_disable (void)
{
        as7265x_device_select( 0);
        uint8_t value = as7265x_vreg_read( AS7265X_LED_CONFIG);
        value &= ~(1<<0);
        as7265x_vreg_write( AS7265X_LED_CONFIG, value);
}

/**
 * Set measurement mode.
 * 
 * @param mode 2 = all 6 channels continuous; 3 = one shot all channels
 */
void as7265x_set_measurement_mode( uint8_t mode)
{
	uint8_t value = as7265x_vreg_read( AS7265X_CONFIG);
	value &= 0b11110011;
	value |= (mode&0b11) << 2;
	as7265x_vreg_write(AS7265X_CONFIG, value);
}

/**
 * Read calibrated value (IEEE 754 float)
 */
float as7265x_get_calibrated_value ( uint8_t device, uint8_t base_addr)
{
	int i;
	uint8_t value;
	uint32_t shift_reg = 0;

	as7265x_device_select( device);

	for (i = base_addr; i < base_addr+4; i++) {
		shift_reg <<= 8;
		value = as7265x_vreg_read( i);
		shift_reg |= value;
	}
	// convert content of shift_reg to floating point
	float ret;
	memcpy (&ret, &shift_reg, sizeof(float));
	return ret;
}

/**
 * Read raw value (16 bit unsigned integer)
 */
int as7265x_get_raw_value ( uint8_t device, uint8_t base_addr)
{
	as7265x_device_select( device);
        uint32_t value = (as7265x_vreg_read( base_addr)<<8);
	value |= as7265x_vreg_read(base_addr+1);
	return value;
}


/**
 * Read all 18 channels. Channels AS72651 (vis): channels 0-5, AS72652 (vis+IR): channels 6-11,
 * AS72653 (vis+UV): channels 12-17.
 */
void as7265x_get_all_calibrated_values ( as7265x_channels_t *channels)
{

	uint8_t base_addr;
	int channel_index = 0;
	uint8_t device;
	float v;

	for (device = 0; device < 3; device++) {
		for (base_addr = 0x14; base_addr < 0x2c; base_addr += 4) {	
			v = as7265x_get_calibrated_value (device, base_addr);
			channels->channel[channel_index] = v;
			channel_index++;
		}
	}

}
/**
 * Read all 18 channels raw ADC. Channels AS72651 (vis): channels 0-5, AS72652 (vis+IR): channels 6-11,
 * AS72653 (vis+UV): channels 12-17
 */

void as7265x_get_all_raw_values (as7265x_raw_channels_t *channels)
{
	int base_addr;
	int device;
	int channel_index = 0;
	for (device = 0; device < 3; device++) {
		for (base_addr = 0x8; base_addr < 0x14; base_addr += 2) {
			channels->channel[channel_index] = (uint16_t)as7265x_get_raw_value( device, base_addr);
			channel_index++;
		}
	}

}


/**
 * Order channels in ascending wavelength.
 */
void as7265x_order_calibrated_channels(as7265x_channels_t *channels)
{
	float buf[18];
	int i;
	for (i = 0; i < 18; i++) {
		buf[i] = channels->channel[as7265x_channel_order_table[i]];
	}
	for (i = 0; i < 18; i++) {
		channels->channel[i] = buf[i];
	}
}

void as7265x_order_raw_channels( as7265x_raw_channels_t *channels)
{
	uint16_t buf[18];
	int i;
	for (i = 0; i < 18; i++) {
		buf[i] = channels->channel[as7265x_channel_order_table[i]];
	}
	for (i = 0; i < 18; i++) {
		channels->channel[i] = buf[i];
	}
}


void as7265x_measure(void)
{
	int i;
	as7265x_set_measurement_mode(AS7265X_MEASUREMENT_MODE_6CHAN_ONE_SHOT);
	for (i = 0; i < 100; i++) {
		if ( as7265x_is_data_available() )
		{
			break;
		}
	}


	as7265x_get_calibrated_value ( AS7265X_R_G_A_CAL, AS72653_UV);

}

void as7265x_soft_reset (void)
{
	as7265x_vreg_write( AS7265X_CONFIG, (1<<7));
}


as7265x_wavelengths_t as7265x_get_unordered_channel_wavelengths (void)
{
	int i = 0;
	as7265x_wavelengths_t ret;
	for (i = 0; i < 18; i++) {
		ret.channel[i] = as7265x_unordered_channel_wavelength[i];
	}
	// TODO: is this allowed in C?
	return ret;
}

as7265x_wavelengths_t as7265x_get_ordered_channel_wavelengths (void)
{
	int i = 0;
	as7265x_wavelengths_t ret;
	for (i = 0; i < 18; i++) {
		ret.channel[i] = as7265x_unordered_channel_wavelength[i];
	}
	return ret;
}

