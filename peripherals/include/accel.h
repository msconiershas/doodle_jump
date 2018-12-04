#ifndef __WIRELESS_H__
#define __WIRELESS_H__

#include <stdint.h>
#include <stdio.h>
#include "TM4C123GH6PM.h"

#include "gpio_port.h"
#include "spi.h"


//*****************************************************************************
// Transmits 4 bytes of data to the remote device.
//*****************************************************************************
int8_t accel_read_reg(uint8_t reg);

	//*****************************************************************************
// Transmits 4 bytes of data to the remote device.
//*****************************************************************************
void accel_write_reg(uint8_t reg, uint8_t data);
	
void accel_initialize(void);

#endif
