/*
 * power.h
 *
 * Functions to handle the power subsystem and battery charger of the board
 */

#ifndef POWER_H_
#define POWER_H_

#include <stdint.h>
#include <stdbool.h>

extern bool i2c_available;

bool power_init (void);

void power_set_regulator_mode  (uint8_t mode); // mode: bit 0 activates DCDC0, bit 1 activates regular DCDC.
void power_set_digital_voltage (uint8_t mode); // mode: BCD coded desired voltage: 0x18 -> 0x33 in steps of 0x03.

enum system_off_modes {
	system_shutdown,  // enters ship mode: all power rails deactivated.
	system_standby,   // VSYS powered, but MCU in system off.
};

void power_off (enum system_off_modes mode); // does not return!

#endif /* POWER_H_ */
