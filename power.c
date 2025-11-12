/*
 * power.c
 *
 * Functions to handle the power subsystem and battery charger of the board
 * (C) 2019-2021 see CONTRIBUTORS.
 */

#include <stdbool.h>
#include <stdint.h>
#include "boards.h"


#include <nrfx_saadc.h>
#include <nrfx_power.h>
#include <nrf_temp.h>
#include <nrf_delay.h>

#include "app_error.h"
#include "app_timer.h"
#include "app_scheduler.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"

#include "power.h"


static void power_flash_regout_voltage (uint32_t value);
static void adc_event (nrfx_saadc_evt_t const *p_event);
static void power_timer_handler (void *p_context);

//APP_TIMER_DEF(m_power_timer_id);

nrf_saadc_value_t adc_buf[8];

bool i2c_available;


bool power_init (void)
{
	uint8_t vdd_reg = NRF_UICR->REGOUT0 & 7; // real voltage is (vdd_reg * 0.3 + 1.8)
	if (vdd_reg != 4) power_flash_regout_voltage(4);

	// Initialize GPIOs:
	nrf_gpio_pin_write(LED_PIN, 0);
	nrf_gpio_pin_write(CHRG_STATUS_DRIVE, 0);
	nrf_gpio_cfg(LED_PIN,           NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL,   NRF_GPIO_PIN_S0H1, NRF_GPIO_PIN_NOSENSE);
	nrf_gpio_cfg(CHRG_STATUS_DRIVE, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL,   NRF_GPIO_PIN_D0S1, NRF_GPIO_PIN_NOSENSE);
	nrf_gpio_cfg(CHRG_STATUS_SENSE, NRF_GPIO_PIN_DIR_INPUT,  NRF_GPIO_PIN_INPUT_CONNECT,    NRF_GPIO_PIN_NOPULL,   NRF_GPIO_PIN_H0D1, NRF_GPIO_PIN_NOSENSE);

	// Configure LSM6DSO connections:
	nrf_gpio_pin_write(GYRO_SDA, 1);
	nrf_gpio_pin_write(GYRO_SCL, 1);
	nrf_gpio_cfg(GYRO_SDA,          NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_CONNECT,    NRF_GPIO_PIN_PULLUP,   NRF_GPIO_PIN_S0D1, NRF_GPIO_PIN_NOSENSE);
	nrf_gpio_cfg(GYRO_SCL,          NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_CONNECT,    NRF_GPIO_PIN_PULLUP,   NRF_GPIO_PIN_S0D1, NRF_GPIO_PIN_NOSENSE);
	nrf_gpio_cfg(GYRO_INT1,         NRF_GPIO_PIN_DIR_INPUT,  NRF_GPIO_PIN_INPUT_CONNECT,    NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_S0D1, NRF_GPIO_PIN_SENSE_HIGH);
	nrf_gpio_cfg(GYRO_INT2,         NRF_GPIO_PIN_DIR_INPUT,  NRF_GPIO_PIN_INPUT_CONNECT,    NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_S0D1, NRF_GPIO_PIN_SENSE_HIGH);

	// Configure ADS1293 connections:
	// Since it starts unpowered, keep all digital signals low.
	nrf_gpio_pin_write(EMG_ENABLE, 0);
	nrf_gpio_pin_write(EMG_POWER, 1);
	nrf_gpio_pin_write(EMG_RESET, 0);
	nrf_gpio_pin_write(EMG_CLOCK, 0);
	nrf_gpio_pin_write(EMG_ALARM, 0);
	nrf_gpio_pin_write(EMG_READY, 0);
	nrf_gpio_pin_write(EMG_SCLK, 0);
	nrf_gpio_pin_write(EMG_SS, 0);
	nrf_gpio_cfg(EMG_ENABLE,        NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL,   NRF_GPIO_PIN_S0H1, NRF_GPIO_PIN_NOSENSE);
	nrf_gpio_cfg(EMG_POWER,         NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL,   NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE);
	nrf_gpio_cfg(EMG_RESET,         NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL,   NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE);
	nrf_gpio_cfg(EMG_CLOCK,         NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL,   NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE);
	nrf_gpio_cfg(EMG_ALARM,         NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL,   NRF_GPIO_PIN_S0D1, NRF_GPIO_PIN_NOSENSE);
	nrf_gpio_cfg(EMG_READY,         NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL,   NRF_GPIO_PIN_S0D1, NRF_GPIO_PIN_NOSENSE);
	nrf_gpio_cfg(EMG_SCLK,          NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL,   NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE);
	nrf_gpio_cfg(EMG_SS,            NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL,   NRF_GPIO_PIN_S0S1, NRF_GPIO_PIN_NOSENSE);

	// try and reset I²C bus:
	for (int i = 0; i < 10; ++i) {
		nrf_delay_us(5);
		nrf_gpio_pin_write(GYRO_SCL, 0);
		nrf_delay_us(2);
		if (i == 9) // time to generate stop condition.
			nrf_gpio_pin_write(GYRO_SDA, 0);
		nrf_delay_us(3);
		nrf_gpio_pin_write(GYRO_SCL, 1);
	}
	nrf_delay_us(5);
	nrf_gpio_pin_write(GYRO_SDA, 1);
	nrf_delay_us(5);
	if (nrf_gpio_pin_read(GYRO_SDA) == 0 || nrf_gpio_pin_read(GYRO_SCL) == 0) {
		// I²C bus irrecoverably stuck... abort!
		NRF_LOG_ERROR("I²C BUS stuck!");
		NRF_LOG_FLUSH();
		// disable bus:
		nrf_gpio_cfg(GYRO_SDA, NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_S0D1, NRF_GPIO_PIN_NOSENSE);
		nrf_gpio_cfg(GYRO_SCL, NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_S0D1, NRF_GPIO_PIN_NOSENSE);
	} else {
		i2c_available = true;
	}

	// Initialize ADC:
/*

	// Create timers:
	err_code = app_timer_init();
	APP_ERROR_CHECK(err_code);
	err_code = app_timer_create(&m_power_timer_id, APP_TIMER_MODE_REPEATED, power_timer_handler);
	APP_ERROR_CHECK(err_code);

	err_code = app_timer_start(m_power_timer_id, APP_TIMER_TICKS(1000), NULL);
	APP_ERROR_CHECK(err_code);
*/

	return true;
}


void power_off (enum system_off_modes mode)
{
	nrf_gpio_pin_write(LED_PIN, 0);
	gyro_wakeup_enable();
	nrf_gpio_pin_write(LED_PIN, 0);

	// Go to system-off mode (this function will not return; wakeup will cause a reset).
	sd_power_system_off();
}

static void compute_power_status (void *p_event_data, uint16_t event_size)
{
}

static void adc_event (nrfx_saadc_evt_t const *p_event)
{
}


static void power_timer_handler (void *p_context)
{
    UNUSED_PARAMETER(p_context);
}



/*
 * This function should be called by on_write on the system BLE characteristic.
 */
void power_set_regulator_mode (uint8_t mode)
{
	uint32_t err0, err1;
	switch (mode) {
	case 0:
		err0 = sd_power_dcdc0_mode_set(NRF_POWER_DCDC_DISABLE);
		err1 = sd_power_dcdc_mode_set(NRF_POWER_DCDC_DISABLE);
		break;
	case 1:
		err0 = sd_power_dcdc0_mode_set(NRF_POWER_DCDC_ENABLE);
		err1 = sd_power_dcdc_mode_set(NRF_POWER_DCDC_DISABLE);
		break;
	case 2:
		err0 = sd_power_dcdc0_mode_set(NRF_POWER_DCDC_DISABLE);
		err1 = sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
		break;
	case 3:
		err0 = sd_power_dcdc0_mode_set(NRF_POWER_DCDC_ENABLE);
		err1 = sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
		break;
	default:
		err0 = err1 = 0;
	}
	// TODO: check for errors!
	(void) err0;
	(void) err1;
}

void power_set_digital_voltage (uint8_t mode)
{
	switch (mode) {
	case 0x18: power_flash_regout_voltage(0); break;
	case 0x21: power_flash_regout_voltage(1); break;
	case 0x24: power_flash_regout_voltage(2); break;
	case 0x27: power_flash_regout_voltage(3); break;
	case 0x30: power_flash_regout_voltage(4); break;
	case 0x33: power_flash_regout_voltage(5); break;
	}
}


/*
 * This function sets the REGOUT0 register located in the UICR portion of the flash.
 * It copies the whole UICR register to RAM, sets REGOUT0 there, erases the UICR,
 * and writes back all of the saved content with the change applied.
 * At the end, it resets the MCU to apply the change.
 */
void power_flash_regout_voltage (uint32_t value)
{
	value |= 0xFFFFFFF8; // only the 3 least significant bits are used.

	// disable interrupts:
	__disable_irq();
	__ISB();
	__DSB();

	// backup UICR content:
	NRF_UICR_Type temp = *NRF_UICR;

	// update values:
	temp.REGOUT0 = value;

	// program flash:

	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een; // enable Erase
	while (!NRF_NVMC->READY) ;
	NRF_NVMC->ERASEUICR = 1;
	while (!NRF_NVMC->READY) ;
	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen; // enable Write
	while (!NRF_NVMC->READY) ;

	uint32_t const *src = (uint32_t const *) &temp;
	uint32_t volatile *dst = (uint32_t volatile *) NRF_UICR;
	for (unsigned i = 0; i < sizeof temp / sizeof (uint32_t); ++i) {
		if (src[i] != 0xFFFFFFFF) {
			while (!NRF_NVMC->READYNEXT) ;
			dst[i] = src[i];
		}
	}

	while (!NRF_NVMC->READY) ;
	NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren; // enable Read
	while (!NRF_NVMC->READY) ;

	NVIC_SystemReset();
}

