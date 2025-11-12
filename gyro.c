// LSM6DSO code
/*
 * 18 <= 02 (I3C_Disable)
 * 10 <= 4C (L 104 Hz 8g)
 * 11 <= 40 (G 104 Hz 250°/s)
 * 10 ... <= 4C 40 04 08 60 00 00 00 02 20
 */


#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "boards.h"
#include <stdio.h>

#include <nrfx_twim.h>
#include <nrfx_spim.h>
#include <nrfx_saadc.h>
#include <nrfx_power.h>

#include <nrf_gpiote.h>

#include <nrf_temp.h>

#include "app_util_platform.h"
#include "app_error.h"

#include "app_scheduler.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "gyro.h"

static const nrfx_twim_t gyro = NRFX_TWIM_INSTANCE(0);

uint8_t id_reg = 0x0F;
uint8_t id_val = 0;
uint8_t gyro_setup[] = {0x10, 0x4C, 0x44, 0x44, 0x08, 0x60, 0x00, 0x00, 0x00, 0x02, 0x20};
//uint8_t gyro_setup[] = {0x10, 0x10, 0x00, 0x44, 0x08, 0x60, 0x00, 0x00, 0x04, 0x02, 0x20};
uint8_t gyro_read[]  = {0x20};
uint8_t gyro_data[14];

volatile bool i2c_finished;
static bool gyro_stream;

volatile uint32_t gyro_buf_head;
volatile uint32_t gyro_buf_tail;
gyro_sample_t gyro_buf[GYRO_BUF_SIZE];

extern void received_gyro_data_block (void *p_event_data, uint16_t event_size);

static void i2c_event (nrfx_twim_evt_t const *p_event, void *p_context)
{
	if (p_event->type == NRFX_TWIM_EVT_DONE) {
		if (gyro_stream) {
			// queue data:
			gyro_sample_t *p = gyro_buf + gyro_buf_tail++;
			gyro_buf_tail &= GYRO_BUF_IDX_MASK;
			p->timestamp = NRF_TIMER1->CC[3];
			p->status = 0;
			memcpy(p->data, gyro_data, 14);
			if (((gyro_buf_tail - gyro_buf_head) & GYRO_BUF_IDX_MASK) >= 2) {
				app_sched_event_put(NULL, 0, received_gyro_data_block);
			}
		} else
			i2c_finished = true;
	}
}

static void i2c_wait (void)
{
	while (!i2c_finished) __WFI(); // TODO: sd_app_evt_wait()
	i2c_finished = false;
}




void gyro_wakeup_enable (void)
{
	uint8_t conf[][2] = {
		{0x10, 0x00},
		{0x11, 0x00},
		{0x14, 0x80}, // 0x80 ULP
		{0x15, 0x00},
		{0x17, 0x04}, // CTRL8_XL = HP_SLOPE_XL_EN
		{0x5B, 0x0F}, // WAKE_UP_THS
		{0x5C, 0x10}, // WAKE_UP_DUR
		{0x56, 0x40}, // TAP_CFG_0
		{0x10, 0x70}, // CTRL1_XL
		{0,    4},    // wait 4 ms
		{0x10, 0x10}, // CTRL1_XL
		{0x58, 0x80}, // TAP_CFG_2
		{0x5E, 0x20}, // MD1_CFG
	};

	nrfx_twim_enable(&gyro);
	for (unsigned i = 0; i < sizeof conf / sizeof *conf; ++i) {
		if (!conf[i][0]) {
			nrf_delay_ms(conf[i][1]);
		} else {
			nrfx_twim_tx(&gyro, GYRO_ADDRESS, &conf[i][0], 2, false);
			i2c_wait();
			nrf_delay_ms(1);
		}
	}

//	uint8_t data[8]={0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};
	nrfx_twim_tx(&gyro, GYRO_ADDRESS, &conf[6][0], 1, true);
	i2c_wait();
	nrf_delay_ms(1);
//	nrfx_twim_rx(&gyro, GYRO_ADDRESS, data, 8);
//	i2c_wait();
//	nrf_delay_ms(1000);
//	NRF_LOG_HEXDUMP_INFO(data, 8);
//	NRF_LOG_FLUSH();

	nrfx_twim_disable(&gyro);

}

int gyro_probe (void)
{
	nrfx_twim_config_t conf = {
		.scl = GYRO_SCL,
		.sda = GYRO_SDA,
		.frequency = NRF_TWIM_FREQ_400K,
		.interrupt_priority = APP_IRQ_PRIORITY_HIGH,
		.hold_bus_uninit = true
	};

	nrfx_err_t err = nrfx_twim_init(&gyro, &conf, i2c_event, NULL);

	nrfx_twim_enable(&gyro);

	uint8_t conf_main[] = {0x01, 0};
	err = nrfx_twim_tx(&gyro, GYRO_ADDRESS, conf_main, sizeof conf_main, false);
	i2c_wait();

	nrfx_twim_xfer_desc_t transfer = {
			.type = NRFX_TWIM_XFER_TXRX,
			.address = GYRO_ADDRESS,
			.primary_length = 1,
			.secondary_length = 1,
			.p_primary_buf = &id_reg,
			.p_secondary_buf = &id_val
	};

	err = nrfx_twim_xfer(&gyro, &transfer, 0);
	i2c_wait();

	if (id_val != 0x6C) {
		NRF_LOG_INFO("GYRO NOT FOUND!");
		NRF_LOG_FLUSH();
		return 1;
	}

	// turn off gyroscope:
	uint8_t conf_i2c[] = {0x18, 2};
	err = nrfx_twim_tx(&gyro, GYRO_ADDRESS, conf_i2c, sizeof conf_i2c, false);
	i2c_wait();
	uint8_t conf_off[] = {0x10, 0, 0};
	err = nrfx_twim_tx(&gyro, GYRO_ADDRESS, conf_off, sizeof conf_off, false);
	i2c_wait();

	// reset:
	uint8_t conf3[] = {0x12, 0x01};
	err = nrfx_twim_tx(&gyro, GYRO_ADDRESS, conf3, sizeof conf3, false);
	i2c_wait();
	nrf_delay_ms(1);
	// reboot:
	uint8_t conf1[] = {0x0D, 0x04};
	err = nrfx_twim_tx(&gyro, GYRO_ADDRESS, conf1, sizeof conf1, false);
	i2c_wait();
	uint8_t conf2[] = {0x12, 0x80};
	err = nrfx_twim_tx(&gyro, GYRO_ADDRESS, conf2, sizeof conf2, false);
	i2c_wait();
	while (nrf_gpio_pin_read(GYRO_INT1)) ;
	// reset:
	err = nrfx_twim_tx(&gyro, GYRO_ADDRESS, conf3, sizeof conf3, false);
	i2c_wait();
	nrf_delay_ms(1);

	nrfx_twim_disable(&gyro);

	return 0;
}

void gyro_enable (void)
{
	nrfx_twim_enable(&gyro);

	nrfx_twim_xfer_desc_t transfer2 = {
			.type = NRFX_TWIM_XFER_TX,
			.address = GYRO_ADDRESS,
			.primary_length = sizeof gyro_setup,
			.secondary_length = 0,
			.p_primary_buf = gyro_setup,
			.p_secondary_buf = NULL
	};
	nrfx_err_t err = nrfx_twim_xfer(&gyro, &transfer2, 0);
	i2c_wait();

	uint8_t confint2[][2] = {
		{0x0B, 0x80},
		{0x0E, 0x01},
	};
	nrfx_twim_tx(&gyro, GYRO_ADDRESS, &confint2[0][0], 2, false);
	i2c_wait();
	nrfx_twim_tx(&gyro, GYRO_ADDRESS, &confint2[1][0], 2, false);
	i2c_wait();

	nrfx_twim_xfer_desc_t transfer3 = {
			.type = NRFX_TWIM_XFER_TXRX,
			.address = GYRO_ADDRESS,
			.primary_length = sizeof gyro_read,
			.secondary_length = sizeof gyro_data,
			.p_primary_buf = gyro_read,
			.p_secondary_buf = gyro_data
	};
	err = nrfx_twim_xfer(&gyro, &transfer3, NRFX_TWIM_FLAG_HOLD_XFER | NRFX_TWIM_FLAG_REPEATED_XFER);

	nrf_gpiote_event_configure(2, GYRO_INT2, NRF_GPIOTE_POLARITY_LOTOHI);
	nrf_gpiote_event_enable(2);
	// and use it to start I2C transaction:
 	sd_ppi_channel_assign(3, (const volatile void *) nrf_gpiote_event_addr_get(NRF_GPIOTE_EVENTS_IN_2), (const volatile void *) nrfx_twim_start_task_get(&gyro, NRFX_TWIM_XFER_TXRX));
 	NRF_PPI->FORK[3].TEP = (uint32_t) &NRF_TIMER1->TASKS_CAPTURE[3];
 	sd_ppi_channel_enable_set(1 << 3);
 	gyro_stream = true;
}

void gyro_disable (void)
{
	if (!gyro_stream) return;
 	sd_ppi_channel_enable_clr(1 << 3);
 	nrf_gpiote_event_disable(2);
	uint8_t conf_off[] = {0x10, 0, 0};
	nrfx_twim_tx(&gyro, GYRO_ADDRESS, conf_off, sizeof conf_off, false);
 	gyro_stream = false;
	i2c_wait();
	nrfx_twim_disable(&gyro);
}

