// emg.c

#include <stdbool.h>
#include <stdint.h>
#include "boards.h"

#include <nrfx_spim.h>
#include <nrf_pwm.h>
#include <nrf_gpiote.h>
#include <nrf_soc.h>
#include <nrf_delay.h>

#include "app_scheduler.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "emg.h"

static NRF_PWM_Type * const pwm = NRF_PWM0;
static const nrfx_spim_t emg = NRFX_SPIM_INSTANCE(2);

uint8_t emg_id   = 0x40 + 0x80;
uint8_t emg_read = 0x30 + 0x80;
uint8_t emg_reply[17];
volatile bool spi_finished;

volatile uint32_t emg_buf_head;
volatile uint32_t emg_buf_tail;
emg_sample_t emg_buf[EMG_BUF_SIZE];

unsigned emg_samples_per_packet = 2;
unsigned emg_channels_per_sample = 3;
unsigned emg_compression_type = 0;
unsigned emg_compression_mode = 0;

int emg_status;

extern void received_emg_data_block (void *p_event_data, uint16_t event_size);

static volatile uint32_t last_datardy_timestamp;

uint8_t emg_online_reconf_data[32];
uint8_t emg_online_reconf_size;
uint8_t emg_local_buf_head;

#ifdef USE_FAKE_EMG_DATA
static const uint8_t fake_data[256][9] = {
#include "fakedata.h"
};

static uint8_t fake_index;
#endif

static void spi_event (nrfx_spim_evt_t const *p_event, void *p_context)
{
	int emg_status = *(int *) p_context;

	if (emg_status == 0) {
		if (p_event->type == NRFX_SPIM_EVENT_DONE) spi_finished = true;
	} else if (emg_status == 3) {
		if (p_event->type == NRFX_SPIM_EVENT_DONE) {
			// on-line reconfiguration done, re-enable streaming:
			nrfx_spim_xfer_desc_t transfer_emg = {
				.p_tx_buffer = &emg_read,
				.tx_length = 1,
				.p_rx_buffer = emg_reply,
				.rx_length = 17,
			};
			nrfx_spim_xfer(&emg, &transfer_emg, NRFX_SPIM_FLAG_HOLD_XFER | NRFX_SPIM_FLAG_REPEATED_XFER);
			*(int *) p_context = 2;
			nrf_gpiote_event_enable(0); // re-enable interrupt generation.
			nrf_gpiote_task_enable(1);
		}
	} else if (emg_status >= 1) {
		if (p_event->type == NRFX_SPIM_EVENT_DONE) {
			uint32_t delay = NRF_TIMER1->CC[1] - last_datardy_timestamp;
			last_datardy_timestamp = NRF_TIMER1->CC[1];
			if (emg_status == 1) { // first sample arrived
				*(int *) p_context = 2;
				NRF_LOG_INFO("ADS1293 started after %u µs.", delay);
				NRF_LOG_FLUSH();
				delay = 1;
			} else {
/*				if (delay != 1250) {
					NRF_LOG_ERROR("ADS1293 data timing error: %u µs.", delay);
					NRF_LOG_FLUSH();
				}
				delay /= 1250;*/
			}
			// queue data:
			emg_sample_t *p = emg_buf + emg_buf_tail++;
			emg_buf_tail &= EMG_BUF_IDX_MASK;
			p->timestamp = last_datardy_timestamp;
			p->status = emg_reply[1];
#ifdef USE_FAKE_EMG_DATA
			memcpy(p->data, fake_data + fake_index++, 9);
#else
			memcpy(p->data, emg_reply + 8, sizeof p->data);
#endif
			uint8_t queue_used = (emg_buf_tail - emg_local_buf_head) & EMG_BUF_IDX_MASK;
			if (queue_used >= emg_samples_per_packet) {
				emg_local_buf_head = emg_buf_tail;
				if (queue_used > 12)
					NRF_LOG_INFO("QUEUE WARN: %d", queue_used);
				app_sched_event_put(NULL, 0, received_emg_data_block);
				emg_online_reconf_send();
			}
		}
	}
}

void emg_online_reconf_send (void)
{
	// check if there are data to be sent over SPI while streaming is in progress:
	if (emg_online_reconf_size) {
		nrf_gpiote_event_disable(0); // temporarily disable interrupt generation.
		nrf_gpiote_task_disable(1);

		emg_status = 3;
		nrfx_spim_xfer_desc_t transfer_emg = {
			.p_tx_buffer = emg_online_reconf_data,
			.tx_length = emg_online_reconf_size,
			.p_rx_buffer = NULL,
			.rx_length = 0
		};
		nrfx_spim_xfer(&emg, &transfer_emg, 0);
		emg_online_reconf_size = 0;
	}
}

static void spi_wait (void)
{
	while (!spi_finished) ; // TODO: sd_app_evt_wait()
	spi_finished = false;
}

static void clock_start (void)
{
	// start a 409.6 kHz clock for ADS1293.
	nrf_pwm_enable(pwm);
	uint32_t pwm_pins[4] = {
		EMG_CLOCK,
#ifdef USE_FAKE_EMG_DATA
		NRF_PWM_PIN_NOT_CONNECTED,
#else
		LED_PIN,
#endif
		NRF_PWM_PIN_NOT_CONNECTED,
		NRF_PWM_PIN_NOT_CONNECTED
	};
	nrf_pwm_pins_set(pwm, pwm_pins);
	nrf_pwm_configure(pwm, NRF_PWM_CLK_16MHz, NRF_PWM_MODE_UP, 39);
	// To obtain 409.6 kHz (2441.406250 ns) from a 16 MHz (62.5 ns) clock
	// we need to count 39.0625 periods of the base clock.
	// So we count up to 39 for 15 periods and then up to 40 for another (single) period:
	// (39 × 15 + 40 × 1) ÷ (15 + 1) = 625 ÷ 16 = 39.0625
	// 50% duty cycle is approximated by using pulse widths alternating between 19 and 20,
	// obtaining an average 19.5 ÷ 39.0625 = 49.92%
	// The indicator LED is also driven by the same PWM at a low duty cycle.
	// In the following defines, 0x8000 is added to request positive pulses.
	#define PWMPHA (0x8000 + 19)
	#define PWMPHB (0x8000 + 20)
#ifdef USE_FAKE_EMG_DATA
	#define PWMLED (0x8000 +  0)
#else
	#define PWMLED (0x8000 +  2) // WAS: 4 @ 1.8 V
#endif
	static nrf_pwm_values_wave_form_t pwm_sequence[] = {
		{PWMPHA, PWMLED, 0, 39}, {PWMPHB, PWMLED, 0, 39}, {PWMPHA, PWMLED, 0, 39}, {PWMPHB, PWMLED, 0, 39},
		{PWMPHA, PWMLED, 0, 39}, {PWMPHB, PWMLED, 0, 39}, {PWMPHA, PWMLED, 0, 39}, {PWMPHB, PWMLED, 0, 39},
		{PWMPHA, PWMLED, 0, 39}, {PWMPHB, PWMLED, 0, 39}, {PWMPHA, PWMLED, 0, 39}, {PWMPHB, PWMLED, 0, 39},
		{PWMPHA, PWMLED, 0, 39}, {PWMPHB, PWMLED, 0, 39}, {PWMPHA, PWMLED, 0, 39}, {PWMPHB, PWMLED, 0, 40},
	};
	nrf_pwm_sequence_t const sequence =
	{
		.values.p_wave_form = pwm_sequence,
		.length             = NRF_PWM_VALUES_LENGTH(pwm_sequence),
		.repeats            = 0,
		.end_delay          = 0
	};
	nrf_pwm_sequence_set(pwm, 0, &sequence);
	nrf_pwm_seq_cnt_set(pwm, 0, sequence.length);
	nrf_pwm_sequence_set(pwm, 1, &sequence);
	nrf_pwm_seq_cnt_set(pwm, 1, sequence.length);
	nrf_pwm_decoder_set(pwm, NRF_PWM_LOAD_WAVE_FORM, NRF_PWM_STEP_AUTO);
	nrf_pwm_shorts_enable(pwm, NRF_PWM_SHORT_LOOPSDONE_SEQSTART0_MASK);
	nrf_pwm_loop_set(pwm, 1);
	nrf_pwm_task_trigger(pwm, NRF_PWM_TASK_SEQSTART0);
}

static void clock_stop (void)
{
	nrf_pwm_disable(pwm);
}

static void clock_start_timer (void)
{
	// Generate ADC clock using a timer instead of PWM peripheral
	nrf_gpiote_task_configure(3, EMG_CLOCK, NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIOTE_INITIAL_VALUE_LOW);
	nrf_gpiote_task_enable(3);
	// Configure timer
	NRF_TIMER3->MODE = 0;
	NRF_TIMER3->BITMODE = 1;
	NRF_TIMER3->PRESCALER = 0;
	NRF_TIMER3->CC[0] = 20;
	NRF_TIMER3->TASKS_CLEAR = 1;
	NRF_TIMER3->SHORTS = 1;
	// Configure PPI
	sd_ppi_channel_assign(4, &NRF_TIMER3->EVENTS_COMPARE[0], (const volatile void *) nrf_gpiote_task_addr_get(NRF_GPIOTE_TASKS_OUT_3));
	sd_ppi_channel_enable_set(1 << 4);
	// enable timer
	NRF_TIMER3->TASKS_START = 1;
}


static void clock_stop_timer (void)
{
	sd_ppi_channel_enable_clr(1 << 4);
	NRF_TIMER3->TASKS_STOP = 1;
	nrf_gpiote_task_disable(3);
}



static uint8_t conf0[] = {
	0x00, // STARTING REGISTER NUMBER
	0x00,
	0100, // positive test signal
	0200, // negative test signal
	0300, // zero test signal
	0300,
	4,    // FLEX_VBAT_CN (4 for CH3)
	0x08, 0x00, 0x00, 0x00, // LOD off,
	0x00, // CMDET_EN
	0,    // CMDET_CN
	0x70,    // RLD_CN
	0, 0, 0, 0, // WILSON
	0, // REF_CN (enabled)
	2, // OSC_CN (write 6 at end to start clock!)
	0077, // AFE_RES
	0004, // AFE_SHDN_CN (4 for CH3)
	0, // AFE_FAULT_CN
	0, // RESERVED
	6  // AFE_PACE_CN
};

static uint8_t conf1[] = {
	0x00, // STARTING REGISTER NUMBER
	0x00,
	0300, // zero test signal
	0300, // zero test signal
	0300, // zero test signal
	0300,
	0,    // FLEX_VBAT_CN (4 for CH3)
	0x08, 0x00, 0x00, 0x00, // LOD off,
	0x00, // CMDET_EN
	0,    // CMDET_CN
	0x70, // RLD_CN
	0, 0, 0, 0, // WILSON
	0, // REF_CN (enabled)
	2, // OSC_CN (write 6 at end to start clock!)
	0077, // AFE_RES
	0000, // AFE_SHDN_CN (4 for CH3)
	0, // AFE_FAULT_CN
	0, // RESERVED
	6  // AFE_PACE_CN
};

static uint8_t conf2[] = {
	0x21, // STARTING REGISTER NUMBER
	1,  // R2_RATE (div by 4)
	16, // R3_RATE_CH1 (div by 16)
	16, // R3_RATE_CH2 (div by 16)
	16, // R3_RATE_CH3 (div by 16)
	0,  // R1_RATE (div by 4)
	0,  // ECG_FILTER (enabled)
	32, // DRDYB_SRC (CH3)
	32, // SYNCB_CN  (CH3)
	0, 0 // ALARM
};

static uint8_t conf3[] = {
	0x12, // STARTING REGISTER NUMBER
	6, // OSC_CN
};

static uint8_t conf4[] = {
	0x00, // STARTING REGISTER NUMBER
	1, // CONFIG (start)
};

static uint8_t conf_channels[] = {
	0x01, // STARTING REGISTER NUMBER
	0012, // CH1 PIN 1+ 2-
	0034, // CH2 PIN 3+ 4-
	0056, // CH3 PIN 5+ 6-
};


static void emg_power_on (void)
{
	// power up chip:
	nrf_gpio_pin_write(EMG_POWER, 0);
	nrf_delay_ms(5);
	nrf_gpio_pin_write(EMG_ENABLE, 1);
	nrf_delay_ms(5);

	clock_start();

	nrf_gpio_cfg(EMG_READY, NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_INPUT_CONNECT, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_S0D1, NRF_GPIO_PIN_NOSENSE);
	nrf_gpio_cfg(EMG_ALARM, NRF_GPIO_PIN_DIR_INPUT, NRF_GPIO_PIN_INPUT_CONNECT, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_S0D1, NRF_GPIO_PIN_NOSENSE);

	nrfx_spim_config_t conf_emg = {
		.sck_pin  = EMG_SCLK,
		.mosi_pin = EMG_MOSI,
		.miso_pin = EMG_MISO,
		.ss_pin   = EMG_SS,
		.ss_active_high = false,
		.irq_priority = APP_IRQ_PRIORITY_HIGH,
		.orc = 0x00, // because there are pull-down resistors
		.frequency = NRF_SPIM_FREQ_8M,
		.mode = NRF_SPIM_MODE_0,
		.bit_order = NRF_SPIM_BIT_ORDER_MSB_FIRST
	};

	nrfx_err_t err = nrfx_spim_init(&emg, &conf_emg, spi_event, &emg_status);
    APP_ERROR_CHECK(err);

	nrf_gpio_pin_write(EMG_RESET, 1);
	nrf_delay_ms(1);
}

static void emg_power_off (void)
{
	// turn off chip:
	nrf_gpio_pin_write(EMG_RESET, 0);
	nrfx_spim_uninit(&emg);
	nrf_gpio_pin_write(EMG_SS, 0);
	nrf_gpio_cfg(EMG_ALARM, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0D1, NRF_GPIO_PIN_NOSENSE);
	nrf_gpio_cfg(EMG_READY, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_S0D1, NRF_GPIO_PIN_NOSENSE);
	clock_stop();
	nrf_gpio_pin_write(EMG_ENABLE, 0);
	nrf_gpio_pin_write(EMG_POWER, 1);
}

int emg_probe (void)
{
	nrfx_err_t err;

	emg_power_on();

	nrfx_spim_xfer_desc_t transfer_emg = {
		.p_tx_buffer = &emg_id,
		.tx_length = 1,
		.p_rx_buffer = emg_reply,
		.rx_length = 2
	};

	err = nrfx_spim_xfer(&emg, &transfer_emg, 0);
	spi_wait();
	NRF_LOG_INFO("SPI-40: %d.", emg_reply[1]);
	NRF_LOG_FLUSH();

	if (emg_reply[1] != 0x01) {
		NRF_LOG_INFO("EMG NOT FOUND!");
		NRF_LOG_FLUSH();
		emg_power_off();
		return 1;
	}

	transfer_emg.p_tx_buffer = conf0;
	transfer_emg.tx_length = sizeof conf0,
	transfer_emg.p_rx_buffer = NULL;
	transfer_emg.rx_length = 0;
	err = nrfx_spim_xfer(&emg, &transfer_emg, 0);
	spi_wait();

	transfer_emg.p_tx_buffer = conf2;
	transfer_emg.tx_length = sizeof conf2,
	transfer_emg.p_rx_buffer = NULL;
	transfer_emg.rx_length = 0;
	err = nrfx_spim_xfer(&emg, &transfer_emg, 0);
	spi_wait();

	transfer_emg.p_tx_buffer = conf3;
	transfer_emg.tx_length = sizeof conf3,
	transfer_emg.p_rx_buffer = NULL;
	transfer_emg.rx_length = 0;
	err = nrfx_spim_xfer(&emg, &transfer_emg, 0);
	spi_wait();

	transfer_emg.p_tx_buffer = conf4;
	transfer_emg.tx_length = sizeof conf4,
	transfer_emg.p_rx_buffer = NULL;
	transfer_emg.rx_length = 0;
	err = nrfx_spim_xfer(&emg, &transfer_emg, 0);
	spi_wait();

	nrf_delay_ms(100);

	transfer_emg.p_tx_buffer = &emg_read;
	transfer_emg.tx_length = 1,
	transfer_emg.p_rx_buffer = emg_reply;
	transfer_emg.rx_length = 17;
	err = nrfx_spim_xfer(&emg, &transfer_emg, 0);
	APP_ERROR_CHECK(err);
	spi_wait();

	int32_t vdd = (emg_reply[14] << 16) + (emg_reply[15] << 8) + (emg_reply[16]);
	if (vdd && abs(vdd - 0x4EC995) > 107679) {
		// check if VDD is within 3.3 V +/- 0.1 V
		NRF_LOG_INFO("ADS1293 VDD out of range!");
	}
	vdd = ((vdd - 0x400000 + 8) / 16 * 4869 + 0x28000) / 0x50000 + 2400;
	NRF_LOG_INFO("VDD=%d mV", vdd);

	NRF_LOG_HEXDUMP_INFO(emg_reply + 1, 16);
	NRF_LOG_FLUSH();

	emg_power_off();
	return 0;
}

int emg_configure (int rate, int samples_per_packet, uint32_t ch_config)
{
	uint8_t r3_rate = 0;
	switch (rate) {
		case  100: r3_rate = 0x80; break;
		case  200: r3_rate = 0x40; break;
		case  400: r3_rate = 0x20; break;
		case  800: r3_rate = 0x10; break;
		case 1600: r3_rate = 0x04; break;
		case 3200: r3_rate = 0x01; break;
		return -1;
	}
	int channels = !!(ch_config & 0x0000FF) + !!(ch_config & 0x00FF00) + !!(ch_config & 0xFF0000);
	switch (channels) {
		case 0: return -2;
		case 1: if (ch_config & 0xFFFF00) return -2; else break;
		case 2: if (ch_config & 0xFF0000) return -2; else break;
	}
	emg_compression_type = samples_per_packet >> 4;
	emg_compression_mode = samples_per_packet & 0x0F;
	if (samples_per_packet >= 0x10) {
		samples_per_packet = 4;
	} else {
		if (samples_per_packet * channels > 6) return -2;
	}
	emg_samples_per_packet = samples_per_packet;
	emg_channels_per_sample = channels;
	conf_channels[1] = (ch_config & 0x0000FF) >>  0;
	conf_channels[2] = (ch_config & 0x00FF00) >>  8;
	conf_channels[3] = (ch_config & 0xFF0000) >> 16;
	conf2[2] = r3_rate;
	conf2[3] = r3_rate;
	conf2[4] = r3_rate;
	uint8_t afe_shdn_cn = 011 * !(ch_config & 0x0000FF) + 022 * !(ch_config & 0x00FF00) + 044 * !(ch_config & 0xFF0000);
	conf1[21] = afe_shdn_cn;
	uint8_t drdyb_src = (ch_config & 0x0000FF) ? 010 : (ch_config & 0x00FF00) ? 020 : (ch_config & 0xFF0000) ? 040 : 0;
	conf2[7] = drdyb_src;
	conf2[8] = drdyb_src;
	return 0;
}

void emg_enable (void)
{
	nrfx_err_t err;

	emg_power_on();

	nrfx_spim_xfer_desc_t transfer_emg = {
		.p_tx_buffer = &emg_id,
		.tx_length = 1,
		.p_rx_buffer = emg_reply,
		.rx_length = 2
	};

	err = nrfx_spim_xfer(&emg, &transfer_emg, 0);
	spi_wait();
	NRF_LOG_INFO("SPI-40: %d.", emg_reply[1]);
	NRF_LOG_FLUSH();

	transfer_emg.p_tx_buffer = conf1;
	transfer_emg.tx_length = sizeof conf1,
	transfer_emg.p_rx_buffer = NULL;
	transfer_emg.rx_length = 0;
	err = nrfx_spim_xfer(&emg, &transfer_emg, 0);
	spi_wait();

	transfer_emg.p_tx_buffer = conf2;
	transfer_emg.tx_length = sizeof conf2,
	transfer_emg.p_rx_buffer = NULL;
	transfer_emg.rx_length = 0;
	err = nrfx_spim_xfer(&emg, &transfer_emg, 0);
	spi_wait();

	transfer_emg.p_tx_buffer = conf3;
	transfer_emg.tx_length = sizeof conf3,
	transfer_emg.p_rx_buffer = NULL;
	transfer_emg.rx_length = 0;
	err = nrfx_spim_xfer(&emg, &transfer_emg, 0);
	spi_wait();

	transfer_emg.p_tx_buffer = conf_channels;
	transfer_emg.tx_length = sizeof conf_channels,
	transfer_emg.p_rx_buffer = NULL;
	transfer_emg.rx_length = 0;
	err = nrfx_spim_xfer(&emg, &transfer_emg, 0);
	spi_wait();

	transfer_emg.p_tx_buffer = conf4;
	transfer_emg.tx_length = sizeof conf4,
	transfer_emg.p_rx_buffer = NULL;
	transfer_emg.rx_length = 0;
	err = nrfx_spim_xfer(&emg, &transfer_emg, 0);
	spi_wait();

	emg_status = 1;

	transfer_emg.p_tx_buffer = &emg_read;
	transfer_emg.tx_length = 1,
	transfer_emg.p_rx_buffer = emg_reply;
	transfer_emg.rx_length = 17;
	err = nrfx_spim_xfer(&emg, &transfer_emg, NRFX_SPIM_FLAG_HOLD_XFER | NRFX_SPIM_FLAG_REPEATED_XFER);
	APP_ERROR_CHECK(err);

	// Enable events on channel 0 for data ready pin:
	nrf_gpiote_event_configure(0, EMG_READY, NRF_GPIOTE_POLARITY_HITOLO);
	nrf_gpiote_event_enable(0);
	nrf_gpiote_task_configure(1, EMG_SS, NRF_GPIOTE_POLARITY_HITOLO, NRF_GPIOTE_INITIAL_VALUE_HIGH); // TODO: ignored config???
	nrf_gpiote_task_enable(1);
	// and use it to start SPI transaction:
 	sd_ppi_channel_assign(0, (const volatile void *) nrf_gpiote_event_addr_get(NRF_GPIOTE_EVENTS_IN_0), (const volatile void *) nrfx_spim_start_task_get(&emg));
 	sd_ppi_channel_assign(1, (const volatile void *) nrf_gpiote_event_addr_get(NRF_GPIOTE_EVENTS_IN_0), (const volatile void *) nrf_gpiote_task_addr_get(NRF_GPIOTE_TASKS_CLR_1));
 	NRF_PPI->FORK[1].TEP = (uint32_t) &NRF_TIMER1->TASKS_CAPTURE[1];
 	sd_ppi_channel_assign(2, (const volatile void *) nrfx_spim_end_event_get(&emg), (const volatile void *) nrf_gpiote_task_addr_get(NRF_GPIOTE_TASKS_SET_1));
 	sd_ppi_channel_enable_set((1 << 0) | (1 << 1) | (1 << 2));
 	// store initial timestamp:

 	NRF_TIMER1->MODE = 0; // timer
    NRF_TIMER1->BITMODE = 2; // 24 bit fot test, must be: 3; // 32 bit
    NRF_TIMER1->PRESCALER = 4; // 1 MHz
    NRF_TIMER1->TASKS_CLEAR = 1;
    NRF_TIMER1->TASKS_START = 1;

	NRF_TIMER1->TASKS_CAPTURE[1] = 1;
	last_datardy_timestamp = NRF_TIMER1->CC[1];
}

void emg_disable (void)
{
 	sd_ppi_channel_enable_clr((1 << 0) | (1 << 1) | (1 << 2));
	nrf_gpiote_event_disable(0);
	nrf_gpiote_task_disable(1);
	emg_power_off();
    NRF_TIMER1->TASKS_STOP = 1;
	emg_status = 0;
}
