
#include <stdbool.h>
#include <stdint.h>
#include "nrf_delay.h"
#include "boards.h"

#include <nrfx_saadc.h>
#include <nrfx_power.h>
#include <nrf_temp.h>
#include <nrf_clock.h>
#include <nrf_pwr_mgmt.h>

#include "app_util_platform.h"
#include "app_error.h"
#include "app_timer.h"
#include "app_scheduler.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "bluetooth.h"
#include "gyro.h"
#include "emg.h"
#include "power.h"

#include "usb.h"


#define FPU_INTERRUPT_MODE // This is needed to allow the CPU to enter power saving after an FPU exception.

#ifdef FPU_INTERRUPT_MODE

#define FPU_EXCEPTION_MASK               0x0000009F                      //!< FPU exception mask used to clear exceptions in FPSCR register.
#define FPU_FPSCR_REG_STACK_OFF          0x40                            //!< Offset of FPSCR register stacked during in
/**
 * @brief FPU Interrupt handler. Clearing exception flag at the stack.
 *
 * Function clears exception flag in FPSCR register and at the stack. During interrupt handler
 * execution FPU registers might be copied to the stack (see lazy stacking option) and
 * it is necessary to clear data at the stack which will be recovered in the return from
 * interrupt handling.
 */
void FPU_IRQHandler(void)
{
	// Prepare pointer to stack address with pushed FPSCR register.
	uint32_t * fpscr = (uint32_t * )(FPU->FPCAR + FPU_FPSCR_REG_STACK_OFF);
	// Execute FPU instruction to activate lazy stacking.
	(void)__get_FPSCR();
	// Clear flags in stacked FPSCR register.
	*fpscr = *fpscr & ~(FPU_EXCEPTION_MASK);
}

#endif


// Define a buffer for the scheduler:
#define SCHED_MAX_EVENT_DATA_SIZE MAX(2 * sizeof (uint32_t), APP_TIMER_SCHED_EVENT_DATA_SIZE)
#define SCHED_QUEUE_SIZE 8
static uint32_t scheduler_buffer[CEIL_DIV(
	APP_SCHED_BUF_SIZE(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE),
	sizeof (uint32_t)
)];


static nrfx_saadc_channel_t adc_channels[] = {
		NRFX_SAADC_DEFAULT_CHANNEL_SE(NRF_SAADC_INPUT_AIN0,     0), // VBUS ÷ 5.02
		NRFX_SAADC_DEFAULT_CHANNEL_SE(NRF_SAADC_INPUT_AIN1,     1), // IBAT × 10 Ω
		NRFX_SAADC_DEFAULT_CHANNEL_SE(NRF_SAADC_INPUT_VDD,      2),
		NRFX_SAADC_DEFAULT_CHANNEL_SE(NRF_SAADC_INPUT_VDDHDIV5, 3),
};

extern nrf_saadc_value_t adc_buf[4];
volatile bool calibrated = false;
int32_t tsoc;

static void compute_power_status (void *p_event_data, uint16_t event_size)
{
	int vbus =   6024 * adc_buf[0] / 4096; // mV
	int ibat = 180000 * adc_buf[1] / 4096; // µA
	int vddd =   3600 * adc_buf[2] / 4096; // mV
	int vbat =   6000 * adc_buf[3] / 4096; // mV
	uint32_t usb_status;
	sd_power_usbregstatus_get(&usb_status);
	bool usb = usb_status & NRF_POWER_USBREGSTATUS_VBUSDETECT_MASK;
	bool chrg_on = !nrf_gpio_pin_read(CHRG_STATUS_SENSE);
	nrf_gpio_pin_write(CHRG_STATUS_DRIVE, 1);
	bool chrg_charging = !nrf_gpio_pin_read(CHRG_STATUS_SENSE);
	nrf_gpio_pin_write(CHRG_STATUS_DRIVE, 0);
	const char *status = "Error";
	if (chrg_on && chrg_charging) status = "Charging";
	if (chrg_on && !chrg_charging) status = "Charged";
	if (!chrg_on && !chrg_charging) status = "Off";
	if (usb != chrg_on) status = "USB-Error";
	int32_t t = tsoc * 100 / 4;
	// Compute battery level (dummy calculation!)
	// TODO: real calculation!
	static const int vbat_max = 4200;
	static const int vbat_min = 3500;
	int bat_level = (vbat - vbat_min) * 100 / (vbat_max - vbat_min);
	bt_update_battery_level(bat_level);
	uint32_t clk;
	sd_clock_hfclk_is_running(&clk);
	if (clk) status = "HF-XTAL=ON";
	NRF_LOG_INFO("T=%d VBUS=%dmV VBAT=%dmV IBAT=%dµA VDDD=%dmV %s", t, vbus, vbat, ibat, vddd, status);
//	NRF_LOG_INFO("INT1=%d INT2=%d", nrf_gpio_pin_read(GYRO_INT1), nrf_gpio_pin_read(GYRO_INT2));
	NRF_LOG_FLUSH();
}

static void adc_event (nrfx_saadc_evt_t const *p_event)
{
	if (p_event->type == NRFX_SAADC_EVT_DONE) {
		// Applying workaround from Errata 212, otherwise current is stuck at 4-500uA during sleep after first sample.
		volatile uint32_t temp1;
		volatile uint32_t temp2;
		volatile uint32_t temp3;

		temp1 = *(volatile uint32_t *)0x40007640ul;
		temp2 = *(volatile uint32_t *)0x40007644ul;
		temp3 = *(volatile uint32_t *)0x40007648ul;

		*(volatile uint32_t *)0x40007FFCul = 0ul;
		(void) *(volatile uint32_t *)0x40007FFCul;
		*(volatile uint32_t *)0x40007FFCul = 1ul;

		*(volatile uint32_t *)0x40007640ul = temp1;
		*(volatile uint32_t *)0x40007644ul = temp2;
		*(volatile uint32_t *)0x40007648ul = temp3;
		// End of workaround.
		app_sched_event_put(NULL, 0, compute_power_status);
	}
	if (p_event->type == NRFX_SAADC_EVT_CALIBRATEDONE) {
		calibrated = true;
	}
}

APP_TIMER_DEF(m_power_timer_id);


static void power_timer_handler (void *p_context)
{
	UNUSED_PARAMETER(p_context);
//	nrf_gpio_pin_toggle(LED_PIN);
	if (calibrated) {
		ret_code_t err_code;
		sd_temp_get(&tsoc);
		nrfx_saadc_channels_config(adc_channels, 4);
		err_code = nrfx_saadc_simple_mode_set((1<<0|1<<1|1<<2|1<<3), NRF_SAADC_RESOLUTION_12BIT, NRF_SAADC_OVERSAMPLE_4X, adc_event);
		APP_ERROR_CHECK(err_code);
		nrfx_saadc_buffer_set(adc_buf, sizeof adc_buf / sizeof *adc_buf);
		nrfx_saadc_mode_trigger();
	}
}

#include <nrfx_power.h>
#include <ble.h>

static const char * const ble_versions[] = {
	"?", "", "", "", "", "", "4.0", "4.1", "4.2", "5.0", "5.1", "5.2"
};

void show_reset_info (void)
{
	uint32_t r = NRF_POWER->RESETREAS;
	NRF_POWER->RESETREAS = r;
	char reason[64] = {0};
	if (r) strcat(reason, " by");
	if (r & NRF_POWER_RESETREAS_RESETPIN_MASK) strcat(reason, " PIN");
	if (r & NRF_POWER_RESETREAS_DOG_MASK)      strcat(reason, " DOG");
	if (r & NRF_POWER_RESETREAS_SREQ_MASK)     strcat(reason, " SRQ");
	if (r & NRF_POWER_RESETREAS_LOCKUP_MASK)   strcat(reason, " LCK");
	if (r & NRF_POWER_RESETREAS_OFF_MASK)      strcat(reason, " INT");
	if (r & NRF_POWER_RESETREAS_LPCOMP_MASK)   strcat(reason, " ANA");
	if (r & NRF_POWER_RESETREAS_DIF_MASK)      strcat(reason, " DBG");
	if (r & NRF_POWER_RESETREAS_NFC_MASK)      strcat(reason, " NFC");
	if (r & NRF_POWER_RESETREAS_VBUS_MASK)     strcat(reason, " USB");
	NRF_LOG_INFO("Program started%s", reason);
	NRF_LOG_FLUSH();
}

void show_softdevice_info (void)
{
	ble_version_t v = {0};
	uint32_t e = sd_ble_version_get(&v);
	APP_ERROR_CHECK(e);
	if (v.version_number >= sizeof ble_versions / sizeof *ble_versions)
		v.version_number = 0;
	NRF_LOG_INFO("SoftDevice version %04X (BT %s)", v.subversion_number, ble_versions[v.version_number]);
	NRF_LOG_FLUSH();
}

//----------------------------------------------------------------------------------
int main(void)
{
	ret_code_t err_code;

#ifdef FPU_INTERRUPT_MODE
	// Enable FPU interrupt:
	NVIC_SetPriority(FPU_IRQn, APP_IRQ_PRIORITY_LOWEST);
	NVIC_ClearPendingIRQ(FPU_IRQn);
	NVIC_EnableIRQ(FPU_IRQn);
#endif
	// Initialize logging system:
	APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
	NRF_LOG_DEFAULT_BACKENDS_INIT();

//rimetti decommentato
    //nrf_clock_lf_src_set(NRF_CLOCK_LFCLK_Xtal);
    //nrf_clock_task_trigger(NRF_CLOCK_TASK_LFCLKSTART);

	// Initialize timers and schedulers:
	err_code = app_timer_init();
	APP_ERROR_CHECK(err_code);
	err_code = app_sched_init(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE, scheduler_buffer);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_pwr_mgmt_init();
	APP_ERROR_CHECK(err_code);

	// Initialize subsystems:
	usb_init();
	ble_init();
	power_init();

	// Start them:
	//power_start();
	ble_start();
	usb_start();


    // Initialize ADC:
	err_code = nrfx_saadc_init(NRFX_SAADC_CONFIG_IRQ_PRIORITY);
	APP_ERROR_CHECK(err_code);

    adc_channels[0].channel_config.gain = NRF_SAADC_GAIN1_2; // FS = 6 V
    adc_channels[1].channel_config.gain = NRF_SAADC_GAIN1_3; // FS = 180 mA
    adc_channels[2].channel_config.gain = NRF_SAADC_GAIN1_6; // FS = 3.6 V
    adc_channels[3].channel_config.gain = NRF_SAADC_GAIN1_2; // FS = 6 V

	err_code = nrfx_saadc_channels_config(adc_channels, 4);
	APP_ERROR_CHECK(err_code);

	nrfx_saadc_offset_calibrate(adc_event);

	// Probe external sensors:

	if (i2c_available) {
		gyro_probe();
	}


//	emg_probe();

	// Initialize rest of application:

    // Create timers:
    err_code = app_timer_create(&m_power_timer_id, APP_TIMER_MODE_REPEATED, power_timer_handler);
    APP_ERROR_CHECK(err_code);

    err_code = app_timer_start(m_power_timer_id, APP_TIMER_TICKS(1000), NULL);
    APP_ERROR_CHECK(err_code);

	sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);

//-----------------------------------------------------------------------
    // Enter main loop.
	for (;;) {
		app_sched_execute();
		while (app_usbd_event_queue_process());
         

		if (NRF_LOG_PROCESS() == false)
			nrf_pwr_mgmt_run();
	}
	
}