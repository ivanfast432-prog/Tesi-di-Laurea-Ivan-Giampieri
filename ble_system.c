#include "sdk_common.h"
#include "ble_err.h"
#include "ble_system.h"
#include <string.h>
#include "ble_srv_common.h"
#include "boards.h"
#include "peer_manager.h"

#include "nrf_log.h"

#include "power.h"
#include "bluetooth.h"
#include "ble_system.h"

#include <nrf_delay.h>

static void on_connect (ble_sys_t *p_sys, ble_evt_t const *p_ble_evt)
{
	p_sys->conn_handle = p_ble_evt->evt.gatts_evt.conn_handle;
}


static void on_disconnect (ble_sys_t *p_sys, ble_evt_t const *p_ble_evt)
{
	UNUSED_PARAMETER(p_ble_evt);
	p_sys->conn_handle = BLE_CONN_HANDLE_INVALID;
}

static void on_write (ble_sys_t *p_sys, ble_evt_t const *p_ble_evt)
{
	ble_gatts_evt_write_t const *p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;
	extern bool dac_write_read_reg (uint8_t *buff, bool read);
	uint16_t len = p_evt_write->len;

	if (p_evt_write->handle == p_sys->state_handles.cccd_handle) {
/*		if (len == 2)
			// CCCD written, send initial notification if enabled:
			if (ble_srv_is_notification_enabled(p_evt_write->data))
				power_status_update();
*/
	} else if (p_evt_write->handle == p_sys->radio_handles.cccd_handle) {
		if (len == 2) {
			if (ble_srv_is_notification_enabled(p_evt_write->data))
				sd_ble_gap_rssi_start(p_ble_evt->evt.gap_evt.conn_handle, BLE_GAP_RSSI_THRESHOLD_INVALID, 0);
			else
				sd_ble_gap_rssi_stop(p_ble_evt->evt.gap_evt.conn_handle);
		}
	} else if (p_evt_write->handle == p_sys->power_handles.value_handle) {
		// handle writeback of wall-clock time:
		if (len == 8 && p_evt_write->data[0] == 'T') {
//			memcpy(&realtime_offset, p_evt_write->data + 2, 6);
//			realtime_offset -= last_sys_timestamp;
		// handle connection interval change request:
		} else if (len == 2 && p_evt_write->data[0] == 'B' && p_evt_write->data[1] == 'D') {
			pm_peers_delete();
		} else if (len == 2 && p_evt_write->data[0] == 'C') {
			uint16_t data = p_evt_write->data[1] + p_evt_write->data[2] * 0x100;
			extern ble_gap_conn_params_t gap_conn_params;
			gap_conn_params.min_conn_interval = data;
			gap_conn_params.max_conn_interval = data;
			sd_ble_gap_conn_param_update(p_ble_evt->evt.gap_evt.conn_handle, &gap_conn_params);
		// handle direct led color control:
		} else if (len >= 2 && p_evt_write->data[0] == 'L') {
			uint8_t led = p_evt_write->data[1];
			if (len == 3) {
				uint8_t data = p_evt_write->data[2];
				if (led == 0x01) nrf_gpio_pin_write(LED_PIN, data);
			}
		// handle reset and power off commands:
		} else if (len >= 1 && p_evt_write->data[0] == 'R') {
			nrf_gpio_pin_write(LED_PIN, 0);
			if (len == 2 && p_evt_write->data[1] == 'P') {
				NRF_GPIO->OUTCLR = 1 << RESET_PIN;
				NRF_GPIO->DIRSET = 1 << RESET_PIN;
				nrf_delay_ms(100); // give time to discharge reset capacitor
			} else if (len == 2 && p_evt_write->data[1] == 'F') {
				sd_power_gpregret_set(0, 0xB1); // Enter DFU.
			}
			NVIC_SystemReset();
		} else if (len == 2 && p_evt_write->data[0] == 'P') {
			power_off(p_evt_write->data[0] & 7);
		}
	}
}



struct conn_info conn_info_data;


bool ble_sys_send_radio (void)
{
	// compute RSSI average across channels:
	int n = 0;
	int average = 0;
	for (unsigned i = 0; i < 40; ++i)
		if (rssi[i] && ++n) average += rssi[i];
	if (n) average /= n;
	conn_info_data.rssi = average;
	return !ble_sys_send_notification(2, &conn_info_data, sizeof conn_info_data);
}

void ble_sys_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context)
{
    ble_sys_t * p_sys = (ble_sys_t *)p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_sys, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_sys, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_sys, p_ble_evt);
            break;

        case BLE_GAP_EVT_PHY_UPDATE:
			conn_info_data.ble_phy = p_ble_evt->evt.gap_evt.params.phy_update.rx_phy;
			ble_sys_send_radio();
        	break;

        case BLE_GAP_EVT_CONN_PARAM_UPDATE:
			conn_info_data.interval = p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.max_conn_interval,
			conn_info_data.latency  = p_ble_evt->evt.gap_evt.params.conn_param_update.conn_params.slave_latency,
			ble_sys_send_radio();
        	break;

        default:
            // No implementation needed.
            break;
    }
}


#define CREATE_CHAR_USER_DESC(string) {     \
	.max_size         = sizeof string - 1,  \
	.size             = sizeof string - 1,  \
   	.p_char_user_desc = (uint8_t *) string, \
   	.char_props.read  = 1,                  \
   	.read_access      = SEC_OPEN,           \
   	.is_value_user    = true                \
}

uint32_t system_hwstatus; // TODO!

uint32_t ble_sys_init (ble_sys_t *p_sys)
{
	uint32_t err_code;

	// Initialize service structure
	p_sys->conn_handle = BLE_CONN_HANDLE_INVALID;

	// Add service
	ble_uuid128_t base_uuid = {SERVICE_SYS_UUID_BASE};
	ble_uuid_t ble_uuid = {.uuid = SERVICE_SYS_UUID_SERVICE};
	err_code = sd_ble_uuid_vs_add(&base_uuid, &ble_uuid.type);
	VERIFY_SUCCESS(err_code);

	err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_sys->service_handle);
	VERIFY_SUCCESS(err_code);

    // Add system characteristics:

	ble_add_char_params_t add_char_params;

	memset(&add_char_params, 0, sizeof add_char_params);
	ble_add_char_user_desc_t ud1        = CREATE_CHAR_USER_DESC("System state");
	add_char_params.uuid                = SERVICE_SYS_UUID_CHAR1;
	add_char_params.uuid_type           = ble_uuid.type;
	add_char_params.max_len             = sizeof system_hwstatus;
	add_char_params.char_props.read     = 1;
	add_char_params.char_props.notify   = 1;
	add_char_params.p_init_value        = (uint8_t *) &system_hwstatus;
	add_char_params.init_len            = sizeof system_hwstatus;
	add_char_params.read_access         = SEC_MITM;
	add_char_params.cccd_write_access   = SEC_MITM;
	add_char_params.p_user_descr        = &ud1;

	err_code = characteristic_add(p_sys->service_handle, &add_char_params, &p_sys->state_handles);
	VERIFY_SUCCESS(err_code);

	memset(&add_char_params, 0, sizeof add_char_params);
	ble_add_char_user_desc_t ud2        = CREATE_CHAR_USER_DESC("Radio state");
	add_char_params.uuid                = SERVICE_SYS_UUID_CHAR2;
	add_char_params.uuid_type           = ble_uuid.type;
	add_char_params.max_len             = sizeof conn_info_data;
	add_char_params.char_props.read     = 1;
	add_char_params.char_props.notify   = 1;
	add_char_params.p_init_value        = (uint8_t *) &conn_info_data;
	add_char_params.init_len            = sizeof conn_info_data;
	add_char_params.read_access         = SEC_MITM;
	add_char_params.cccd_write_access   = SEC_MITM;
	add_char_params.p_user_descr        = &ud2;

	err_code = characteristic_add(p_sys->service_handle, &add_char_params, &p_sys->radio_handles);
	VERIFY_SUCCESS(err_code);

	memset(&add_char_params, 0, sizeof add_char_params);
	ble_add_char_user_desc_t ud3        = CREATE_CHAR_USER_DESC("Telemetry data");
	add_char_params.uuid                = SERVICE_SYS_UUID_CHAR3;
	add_char_params.uuid_type           = ble_uuid.type;
	add_char_params.max_len             = 20;
	add_char_params.is_var_len          = true;
	add_char_params.char_props.write    = 1;
	add_char_params.char_props.notify   = 1;
	add_char_params.write_access        = SEC_MITM;
	add_char_params.cccd_write_access   = SEC_MITM;
	add_char_params.p_user_descr        = &ud3;

	err_code = characteristic_add(p_sys->service_handle, &add_char_params, &p_sys->power_handles);
	VERIFY_SUCCESS(err_code);

	return NRF_SUCCESS;
}


uint32_t ble_sys_send_notification (int characteristic, void const *data, size_t len)
{
    uint32_t err_code;
    extern ble_sys_t *p_sys;
    ble_gatts_char_handles_t *ch = &p_sys->state_handles + (characteristic - 1);

    if (p_sys->conn_handle == BLE_CONN_HANDLE_INVALID) {
		ble_gatts_value_t val = {
			.len = len,
			.p_value = (uint8_t *) data,
		};
		err_code = sd_ble_gatts_value_set(p_sys->conn_handle, ch->value_handle, &val);
    } else {
		uint16_t hlen = len;
		ble_gatts_hvx_params_t hvx_params = {
			.handle = ch->value_handle,
			.type   = BLE_GATT_HVX_NOTIFICATION,
			.p_len  = &hlen,
			.p_data = data,
		};
		if (pkt_q_lens[BT_ID_SYS >> 28] < 6)
			err_code = sd_ble_gatts_hvx(p_sys->conn_handle, &hvx_params);
		else
			err_code = NRF_ERROR_BUSY;
		if (err_code == NRF_SUCCESS)
			bt_queued_pkt(BT_ID_SYS + characteristic);
    }
    return err_code;
}

