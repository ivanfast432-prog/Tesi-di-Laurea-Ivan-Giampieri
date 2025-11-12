#include <sdk_common.h>
#include <ble_srv_common.h>
#include <ble_err.h>
#include "ble_gyro.h"

#include "nrf_log.h"

#include "bluetooth.h"
#include "gyro.h"


/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_gyro      GYRO Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect (ble_gyro_t *p_gyro, ble_evt_t const *p_ble_evt)
{
    p_gyro->conn_handle = p_ble_evt->evt.gatts_evt.conn_handle;
    p_gyro->buf_head = 0;
    p_gyro->buf_tail = 0;
    p_gyro->pkt_last = 0xFF;
    p_gyro->pkt_next = 0;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_gyro      GYRO Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect (ble_gyro_t *p_gyro, ble_evt_t const *p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
	gyro_disable();
    p_gyro->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Function for handling the Write event.
 *
 * @param[in]   p_gyro      GYRO Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_write (ble_gyro_t *p_gyro, ble_evt_t const *p_ble_evt)
{
	ble_gatts_evt_write_t const *p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

	if (p_evt_write->handle == p_gyro->meas_handles.cccd_handle) {
	    if (p_evt_write->len == 2) { // CCCD written
	    	if (ble_srv_is_notification_enabled(p_evt_write->data)) {
	    		gyro_enable();
	    	} else {
	    		gyro_disable();
	    	}
	    }
    } else if (p_evt_write->handle == p_gyro->ctrl_handles.value_handle) {
    	// TODO: configure gyroscope!
//    	NRF_LOG_INFO("Wrote %d", p_evt_write->data[0]);
//    	NRF_LOG_FLUSH();
    }
}

void ble_gyro_on_ble_evt (ble_evt_t const *p_ble_evt, void *p_context)
{
    ble_gyro_t *p_gyro = (ble_gyro_t *) p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_gyro, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_gyro, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_gyro, p_ble_evt);
            break;

        case BLE_GATTS_EVT_HVC:
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

uint32_t ble_gyro_init (ble_gyro_t *p_gyro, ble_gyro_init_t const *p_gyro_init)
{
    uint32_t err_code;

    // Initialize service structure
    p_gyro->conn_handle = BLE_CONN_HANDLE_INVALID;

    // Add service
    ble_uuid128_t base_uuid = {SERVICE_GYRO_UUID_BASE};
    ble_uuid_t ble_uuid = {.uuid = SERVICE_GYRO_UUID_SERVICE};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &ble_uuid.type);
    VERIFY_SUCCESS(err_code);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_gyro->service_handle);
    VERIFY_SUCCESS(err_code);


    // Add measurement characteristics:

    ble_add_char_params_t add_char_params;

    ble_add_char_user_desc_t ud = CREATE_CHAR_USER_DESC("Gyroscope data stream");

    memset(&add_char_params, 0, sizeof(add_char_params));

    add_char_params.uuid                = SERVICE_GYRO_UUID_CHAR1;
	add_char_params.uuid_type           = ble_uuid.type;
    add_char_params.max_len             = 20;
    add_char_params.is_var_len          = true;
    add_char_params.char_props.notify   = 1;
    add_char_params.cccd_write_access   = SEC_MITM;
    add_char_params.p_user_descr        = &ud;

    err_code = characteristic_add(p_gyro->service_handle, &add_char_params, &p_gyro->meas_handles);
    VERIFY_SUCCESS(err_code);

	memset(&add_char_params, 0, sizeof(add_char_params));

	add_char_params.uuid                = SERVICE_GYRO_UUID_CHAR2;
	add_char_params.uuid_type           = ble_uuid.type;
	add_char_params.max_len             = 4;
	add_char_params.char_props.write    = 1;
	add_char_params.write_access        = SEC_MITM;

	err_code = characteristic_add(p_gyro->service_handle, &add_char_params, &p_gyro->ctrl_handles);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}


static bool ble_gyro_packet_send (ble_gyro_t *p_gyro, uint8_t pos)
{
	// try to send value only if connected:
    if (p_gyro->conn_handle == BLE_CONN_HANDLE_INVALID) return false;

    ble_gyro_pkt_t *pkt = p_gyro->buffer + pos;
	uint16_t len = pkt->size;

	ble_gatts_hvx_params_t hvx_params;
	memset(&hvx_params, 0, sizeof(hvx_params));

	hvx_params.handle = p_gyro->meas_handles.value_handle;
	hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
	hvx_params.offset = 0;
	hvx_params.p_len  = &len;
	hvx_params.p_data = (uint8_t const *) (&pkt->seq);

    uint32_t err_code;
	if (pkt_q_lens[2] < 3) {
		err_code = sd_ble_gatts_hvx(p_gyro->conn_handle, &hvx_params);
	} else {
		err_code = NRF_ERROR_BUSY;
	}

	if (err_code == NRF_SUCCESS) {
		bt_queued_pkt(pkt->seq + BT_ID_GYR);
		return true;
	} else {
		return false;
	}

}

void ble_gyro_packet_prepare (ble_gyro_t *p_gyro)
{
	unsigned const index = p_gyro->pkt_next++;
	uint8_t   const seq = index & 0xFF;
	uint8_t   const pos = p_gyro->buf_tail;
	ble_gyro_pkt_t *pkt = p_gyro->buffer + pos;
	pkt->seq = seq;
	pkt->mode = 0;
	gyro_sample_t *p = gyro_buf + gyro_buf_head++;
	gyro_buf_head &= GYRO_BUF_IDX_MASK;
	memcpy(pkt->data, p->data, 14);
	pkt->size = 16;
	pkt->time = p->timestamp;

	if ((seq & 0x0F) == 0x00) {
		pkt->size += 1;
		pkt->time = 0x46; // TODO: config word!
	}
	if ((seq & 0x0F) == 0x02) {
		pkt->size += 4;
		// time already in!
	}

	if (p_gyro->pkt_last != seq && ble_gyro_packet_send(p_gyro, pos)) {
		p_gyro->pkt_last = seq;
	} else {
		pkt->seq <<= 1;
		pkt->seq |= 0x01;
		pkt->time = (uint8_t) (index >> 7) + 256 * (uint8_t) (p_gyro->buf_tail - p_gyro->buf_head);
		pkt->size = 18;
		if (++p_gyro->buf_tail == p_gyro->buf_head) ++p_gyro->buf_head;
	}

}

void ble_gyro_packet_retry (ble_gyro_t *p_gyro)
{
	if (p_gyro->buf_tail == p_gyro->buf_head) return;
	if (ble_gyro_packet_send(p_gyro, p_gyro->buf_head)) ++p_gyro->buf_head;
}

