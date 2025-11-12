#include <sdk_common.h>
#include <ble_srv_common.h>
#include <ble_err.h>
#include "ble_emg.h"

#include <nrf_log.h>
#include <nrf_soc.h> // to start HF clock

#include "bluetooth.h"
#include "emg.h"

uint32_t emg_ts;
bool emg_ts_valid;


#include "FLAC/stream_encoder.h"

#define            databuf_blocksize  200
#define            databuf_stride    (databuf_blocksize * 3)
static unsigned    databuf_index; // index within buffer
static unsigned    databuf_sel;   // double buffer selector
static FLAC__int32 databuf_data[databuf_stride * 2];



extern ble_emg_t * const ble_emg_pointer;

static void encoded_output (void const *buffer, uint16_t len)
{
	ble_emg_t * const p_emg = ble_emg_pointer;
	unsigned const index = p_emg->pkt_next++;
	uint8_t  const seq = index & 0xFF;
	uint8_t  const pos = p_emg->buf_tail;
	ble_emg_pkt_t *pkt = p_emg->buffer + pos;
	pkt->seq = seq;
	memcpy(pkt->data, buffer, len);
	pkt->size = len;
	if (++p_emg->buf_tail == p_emg->buf_head) ++p_emg->buf_head;
	ble_emg_packet_retry(p_emg);
}


static FLAC__StreamEncoderWriteStatus flac_callback (const FLAC__StreamEncoder *encoder, const FLAC__byte buffer[], size_t bytes, long unsigned samples, long unsigned current_frame, void *client_data)
{
	size_t sent = 0;
	NRF_LOG_INFO("FLAC write frame %u size %u", current_frame, bytes);
	while (sent < bytes) {
		uint16_t rem = bytes - sent;
		if (rem > 18) rem = 18;
		encoded_output(buffer + sent, rem);
		sent += rem;
	}
	return FLAC__STREAM_ENCODER_WRITE_STATUS_OK;
}

FLAC__StreamEncoder *encoder = 0;

static int flac_init (unsigned sample_rate, unsigned channels)
{
	databuf_index = 0;
	databuf_sel = 0;

	encoder = FLAC__stream_encoder_new();
	if (!encoder) return -1;

	FLAC__bool ok = true;
	ok &= FLAC__stream_encoder_set_compression_level(encoder, emg_compression_mode & 7);
	ok &= FLAC__stream_encoder_set_blocksize(encoder, databuf_blocksize);
	ok &= FLAC__stream_encoder_set_channels(encoder, channels);
	ok &= FLAC__stream_encoder_set_bits_per_sample(encoder, emg_compression_mode < 8 ? 24 : 20);
	ok &= FLAC__stream_encoder_set_sample_rate(encoder, sample_rate);

	if (!ok) goto cleanup;

	FLAC__StreamEncoderInitStatus init_status = FLAC__stream_encoder_init_stream(encoder, flac_callback, NULL, NULL, NULL, NULL);
	if (init_status != FLAC__STREAM_ENCODER_INIT_STATUS_OK) goto cleanup;
	return 0;

cleanup:
	FLAC__stream_encoder_delete(encoder);
	encoder = NULL;
	return -1;
}

static void flac_stop (void)
{
	if (!encoder) return;
	FLAC__stream_encoder_delete(encoder);
	encoder = NULL;
}

static int flac_test (void)
{
	const int count = 1;
	int left = count;

	FLAC__stream_encoder_set_buffer(encoder, databuf_data);

	NRF_TIMER1->TASKS_CAPTURE[2] = 1;
	uint32_t start_timestamp = NRF_TIMER1->CC[2];

	while( left--)
		FLAC__stream_encoder_process(encoder);

	NRF_TIMER1->TASKS_CAPTURE[2] = 1;
	uint32_t end_timestamp = NRF_TIMER1->CC[2];

	NRF_LOG_INFO("FLAC x %d time: %u", count, end_timestamp - start_timestamp);

	return 0;
}

void VLDE__encode (int32_t const *buffer)
{
	static uint8_t out[64];
	static uint8_t idx = 0;
	int32_t const *p[3] = {
			buffer + databuf_blocksize * 0,
			buffer + databuf_blocksize * 1,
			buffer + databuf_blocksize * 2,
	};
	static int32_t prevs[3] = {};
	for (int i = 0; i < databuf_index; ++i) {
		for (int c = 0; c < emg_channels_per_sample; ++c) {
			int32_t x = p[c][i];
			int32_t d = x - prevs[c];
			prevs[c] = x;
			uint32_t u = d < 0 ? -d : d;
			if (u < 0x40) {
				// 0SDDDDDD
				out[idx++] =  (d >>  0) & 0x7F;
			} else if (u < 0x2000) {
				// 10SDDDDD DDDDDDDD
				out[idx++] = ((d >>  8) & 0x3F) | 0x80;
				out[idx++] =  (d >>  0) & 0xFF;
			} else if (u < 0x200000) {
				// 11SDDDDD DDDDDDDD DDDDDDDD
				out[idx++] = ((d >> 16) & 0x3F) | 0xC0;
				out[idx++] =  (d >>  8) & 0xFF;
				out[idx++] =  (d >>  0) & 0xFF;
			} else {
				//TODO!
			}
			if (idx >= 18) {
				encoded_output(out, 18);
				idx -= 18;
				memcpy(out, out + 18, idx);
			}
		}
	}
}

/*
const float sense_matrix[databuf_blocksize / 2][databuf_blocksize] = {
#include "matrix.h"
};

void RBCS__encode (int32_t const *buffer)
{
	float out[4];
	for (int c = 0; c < emg_channels_per_sample; ++c) {
		int32_t const *p = buffer + databuf_blocksize * c;
		for (int j = 0; j < databuf_blocksize / 2; ++j) {
			float a = 0;
			for (int i = 0; i < databuf_blocksize; ++i)
				a += sense_matrix[j][i] * p[i];
			out[j % 4] = a;
			if (j % 4 == 3) encoded_output(out, sizeof out);
		}
	}
}
*/

/**@brief Function for handling the Connect event.
 *
 * @param[in]   p_emg       EMG Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_connect (ble_emg_t *p_emg, ble_evt_t const *p_ble_evt)
{
    p_emg->conn_handle = p_ble_evt->evt.gatts_evt.conn_handle;
    p_emg->buf_head = 0;
    p_emg->buf_tail = 0;
    p_emg->pkt_last = 0xFF;
    p_emg->pkt_next = 0;
    emg_ts_valid = false;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in]   p_emg       EMG Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
static void on_disconnect (ble_emg_t *p_emg, ble_evt_t const *p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_emg->conn_handle = BLE_CONN_HANDLE_INVALID;
    flac_stop();
}


/**@brief Function for handling the Write event.
 *
 * @param[in]   p_emg       Health Thermometer Service structure.
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 */
extern uint8_t emg_online_reconf_data[32];
extern uint8_t emg_online_reconf_size;
static uint32_t emg_cfgword = 0x2E1C0A34; // NOTE: keep in sync with default in emg.c!

static void on_write (ble_emg_t *p_emg, ble_evt_t const *p_ble_evt)
{
	ble_gatts_evt_write_t const *p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

	if (p_evt_write->handle == p_emg->meas_handles.cccd_handle) {
		// CCCD written, update state
		if (p_evt_write->len == 2) {
			if (ble_srv_is_notification_enabled(p_evt_write->data)) {
				sd_clock_hfclk_request();
	            emg_enable();
			} else {
	            emg_disable();
				sd_clock_hfclk_release();
			}
		}
    } else if (p_evt_write->handle == p_emg->ctrl_handles.value_handle) {
    	if (p_evt_write->len == 1 && p_evt_write->data[0] == 0x00) {
    		ble_emg_packet_retry(p_emg);
    	}
    	if (p_evt_write->len == 1 && p_evt_write->data[0] == 0xFC) {
    		flac_test();
    	}
    	// check for a direct SPI command (0xDC <raw SPI data>):
    	if (p_evt_write->len > 1 && p_evt_write->data[0] == 0xDC && p_evt_write->len - 1 <= sizeof emg_online_reconf_data) {
    		memcpy(emg_online_reconf_data, p_evt_write->data + 1, p_evt_write->len - 1);
    		emg_online_reconf_size = p_evt_write->len - 1;
    	}
    	// check for a channel configuration command (0xCC <SR> <NS> <CH1> <CH2> <CH3>):
    	if (p_evt_write->len == 6 && p_evt_write->data[0] == 0xCC) {
    		int rate = p_evt_write->data[1] * 100; // sample  rate
    		int size = p_evt_write->data[2]; // samples per packet [ >= 0x10 means compressed data ]
    		uint32_t ch_config = p_evt_write->data[3] + p_evt_write->data[4] * 0x100 + p_evt_write->data[5] * 0x10000;
    		emg_configure(rate,	size, ch_config);
    		/*	BYTE 0:   // ADC configuration:
    		 *		reserved     : 2; // set to zero for now
    		 *		num_channels : 2; // 1 ÷ 3
    		 *		odr_index    : 4; // 0 ÷ 7 => 50 100 200 400 800 1600 3200 6400
    		 *	BYTE 1÷3: // EMG channels 1÷3 definitions as follows:
    		 *		test_mode    : 2; // 0 ÷ 3 => normal pos_test neg_test zero_test
    		 *		pos_channel  : 3; // 0 => OFF, 1÷6 => electrode number, 7 => battery
    		 *		neg_channel  : 3; // 0 => OFF, 1÷6 => electrode number, 7 => battery
    		 */
    		emg_cfgword = 0xC0;
    		switch (rate) {
    			case   50: emg_cfgword = 0; break;
    			case  100: emg_cfgword = 1; break;
    			case  200: emg_cfgword = 2; break;
    			case  400: emg_cfgword = 3; break;
    			case  800: emg_cfgword = 4; break;
    			case 1600: emg_cfgword = 5; break;
    			case 3200: emg_cfgword = 6; break;
    			case 6400: emg_cfgword = 7; break;
    		}
    		int channels = !!(p_evt_write->data[3]) + !!(p_evt_write->data[4]) + !!(p_evt_write->data[5]);
    		emg_cfgword |= channels << 4;
    		emg_cfgword |= ch_config << 8;
    		if (emg_compression_type == 1) flac_init(rate, channels);
    	}
    }
}


void ble_emg_on_ble_evt (ble_evt_t const *p_ble_evt, void *p_context)
{
    ble_emg_t *p_emg = (ble_emg_t *) p_context;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_emg, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_emg, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_emg, p_ble_evt);
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

uint32_t ble_emg_init (ble_emg_t *p_emg, ble_emg_init_t const *p_emg_init)
{
	uint32_t err_code;

	// Initialize service structure
	p_emg->conn_handle = BLE_CONN_HANDLE_INVALID;

	// Add service
	ble_uuid128_t base_uuid = {SERVICE_EMG_UUID_BASE};
	ble_uuid_t ble_uuid = {.uuid = SERVICE_EMG_UUID_SERVICE};
	err_code = sd_ble_uuid_vs_add(&base_uuid, &ble_uuid.type);
	VERIFY_SUCCESS(err_code);

	err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_emg->service_handle);
	VERIFY_SUCCESS(err_code);


	// Add measurement characteristics:

	ble_add_char_params_t add_char_params;

	ble_add_char_user_desc_t ud = CREATE_CHAR_USER_DESC("Electromyography data stream");

	memset(&add_char_params, 0, sizeof(add_char_params));

	add_char_params.uuid                = SERVICE_EMG_UUID_CHAR1;
	add_char_params.uuid_type           = ble_uuid.type;
	add_char_params.max_len             = BLE_EMG_MAX_LENGTH;
	add_char_params.is_var_len          = true;
	add_char_params.char_props.notify   = 1;
	add_char_params.cccd_write_access   = SEC_MITM; 
	add_char_params.p_user_descr        = &ud;

	err_code = characteristic_add(p_emg->service_handle, &add_char_params, &p_emg->meas_handles);
	VERIFY_SUCCESS(err_code);

	memset(&add_char_params, 0, sizeof(add_char_params));

	add_char_params.uuid                = SERVICE_EMG_UUID_CHAR2;
	add_char_params.uuid_type           = ble_uuid.type;
	add_char_params.max_len             = 20;
	add_char_params.char_props.write    = 1;
	add_char_params.write_access        = SEC_MITM;

	err_code = characteristic_add(p_emg->service_handle, &add_char_params, &p_emg->ctrl_handles);
	VERIFY_SUCCESS(err_code);

	return NRF_SUCCESS;
}


static bool ble_emg_packet_send (ble_emg_t *p_emg, uint8_t pos)
{
    // try to send value only if connected:
    if (p_emg->conn_handle == BLE_CONN_HANDLE_INVALID) return false;

    ble_emg_pkt_t *pkt = p_emg->buffer + pos;
	uint16_t len = pkt->size + 1;

	ble_gatts_hvx_params_t hvx_params;
	memset(&hvx_params, 0, sizeof hvx_params);

	hvx_params.handle = p_emg->meas_handles.value_handle;
	hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
	hvx_params.offset = 0;
	hvx_params.p_len  = &len;
	hvx_params.p_data = (uint8_t const *) (&pkt->seq);

    uint32_t err_code;
	if (pkt_q_lens[1] < 6) {
		err_code = sd_ble_gatts_hvx(p_emg->conn_handle, &hvx_params);
	} else {
		err_code = NRF_ERROR_BUSY;
	}

	if (err_code == NRF_SUCCESS) {
		bt_queued_pkt(pkt->seq + BT_ID_EMG);
		return true;
	} else {
		return false;
	}

}

static uint32_t emg_0_ts;

static inline int32_t emg_adc_convert_24 (uint8_t const *data)
{
	return (int32_t) ((data[0] << 16) + (data[1] << 8) + (data[2] >> 0)) - 0x400000;
}

static inline int32_t emg_adc_convert_20 (uint8_t const *data)
{
	return (int32_t) ((data[0] << 12) + (data[1] << 4) + (data[2] >> 4)) - 0x40000;
}

void ble_emg_packet_prepare (ble_emg_t *p_emg)
{
	if (!emg_samples_per_packet) {
		// Do not stream data - only for baseline power profiling
		emg_buf_head = emg_buf_tail;
		return;
	}
	if (emg_compression_type) {
		// buffer data:
		int32_t *buffer = databuf_data + databuf_sel * databuf_stride;
		for (
			int32_t *target = buffer + databuf_index;
			emg_buf_head != emg_buf_tail &&
			databuf_index++ < databuf_blocksize;
			++target
		) {
			emg_sample_t *p = emg_buf + emg_buf_head++;
			emg_buf_head &= EMG_BUF_IDX_MASK;
			if (emg_compression_mode >= 8) {
				target[0 * databuf_blocksize] = emg_adc_convert_20(p->data + 0);
				target[1 * databuf_blocksize] = emg_adc_convert_20(p->data + 3);
				target[2 * databuf_blocksize] = emg_adc_convert_20(p->data + 6);
			} else {
				target[0 * databuf_blocksize] = emg_adc_convert_24(p->data + 0);
				target[1 * databuf_blocksize] = emg_adc_convert_24(p->data + 3);
				target[2 * databuf_blocksize] = emg_adc_convert_24(p->data + 6);
			}
		}
		if (emg_compression_type == 2) {
			VLDE__encode(buffer);
			databuf_index = 0;
			databuf_sel ^= 1;
			return;
		}
		if (databuf_index == databuf_blocksize) {
			databuf_index = 0;
			databuf_sel ^= 1;
			if (emg_compression_type == 1) {
				FLAC__stream_encoder_set_buffer(encoder, buffer);
				FLAC__stream_encoder_process(encoder);
			}
//			if (emg_compression_type == 3) {
//				RBCS__encode(buffer);
//			}
		}
		return;
	}
	// otherwise just stream raw data:
	unsigned const sample_size = emg_channels_per_sample * 3;
	unsigned const index = p_emg->pkt_next++;
	uint8_t  const seq = index & 0xFF;
	uint8_t  const pos = p_emg->buf_tail;
	ble_emg_pkt_t *pkt = p_emg->buffer + pos;
	pkt->seq = seq;
	unsigned size = 0;
	for (
		unsigned count = 0;
		emg_buf_head != emg_buf_tail &&
		size <= sizeof p_emg->buffer->data - sample_size &&
		count++ < emg_samples_per_packet;
		size += sample_size
	) {
		emg_sample_t *p = emg_buf + emg_buf_head++;
		emg_buf_head &= EMG_BUF_IDX_MASK;
		if (!seq && !size) emg_0_ts = p->timestamp;
		memcpy(pkt->data + size, p->data, sample_size);
	}
	int8_t subcnt = -1;
	if ((seq & 0xFC) == 0x00) {
		subcnt = seq & 0x03;
		pkt->data[size++] = ((uint8_t *) &emg_cfgword)[subcnt];
	}
	if ((seq & 0xFC) == 0x08 && emg_ts_valid) {
		subcnt = seq & 0x03;
		pkt->data[size++] = ((uint8_t *) &emg_0_ts)[subcnt];
	}
	if ((seq & 0xFC) == 0x10 && emg_ts_valid) {
		subcnt = seq & 0x03;
		pkt->data[size++] = ((uint8_t *) &emg_ts)[subcnt];
		if (subcnt == 3) emg_ts_valid = false;
	}
	pkt->size = size;

	if (p_emg->pkt_last != seq && ble_emg_packet_send(p_emg, pos)) {
		p_emg->pkt_last = seq;
	} else {
		pkt->seq |= 0x80;
		if (subcnt < 0)
			++pkt->size;
		else
			--size;
		pkt->data[size++] = index >> 7;
		if (++p_emg->buf_tail == p_emg->buf_head) ++p_emg->buf_head;
	}
}

void ble_emg_packet_retry (ble_emg_t *p_emg)
{
	if (p_emg->buf_tail == p_emg->buf_head) return;
	if (ble_emg_packet_send(p_emg, p_emg->buf_head)) ++p_emg->buf_head;
}
