#ifndef BLE_EMG_H__
#define BLE_EMG_H__

#include <stdint.h>
#include <stdbool.h>
#include <ble.h>
#include <ble_srv_common.h>
#include <nrf_sdh_ble.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BLE_EMG_DEF(_name)                                                                          \
static ble_emg_t _name;                                                                             \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_BAS_BLE_OBSERVER_PRIO,                                                     \
                     ble_emg_on_ble_evt, &_name)

//      SERVICE_EMG_UUID_BASE  = UUID5(X500, "CN=EMG,CN=EMGyro2,OU=DII,O=UnivPM,L=Ancona,C=IT");
#define SERVICE_EMG_UUID_BASE    {0x0F, 0xD2, 0x9B, 0x77, 0x1A, 0xF4, 0xB4, 0x9A, 0xBE, 0x5A, 0x79, 0x41, 0xFF, 0x2F, 0xF9, 0xCE}
#define SERVICE_EMG_UUID_SERVICE  0x2FFF
#define SERVICE_EMG_UUID_CHAR1    0x2FF1
#define SERVICE_EMG_UUID_CHAR2    0x2FF2
#define SERVICE_EMG_UUID_CHAR3    0x2FF3
#define SERVICE_EMG_UUID_CHAR4    0x2FF4

#define BLE_EMG_MAX_LENGTH 20

// Forward declaration of the ble_emg_t type.
typedef struct ble_emg_s ble_emg_t;

/**@brief EMG Service initialization structure. */
typedef struct
{
	security_req_t               emg_cccd_wr_sec;                           /**< Security requirement for writing EMG measurement characteristic CCCD. */
	security_req_t               emg_conf_wr_sec;                           /**< Security requirement for writing EMG configuration characteristic. */
} ble_emg_init_t;


/**@brief EMG Service data stream packet structure. */
typedef struct ble_emg_pkt_s
{
	uint8_t size;
	uint8_t seq;
	uint8_t data[19];
} ble_emg_pkt_t;

/**@brief EMG Service status structure. */
struct ble_emg_s
{
    uint16_t                     conn_handle;                               /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    uint16_t                     service_handle;                            /**< Handle of EMG Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t     meas_handles;                              /**< Handles related to the EMG measurement characteristic. */
    ble_gatts_char_handles_t     ctrl_handles;                              /**< Handles related to the EMG configuration characteristic. */
//	data buffer for retransmissions:
    ble_emg_pkt_t                buffer[256]; // contains failed packets for retransmissions
    uint8_t  buf_head;
    uint8_t  buf_tail;
    uint8_t  pkt_last;
    unsigned pkt_next;
};

extern uint32_t emg_ts; // TX timestamp of pkt #0.
extern bool emg_ts_valid;

/**@brief Function for initializing the EMG Service. */
uint32_t ble_emg_init (ble_emg_t *p_emg, const ble_emg_init_t *p_emg_init);


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the EMG Service.
 *
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 * @param[in]   p_context   EMG Service structure.
 */
void ble_emg_on_ble_evt (ble_evt_t const *p_ble_evt, void *p_context);


/**@brief Functions for sending EMG measurement if notification has been enabled. */
void ble_emg_packet_prepare (ble_emg_t *p_emg); // prepare and try to send a packet
void ble_emg_packet_retry   (ble_emg_t *p_emg); // try to retransmit a packet


#ifdef __cplusplus
}
#endif

#endif // BLE_EMG_H__

/** @} */
