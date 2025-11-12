#ifndef BLE_GYRO_H__
#define BLE_GYRO_H__

#include <stdint.h>
#include <stdbool.h>

#include "ble_srv_common.h"

#ifdef __cplusplus
extern "C" {
#endif

/**@brief   Macro for defining a ble_gyro instance.
 *
 * @param   _name   Name of the instance.
 * @hideinitializer
 */
#define BLE_GYRO_DEF(_name)                                                                         \
static ble_gyro_t _name;                                                                            \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_HTS_BLE_OBSERVER_PRIO,                                                     \
                     ble_gyro_on_ble_evt, &_name)

//      SERVICE_GYRO_UUID_BASE  = UUID5(X500, "CN=gyro,CN=EMGyro2,OU=DII,O=UnivPM,L=Ancona,C=IT");
#define SERVICE_GYRO_UUID_BASE    {0xF3, 0x4E, 0x58, 0x54, 0xAC, 0xA1, 0xD9, 0x8C, 0x1E, 0x58, 0xC8, 0x74, 0x1D, 0x62, 0x78, 0x23}
#define SERVICE_GYRO_UUID_SERVICE  0x621D
#define SERVICE_GYRO_UUID_CHAR1    0x6211
#define SERVICE_GYRO_UUID_CHAR2    0x6212
#define SERVICE_GYRO_UUID_CHAR3    0x6213
#define SERVICE_GYRO_UUID_CHAR4    0x6214

// Forward declaration of the ble_gyro_t type.
typedef struct ble_gyro_s ble_gyro_t;

/**@brief Gyro Service init structure. This contains all options and data
 *        needed for initialization of the service. */
typedef struct
{
	// currently empty...
} ble_gyro_init_t;

/**@brief GYRO Service data stream packet structure. */
typedef struct ble_gyro_pkt_s
{
	uint8_t seq;
	uint8_t mode;
	uint8_t data[14];
	uint32_t time;
	uint8_t size;
} ble_gyro_pkt_t;

/**@brief Gyro Service structure. This contains various status information for
 *        the service. */
struct ble_gyro_s
{
    uint16_t                     conn_handle;                               /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    uint16_t                     service_handle;                            /**< Handle of Gyro Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t     meas_handles;                              /**< Handles related to the Gyro Measurement characteristic. */
    ble_gatts_char_handles_t     ctrl_handles;                              /**< Handles related to the Gyro Configuration characteristic. */
    //	data buffer for retransmissions:
	ble_gyro_pkt_t               buffer[256];
    uint8_t  buf_head;
    uint8_t  buf_tail;
    uint8_t  pkt_last;
    unsigned pkt_next;
};

/**@brief Function for initializing the Gyro Service.
 *
 * @param[out]  p_gyro       Gyro Service structure. This structure will have to
 *                           be supplied by the application. It will be initialized by this function,
 *                           and will later be used to identify this particular service instance.
 * @param[in]   p_gyro_init  Information needed to initialize the service.
 *
 * @return      NRF_SUCCESS on successful initialization of service, otherwise an error code.
 */
uint32_t ble_gyro_init(ble_gyro_t * p_gyro, const ble_gyro_init_t * p_gyro_init);


/**@brief Function for handling the Application's BLE Stack events.
 *
 * @details Handles all events from the BLE stack of interest to the Gyro Service.
 *
 * @param[in]   p_ble_evt   Event received from the BLE stack.
 * @param[in]   p_context   Gyro Service structure.
 */
void ble_gyro_on_ble_evt(ble_evt_t const * p_ble_evt, void * p_context);



/**@brief Functions for sending GYRO measurement if notification has been enabled. */
void ble_gyro_packet_prepare (ble_gyro_t *p_gyro); // prepare and try to send a packet
void ble_gyro_packet_retry   (ble_gyro_t *p_gyro); // try to retransmit a packet



#ifdef __cplusplus
}
#endif

#endif // BLE_GYRO_H__

/** @} */
