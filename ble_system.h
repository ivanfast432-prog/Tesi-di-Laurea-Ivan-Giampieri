#ifndef BLE_SYS_H__
#define BLE_SYS_H__

#include <stdint.h>
#include <stdbool.h>
#include <ble.h>
#include <ble_srv_common.h>
#include <nrf_sdh_ble.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BLE_SYS_DEF(_name)                                                                          \
static ble_sys_t _name;                                                                             \
NRF_SDH_BLE_OBSERVER(_name ## _obs,                                                                 \
                     BLE_BAS_BLE_OBSERVER_PRIO,                                                     \
                     ble_sys_on_ble_evt, &_name)

//      SERVICE_SYS_UUID_BASE  = UUID5(X500, "CN=system,CN=EMGyro2,OU=DII,O=UnivPM,L=Ancona,C=IT");
#define SERVICE_SYS_UUID_BASE    {0xE7, 0xF0, 0x0F, 0xBA, 0x02, 0x04, 0x32, 0xB7, 0x75, 0x5D, 0xE3, 0xBF, 0x36, 0x3A, 0x7F, 0xB7}
#define SERVICE_SYS_UUID_SERVICE  0x3A36
#define SERVICE_SYS_UUID_CHAR1    0x3A31
#define SERVICE_SYS_UUID_CHAR2    0x3A32
#define SERVICE_SYS_UUID_CHAR3    0x3A33

// Forward declaration of the ble_sys_t type.
typedef struct ble_sys_s ble_sys_t;

/**@brief SYS Service status structure. */
struct ble_sys_s
{
    uint16_t                     conn_handle;                               /**< Handle of the current connection (as provided by the BLE stack, is BLE_CONN_HANDLE_INVALID if not in a connection). */
    uint16_t                     service_handle;                            /**< Handle of SYS Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t     state_handles;                             /**< Handles related to the STATE  characteristic. */
    ble_gatts_char_handles_t     radio_handles;                             /**< Handles related to the RADIO  characteristic. */
    ble_gatts_char_handles_t     power_handles;                             /**< Handles related to the POWER  characteristic. */
};

/**@brief Function for initializing the SYSTEM Service. */
uint32_t ble_sys_init (ble_sys_t *p_sys);

/**@brief Function for handling the Application's BLE Stack events. */
void ble_sys_on_ble_evt (ble_evt_t const *p_ble_evt, void *p_context);

bool ble_sys_send_radio (void);
uint32_t ble_sys_send_notification (int characteristic, void const *data, size_t len);

#ifdef __cplusplus
}
#endif

#endif // BLE_SYS_H__

/** @} */
