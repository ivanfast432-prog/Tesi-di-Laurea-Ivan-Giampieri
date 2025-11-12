/**
 * Copyright (c) 2014 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup ble_sdk_app_template_main main.c
 * @{
 * @ingroup ble_sdk_app_template
 * @brief Template project main file.
 *
 * This file contains a template for creating a new application. It has the code necessary to wakeup
 * from button, advertise, get a connection restart advertising on disconnect and if no new
 * connection created go back to system-off mode.
 * It can easily be used as a starting point for creating a new application, the comments identified
 * with 'YOUR_JOB' indicates where and how you can customize.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "boards.h"

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdm.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "fds.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"

#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "app_timer.h"
#include "app_scheduler.h"

#include "ble_bas.h"
#include "ble_dis.h"

#include "ble_emg.h"
#include "ble_gyro.h"
#include "ble_system.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "emg.h"
#include "gyro.h"

#include "bluetooth.h"
#include "usb.h"
void show_softdevice_info(void);
#define PASSKEY_LENGTH 6
#define PASSKEY_TXT    "Passkey:"
static ble_gap_sec_params_t m_sec_params;
#include "peer_manager.h"
#include "peer_manager_types.h"
#include "ble_gap.h"


#define DEVICE_NAME                     "EMGyro2"                               /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME               "DII@UnivPM"                            /**< Manufacturer. Will be passed to Device Information Service. */

#define APP_BLE_OBSERVER_PRIO           3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG            1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(7.5, UNIT_1_25_MS)          /**< Minimum acceptable connection interval (7.5 ms). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(7.5, UNIT_1_25_MS)          /**< Maximum acceptable connection interval (7.5 ms). */
#define SLAVE_LATENCY                   32                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(1000)                   /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(2000)                   /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

//#define SEC_PARAM_BOND                  1                                       /**< Perform bonding. */
//#define SEC_PARAM_MITM                  0                                       /**< Man In The Middle protection not required. */
//#define SEC_PARAM_LESC                  0                                       /**< LE Secure Connections not enabled. */
//#define SEC_PARAM_KEYPRESS              0                                       /**< Keypress notifications not enabled. */
//#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
//#define SEC_PARAM_OOB                   0                                       /**< Out Of Band data not available. */
//#define SEC_PARAM_MIN_KEY_SIZE          7                                       /**< Minimum encryption key size. */
//#define SEC_PARAM_MAX_KEY_SIZE          16                                      /**< Maximum encryption key size. */

#define SEC_PARAM_BOND            1   // bonding
#define SEC_PARAM_MITM            1   // MITM protection
#define SEC_PARAM_LESC            0   // LE Secure Connections
#define SEC_PARAM_KEYPRESS        0   // keypress notifica disabilitata
#define SEC_PARAM_IO_CAPABILITIES BLE_GAP_IO_CAPS_DISPLAY_ONLY // display only
#define SEC_PARAM_OOB             0   // no OOB
#define SEC_PARAM_MIN_KEY_SIZE    7
#define SEC_PARAM_MAX_KEY_SIZE    16

#define DEAD_BEEF                       0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */


NRF_BLE_GATT_DEF(m_gatt);                                                       /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                         /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                             /**< Advertising module instance. */

BLE_BAS_DEF(m_bas);
BLE_EMG_DEF(m_emg);
BLE_GYRO_DEF(m_gyro);
BLE_SYS_DEF(m_system);

ble_emg_t * const ble_emg_pointer = &m_emg;
ble_sys_t * const p_sys = &m_system;

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;                        /**< Handle of the current connection. */
static bool m_advertising_active = false;

// YOUR_JOB: Use UUIDs for service(s) used in your application.
static ble_uuid_t m_adv_uuids[] =                                               /**< Universally unique service identifiers. */
{
    {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE},
//    {CUSTOM_SERVICE_UUID, BLE_UUID_TYPE_VENDOR_BEGIN}
};

static  int8_t rssi_power;
static uint8_t rssi_channel;
int8_t rssi[40];     // per-channel RSSI value

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num   Line number of the failing ASSERT call.
 * @param[in] file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            break;

        default:
            break;
    }
}


ble_gap_conn_params_t   gap_conn_params;

/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    /* YOUR_JOB: Use an appearance value matching the application's use case.
       err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_);
       APP_ERROR_CHECK(err_code); */

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for handling the YYY Service events.
 * YOUR_JOB implement a service handler function depending on the event the service you are using can generate
 *
 * @details This function will be called for all YY Service events which are passed to
 *          the application.
 *
 * @param[in]   p_yy_service   YY Service structure.
 * @param[in]   p_evt          Event received from the YY Service.
 *
 *
static void on_yys_evt(ble_yy_service_t     * p_yy_service,
                       ble_yy_service_evt_t * p_evt)
{
    switch (p_evt->evt_type)
    {
        case BLE_YY_NAME_EVT_WRITE:
            APPL_LOG("[APPL]: charact written with value %s. ", p_evt->params.char_xx.value.p_str);
            break;

        default:
            // No implementation needed.
            break;
    }
}
*/

/**@brief Function for initializing services that will be used by the application.
 */
static void services_init (void)
{
	ret_code_t err_code;

	// Initialize Queued Write Module.

	nrf_ble_qwr_init_t qwr_init;
	memset(&qwr_init, 0, sizeof qwr_init);

	qwr_init.error_handler = nrf_qwr_error_handler;

	err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
	APP_ERROR_CHECK(err_code);

    // Initialize Battery Service.

    ble_bas_init_t bas_init;
    memset(&bas_init, 0, sizeof bas_init);

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 0;
    bas_init.bl_rd_sec        = SEC_OPEN;
    bas_init.bl_cccd_wr_sec   = SEC_OPEN;
    bas_init.bl_report_rd_sec = SEC_OPEN;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Device Information Service.

    ble_dis_init_t dis_init;
	memset(&dis_init, 0, sizeof dis_init);

	ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)MANUFACTURER_NAME);
	dis_init.dis_char_rd_sec = SEC_OPEN;

	err_code = ble_dis_init(&dis_init);
	APP_ERROR_CHECK(err_code);

	// Initialize EMG Service:

	ble_emg_init_t emg_init;
	memset(&emg_init, 0, sizeof emg_init);

	emg_init.emg_conf_wr_sec = SEC_OPEN;
	emg_init.emg_cccd_wr_sec = SEC_OPEN;

	err_code = ble_emg_init(&m_emg, &emg_init);
	APP_ERROR_CHECK(err_code);

	// Initialize Gyro Service:

	ble_gyro_init_t gyro_init;
	memset(&gyro_init, 0, sizeof gyro_init);

	//gyro_init.emg_conf_wr_sec = SEC_OPEN;
	//gyro_init.emg_cccd_wr_sec = SEC_OPEN;

	err_code = ble_gyro_init(&m_gyro, &gyro_init);
	APP_ERROR_CHECK(err_code);

	// Initialize System Service:
	err_code = ble_sys_init(&m_system);
	APP_ERROR_CHECK(err_code);

}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
	if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED) {
//		ret_code_t err_code;
//		nrf_gpio_pin_set(LED_PIN);
//		err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
//		APP_ERROR_CHECK(err_code);
	}
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
#include <nrfx_power.h>
static void sleep_mode_enter (void)
{
	// TODO: Prepare wakeup buttons.
	nrf_gpio_pin_write(LED_PIN, 0);
	gyro_wakeup_enable();
	nrf_gpio_pin_write(LED_PIN, 0);

	// Go to system-off mode (this function will not return; wakeup will cause a reset).
	sd_power_system_off();
	// If the function returns, we have a debug probe attached that is preventing system off mode.
	nrf_gpio_pin_write(LED_PIN, 1);
	sd_softdevice_disable();
	app_timer_stop_all();
	nrf_pwr_mgmt_run();
	NVIC_SystemReset();
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
	switch (ble_adv_evt) {
		case BLE_ADV_EVT_FAST:
			NRF_LOG_INFO("Fast advertising.");
			break;
		case BLE_ADV_EVT_SLOW:
			NRF_LOG_INFO("Slow advertising.");
			break;
		case BLE_ADV_EVT_IDLE:
			if (m_advertising_active) {
				uint32_t usb_status;
				sd_power_usbregstatus_get(&usb_status);
				if (usb_status & NRF_POWER_USBREGSTATUS_VBUSDETECT_MASK) {
					ble_advertising_start(&m_advertising, BLE_ADV_EVT_SLOW);
					break;
				}
			}
			// check if we have debug probe attached:
			if (CoreDebug->DHCSR & CoreDebug_DHCSR_C_DEBUGEN_Msk)
				ble_advertising_start(&m_advertising, BLE_ADV_EVT_SLOW);
			else
				sleep_mode_enter();
			break;
		default:
			break;
	}
}

static uint32_t pkt_queue[256];
static int pkt_head, pkt_tail;
static uint8_t top_len = 16;
int pkt_q_lens[8];

void bt_queued_pkt (uint32_t id)
{
	++pkt_q_lens[(id >> 28) & 0x07];
	pkt_queue[pkt_head++] = id;
	pkt_head &= 0xFF;
	uint8_t len = pkt_head - pkt_tail;
	if (len > top_len) {
		top_len = len;
		NRF_LOG_INFO("pkt_queue max_len = %d", top_len);
	}
}

static uint32_t bt_dequeue_pkt (uint8_t count)
{
	uint32_t id = 0;
	while (count--) {
		if (pkt_head == pkt_tail) {
			NRF_LOG_ERROR("pkt_queue under-run");
			return 0;
		}
		id = pkt_queue[pkt_tail++];
		pkt_tail &= 0xFF;
		--pkt_q_lens[(id >> 28) & 0x07];
	}
	return id;
}

unsigned bt_queue_len (void)
{
	uint8_t len = pkt_head - pkt_tail;
	return len;
}

/*-----------------------------------------------------------------------------------
    Scheduler BLE e gestione sicurezza:
    Questo blocco contiene funzioni che vengono eseguite nel contesto dello scheduler
    per avviare la sicurezza sulle connessioni BLE e gestire i bond. In particolare:
    - start_secure_conn_sched: richiama pm_conn_secure per avviare il pairing/bonding
      se esiste una connessione valida.
    - Variabili come m_peer_to_be_deleted gestiscono lo stato dei peer da cancellare.
    - Funzioni come ble_forget_bonds, ble_list_bonds, ble_bond_info e ble_clear_bond
      permettono di manipolare i bond salvati e inviare informazioni tramite USB. */

/*  Scheduled function: avvia la security in un contesto sicuro
    Questa funzione è una callback pianificata che verifica se c'è una connessione BLE attiva.
    Se sì, tenta di avviare la procedura di sicurezza (pairing/bonding).
    Logga il risultato per debug */

// scheduled function: avvia la security in un contesto sicuro
void start_secure_conn_sched(void * p_event_data, uint16_t event_size)
{
    (void)p_event_data; // parametro non usato
    (void)event_size;   // parametro non usato

    if (m_conn_handle == BLE_CONN_HANDLE_INVALID) {
        NRF_LOG_WARNING("start_secure_conn_sched: no conn handle"); // nessuna connessione attiva
        return;
    }

    ret_code_t err = pm_conn_secure(m_conn_handle, false); // avvia pairing se necessario
    if (err != NRF_SUCCESS) {
        NRF_LOG_ERROR("pm_conn_secure failed: 0x%08X", err); // log errore
    } else {
        NRF_LOG_INFO("pm_conn_secure scheduled (requested)."); // log successo
    }
}

// Peer ID in corso di cancellazione, usato dal Peer Manager
static pm_peer_id_t m_peer_to_be_deleted = PM_PEER_ID_INVALID;
//Questa variabile è un segnaposto temporaneo per sapere quale peer si sta eliminando, utile per gestire correttamente gli eventi asincroni del Peer Manager.

void ble_forget_bonds(void)  
{
    ret_code_t err_code = pm_peers_delete(); // Cancella tutti i peer salvati (bond)
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("Bond eliminati. I dispositivi devono eseguire un nuovo pairing.");
    NRF_LOG_FLUSH();
}


// Funzione per stampare la lista dei dispositivi associati
void ble_list_bonds(void)
{
    // Ottiene l'ID del primo peer (dispositivo associato) valido.
    // PM_PEER_ID_INVALID indica che non ci sono peer ancora registrati.
    pm_peer_id_t peer_id = pm_next_peer_id_get(PM_PEER_ID_INVALID);

    if (peer_id == PM_PEER_ID_INVALID)
    {
        usb_send("Nessun dispositivo associato.\r\n", 31);
        return;
    }
    usb_send("Ecco la lista:\r\n", 16);

    while (peer_id != PM_PEER_ID_INVALID)
    {
        pm_peer_data_bonding_t bonding_data;  // Struttura per contenere i dati di bonding
        
        // Carica i dati di bonding del peer corrente
        if (pm_peer_data_bonding_load(peer_id, &bonding_data) == NRF_SUCCESS)
        {
            char msg[128];   // Buffer per creare il messaggio da inviare via USB
            // Formatta il messaggio con l'ID del peer e il suo indirizzo MAC
            // Nota: l'indirizzo MAC viene stampato in ordine inverso rispetto a come è memorizzato
            snprintf(msg, sizeof(msg),
                     "ID %d: %02X:%02X:%02X:%02X:%02X:%02X\r\n",
                     peer_id,
                     bonding_data.peer_ble_id.id_addr_info.addr[5],
                     bonding_data.peer_ble_id.id_addr_info.addr[4],
                     bonding_data.peer_ble_id.id_addr_info.addr[3],
                     bonding_data.peer_ble_id.id_addr_info.addr[2],
                     bonding_data.peer_ble_id.id_addr_info.addr[1],
                     bonding_data.peer_ble_id.id_addr_info.addr[0]);
            usb_send(msg, strlen(msg));
        }
        else
        {
            usb_send("Errore nel caricamento del bond.\r\n", 37); 
        }

        peer_id = pm_next_peer_id_get(peer_id); // Passa al peer successivo
    }

    usb_send("\r\n", 2);
}

// Funzione per stampare informazioni dettagliate sui bond (senza IRK)
void ble_bond_info(void)
{
    pm_peer_id_t peer_id = pm_next_peer_id_get(PM_PEER_ID_INVALID);  // Ottiene l'ID del primo peer valido
    uint8_t index = 0;  // Contatore per numerare i peer nella stampa

    if (peer_id == PM_PEER_ID_INVALID)   // Se non ci sono dispositivi associati, invia un messaggio e termina
    {
        usb_send("Nessun dispositivo associato.\r\n", 31);
        return;
    }

    usb_send("Dispositivi associati:\r\n", 24);

    while (peer_id != PM_PEER_ID_INVALID)  // Ciclo su tutti i peer finché non si arriva alla fine della lista
    {
        pm_peer_data_bonding_t bonding_data;  // Struttura per contenere i dati di bonding

        if (pm_peer_data_bonding_load(peer_id, &bonding_data) == NRF_SUCCESS) // Carica i dati di bonding del peer corrente
        {
            char msg[256];  // Buffer per formattare il messaggio

            // Formatta tutte le informazioni principali del peer:
            // - Index e ID peer
            // - Indirizzo MAC
            // - Presenza di LTK (Long Term Key)
            // - MITM (Man-In-The-Middle protection)
            // - Dimensione chiave
            // - Stato crittografia (encrypted)
            snprintf(msg, sizeof(msg),
                     "- Peer %d (ID: %d)\r\n"
                     "  MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n"
                     "  LTK presente: %s\r\n"
                     "  MITM: %s\r\n"
                     "  Key size: %d\r\n"
                     "  Encrypted: %s\r\n",
                     index,
                     peer_id,
                     bonding_data.peer_ble_id.id_addr_info.addr[5],
                     bonding_data.peer_ble_id.id_addr_info.addr[4],
                     bonding_data.peer_ble_id.id_addr_info.addr[3],
                     bonding_data.peer_ble_id.id_addr_info.addr[2],
                     bonding_data.peer_ble_id.id_addr_info.addr[1],
                     bonding_data.peer_ble_id.id_addr_info.addr[0],
                     bonding_data.peer_ltk.enc_info.ltk_len > 0 ? "si" : "no",
                     bonding_data.peer_ltk.master_id.ediv ? "si" : "no",
                     bonding_data.peer_ltk.enc_info.ltk_len,
                     bonding_data.peer_ltk.master_id.ediv ? "si" : "no");

            usb_send(msg, strlen(msg));
        }
        else {
            usb_send("Errore caricamento bond data.\r\n", 31);
        }
        index++;  // Incrementa il contatore per il prossimo peer
        peer_id = pm_next_peer_id_get(peer_id);
    }
    usb_send("\r\n", 2);
}

// Funzione per cancellare un bond specifico dato il suo peer ID
void ble_clear_bond(uint8_t peer_id)
{
    // Controlla se il peer esiste
    pm_peer_id_t id = pm_next_peer_id_get(PM_PEER_ID_INVALID); // Ottiene l'ID del primo peer valido
    bool found = false;       // Variabile per indicare se il peer da cancellare esiste

   // Ciclo su tutti i peer associati per verificare se il peer richiesto esiste
    while (id != PM_PEER_ID_INVALID)
    {
        if (id == peer_id) // Se il peer corrente corrisponde all'ID da cancellare
        {
            found = true;  // Segnala che il peer è stato trovato
            break;         // Esce dal ciclo
        }
        id = pm_next_peer_id_get(id); // Passa al peer successivo
    }

    // Se il peer non esiste, invia un messaggio di errore e termina la funzione
    if (!found)
    {
        usb_send("Errore: nessun bond con questo ID.\r\n", 36);
        return;
    }

    // Cancella il peer specificato tramite la funzione del Peer Manager
    ret_code_t err = pm_peer_delete(peer_id);

    if (err == NRF_SUCCESS) // Se la cancellazione ha avuto successo
    {
        char msg[64];
        snprintf(msg, sizeof(msg),
                 "Bond %d eliminato con successo.\r\n", peer_id);
        usb_send(msg, strlen(msg));
    }
    else // Se c'è stato un errore nella cancellazione
    {
        char msg[64];
        snprintf(msg, sizeof(msg),
                 "Errore cancellazione bond %d (codice %d)\r\n", peer_id, err);
        usb_send(msg, strlen(msg));
    }

    // Prompt finale per indicare che il dispositivo è pronto per ulteriori comandi
    usb_send("> ", 2);
}


/* buffer statico per passkey da mostrare (scheduler consumer) */
char scheduled_passkey[PASSKEY_LENGTH + 1] = {0};

/* funzione che sarà eseguita nel contesto dello scheduler */
void usb_show_passkey_sched(void * p_event_data, uint16_t event_size)
{
    (void)p_event_data;  //Questi due parametri sono ignorati perché non servono in questo contesto. 
    (void)event_size;   //La funzione è probabilmente chiamata da un scheduler/event handler.
    
    if (scheduled_passkey[0] == '\0') {
        return;
    }

    /* Log su NRF log */
    NRF_LOG_INFO("%s %s", PASSKEY_TXT, scheduled_passkey);
    NRF_LOG_FLUSH();

    /* Invia anche sulla porta USB */
    char usb_msg[64];
    size_t len = snprintf(usb_msg, sizeof(usb_msg), "%s %s\r\n", PASSKEY_TXT, scheduled_passkey);
    // Usa snprintf per scrivere nel buffer una stringa formattata.
    // %s %s\r\n indica che verranno concatenate due stringhe (PASSKEY_TXT e scheduled_passkey) separate da uno spazio, seguite da ritorno a capo (\r\n).
    // len conterrà la lunghezza effettiva del messaggio scritto.
    (void)usb_send(usb_msg, len);

    /* mostra prompt per facilità utente */
    const char *prompt = "> ";
    (void)usb_send(prompt, strlen(prompt));

    /* pulisci buffer */
    scheduled_passkey[0] = '\0';
}

/*  Gestione eventi BLE:
    Funzione centrale ble_evt_handler che intercetta tutti gli eventi BLE.
    Aggiorna connessione, gestisce passkey, timeout, PHY update, e HVN TX complete.
    Tutte le azioni vengono loggate e in alcuni casi passate allo scheduler per esecuzione sicura.

 * Funzione: ble_evt_handler
 * -------------------------
 * Gestisce tutti gli eventi BLE (Bluetooth Low Energy) generati dallo stack SoftDevice.
 * Ogni evento ricevuto dal BLE viene analizzato in base all’identificativo (evt_id)
 * e gestito con un caso specifico all’interno dello switch.
 *
 * Eventi principali gestiti:
 *  - BLE_GAP_EVT_PASSKEY_DISPLAY: visualizzazione della passkey per pairing sicuro
 *  - BLE_GAP_EVT_CONNECTED / DISCONNECTED: gestione connessione e disconnessione
 *  - BLE_GAP_EVT_PHY_UPDATE / REQUEST: aggiornamento velocità fisica
 *  - BLE_GATTS_EVT_TIMEOUT / GATTC_EVT_TIMEOUT: timeout GATT lato server/client
 *  - BLE_GATTS_EVT_HVN_TX_COMPLETE: completamento invio notifiche
 *
 * La funzione utilizza app_scheduler per rinviare alcune operazioni al main loop,
 * evitando race conditions e garantendo la sincronizzazione tra stack BLE e logica utente. */

static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code = NRF_SUCCESS;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_PASSKEY_DISPLAY:
            {
            NRF_LOG_INFO("Prova3");
            NRF_LOG_FLUSH();
            /* copia passkey nello statico per lo scheduler */
            memcpy(scheduled_passkey,p_ble_evt->evt.gap_evt.params.passkey_display.passkey,PASSKEY_LENGTH);
            scheduled_passkey[PASSKEY_LENGTH] = '\0';

            /* metti in coda l'azione di visualizzazione (verrà eseguita nel main loop) */
            app_sched_event_put(NULL, 0, usb_show_passkey_sched);
            //char password[64];
            //size_t len = snprintf(password, sizeof(password), "%s %s\r\n", PASSKEY_TXT, scheduled_passkey);
            //usb_send(password, len);
            //le ultime 3 righe non sono necessarie perchè la visualizzazione avviene nello scheduler e non serve stampare qui.
            char messaggio ="\r\n"
                            "Inserisci la password sul dispositivo per completare\r\n"
                            "l’associazione sicura con la scheda EMGyro2 tramite\r\n" 
                            "connessione Bluetooth criptata.\r\n"
                            "> ";
            usb_send(messaggio, strlen(messaggio));

            /* log veloce anche qui (non obbligatorio) */
            NRF_LOG_DEBUG("Passkey event scheduled for display.");
            } break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected.");
            // LED indication will be changed when advertising starts.
            emg_disable();
    		sd_clock_hfclk_release();
            break;

        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected.");
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);

            /* Schedula la richiesta di sicurezza per evitare race condition */
            app_sched_event_put(NULL, 0, start_secure_conn_sched);

            // Aggiorna PHY
            {
                ble_gap_phys_t const phys = {
                    .rx_phys = BLE_GAP_PHY_2MBPS,
                    .tx_phys = BLE_GAP_PHY_2MBPS,
                };
                sd_ble_gap_phy_update(m_conn_handle, &phys);
            } break;


        case BLE_GAP_EVT_PHY_UPDATE:
        {
        	NRF_LOG_INFO("PHY: %d %d %d",
        			p_ble_evt->evt.gap_evt.params.phy_update.status,
        			p_ble_evt->evt.gap_evt.params.phy_update.rx_phy,
        			p_ble_evt->evt.gap_evt.params.phy_update.tx_phy
			);
        } break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_INFO("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_HVN_TX_COMPLETE:
        {
        	uint32_t ts = NRF_TIMER1->CC[0];
			uint8_t cnt = p_ble_evt->evt.gatts_evt.params.hvn_tx_complete.count;
			uint32_t id = bt_dequeue_pkt(cnt);
			if (id == BT_ID_EMG) { // executes every 256 packets when SEQ == 0:
				emg_ts = ts;
				NRF_TIMER1->TASKS_CAPTURE[0] = 1;
				uint32_t ts2 = NRF_TIMER1->CC[0];

				emg_ts_valid = true;
				NRF_LOG_INFO("TX %u %u", (unsigned) ts2 - ts, ts);
			}
			ble_emg_packet_retry(&m_emg);
        }
            break;

        default:
            // No implementation needed.
            break;
    }
}

/*  Inizializzazione BLE stack:
    Le funzioni ble_stack_init e peer_manager_init configurano il SoftDevice BLE,
    impostano parametri di sicurezza, registrano callbacks per eventi BLE e Peer Manager.*/
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


//Function for the Peer Manager initialization.
static void peer_manager_init(void)
{
    ret_code_t err_code;

    //Inizializza il Peer Manager
    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    //Imposta i parametri di sicurezza
    ble_gap_sec_params_t sec_param;
    memset(&sec_param, 0, sizeof(sec_param));

    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    //Copia nel globale m_sec_params (per ble_evt_handler)
    memcpy(&m_sec_params, &sec_param, sizeof(sec_param));

    //Registra i parametri nel Peer Manager
    err_code = pm_sec_params_set(&m_sec_params);
    APP_ERROR_CHECK(err_code);

    //Registra il callback per gli eventi del Peer Manager
    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}

static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated when button is pressed.
 */
/*
static void bsp_event_handler(bsp_event_t event)
{
    ret_code_t err_code;

    switch (event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break; // BSP_EVENT_SLEEP

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break; // BSP_EVENT_DISCONNECT

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break; // BSP_EVENT_KEY_0

        default:
            break;
    }
}
*/

/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    ble_advertising_init_t init;
    memset(&init, 0, sizeof init);

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof m_adv_uuids / sizeof m_adv_uuids[0];
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

	init.config.ble_adv_fast_enabled  = true;
	init.config.ble_adv_fast_interval = 244;   // in units of  0.625 ms.
	init.config.ble_adv_fast_timeout  = 3000;  // in units of 10.000 ms.
	init.config.ble_adv_slow_enabled  = true;
	init.config.ble_adv_slow_interval = 2056;  // in units of  0.625 ms.
	init.config.ble_adv_slow_timeout  = 3000;  // in units of 10.000 ms.

    init.evt_handler = on_adv_evt;

    ret_code_t err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}



/**@brief Function for starting advertising.
 */
void advertising_start (void)
{
	if (!m_advertising_active) {
		ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
		APP_ERROR_CHECK(err_code);
		m_advertising_active = true;
	}
}

void advertising_stop (void)
{
	m_advertising_active = false;
	sd_ble_gap_adv_stop(m_advertising.adv_handle);
}


static volatile bool bt_started = false;

void bt_update_battery_level (uint8_t level)
{
    ret_code_t err_code;

    if (!bt_started) return;

    err_code = ble_bas_battery_level_update(&m_bas, level, BLE_CONN_HANDLE_ALL);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
}

void received_emg_data_block (void *p_event_data, uint16_t event_size)
{
//	NRF_LOG_INFO("B:%d E:%d L:%d", bt_queue_len(), pkt_q_lens[1], (uint8_t) (m_emg.buf_tail - m_emg.buf_head));
	if (pkt_q_lens[1] == 1) ble_emg_packet_retry(&m_emg);
	ble_emg_packet_prepare(&m_emg);
}

void received_gyro_data_block (void *p_event_data, uint16_t event_size)
{
	if (pkt_q_lens[2] <= 1) ble_gyro_packet_retry(&m_gyro);
	ble_gyro_packet_prepare(&m_gyro);
}

/**@brief Function for initializing Radio Notification Software Interrupts.
 */
uint32_t radio_notification_init (uint32_t irq_priority, uint8_t notification_type, uint8_t notification_distance)
{
    uint32_t err_code;

    err_code = sd_nvic_ClearPendingIRQ(SWI1_IRQn);
    if (err_code != NRF_SUCCESS) return err_code;

    err_code = sd_nvic_SetPriority(SWI1_IRQn, irq_priority);
    if (err_code != NRF_SUCCESS) return err_code;

    err_code = sd_nvic_EnableIRQ(SWI1_IRQn);
    if (err_code != NRF_SUCCESS) return err_code;

    // Configure the event
    return sd_radio_notification_cfg_set(notification_type, notification_distance);
}

/**@brief Software interrupt 1 IRQ Handler, handles radio notification interrupts.
 */
void SWI1_IRQHandler (uint32_t radio_evt)
{
	static bool on = false;
	on = !on;
	if (radio_evt) {
//		uint32_t ts1 = NRF_TIMER1->CC[0];
//    	NRF_TIMER1->TASKS_CAPTURE[0] = 1;
//		uint32_t ts2 = NRF_TIMER1->CC[0];
//		if (automatic_dcdc)
//			NRF_POWER->DCDCEN0 = on;
//		NRF_LOG_INFO("R%d %u %u", on, ts1, ts2);
	}
}


/*------------------------------------------------------------------------------------------
    Funzioni di inizializzazione e avvio BLE:
    ble_init imposta stack, GAP, GATT, advertising, services, conn_params, Peer Manager
    e mostra informazioni sul softdevice. ble_start avvia l'advertising. */

void ble_init (void)
{
	// Initialize.
	ble_stack_init();
//	radio_notification_init(3, NRF_RADIO_NOTIFICATION_TYPE_INT_ON_BOTH, NRF_RADIO_NOTIFICATION_DISTANCE_800US);
	gap_params_init();
	gatt_init();
	advertising_init();
	services_init();
	conn_params_init();
	peer_manager_init();

    show_softdevice_info();

//	sd_ppi_channel_assign(4, (const volatile void *) &NRF_RADIO->EVENTS_END, (const volatile void *)  &NRF_TIMER1->TASKS_CAPTURE[0]);
//	sd_ppi_channel_enable_set(1 << 4);

	bt_started = true;
}

void ble_start (void)
{
        advertising_start();
        bt_started = true;
}