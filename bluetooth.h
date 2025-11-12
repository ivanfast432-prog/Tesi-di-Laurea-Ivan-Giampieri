/*
 * bluetooth.h
 *
 *  Created on: Aug 29, 2019
 *      Author: giorgio
 */

#ifndef BLUETOOTH_H_
#define BLUETOOTH_H_

void bt_update_battery_level (uint8_t level);

void ble_init (void);
void ble_start (void);
//void advertising_start (void);
void ble_forget_bonds(void);


extern bool automatic_dcdc;

// The following function MUST be called immediately after a call to sd_ble_gatts_hvx succeeds.
// The id will be used to identify the packet just queued for transmission,
// so that it can be identified when transmission is completed and the HW time stamp associated to it.
void bt_queued_pkt (uint32_t id);
unsigned bt_queue_len (void);
extern int pkt_q_lens[8];
// Flags to identify the service/characteristic to which notifications are being sent:
// (pick at most one)
#define BT_ID_EMG 0x10000000
#define BT_ID_GYR 0x20000000
#define BT_ID_SYS 0x30000000
// Flag to identify retransmission (may be ORed with previous ones):
#define BT_ID_ARQ 0x80000000

// RSSI handling:
extern int8_t rssi[40];

struct conn_info {
	uint16_t interval;       // 6 ÷ 3200 in units of 1.25 ms
	uint16_t latency  : 12;  // 0 ÷  499 in "intervals"
	uint16_t ble_phy  :  4;  // 1=1M 2=2M 4=coded
	 int16_t drift;          // RTC drift in units of 2^-24
	  int8_t rssi;           // [dBm]
	 uint8_t rtc;
} extern conn_info_data;


#endif /* BLUETOOTH_H_ */
