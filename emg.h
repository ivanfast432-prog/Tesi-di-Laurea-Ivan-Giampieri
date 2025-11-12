/*
 * emg.h
 *
 *  Created on: Aug 10, 2019
 *      Author: giorgio
 */

#ifndef EMG_H_
#define EMG_H_

#include <stdint.h>
#include <stdbool.h>

int  emg_probe   (void);
void emg_enable  (void);
void emg_disable (void);
void emg_online_reconf_send (void);

int emg_configure (int rate, int samples_per_packet, uint32_t ch_config);

typedef struct emg_sample_s
{
	uint32_t timestamp;
	uint8_t status;
	uint8_t data[9];
} emg_sample_t;

#define EMG_BUF_IDX_MASK 0x3F
#define EMG_BUF_SIZE (EMG_BUF_IDX_MASK + 1)
extern emg_sample_t emg_buf[EMG_BUF_SIZE];
extern volatile uint32_t emg_buf_head;
extern volatile uint32_t emg_buf_tail;
extern unsigned emg_samples_per_packet;
extern unsigned emg_channels_per_sample;
extern unsigned emg_compression_type;
extern unsigned emg_compression_mode;

#endif /* EMG_H_ */
