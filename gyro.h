/*
 * gyro.h
 *
 *  Created on: Aug 10, 2019
 *      Author: giorgio
 */

#ifndef GYRO_H_
#define GYRO_H_

int  gyro_probe   (void);
void gyro_enable  (void);
void gyro_disable (void);

void gyro_wakeup_enable (void);

typedef struct gyro_sample_s
{
	uint32_t timestamp;
	uint16_t status;
	uint8_t data[14];
} gyro_sample_t;

#define GYRO_BUF_IDX_MASK 0x0F
#define GYRO_BUF_SIZE (GYRO_BUF_IDX_MASK + 1)
extern gyro_sample_t gyro_buf[GYRO_BUF_SIZE];
extern volatile uint32_t gyro_buf_head;
extern volatile uint32_t gyro_buf_tail;

#endif /* GYRO_H_ */
