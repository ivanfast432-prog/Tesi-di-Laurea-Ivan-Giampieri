#ifndef USB_H_
#define USB_H_

#include <app_usbd.h>
#include <app_usbd_cdc_acm.h>

#define TX_BUFFER_SIZE 64
#define READ_SIZE 1

// Prototipi delle funzioni che vuoi usare da altri file
int usb_init(void);
int usb_start(void);

int usb_send(const char *data, size_t size);

extern bool m_send_flag;
extern char m_tx_buffer[NRF_DRV_USBD_EPSIZE];
extern char m_rx_buffer[READ_SIZE];


// Se vuoi rendere visibile l'istanza CDC-ACM (solo se non è static!)
// extern app_usbd_cdc_acm_t m_app_cdc_acm;

#endif // USB_H
