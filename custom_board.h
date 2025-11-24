#ifndef CUSTOM_BOARD_H
#define CUSTOM_BOARD_H

// microcontroller:
#define RESET_PIN          18 // P0.18

// EMGyro2 board:

// charger control:
#define LED_PIN             5 // P0.05 (O - active high)
#define CHRG_STATUS_DRIVE  46 // P1.14 (O - p-type open drain: 1 or Z)
#define CHRG_STATUS_SENSE  47 // P1.15 (I - externally pulled high)
#define CHRG_ADC_CH_VBUS    0 // P0.02 (AIN0) × 5.02
#define CHRG_ADC_CH_IBAT    1 // P0.03 (AIN1) ÷ 10 Ω

// gyroscope:
#define GYRO_SDA           26 // P0.26
#define GYRO_SCL           27 // P0.27
#define GYRO_INT1          12 // P0.12 (I - active high)
#define GYRO_INT2          37 // P1.05 (I - active high)
#define GYRO_ADDRESS     0x6A // I²C bus address of chip

// electromiography:
#define EMG_POWER          10 // P0.10 (O - active low)
#define EMG_ENABLE         34 // P1.02 (O - active high)
#define EMG_RESET          33 // P1.01 (O - active low)
#define EMG_ALARM          36 // P1.04 (I - active low)
#define EMG_READY          42 // P1.10 (I - active low)
#define EMG_CLOCK          38 // P1.06 (O - 409.6 kHz)
#define EMG_SCLK           35 // P1.03
#define EMG_SS              8 // P0.08
#define EMG_MISO           40 // P1.08
#define EMG_MOSI            7 // P0.07


// PCA10056 board:
#if 0
#define LED_PIN            13
#define EMG_POWER          14
#define GYRO_INT1          10
#define GYRO_INT2          39
#define EMG_SCLK           11
#define EMG_SS             12
#define EMG_MISO           24
#define EMG_MOSI           25

#define RADIO_RDY          28
#define RADIO_END          29
#define RADIO_INT          30
#define RADIO_EVT          31

#endif

#endif // CUSTOM_BOARD_H
