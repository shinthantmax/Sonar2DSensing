#ifndef ADXL335_H
#define ADXL335_H

#include "pico/stdlib.h"
#include "hardware/adc.h"

/**
 * ADXL335 3-Axis Analog Accelerometer Driver
 * Platform: Raspberry Pi Pico 2 (Pico SDK / C)
 *
 * The ADXL335 outputs three analog voltages (X, Y, Z) proportional to
 * acceleration. These are read via the Pico's 12-bit ADC channels.
 *
 * Wiring (3.3V supply – recommended for Pico compatibility):
 *   VCC  -> 3.3V (pin 36)
 *   GND  -> GND
 *   XOUT -> GP26 / ADC0 (pin 31)
 *   YOUT -> GP27 / ADC1 (pin 32)
 *   ZOUT -> GP28 / ADC2 (pin 34)
 *
 * Sensor characteristics (Vs = 3.3V):
 *   Zero-g output voltage : Vs / 2  = 1.65 V
 *   Sensitivity           : 300 mV/g (typical)
 *   Measurement range     : ±3 g
 *
 * ADC on Pico 2:
 *   Resolution : 12-bit  (0 – 4095)
 *   Reference  : 3.3V
 *   LSB size   : 3.3V / 4096 ≈ 0.000806 V
 */

/* ------------------------------------------------------------------ */
/*  Pin defaults  (ADC-capable GPIOs only: GP26, GP27, GP28, GP29)    */
/* ------------------------------------------------------------------ */
#define ADXL335_DEFAULT_X_PIN   26   /* ADC0 */
#define ADXL335_DEFAULT_Y_PIN   27   /* ADC1 */
#define ADXL335_DEFAULT_Z_PIN   28   /* ADC2 */

/* ------------------------------------------------------------------ */
/*  Sensor constants                                                    */
/* ------------------------------------------------------------------ */
#define ADXL335_VCC_V           3.3f          /* Supply voltage (V)   */
#define ADXL335_ZERO_G_V        1.65f         /* 0g output = VCC/2    */
#define ADXL335_ONE_G_V         1.95f         /* 1g output approx    */
#define ADXL335_SENSITIVITY_V   0.300f        /* 300 mV/g typical     */
#define ADXL335_ADC_BITS        12
#define ADXL335_ADC_MAX         4095.0f       /* 2^12 - 1             */

/* Number of samples averaged per reading (power-of-2 recommended) */
#define ADXL335_DEFAULT_SAMPLES 32

/* ------------------------------------------------------------------ */
/*  Data structures                                                     */
/* ------------------------------------------------------------------ */

/** Raw 12-bit ADC counts for each axis */
typedef struct {
    uint16_t x;
    uint16_t y;
    uint16_t z;
} ADXL335_Raw;

/** Acceleration in g for each axis */
typedef struct {
    float x;
    float y;
    float z;
} ADXL335_Data;

/** Driver configuration / state */
typedef struct {
    uint  x_pin;          /* GPIO for X output  */
    uint  y_pin;          /* GPIO for Y output  */
    uint  z_pin;          /* GPIO for Z output  */
    uint  x_adc_ch;       /* ADC channel (0-2)  */
    uint  y_adc_ch;
    uint  z_adc_ch;
    float vcc;            /* Supply voltage (V) */
    float x_zero_g_v;     /* 0g reference (V) for X */
    float y_zero_g_v;     /* 0g reference (V) for Y */
    float z_zero_g_v;     /* 0g reference (V) for Z */
    float sensitivity_v;  /* V per g             */
    uint  num_samples;    /* Averaging samples  */
} ADXL335;

/* ------------------------------------------------------------------ */
/*  API                                                                 */
/* ------------------------------------------------------------------ */

/**
 * Initialise the ADXL335 driver.
 *
 * Calls adc_init() and configures the three ADC GPIO pins.
 * Safe to call even if adc_init() was called earlier in your project.
 *
 * @param dev         Pointer to an ADXL335 struct to populate
 * @param x_pin       GPIO connected to XOUT (must be ADC-capable)
 * @param y_pin       GPIO connected to YOUT
 * @param z_pin       GPIO connected to ZOUT
 * @param num_samples Samples to average per reading (1 = no averaging)
 */
void adxl335_init(ADXL335 *dev,
                  uint x_pin, uint y_pin, uint z_pin,
                  uint num_samples);
/**
 * Read raw 12-bit ADC values for all three axes with one sample only.
 *
 * @param dev  Pointer to an initialised ADXL335 struct
 * @param out  Pointer to ADXL335_Raw struct to fill
 */
void adxl335_read_raw(const ADXL335 *dev, ADXL335_Raw *out);
/**
 * Read avg 12-bit ADC values for all three axes.
 *
 * @param dev  Pointer to an initialised ADXL335 struct
 * @param out  Pointer to ADXL335_Raw struct to fill
 */
void adxl335_read_avg(const ADXL335 *dev, ADXL335_Raw *out);

/**
 * Read acceleration in g for all three axes.
 *
 * @param dev  Pointer to an initialised ADXL335 struct
 * @param out  Pointer to ADXL335_Data struct to fill
 */
void adxl335_read_g(const ADXL335 *dev, ADXL335_Data *out);

/**
 * Read acceleration in m/s² for all three axes.
 *
 * @param dev  Pointer to an initialised ADXL335 struct
 * @param out  Pointer to ADXL335_Data struct to fill
 */
void adxl335_read_ms2(const ADXL335 *dev, ADXL335_Data *out);

/**
 * Compute total (vector) magnitude of acceleration in g.
 *
 * @param data  Pointer to an ADXL335_Data struct (already read)
 * @return      sqrt(x² + y² + z²) in g
 */
float adxl335_magnitude_g(const ADXL335_Data *data);

/**
 * Perform a simple runtime calibration.
 *
 * Place the sensor flat (Z axis pointing up) and call this function.
 * It stores the calibrated zero-g offset for X, Y and Z. 
 *
 *
 * @param dev          Pointer to an initialised ADXL335 struct
 * @param cal_samples  Number of samples to average for calibration
 */
void adxl335_calibrate(ADXL335 *dev, uint cal_samples);

#endif /* ADXL335_H */