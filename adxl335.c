#include "adxl335.h"
#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <math.h>

/* ------------------------------------------------------------------ */
/*  Private helpers                                                     */
/* ------------------------------------------------------------------ */

/**
 * Convert a GPIO pin number to its ADC channel index.
 * Pico ADC GPIOs: GP26=ch0, GP27=ch1, GP28=ch2, GP29=ch3 (VSYS monitor)
 */
static inline uint _gpio_to_adc_channel(uint pin)
{
    return pin - 26u;
}

/**
 * Read a single ADC channel, averaged over num_samples readings.
 * Switching the ADC input mux before each sample is intentional so
 * the mux settles properly on every call.
 */
static uint16_t _adc_read_avg(uint channel, uint num_samples)
{
    uint16_t avg = 0;
    
    adc_select_input(channel);
    avg = adc_read();
    sleep_us(100); //necessary delay for accurate readings
    for (uint i = 0; i < num_samples; i++) {
        avg = (avg + adc_read()) >> 1; //divided by 2
        sleep_us(100); //necessary delay for accurate readings
    }
    return avg;
}

/**
 * Convert a raw ADC count to voltage.
 */
static inline float _raw_to_voltage(uint16_t raw, float vcc)
{
    return (float)raw * (vcc / ADXL335_ADC_MAX);
}

/**
 * Convert a voltage to acceleration in g.
 */
static inline float _voltage_to_g(float voltage, float zero_g_v, float sens_v)
{
    return (voltage - zero_g_v) / sens_v;
}

/* ------------------------------------------------------------------ */
/*  Public API                                                          */
/* ------------------------------------------------------------------ */

void adxl335_init(ADXL335 *dev,
                  uint x_pin, uint y_pin, uint z_pin,
                  uint num_samples)
{
    /* Store configuration */
    dev->x_pin        = x_pin;
    dev->y_pin        = y_pin;
    dev->z_pin        = z_pin;
    dev->x_adc_ch     = _gpio_to_adc_channel(x_pin);
    dev->y_adc_ch     = _gpio_to_adc_channel(y_pin);
    dev->z_adc_ch     = _gpio_to_adc_channel(z_pin);
    dev->vcc          = ADXL335_VCC_V;
    dev->x_zero_g_v     = ADXL335_ZERO_G_V;
    dev->y_zero_g_v     = ADXL335_ZERO_G_V;
    dev->z_zero_g_v     = ADXL335_ZERO_G_V;
    dev->sensitivity_v = ADXL335_SENSITIVITY_V;
    dev->num_samples  = (num_samples > 0) ? num_samples : 1;

    /* Initialise ADC hardware */
    adc_init();
    adc_gpio_init(x_pin);
    adc_gpio_init(y_pin);
    adc_gpio_init(z_pin);
}

void adxl335_read_raw(const ADXL335 *dev, ADXL335_Raw *out)
{
    out->x = _adc_read_avg(dev->x_adc_ch, dev->num_samples);
    out->y = _adc_read_avg(dev->y_adc_ch, dev->num_samples);
    out->z = _adc_read_avg(dev->z_adc_ch, dev->num_samples);
}

void adxl335_read_g(const ADXL335 *dev, ADXL335_Data *out)
{
    ADXL335_Raw raw;
    adxl335_read_raw(dev, &raw);

    float vx = _raw_to_voltage(raw.x, dev->vcc);
    float vy = _raw_to_voltage(raw.y, dev->vcc);
    float vz = _raw_to_voltage(raw.z, dev->vcc);

    out->x = _voltage_to_g(vx, dev->x_zero_g_v, dev->sensitivity_v);
    out->y = _voltage_to_g(vy, dev->y_zero_g_v, dev->sensitivity_v);
    out->z = _voltage_to_g(vz, dev->z_zero_g_v, dev->sensitivity_v) + 1; //norminal is one
}

void adxl335_read_ms2(const ADXL335 *dev, ADXL335_Data *out)
{
    adxl335_read_g(dev, out);
    out->x *= 9.80665f;
    out->y *= 9.80665f;
    out->z *= 9.80665f;
}

float adxl335_magnitude_g(const ADXL335_Data *data)
{
    return sqrtf(data->x * data->x +
                 data->y * data->y +
                 data->z * data->z);
}

void adxl335_calibrate(ADXL335 *dev, uint cal_samples)
{
    /*
     * Calibration procedure (sensor must be flat, Z-axis pointing up):
     *
     *  1. X and Y should read 0 g  → measure their actual voltage as zero_g_v.
     *     We compute a per-axis offset but store a single averaged zero_g_v
     *     (the sensor's mid-supply reference) for simplicity.
     *  2. Z should read +1 g       → use this to back-calculate sensitivity.
     *
     * For a more advanced calibration (per-axis offsets) you can extend
     * the ADXL335 struct with x_offset_v / y_offset_v / z_offset_v fields.
     */

    uint saved_samples  = dev->num_samples;
    dev->num_samples    = cal_samples;

    ADXL335_Raw raw;
    adxl335_read_raw(dev, &raw);

    dev->num_samples = saved_samples;   /* restore */

    float vx = _raw_to_voltage(raw.x, dev->vcc);
    float vy = _raw_to_voltage(raw.y, dev->vcc);
    float vz = _raw_to_voltage(raw.z, dev->vcc);

    /*
     * X and Y are at 0 g → their voltages should equal the zero-g reference.
     * Average them to get a refined zero_g_v.
     */
    dev->x_zero_g_v = vx;
    dev->y_zero_g_v = vy;
    dev->z_zero_g_v = vz;

    /*
     * Z is at +1 g → sensitivity = (Vz - zero_g_v) / 1g
     * Guard against a nonsensical reading (sensor not flat or bad wiring).
     */
    // float sens = vz - dev->z_zero_g_v;
    // if (sens > 0.150f && sens < 0.450f) {   /* sanity: 150–450 mV/g */
    //     dev->sensitivity_v = sens;
    // }
    /* else: keep the datasheet default (300 mV/g) */
}