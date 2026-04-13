#include <stdio.h>
#include "pico/stdlib.h"
#include "hcsr04.h"
#include "adxl335.h"


int main(void)
{
    stdio_init_all();

    /* Give USB-serial a moment to connect (optional) */
    sleep_ms(2000);

 
    /* Initialise sensor on GP26/27/28 with 64-sample averaging */
    ADXL335 accel;
    adxl335_init(&accel,
                 ADXL335_DEFAULT_X_PIN,   /* GP26 – ADC0 */
                 ADXL335_DEFAULT_Y_PIN,   /* GP27 – ADC1 */
                 ADXL335_DEFAULT_Z_PIN,   /* GP28 – ADC2 */
                 ADXL335_DEFAULT_SAMPLES);
 
    /* Optional: calibrate with sensor flat, Z-axis up (512 samples) */
    printf("Calibrating – keep sensor flat...\n");
    adxl335_calibrate(&accel, 512);
    // // printf("Calibration done. zero_g=%.4f V  sensitivity=%.4f V/g\n\n",
    //        accel.zero_g_v, accel.sensitivity_v);
 
    ADXL335_Data  g_data;
    ADXL335_Data  ms2_data;
    ADXL335_Raw   raw;

    printf("HC-SR04 demo – Raspberry Pi Pico 2\n");

    HCSR04 sensor;
    hcsr04_init(&sensor,
                HCSR04_DEFAULT_TRIG_PIN,   /* GP14 */
                HCSR04_DEFAULT_ECHO_PIN,   /* GP15 */
                HCSR04_TIMEOUT_US);

    while (true) {
        float dist_cm = hcsr04_read_cm(&sensor);

        if (dist_cm == HCSR04_OUT_OF_RANGE) {
            printf("Out of range (> ~4 m) or no echo received\n");
        } else {
            printf("Distance: %.1f cm  \n", dist_cm);
        }
        

        adxl335_read_raw(&accel, &raw);
        /* Acceleration in g */
        adxl335_read_g(&accel, &g_data);
        float magnitude = adxl335_magnitude_g(&g_data);
 
        /* Acceleration in m/s² */
        adxl335_read_ms2(&accel, &ms2_data);
 
        printf("Raw   | X: %4u  Y: %4u  Z: %4u\n",
               raw.x, raw.y, raw.z);
 
        printf("Accel | X: %+6.3f g  Y: %+6.3f g  Z: %+6.3f g  |Mag|: %.3f g\n",
               g_data.x, g_data.y, g_data.z, magnitude);
 
        printf("SI    | X: %+7.3f  Y: %+7.3f  Z: %+7.3f  m/s²\n\n",
               ms2_data.x, ms2_data.y, ms2_data.z);


        sleep_ms(500);   /* Minimum recommended interval: 60 ms */
    }

    return 0;
}
