#include <stdio.h>
#include "pico/stdlib.h"
#include "hcsr04.h"
#include "adxl335.h"
#include "pico/multicore.h"

// Core 1 Main Code
void core1_entry() {

    HCSR04 sensor;
    hcsr04_init(&sensor,
                HCSR04_DEFAULT_TRIG_PIN,   /* GP14 */
                HCSR04_DEFAULT_ECHO_PIN,   /* GP15 */
                HCSR04_TIMEOUT_US);
    
    while(1){
        float dist_cm = hcsr04_read_cm(&sensor);
        if (multicore_fifo_wready()) {
            multicore_fifo_push_blocking(dist_cm);
        }
        sleep_ms(30);
    }

}

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
 
    
    printf("Calibrating – keep sensor flat...\n");
    adxl335_calibrate(&accel, 512);
 
 
    ADXL335_Data  g_data;
    ADXL335_Data  ms2_data;
    ADXL335_Raw   raw;
    
    multicore_launch_core1(core1_entry); //start core 1

    while (true) {
        

        if (multicore_fifo_rvalid()) {
            float dist_cm = multicore_fifo_pop_blocking();
            printf("Distance: %.1f cm  \n", dist_cm);
        }

        adxl335_read_g(&accel, &g_data);
        printf("Accel | X: %+6.3f g  Y: %+6.3f g  Z: %+6.3f g\n",
               g_data.x, g_data.y, g_data.z);


        sleep_ms(5);   /* Minimum recommended interval: 60 ms */
    }

    return 0;
}
