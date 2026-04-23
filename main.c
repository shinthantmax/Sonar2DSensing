#include <stdio.h>
#include "pico/stdlib.h"
#include "hcsr04.h"
#include "adxl335.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"

#define HCSR04_ECHO_PIN1 13
#define HCSR04_ECHO_PIN2 12
#define HCSR04_RECEIVER_EXTERNAL 1
// Core 1 Main Code


HCSR04 receiver1; //the pair of tranmitter and receiver 1
HCSR04 receiver2; //the pair of tranmitter and receiver 1




float g_sonardist[2];
void core1_entry() {

    
    hcsr04_init(&receiver1,
                HCSR04_DEFAULT_TRIG_PIN,   /* GP14 */
                HCSR04_ECHO_PIN1,   /* GP13 */
                HCSR04_TIMEOUT_US,
                HCSR04_RECEIVER_EXTERNAL);
    
   
    hcsr04_init(&receiver2,
                HCSR04_DEFAULT_TRIG_PIN,   /* GP14 */
                HCSR04_ECHO_PIN2,   /* GP12 */
                HCSR04_TIMEOUT_US,
                HCSR04_RECEIVER_EXTERNAL);
    
    while(1){
     
        hcsr04_trigger(&receiver1);
        g_sonardist[0] = hcsr04_read_cm(&receiver1);
        g_sonardist[1] = hcsr04_read_cm(&receiver2);
        if(g_sonardist[0] != HCSR04_OUT_OF_RANGE &&
             g_sonardist[1] != HCSR04_OUT_OF_RANGE){
                if (multicore_fifo_wready()) {
                    multicore_fifo_push_blocking(1); //ready flag
                }
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
            if(multicore_fifo_pop_blocking()){ //there is a new value
                printf("Distance: %.1f cm   %.1f cm\n", g_sonardist[0], g_sonardist[1]);
            }
        }

        adxl335_read_raw(&accel, &raw);
        // printf("Accel Raw ADC | X: %d   Y: %d   Z: %d \n",
        //        raw.x, raw.y, raw.z);


        // adxl335_read_g(&accel, &g_data);
        // printf("Accel | X: %6.3f   Y: %6.3f   Z: %6.3f \n",
        //        g_data.x, g_data.y, g_data.z);

        sleep_ms(5);   /* Minimum recommended interval: 60 ms */
    }

    return 0;
}
