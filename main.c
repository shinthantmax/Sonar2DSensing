#include <stdio.h>
#include "pico/stdlib.h"
#include "hcsr04.h"
#include "adxl335.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"
#include "icm20948_spi.h"

#define HCSR04_ECHO_PIN1 13
#define HCSR04_ECHO_PIN2 12
#define HCSR04_RECEIVER_EXTERNAL 1
// Core 1 Main Code


HCSR04 receiver1; //the pair of tranmitter and receiver 1
HCSR04 receiver2; //the pair of tranmitter and receiver 1



#define PIN_CS   5
#define PIN_SCK  2
#define PIN_MOSI 3
#define PIN_MISO 4
 

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
    
    multicore_launch_core1(core1_entry); //start core 1
 
    /* Initialise sensor on GP26/27/28 with 64-sample averaging */
    // ADXL335 accel;
    // adxl335_init(&accel,
    //              AXDL335_DEFAULT_X_PIN,   /* GP26 – ADC0 */
    //              ADXL335_DEFAULT_Y_PIN,   /* GP27 – ADC1 */
    //              ADXL335_DEFAULT_Z_PIN,   /* GP28 – ADC2 */
    //              ADXL335_DEFAULT_SAMPLES);
 
    
    // printf("Calibrating – keep sensor flat...\n");
    // adxl335_calibrate(&accel, 512);
 
 
    // ADXL335_Data  g_data;
    // ADXL335_Data  ms2_data;
    // ADXL335_Raw   raw;
    
    
    /*Initialize icm20948*/
    /* Build configuration */
    icm20948_config_t cfg = ICM20948_CONFIG_DEFAULT(spi0);
    // cfg.gyro_fs    = ICM20948_GYRO_FS_250;
    // cfg.accel_fs   = ICM20948_ACCEL_FS_4G;
    
    /* Initialise driver */
    icm20948_dev_t imu;
    icm20948_err_t err = icm20948_init(&imu, &cfg);
    printf("Hello1");
    if (err != ICM20948_OK) {
        printf("ERROR: icm20948_init() returned %d\n", err);
        while (1) tight_loop_contents();
    }
    printf("WHO_AM_I = 0x%02X  (expect 0xEA)\n\n", icm20948_who_am_i(&imu));
 
    // /*
    //  * Optional calibration – replace zeros with values from your own
    //  * calibration routine.
    //  */
    icm20948_cal_t cal = {
        .accel_offset = { 0.0f, 0.0f, 0.0f },
        .gyro_offset  = { 0.0f, 0.0f, 0.0f },
        .mag_offset   = { 0.0f, 0.0f, 0.0f },
        .mag_scale    = { 1.0f, 1.0f, 1.0f },
    };
 
    icm20948_raw_t rawdata;
    icm20948_data_t data;

    while (true) {
        

        if (multicore_fifo_rvalid()) {
            if(multicore_fifo_pop_blocking()){ //there is a new value
                // printf("Distance: %.1f cm   %.1f cm\n", g_sonardist[0], g_sonardist[1]);
            }
        }

        // adxl335_read_raw(&accel, &raw);
        // printf("Accel Raw ADC | X: %d   Y: %d   Z: %d \n",
        //        raw.x, raw.y, raw.z);


        // adxl335_read_g(&accel, &g_data);
        // printf("Accel | X: %6.3f   Y: %6.3f   Z: %6.3f \n",
        //        g_data.x, g_data.y, g_data.z);

        err = icm20948_read_all(&imu, &rawdata, &cal);
        
 
        printf("Accel ADC  [%d, %d, %d] \n",
               rawdata.ax, rawdata.ay, rawdata.az);

        icm20948_convert_accel(&imu, &rawdata, &data);
        printf("Accel   [%5.3f, %5.3f, %5.3f] m/s^2 \n",
               data.ax, data.ay, data.az);
        
        printf("Gyro ADC   [%d, %d, %d] \n",
               rawdata.gx, rawdata.gy, rawdata.gz);

        icm20948_convert_gyro(&imu, &rawdata, &data);
        printf("Gyro   [%5.3f, %5.3f, %5.3f] deg/s \n",
               data.gx, data.gy, data.gz);
        // printf("Temp   %.2f °C\n", data.temperature);
 
        printf("---\n");
 

        sleep_ms(300);   /* Minimum recommended interval: 60 ms */
    }

    return 0;
}
