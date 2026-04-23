#include "hcsr04.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

/* Speed of sound: 0.03435 cm/µs at ~20 °C  →  inverse = 29.15 µs/cm */
#define SOUND_SPEED_CM_US  0.03435f
#define SOUND_SPEED_IN_US  0.01352f  /* 0.03435 / 2.54 */

/* ------------------------------------------------------------------ */
/*  Private helpers                                                     */
/* ------------------------------------------------------------------ */

#define HCSR04_ECHO_PIN1 13
#define HCSR04_ECHO_PIN2 12

#define DEVICE_COUNT 2

volatile uint64_t start_time[ DEVICE_COUNT ];
volatile uint64_t pulse_time[ DEVICE_COUNT ];
uint8_t time_ready [DEVICE_COUNT];

//translate gpio pin to array index
uint8_t _get_index(uint gpio){
    if (gpio == HCSR04_ECHO_PIN1) {
       return 0;
    }
    else{
        return 1;
    }
}

// Interrupt callback
void gpio_isr(uint gpio, uint32_t events) {
    int index = _get_index(gpio);
    
    
    // Rising edge → start timing
    if (events & GPIO_IRQ_EDGE_RISE) {
        start_time[index] = time_us_64();
    }

    // Falling edge → end timing
    if (events & GPIO_IRQ_EDGE_FALL) {
        pulse_time[index] = time_us_64() - start_time[index];
        // if(pulse_time[index] > (HCSR04_TIMEOUT_US/2)){ //3000 is arbitary, check if the value is close to timeout
        //     time_ready[index] = false;
        //     pulse_time[index] = 0;
        // }
        time_ready[index] = true;
    }
}

/**
 * Wait for a GPIO level with a timeout.
 *
 * @param pin        GPIO to watch
 * @param level      Desired level (0 or 1)
 * @param timeout_us Max time to wait in µs
 * @return           true if the level was reached, false on timeout
 */
static bool _wait_for_level(uint pin, bool level, uint timeout_us)
{
    absolute_time_t deadline = make_timeout_time_us(timeout_us);
    while (gpio_get(pin) != level) {
        if (time_reached(deadline)) {
            return false;
        }
    }
    return true;
}

/**
 * Core measurement: returns raw echo pulse width in µs,
 * or 0 on timeout.
 */
static uint32_t _measure_pulse_us(const HCSR04 *sensor)
{
    // _trigger(sensor);
    
    /* Wait for ECHO to go HIGH (start of pulse) */
    if (!_wait_for_level(sensor->echo_pin, 1, sensor->timeout_us)) {
        return 0;
    }
    uint64_t start = time_us_64();

    /* Wait for ECHO to go LOW (end of pulse) */
    if (!_wait_for_level(sensor->echo_pin, 0, sensor->timeout_us)) {
        return 0;
    }
    uint64_t end = time_us_64();

    return (uint32_t)(end - start);
}



/* ------------------------------------------------------------------ */
/*  Public API                                                          */
/* ------------------------------------------------------------------ */

void hcsr04_init(HCSR04 *sensor, uint trig_pin, uint echo_pin, uint timeout_us, uint8_t receiver_setup)
{
    sensor->trig_pin   = trig_pin;
    sensor->echo_pin   = echo_pin;
    sensor->timeout_us = timeout_us;
    sensor->receiver_setup = receiver_setup;

    /* TRIG: output, start LOW */
    gpio_init(trig_pin);
    gpio_set_dir(trig_pin, GPIO_OUT);
    gpio_put(trig_pin, 0);

    /* ECHO: input, no pull (external voltage divider handles biasing) */
    gpio_init(echo_pin);
    gpio_set_dir(echo_pin, GPIO_IN);
    gpio_disable_pulls(echo_pin);

     gpio_set_irq_enabled_with_callback(
        sensor->echo_pin,
        GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
        true,&gpio_isr);
    
}

/**
 * Send a 10 µs HIGH pulse on TRIG to start a measurement.
 */
void hcsr04_trigger(const HCSR04 *sensor)
{
    gpio_put(sensor->trig_pin, 0);
    sleep_us(2);                  /* ensure TRIG is LOW before pulse   */
    gpio_put(sensor->trig_pin, 1);
    sleep_us(10);                 /* 10 µs trigger pulse                */
    gpio_put(sensor->trig_pin, 0);
}

bool get_time_ready(){
    
}

float hcsr04_read_cm(const HCSR04 *sensor)
{
    int index = _get_index(sensor->echo_pin);
    uint64_t pulse_us = 0;

    if(time_ready[index]){
        pulse_us = pulse_time[index];
        time_ready[index] = 0;
    }

    if (pulse_us == 0) {
        return HCSR04_OUT_OF_RANGE;
    }

    if(sensor->receiver_setup){
        /*straight line distance between the sensors*/
        return (float)pulse_us * SOUND_SPEED_CM_US; 
    }
    else{
        /* Distance = (pulse_width / 2) * speed_of_sound */
        return (float)pulse_us * SOUND_SPEED_CM_US / 2.0f;
    }
    
}
