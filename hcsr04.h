#ifndef HCSR04_H
#define HCSR04_H

#include "pico/stdlib.h"

/* Default pins*/
#define HCSR04_DEFAULT_TRIG_PIN  14
#define HCSR04_DEFAULT_ECHO_PIN  15

/* Maximum echo wait time (µs). 30 000 µs ≈ 5 m */
#define HCSR04_TIMEOUT_US        30000

/* Returned when measurement times out */
#define HCSR04_OUT_OF_RANGE      -1.0f

/** 
 * The setup of the receiver 
 * 0 --> the receiver is on the same module
 * 1 --> the receiver is on the external module
)*/
#define HCSR04_DEFAULT_RECEIVER 0

typedef struct {
    uint trig_pin;
    uint echo_pin;
    uint timeout_us;
    bool receiver_setup;
} HCSR04;

/**
 * Initialise the sensor.
 *
 * @param sensor      Pointer to an HCSR04 struct to initialise
 * @param trig_pin    GPIO number for TRIG
 * @param echo_pin    GPIO number for ECHO
 * @param timeout_us  Max echo-wait time in µs (use HCSR04_TIMEOUT_US)
 */
void hcsr04_init(HCSR04 *sensor, uint trig_pin, uint echo_pin, uint timeout_us, uint8_t receiver_setup);


/**
 * Send a 10 µs HIGH pulse on TRIG to start a measurement.
 * @param sensor  Pointer to an initialised HCSR04 struct
 */
void hcsr04_trigger(const HCSR04 *sensor);

/**
 * Trigger a measurement and return the distance in centimetres.
 *
 * @param sensor  Pointer to an initialised HCSR04 struct
 * @return        Distance in cm, or HCSR04_OUT_OF_RANGE on timeout
 */
float hcsr04_read_cm(const HCSR04 *sensor);

#endif /* HCSR04_H */
