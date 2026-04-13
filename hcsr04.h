#ifndef HCSR04_H
#define HCSR04_H

#include "pico/stdlib.h"

/**
 * HC-SR04 Ultrasonic Distance Sensor Driver
 * Platform: Raspberry Pi Pico 2 (Pico SDK / C)
 *
 * Wiring:
 *   VCC  -> VBUS (5V, pin 40) or 3.3V (pin 36)
 *   GND  -> GND
 *   TRIG -> Any GPIO (default GP14)
 *   ECHO -> Any GPIO (default GP15)
 *
 * ⚠ IMPORTANT: The HC-SR04 ECHO line is 5V. Use a voltage divider
 *   (e.g. 1kΩ + 2kΩ) to reduce it to ~3.3V and protect the Pico GPIO.
 */

/* Default pins – change to match your wiring */
#define HCSR04_DEFAULT_TRIG_PIN  14
#define HCSR04_DEFAULT_ECHO_PIN  15

/* Maximum echo wait time (µs). 30 000 µs ≈ 5 m */
#define HCSR04_TIMEOUT_US        30000

/* Returned when measurement times out */
#define HCSR04_OUT_OF_RANGE      -1.0f

typedef struct {
    uint trig_pin;
    uint echo_pin;
    uint timeout_us;
} HCSR04;

/**
 * Initialise the sensor.
 *
 * @param sensor      Pointer to an HCSR04 struct to initialise
 * @param trig_pin    GPIO number for TRIG
 * @param echo_pin    GPIO number for ECHO
 * @param timeout_us  Max echo-wait time in µs (use HCSR04_TIMEOUT_US)
 */
void hcsr04_init(HCSR04 *sensor, uint trig_pin, uint echo_pin, uint timeout_us);

/**
 * Trigger a measurement and return the distance in centimetres.
 *
 * @param sensor  Pointer to an initialised HCSR04 struct
 * @return        Distance in cm, or HCSR04_OUT_OF_RANGE on timeout
 */
float hcsr04_read_cm(const HCSR04 *sensor);

#endif /* HCSR04_H */
