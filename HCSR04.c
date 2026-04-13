// #include <stdio.h>
// #include "pico/stdlib.h"


// int main()
// {
//     stdio_init_all();

//     while (true) {
//         printf("Hello, world!\n");
//         sleep_ms(1000);
//     }
// }
/**
 * HC-SR04 Ultrasonic Distance Sensor Driver (Polling — No ISR)
 * Platform: Raspberry Pi Pico 2 (RP2350)
 * SDK:       pico-sdk 2.x
 *
 * Wiring:
 *   VCC  -> 5V (VBUS)
 *   GND  -> GND
 *   TRIG -> GPIO 14
 *   ECHO -> GPIO 15
 *
 *   ⚠️  HC-SR04 ECHO outputs 5V. Use a voltage divider or level shifter:
 *       ECHO ---[1kΩ]--- GPIO15 ---[2kΩ]--- GND
 *
 * Method:
 *   Pure polling with time_us_64(). No interrupts, no PIO, no DMA.
 *   Simple and portable — easy to read and modify.
 *
 * CMakeLists.txt — target_link_libraries:
 *   pico_stdlib  hardware_gpio  hardware_timer
 */

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"

/* ── Pin Configuration ───────────────────────────────────────────────── */
#define TRIG_PIN            14
#define ECHO_PIN            15

/* ── Timing / Range Constants ────────────────────────────────────────── */
#define TRIGGER_PULSE_US    10          /* HC-SR04 needs >= 10 us trigger */
#define ECHO_TIMEOUT_US     30000ULL    /* 30 ms  ~= 5 m max range        */
#define SPEED_OF_SOUND_CM   0.0343f     /* cm/us at ~20 C                 */
#define MIN_DISTANCE_CM     2.0f        /* sensor blind spot              */
#define MAX_DISTANCE_CM     400.0f      /* rated max range                */

/* --------------------------------------------------------------------- *
 * hcsr04_init()
 * Configure TRIG as output (LOW) and ECHO as input.
 * --------------------------------------------------------------------- */
void hcsr04_init(void)
{
    gpio_init(TRIG_PIN);
    gpio_set_dir(TRIG_PIN, GPIO_OUT);
    gpio_put(TRIG_PIN, 0);

    gpio_init(ECHO_PIN);
    gpio_set_dir(ECHO_PIN, GPIO_IN);
    gpio_pull_down(ECHO_PIN);   /* pull-down: safe default when idle */
}

/* --------------------------------------------------------------------- *
 * hcsr04_measure_us()
 *
 * Fire one measurement and return the raw echo pulse width in us.
 * Returns 0 on timeout (nothing in range or sensor fault).
 * --------------------------------------------------------------------- */
uint64_t hcsr04_measure_us(void)
{
    uint64_t t_start, t_end, deadline;

    /* 1. Send 10 us trigger pulse */
    gpio_put(TRIG_PIN, 1);
    sleep_us(TRIGGER_PULSE_US);
    gpio_put(TRIG_PIN, 0);

    /* 2. Wait for ECHO to go HIGH (rising edge) */
    deadline = time_us_64() + ECHO_TIMEOUT_US;
    while (gpio_get(ECHO_PIN) == 0) {
        if (time_us_64() >= deadline) {
            return 0;   /* Timeout waiting for echo to start */
        }
    }
    t_start = time_us_64();

    /* 3. Wait for ECHO to go LOW (falling edge) */
    deadline = t_start + ECHO_TIMEOUT_US;
    while (gpio_get(ECHO_PIN) == 1) {
        if (time_us_64() >= deadline) {
            return 0;   /* Timeout: echo pulse too long (out of range) */
        }
    }
    t_end = time_us_64();

    return (t_end > t_start) ? (t_end - t_start) : 0;
}

/* --------------------------------------------------------------------- *
 * hcsr04_measure_cm()
 *
 * Returns distance in centimetres, or -1.0 on timeout / error.
 * --------------------------------------------------------------------- */
float hcsr04_measure_cm(void)
{
    uint64_t pulse_us = hcsr04_measure_us();
    if (pulse_us == 0) return -1.0f;

    /* distance = (pulse_us x speed_of_sound) / 2  (round-trip) */
    return (float)pulse_us * SPEED_OF_SOUND_CM / 2.0f;
}

/* --------------------------------------------------------------------- *
 * hcsr04_measure_averaged_cm()
 *
 * Takes `samples` readings, discards any timeouts, and returns the
 * mean distance in cm. Returns -1.0 if all samples timed out.
 *
 * Recommended: samples = 3-5, delay_ms = 20-60
 * --------------------------------------------------------------------- */
float hcsr04_measure_averaged_cm(uint8_t samples, uint32_t delay_ms)
{
    float   sum   = 0.0f;
    uint8_t valid = 0;

    for (uint8_t i = 0; i < samples; i++) {
        float d = hcsr04_measure_cm();
        if (d >= MIN_DISTANCE_CM && d <= MAX_DISTANCE_CM) {
            sum += d;
            valid++;
        }
        if (i < samples - 1) sleep_ms(delay_ms);
    }

    return (valid > 0) ? (sum / valid) : -1.0f;
}

/* ── Main ────────────────────────────────────────────────────────────── */
int main(void)
{
    stdio_init_all();
    sleep_ms(2000);     /* allow USB CDC to enumerate on host */

    printf("=== HC-SR04 -- Pico 2 -- Polling Mode ===\n");
    printf("TRIG: GPIO%d   ECHO: GPIO%d\n\n", TRIG_PIN, ECHO_PIN);

    hcsr04_init();

    while (true) {
        /* --- Single reading ------------------------------------------ */
        float dist_cm = hcsr04_measure_cm();

        if (dist_cm < 0.0f) {
            printf("[TIMEOUT]   No echo received (out of range or disconnected)\n");
        } else if (dist_cm < MIN_DISTANCE_CM) {
            printf("[TOO CLOSE] %.1f cm  (sensor blind spot < 2 cm)\n", dist_cm);
        } else if (dist_cm > MAX_DISTANCE_CM) {
            printf("[TOO FAR]   %.1f cm  (rated max 400 cm)\n", dist_cm);
        } else {
            printf("Distance: %6.1f cm   %5.0f mm   %.3f m\n",
                   dist_cm,
                   dist_cm * 10.0f,
                   dist_cm / 100.0f);
        }

        /* --- Averaged reading (uncomment if preferred) --------------- */
        /*
        float avg = hcsr04_measure_averaged_cm(5, 30);
        if (avg < 0.0f) printf("Averaged: [TIMEOUT]\n");
        else            printf("Averaged: %.1f cm\n", avg);
        */

        sleep_ms(200);  /* 5 Hz -- increase delay to reduce ultrasound noise */
    }

    return 0;
}