/**
 * @file  icm20948_spi.h
 * @brief TDK InvenSense ICM-20948 9-DoF IMU – SPI driver for Raspberry Pi Pico 2
 *
 * Supports:
 *   - Accelerometer  (±2 / ±4 / ±8 / ±16 g)
 *   - Gyroscope      (±250 / ±500 / ±1000 / ±2000 °/s)
 *   - Magnetometer   AK09916 via on-chip I²C master (continuous 100 Hz)
 *   - Die temperature
 *
 * Default wiring (SPI0):
 *   CS   -> GP5
 *   SCK  -> GP2 (SPI0 SCK)
 *   MOSI -> GP3 (SPI0 TX)
 *   MISO -> GP4 (SPI0 RX)
 *
 * Build dependency: pico-sdk  (hardware/spi, hardware/gpio, pico/time)
 *
 * SPDX-License-Identifier: MIT
 */

#ifndef ICM20948_SPI_H
#define ICM20948_SPI_H

#include <stdint.h>
#include <stdbool.h>
#include "hardware/spi.h"
#include "hardware/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * WHO_AM_I
 * ---------------------------------------------------------------------- */
#define ICM20948_WHO_AM_I_VAL   0xEA

/* -------------------------------------------------------------------------
 * Gyroscope full-scale range
 * ---------------------------------------------------------------------- */
typedef enum {
    ICM20948_GYRO_FS_250  = 0,   /**< ±250  °/s  – sensitivity 131.0 LSB/(°/s) */
    ICM20948_GYRO_FS_500  = 1,   /**< ±500  °/s  – sensitivity  65.5 LSB/(°/s) */
    ICM20948_GYRO_FS_1000 = 2,   /**< ±1000 °/s  – sensitivity  32.8 LSB/(°/s) */
    ICM20948_GYRO_FS_2000 = 3,   /**< ±2000 °/s  – sensitivity  16.4 LSB/(°/s) */
} icm20948_gyro_fs_t;

/* -------------------------------------------------------------------------
 * Accelerometer full-scale range
 * ---------------------------------------------------------------------- */
typedef enum {
    ICM20948_ACCEL_FS_2G  = 0,   /**< ±2  g – sensitivity 16384 LSB/g */
    ICM20948_ACCEL_FS_4G  = 1,   /**< ±4  g – sensitivity  8192 LSB/g */
    ICM20948_ACCEL_FS_8G  = 2,   /**< ±8  g – sensitivity  4096 LSB/g */
    ICM20948_ACCEL_FS_16G = 3,   /**< ±16 g – sensitivity  2048 LSB/g */
} icm20948_accel_fs_t;

/* -------------------------------------------------------------------------
 * Return / status codes
 * ---------------------------------------------------------------------- */
typedef enum {
    ICM20948_OK            =  0,
    ICM20948_ERR_WHO_AM_I  = -1,  /**< WHO_AM_I mismatch                  */
    ICM20948_ERR_MAG_ID    = -2,  /**< AK09916 device ID mismatch         */
    ICM20948_ERR_MAG_TO    = -3,  /**< AK09916 I²C master timeout         */
    ICM20948_ERR_MAG_OVF   = -4,  /**< AK09916 magnetic sensor overflow   */
    ICM20948_ERR_MAG_RDY   = -5,  /**< AK09916 data not ready             */
} icm20948_err_t;

/* -------------------------------------------------------------------------
 * Driver configuration
 * ---------------------------------------------------------------------- */
typedef struct {
    spi_inst_t          *spi;         /**< SPI instance – spi0 or spi1     */
    uint                 pin_cs;      /**< Chip-select GPIO (active-low)    */
    uint                 pin_sck;     /**< SPI clock GPIO                   */
    uint                 pin_mosi;    /**< SPI MOSI (TX) GPIO               */
    uint                 pin_miso;    /**< SPI MISO (RX) GPIO               */
    uint32_t             baudrate;    /**< SPI clock frequency (Hz)         */
    icm20948_gyro_fs_t   gyro_fs;    /**< Gyro full-scale range             */
    icm20948_accel_fs_t  accel_fs;   /**< Accel full-scale range            */
    bool                 mag_enable;  /**< Enable AK09916 magnetometer       */
} icm20948_config_t;

/** Default config macro – 8 MHz SPI, ±250°/s gyro, ±4 g accel, mag on */
#define ICM20948_CONFIG_DEFAULT(spi_inst, cs, sck, mosi, miso)  \
    {                                                            \
        .spi        = (spi_inst),                               \
        .pin_cs     = (cs),                                     \
        .pin_sck    = (sck),                                    \
        .pin_mosi   = (mosi),                                   \
        .pin_miso   = (miso),                                   \
        .baudrate   = 8000000,                                  \
        .gyro_fs    = ICM20948_GYRO_FS_250,                     \
        .accel_fs   = ICM20948_ACCEL_FS_4G,                     \
        .mag_enable = true,                                      \
    }

/* -------------------------------------------------------------------------
 * Driver handle (opaque internals exposed for static allocation)
 * ---------------------------------------------------------------------- */
typedef struct {
    icm20948_config_t   cfg;
    int                 cur_bank;
    float               gyro_sens;    /**< LSB / (°/s)  */
    float               accel_sens;   /**< LSB / g       */
} icm20948_dev_t;

/* -------------------------------------------------------------------------
 * Calibration offsets (optional – apply before reading)
 * ---------------------------------------------------------------------- */
typedef struct {
    float accel_offset[3];   /**< m/s² bias  [x, y, z] */
    float gyro_offset[3];    /**< °/s  bias  [x, y, z] */
    float mag_offset[3];     /**< µT   hard-iron [x,y,z] */
    float mag_scale[3];      /**< soft-iron scale  [x,y,z], default {1,1,1} */
} icm20948_cal_t;

/* -------------------------------------------------------------------------
 * Data output structure
 * ---------------------------------------------------------------------- */
typedef struct {
    float ax, ay, az;        /**< Acceleration  (m/s²)  */
    float gx, gy, gz;        /**< Angular rate   (°/s)  */
    float mx, my, mz;        /**< Magnetic field  (µT)  */
    float temperature;        /**< Die temperature (°C)  */
    bool  mag_valid;          /**< true when mag data is fresh & no overflow */
} icm20948_data_t;

/* =========================================================================
 * Public API
 * ====================================================================== */

/**
 * @brief  Initialise the ICM-20948 driver.
 *
 * Configures SPI GPIO, resets the device, applies full-scale settings,
 * and optionally initialises the AK09916 magnetometer.
 *
 * @param  dev  Pointer to driver handle (caller-allocated).
 * @param  cfg  Pointer to configuration struct.
 * @return ICM20948_OK on success, negative error code otherwise.
 */
icm20948_err_t icm20948_init(icm20948_dev_t *dev, const icm20948_config_t *cfg);

/**
 * @brief  Read all sensor data into a data structure.
 *
 * @param  dev   Driver handle.
 * @param  out   Output data (physical units).
 * @param  cal   Optional calibration offsets (NULL to skip).
 * @return ICM20948_OK, or ICM20948_ERR_MAG_* for non-fatal mag issues.
 */
icm20948_err_t icm20948_read_all(icm20948_dev_t *dev,
                                  icm20948_data_t *out,
                                  const icm20948_cal_t *cal);

/** @brief Read acceleration only (m/s²). */
icm20948_err_t icm20948_read_accel(icm20948_dev_t *dev,
                                    float *ax, float *ay, float *az);

/** @brief Read gyroscope only (°/s). */
icm20948_err_t icm20948_read_gyro(icm20948_dev_t *dev,
                                   float *gx, float *gy, float *gz);

/** @brief Read magnetometer only (µT). mag_valid set false on overflow/not-ready. */
icm20948_err_t icm20948_read_mag(icm20948_dev_t *dev,
                                  float *mx, float *my, float *mz,
                                  bool *mag_valid);

/** @brief Read die temperature (°C). */
icm20948_err_t icm20948_read_temp(icm20948_dev_t *dev, float *temp_c);

/** @brief Change gyroscope full-scale range at runtime. */
icm20948_err_t icm20948_set_gyro_fs(icm20948_dev_t *dev, icm20948_gyro_fs_t fs);

/** @brief Change accelerometer full-scale range at runtime. */
icm20948_err_t icm20948_set_accel_fs(icm20948_dev_t *dev, icm20948_accel_fs_t fs);

/**
 * @brief  Set gyro/accel sample-rate dividers.
 *
 * Sample Rate = Base Rate / (1 + divider)
 *   Gyro  base: ~9 kHz (DLPF on)
 *   Accel base: ~1.125 kHz
 *
 * Pass 0 for maximum rate.
 */
icm20948_err_t icm20948_set_sample_rate(icm20948_dev_t *dev,
                                         uint8_t gyro_div,
                                         uint8_t accel_div);

/** @brief Enter sleep mode (draws ~8 µA). */
icm20948_err_t icm20948_sleep(icm20948_dev_t *dev);

/** @brief Wake from sleep mode. */
icm20948_err_t icm20948_wake(icm20948_dev_t *dev);

/** @brief Read WHO_AM_I register (should return 0xEA). */
uint8_t icm20948_who_am_i(icm20948_dev_t *dev);

#ifdef __cplusplus
}
#endif

#endif /* ICM20948_SPI_H */