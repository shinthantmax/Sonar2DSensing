/**
 * @file  icm20948_spi.c
 * @brief TDK InvenSense ICM-20948 9-DoF IMU – SPI driver implementation
 *        Target: Raspberry Pi Pico 2 (RP2350) / Pico SDK / C99
 *
 * SPDX-License-Identifier: MIT
 */

#include "icm20948_spi.h"

#include <string.h>
#include "pico/time.h"
#include "pico/stdlib.h"

/* =========================================================================
 * Internal register map
 * ====================================================================== */

/* ---- Bank 0 ---- */
#define REG_WHO_AM_I          0x00
#define REG_USER_CTRL         0x03
#define REG_PWR_MGMT_1        0x06
#define REG_PWR_MGMT_2        0x07
#define REG_INT_PIN_CFG       0x0F
#define REG_ACCEL_XOUT_H      0x2D
#define REG_GYRO_XOUT_H       0x33
#define REG_TEMP_OUT_H        0x39
#define REG_EXT_SLV_SENS_DATA 0x3B   /* 9 bytes of mag auto-read live here */
#define REG_I2C_MST_STATUS    0x17
#define REG_BANK_SEL          0x7F

/* ---- Bank 2 ---- */
#define REG_GYRO_SMPLRT_DIV    0x00
#define REG_GYRO_CONFIG_1      0x01
#define REG_ACCEL_SMPLRT_DIV_2 0x11
#define REG_ACCEL_CONFIG       0x14

/* ---- Bank 3 ---- */
#define REG_I2C_MST_CTRL  0x01
#define REG_I2C_SLV0_ADDR 0x03
#define REG_I2C_SLV0_REG  0x04
#define REG_I2C_SLV0_CTRL 0x05
#define REG_I2C_SLV4_ADDR 0x13
#define REG_I2C_SLV4_REG  0x14
#define REG_I2C_SLV4_DO   0x15
#define REG_I2C_SLV4_CTRL 0x16
#define REG_I2C_SLV4_DI   0x17

/* ---- AK09916 (magnetometer) ---- */
#define AK09916_I2C_ADDR  0x0C
#define AK_REG_DEVICE_ID  0x01
#define AK_REG_STATUS1    0x10
#define AK_REG_HXL        0x11
#define AK_REG_STATUS2    0x18
#define AK_REG_CNTL2      0x31
#define AK_REG_CNTL3      0x32

#define AK09916_DEVICE_ID 0x09

/* ---- SPI framing ---- */
#define SPI_READ_FLAG  0x80
#define SPI_WRITE_MASK 0x7F

/* =========================================================================
 * Sensitivity look-up tables
 * ====================================================================== */

static const float GYRO_SENS_TABLE[4] = {
    131.0f,   /* ±250  °/s */
     65.5f,   /* ±500  °/s */
     32.8f,   /* ±1000 °/s */
     16.4f,   /* ±2000 °/s */
};

static const float ACCEL_SENS_TABLE[4] = {
    16384.0f, /* ±2  g */
     8192.0f, /* ±4  g */
     4096.0f, /* ±8  g */
     2048.0f, /* ±16 g */
};

#define MAG_SENS_UT  0.15f   /* µT per LSB (16-bit mode) */
#define GRAVITY_MS2  9.80665f

/* =========================================================================
 * Low-level SPI primitives
 * ====================================================================== */

static inline void cs_low(const icm20948_dev_t *dev) {
    gpio_put(dev->cfg.pin_cs, 0);
}

static inline void cs_high(const icm20948_dev_t *dev) {
    gpio_put(dev->cfg.pin_cs, 1);
}

static void spi_write_reg(icm20948_dev_t *dev, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg & SPI_WRITE_MASK, val };
    cs_low(dev);
    spi_write_blocking(dev->cfg.spi, buf, 2);
    cs_high(dev);
}

static uint8_t spi_read_reg(icm20948_dev_t *dev, uint8_t reg) {
    uint8_t tx[2] = { reg | SPI_READ_FLAG, 0x00 };
    uint8_t rx[2] = { 0, 0 };
    cs_low(dev);
    spi_write_read_blocking(dev->cfg.spi, tx, rx, 2);
    cs_high(dev);
    return rx[1];
    
}

static void spi_read_bytes(icm20948_dev_t *dev,
                            uint8_t reg, uint8_t *dst, size_t len) {
    /* tx: [reg|READ] then len dummy bytes */
    uint8_t tx_header = reg | SPI_READ_FLAG;
    cs_low(dev);
    spi_write_blocking(dev->cfg.spi, &tx_header, 1);
    /* Read len bytes – send zeros as dummy */
    uint8_t dummy[32] = {0};
    spi_write_read_blocking(dev->cfg.spi, dummy, dst, len < 32 ? len : 32);
    cs_high(dev);
}

/* =========================================================================
 * Register bank selection
 * ====================================================================== */

static void set_bank(icm20948_dev_t *dev, int bank) {
    if (dev->cur_bank != bank) {
        spi_write_reg(dev, REG_BANK_SEL, (uint8_t)((bank & 0x03) << 4));
        dev->cur_bank = bank;
    }
}

/* =========================================================================
 * Helper: signed 16-bit from two bytes (big-endian)
 * ====================================================================== */

static inline int16_t to_s16(uint8_t hi, uint8_t lo) {
    return (int16_t)((uint16_t)hi << 8 | lo);
}

/* =========================================================================
 * AK09916 I²C-master (SLV4 single-byte transactions)
 * ====================================================================== */

static icm20948_err_t ak_write(icm20948_dev_t *dev, uint8_t reg, uint8_t val) {
    set_bank(dev, 3);
    spi_write_reg(dev, REG_I2C_SLV4_ADDR, AK09916_I2C_ADDR);   /* write */
    spi_write_reg(dev, REG_I2C_SLV4_REG,  reg);
    spi_write_reg(dev, REG_I2C_SLV4_DO,   val);
    spi_write_reg(dev, REG_I2C_SLV4_CTRL, 0x80);                /* trigger */
    set_bank(dev, 0);
    /* Poll I2C_MST_STATUS[6] (SLV4_DONE) */
    for (int i = 0; i < 50; i++) {
        uint8_t s = spi_read_reg(dev, REG_I2C_MST_STATUS);
        if (s & 0x40) return ICM20948_OK;
        sleep_ms(2);
    }
    return ICM20948_ERR_MAG_TO;
}

static icm20948_err_t ak_read(icm20948_dev_t *dev,
                               uint8_t reg, uint8_t *out) {
    set_bank(dev, 3);
    spi_write_reg(dev, REG_I2C_SLV4_ADDR, AK09916_I2C_ADDR | 0x80); /* read */
    spi_write_reg(dev, REG_I2C_SLV4_REG,  reg);
    spi_write_reg(dev, REG_I2C_SLV4_CTRL, 0x80);
    set_bank(dev, 0);
    for (int i = 0; i < 50; i++) {
        uint8_t s = spi_read_reg(dev, REG_I2C_MST_STATUS);
        if (s & 0x40) {
            set_bank(dev, 3);
            *out = spi_read_reg(dev, REG_I2C_SLV4_DI);
            set_bank(dev, 0);
            return ICM20948_OK;
        }
        sleep_ms(2);
    }
    return ICM20948_ERR_MAG_TO;
}

/* =========================================================================
 * Magnetometer initialisation
 * ====================================================================== */

static icm20948_err_t init_magnetometer(icm20948_dev_t *dev) {
    icm20948_err_t err;

    /* Enable I²C master engine */
    set_bank(dev, 0);
    spi_write_reg(dev, REG_USER_CTRL, 0x20);
    sleep_ms(10);

    set_bank(dev, 3);
    spi_write_reg(dev, REG_I2C_MST_CTRL, 0x17);   /* 400 kHz */

    /* Reset AK09916 */
    err = ak_write(dev, AK_REG_CNTL3, 0x01);
    if (err != ICM20948_OK) return err;
    sleep_ms(10);

    /* Verify device ID */
    uint8_t id = 0;
    err = ak_read(dev, AK_REG_DEVICE_ID, &id);
    if (err != ICM20948_OK) return err;
    if (id != AK09916_DEVICE_ID) return ICM20948_ERR_MAG_ID;

    /* Continuous measurement mode 4 → 100 Hz, 16-bit */
    err = ak_write(dev, AK_REG_CNTL2, 0x08);
    if (err != ICM20948_OK) return err;
    sleep_ms(10);

    /*
     * Configure SLV0 to automatically DMA 9 bytes from AK09916 every sample:
     *   ST1 (1) | HX HY HZ (6 LE bytes) | ST2 (1) | padding (1) = 9 bytes
     * Results land at EXT_SLV_SENS_DATA_00 (0x3B) in Bank 0.
     */
    set_bank(dev, 3);
    spi_write_reg(dev, REG_I2C_SLV0_ADDR, AK09916_I2C_ADDR | 0x80); /* read */
    spi_write_reg(dev, REG_I2C_SLV0_REG,  AK_REG_STATUS1);
    spi_write_reg(dev, REG_I2C_SLV0_CTRL, 0x89);  /* enable | 9 bytes */
    set_bank(dev, 0);

    return ICM20948_OK;
}

/* =========================================================================
 * Public API implementation
 * ====================================================================== */

icm20948_err_t icm20948_init(icm20948_dev_t *dev,
                              const icm20948_config_t *cfg) {
    memcpy(&dev->cfg, cfg, sizeof(icm20948_config_t));
    dev->cur_bank   = -1;
    dev->gyro_sens  = GYRO_SENS_TABLE[cfg->gyro_fs];
    dev->accel_sens = ACCEL_SENS_TABLE[cfg->accel_fs];

    /* ---- Configure SPI GPIO ---- */
    spi_init(cfg->spi, cfg->baudrate);
    gpio_set_function(cfg->pin_sck,  GPIO_FUNC_SPI);
    gpio_set_function(cfg->pin_mosi, GPIO_FUNC_SPI);
    gpio_set_function(cfg->pin_miso, GPIO_FUNC_SPI);

    gpio_init(cfg->pin_cs);
    gpio_set_dir(cfg->pin_cs, GPIO_OUT);
    gpio_put(cfg->pin_cs, 1);

    /* ICM-20948 requires CPOL=1, CPHA=1 */
    spi_set_format(cfg->spi, 8, SPI_CPOL_1, SPI_CPHA_1, SPI_MSB_FIRST);

    sleep_ms(10);

    /* ---- Verify chip identity ---- */
    set_bank(dev, 0);
    uint8_t who = spi_read_reg(dev, REG_WHO_AM_I);
    if (who != ICM20948_WHO_AM_I_VAL) return ICM20948_ERR_WHO_AM_I;

    /* ---- Software reset ---- */
    spi_write_reg(dev, REG_PWR_MGMT_1, 0x80);
    sleep_ms(100);
    dev->cur_bank = -1; /* bank unknown after reset */

    /* ---- Wake up, auto clock ---- */
    set_bank(dev, 0);
    spi_write_reg(dev, REG_PWR_MGMT_1, 0x01);
    sleep_ms(30);

    /* ---- Enable accel + gyro ---- */
    spi_write_reg(dev, REG_PWR_MGMT_2, 0x00);

    /* ---- Gyroscope: full-scale, DLPF on ---- */
    set_bank(dev, 2);
    spi_write_reg(dev, REG_GYRO_CONFIG_1,
                  (uint8_t)((cfg->gyro_fs << 1) | 0x01));
    spi_write_reg(dev, REG_GYRO_SMPLRT_DIV, 0x00);  /* max rate */

    /* ---- Accelerometer: full-scale, DLPF on ---- */
    spi_write_reg(dev, REG_ACCEL_CONFIG,
                  (uint8_t)((cfg->accel_fs << 1) | 0x01));
    spi_write_reg(dev, REG_ACCEL_SMPLRT_DIV_2, 0x00);

    set_bank(dev, 0);

    /* ---- Magnetometer ---- */
    if (cfg->mag_enable) {
        icm20948_err_t merr = init_magnetometer(dev);
        if (merr != ICM20948_OK) return merr;
    }

    return ICM20948_OK;
}

/* -------------------------------------------------------------------------- */

icm20948_err_t icm20948_read_accel(icm20948_dev_t *dev,
                                    float *ax, float *ay, float *az) {
    set_bank(dev, 0);
    uint8_t raw[6];
    spi_read_bytes(dev, REG_ACCEL_XOUT_H, raw, 6);
    float s = dev->accel_sens;
    *ax = (float)to_s16(raw[0], raw[1]) / s * GRAVITY_MS2;
    *ay = (float)to_s16(raw[2], raw[3]) / s * GRAVITY_MS2;
    *az = (float)to_s16(raw[4], raw[5]) / s * GRAVITY_MS2;
    return ICM20948_OK;
}

icm20948_err_t icm20948_read_gyro(icm20948_dev_t *dev,
                                   float *gx, float *gy, float *gz) {
    set_bank(dev, 0);
    uint8_t raw[6];
    spi_read_bytes(dev, REG_GYRO_XOUT_H, raw, 6);
    float s = dev->gyro_sens;
    *gx = (float)to_s16(raw[0], raw[1]) / s;
    *gy = (float)to_s16(raw[2], raw[3]) / s;
    *gz = (float)to_s16(raw[4], raw[5]) / s;
    return ICM20948_OK;
}

icm20948_err_t icm20948_read_temp(icm20948_dev_t *dev, float *temp_c) {
    set_bank(dev, 0);
    uint8_t raw[2];
    spi_read_bytes(dev, REG_TEMP_OUT_H, raw, 2);
    int16_t t = to_s16(raw[0], raw[1]);
    *temp_c = ((float)t / 333.87f) + 21.0f;
    return ICM20948_OK;
}

icm20948_err_t icm20948_read_mag(icm20948_dev_t *dev,
                                  float *mx, float *my, float *mz,
                                  bool *mag_valid) {
    if (!dev->cfg.mag_enable) {
        *mag_valid = false;
        return ICM20948_ERR_MAG_RDY;
    }

    /*
     * DMA shadow at EXT_SLV_SENS_DATA_00 (Bank 0, 0x3B):
     * Byte 0: ST1  (DRDY = bit 0)
     * Byte 1: HXL, Byte 2: HXH
     * Byte 3: HYL, Byte 4: HYH
     * Byte 5: HZL, Byte 6: HZH
     * Byte 7: ST2  (HOFL = bit 3)
     */
    set_bank(dev, 0);
    uint8_t raw[9];
    spi_read_bytes(dev, REG_EXT_SLV_SENS_DATA, raw, 9);

    if (!(raw[0] & 0x01)) {     /* DRDY not set */
        *mag_valid = false;
        return ICM20948_ERR_MAG_RDY;
    }
    if (raw[7] & 0x08) {        /* HOFL: overflow */
        *mag_valid = false;
        return ICM20948_ERR_MAG_OVF;
    }

    /* AK09916 data is little-endian */
    *mx = (float)to_s16(raw[2], raw[1]) * MAG_SENS_UT;
    *my = (float)to_s16(raw[4], raw[3]) * MAG_SENS_UT;
    *mz = (float)to_s16(raw[6], raw[5]) * MAG_SENS_UT;
    *mag_valid = true;
    return ICM20948_OK;
}

icm20948_err_t icm20948_read_all(icm20948_dev_t *dev,
                                   icm20948_data_t *out,
                                   const icm20948_cal_t *cal) {
    icm20948_err_t err;

    err = icm20948_read_accel(dev, &out->ax, &out->ay, &out->az);
    if (err != ICM20948_OK) return err;

    err = icm20948_read_gyro(dev, &out->gx, &out->gy, &out->gz);
    if (err != ICM20948_OK) return err;

    err = icm20948_read_temp(dev, &out->temperature);
    if (err != ICM20948_OK) return err;

    /* Mag failures are non-fatal */
    icm20948_read_mag(dev, &out->mx, &out->my, &out->mz, &out->mag_valid);

    /* ---- Apply calibration ---- */
    if (cal) {
        out->ax -= cal->accel_offset[0];
        out->ay -= cal->accel_offset[1];
        out->az -= cal->accel_offset[2];

        out->gx -= cal->gyro_offset[0];
        out->gy -= cal->gyro_offset[1];
        out->gz -= cal->gyro_offset[2];

        if (out->mag_valid) {
            out->mx = (out->mx - cal->mag_offset[0]) * cal->mag_scale[0];
            out->my = (out->my - cal->mag_offset[1]) * cal->mag_scale[1];
            out->mz = (out->mz - cal->mag_offset[2]) * cal->mag_scale[2];
        }
    }

    return ICM20948_OK;
}

/* -------------------------------------------------------------------------- */

icm20948_err_t icm20948_set_gyro_fs(icm20948_dev_t *dev,
                                     icm20948_gyro_fs_t fs) {
    dev->cfg.gyro_fs = fs;
    dev->gyro_sens   = GYRO_SENS_TABLE[fs];
    set_bank(dev, 2);
    spi_write_reg(dev, REG_GYRO_CONFIG_1, (uint8_t)((fs << 1) | 0x01));
    set_bank(dev, 0);
    return ICM20948_OK;
}

icm20948_err_t icm20948_set_accel_fs(icm20948_dev_t *dev,
                                      icm20948_accel_fs_t fs) {
    dev->cfg.accel_fs = fs;
    dev->accel_sens   = ACCEL_SENS_TABLE[fs];
    set_bank(dev, 2);
    spi_write_reg(dev, REG_ACCEL_CONFIG, (uint8_t)((fs << 1) | 0x01));
    set_bank(dev, 0);
    return ICM20948_OK;
}

icm20948_err_t icm20948_set_sample_rate(icm20948_dev_t *dev,
                                         uint8_t gyro_div,
                                         uint8_t accel_div) {
    set_bank(dev, 2);
    spi_write_reg(dev, REG_GYRO_SMPLRT_DIV,    gyro_div);
    spi_write_reg(dev, REG_ACCEL_SMPLRT_DIV_2, accel_div);
    set_bank(dev, 0);
    return ICM20948_OK;
}

icm20948_err_t icm20948_sleep(icm20948_dev_t *dev) {
    set_bank(dev, 0);
    uint8_t pwr = spi_read_reg(dev, REG_PWR_MGMT_1);
    spi_write_reg(dev, REG_PWR_MGMT_1, pwr | 0x40);
    return ICM20948_OK;
}

icm20948_err_t icm20948_wake(icm20948_dev_t *dev) {
    set_bank(dev, 0);
    uint8_t pwr = spi_read_reg(dev, REG_PWR_MGMT_1);
    spi_write_reg(dev, REG_PWR_MGMT_1, pwr & (uint8_t)~0x40);
    return ICM20948_OK;
}

uint8_t icm20948_who_am_i(icm20948_dev_t *dev) {
    set_bank(dev, 0);
    return spi_read_reg(dev, REG_WHO_AM_I);
}