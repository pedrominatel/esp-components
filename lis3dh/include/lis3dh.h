#ifndef LIS3DH_H
#define LIS3DH_H

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

/* I2C Addresses */
#define LIS3DH_ADDR_LOW   0x18  /* SA0 pin = 0 */
#define LIS3DH_ADDR_HIGH  0x19  /* SA0 pin = 1 */

/* Register Addresses */
#define LIS3DH_REG_WHO_AM_I       0x0F
#define LIS3DH_REG_CTRL_REG1      0x20
#define LIS3DH_REG_CTRL_REG2      0x21
#define LIS3DH_REG_CTRL_REG3      0x22
#define LIS3DH_REG_CTRL_REG4      0x23
#define LIS3DH_REG_CTRL_REG5      0x24
#define LIS3DH_REG_CTRL_REG6      0x25
#define LIS3DH_REG_STATUS_REG     0x27
#define LIS3DH_REG_OUT_X_L        0x28
#define LIS3DH_REG_OUT_X_H        0x29
#define LIS3DH_REG_OUT_Y_L        0x2A
#define LIS3DH_REG_OUT_Y_H        0x2B
#define LIS3DH_REG_OUT_Z_L        0x2C
#define LIS3DH_REG_OUT_Z_H        0x2D
#define LIS3DH_REG_INT1_CFG       0x30
#define LIS3DH_REG_INT1_SRC       0x31
#define LIS3DH_REG_INT1_THS       0x32
#define LIS3DH_REG_INT1_DURATION  0x33

/* WHO_AM_I value */
#define LIS3DH_ID 0x33

/* Data rate options */
typedef enum {
    LIS3DH_ODR_POWER_DOWN = 0x00,
    LIS3DH_ODR_1_HZ       = 0x01,
    LIS3DH_ODR_10_HZ      = 0x02,
    LIS3DH_ODR_25_HZ      = 0x03,
    LIS3DH_ODR_50_HZ      = 0x04,
    LIS3DH_ODR_100_HZ     = 0x05,
    LIS3DH_ODR_200_HZ     = 0x06,
    LIS3DH_ODR_400_HZ     = 0x07,
} lis3dh_odr_t;

/* Full scale range options */
typedef enum {
    LIS3DH_FS_2G  = 0x00,
    LIS3DH_FS_4G  = 0x01,
    LIS3DH_FS_8G  = 0x02,
    LIS3DH_FS_16G = 0x03,
} lis3dh_fs_t;

/* Accelerometer data structure */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} lis3dh_accel_t;

/* Device handle */
typedef void* lis3dh_handle_t;

/* Configuration structure */
typedef struct {
    i2c_master_bus_handle_t i2c_bus;
    uint8_t i2c_addr;
    lis3dh_odr_t odr;
    lis3dh_fs_t fs;
} lis3dh_config_t;

/**
 * @brief Initialize LIS3DH sensor
 * @param config Pointer to configuration structure
 * @param out_handle Pointer to device handle
 * @return ESP_OK on success
 */
esp_err_t lis3dh_init(const lis3dh_config_t *config, lis3dh_handle_t *out_handle);

/**
 * @brief Deinitialize LIS3DH sensor
 * @param handle Device handle
 * @return ESP_OK on success
 */
esp_err_t lis3dh_deinit(lis3dh_handle_t handle);

/**
 * @brief Read WHO_AM_I register
 * @param handle Device handle
 * @param id Pointer to store WHO_AM_I value
 * @return ESP_OK on success
 */
esp_err_t lis3dh_whoami(lis3dh_handle_t handle, uint8_t *id);

/**
 * @brief Read accelerometer data
 * @param handle Device handle
 * @param accel Pointer to accelerometer data structure
 * @return ESP_OK on success
 */
esp_err_t lis3dh_read_accel(lis3dh_handle_t handle, lis3dh_accel_t *accel);

/**
 * @brief Set output data rate
 * @param handle Device handle
 * @param odr Output data rate value
 * @return ESP_OK on success
 */
esp_err_t lis3dh_set_odr(lis3dh_handle_t handle, lis3dh_odr_t odr);

/**
 * @brief Set full scale range
 * @param handle Device handle
 * @param fs Full scale range value
 * @return ESP_OK on success
 */
esp_err_t lis3dh_set_fs(lis3dh_handle_t handle, lis3dh_fs_t fs);

#ifdef __cplusplus
}
#endif

#endif /* LIS3DH_H */
