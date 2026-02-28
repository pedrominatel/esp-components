/*
 * SPDX-FileCopyrightText: 2015-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief ST25DV NFC/I2C EEPROM driver
 *
 * ST25DV is an ISO 15693 (Type 5 Tag) NFC/I2C EEPROM family from ST
 * (ST25DV04K: 512 B, ST25DV16K: 2 KB, ST25DV64K: 8 KB).
 *
 * Key characteristics:
 *  - Two I2C addresses: data area (0x53) and system config area (0x57)
 *  - 16-bit register addressing (MSB first) for all I2C transactions
 *  - Configurable GPO pin for interrupts (field detect, RF events, mailbox)
 *  - Energy harvesting output pin
 *  - NDEF support (ISO 15693 Type 5 Tag)
 *  - Configurable security zones with I2C password protection
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"

#if __has_include("i2c_bus.h")
#include "i2c_bus.h"
#endif

/* =========================================================================
 * I2C Addresses (7-bit)
 * ========================================================================= */

/** @brief User EEPROM, mailbox, and dynamic registers */
#define ST25DV_ADDR_DATA            ((uint16_t)0x53)
/** @brief System configuration registers (requires I2C password to write) */
#define ST25DV_ADDR_SYST            ((uint16_t)0x57)

/* =========================================================================
 * System Configuration Register Addresses (accessed via ST25DV_ADDR_SYST)
 * ========================================================================= */

#define ST25DV_GPO1_REG             ((uint16_t)0x0000)  /*!< GPO1 configuration */
#define ST25DV_GPO2_REG             ((uint16_t)0x0001)  /*!< GPO pulse duration (IT_TIME) */
#define ST25DV_EH_MODE_REG          ((uint16_t)0x0002)  /*!< Energy harvesting mode */
#define ST25DV_RF_MNGT_REG          ((uint16_t)0x0003)  /*!< RF management (disable/sleep) */
#define ST25DV_RFA1SS_REG           ((uint16_t)0x0004)  /*!< RF Zone 1 security status */
#define ST25DV_ENDA1_REG            ((uint16_t)0x0005)  /*!< Zone 1 end address (×32 bytes) */
#define ST25DV_RFA2SS_REG           ((uint16_t)0x0006)  /*!< RF Zone 2 security status */
#define ST25DV_ENDA2_REG            ((uint16_t)0x0007)  /*!< Zone 2 end address */
#define ST25DV_RFA3SS_REG           ((uint16_t)0x0008)  /*!< RF Zone 3 security status */
#define ST25DV_ENDA3_REG            ((uint16_t)0x0009)  /*!< Zone 3 end address */
#define ST25DV_RFA4SS_REG           ((uint16_t)0x000A)  /*!< RF Zone 4 security status */
#define ST25DV_I2CZSS_REG           ((uint16_t)0x000B)  /*!< I2C zone security (4 zones × 2 bits) */
#define ST25DV_LOCK_CCFILE_REG      ((uint16_t)0x000C)  /*!< Lock CC file from RF write */
#define ST25DV_MB_MODE_REG          ((uint16_t)0x000D)  /*!< Mailbox mode enable (static) */
#define ST25DV_MB_WDG_REG           ((uint16_t)0x000E)  /*!< Mailbox watchdog delay (×30 ms) */
#define ST25DV_LOCK_CFG_REG         ((uint16_t)0x000F)  /*!< Lock config registers from RF */
#define ST25DV_LOCK_DSFID_REG       ((uint16_t)0x0010)  /*!< DSFID lock flag */
#define ST25DV_LOCK_AFI_REG         ((uint16_t)0x0011)  /*!< AFI lock flag */
#define ST25DV_DSFID_REG            ((uint16_t)0x0012)  /*!< DSFID value */
#define ST25DV_AFI_REG              ((uint16_t)0x0013)  /*!< AFI value */
#define ST25DV_MEM_SIZE_REG         ((uint16_t)0x0014)  /*!< Memory size [15:0] (2 bytes) */
#define ST25DV_BLK_SIZE_REG         ((uint16_t)0x0016)  /*!< Block size minus 1 (1 byte) */
#define ST25DV_IC_REF_REG           ((uint16_t)0x0017)  /*!< IC reference / chip identifier */
#define ST25DV_UID_REG              ((uint16_t)0x0018)  /*!< 8-byte UID (ISO 15693) */
#define ST25DV_IC_REV_REG           ((uint16_t)0x0020)  /*!< IC revision */
#define ST25DV_I2CPASSWD_REG        ((uint16_t)0x0900)  /*!< I2C password register (17-byte frame) */

/* =========================================================================
 * Dynamic Register Addresses (accessed via ST25DV_ADDR_DATA)
 * ========================================================================= */

#define ST25DV_GPO_CTRL_DYN_REG     ((uint16_t)0x2000)  /*!< GPO dynamic control */
#define ST25DV_EH_CTRL_DYN_REG      ((uint16_t)0x2002)  /*!< Energy harvesting dynamic control */
#define ST25DV_RF_MNGT_DYN_REG      ((uint16_t)0x2003)  /*!< RF management dynamic */
#define ST25DV_I2C_SSO_DYN_REG      ((uint16_t)0x2004)  /*!< I2C secure session open flag */
#define ST25DV_IT_STS_DYN_REG       ((uint16_t)0x2005)  /*!< IT interrupt status (read clears) */
#define ST25DV_MB_CTRL_DYN_REG      ((uint16_t)0x2006)  /*!< Mailbox control dynamic */
#define ST25DV_MB_LEN_DYN_REG       ((uint16_t)0x2007)  /*!< Mailbox message length in bytes */
#define ST25DV_MAILBOX_RAM_REG      ((uint16_t)0x2008)  /*!< Mailbox data buffer (up to 256 bytes) */

/* =========================================================================
 * EH_CTRL_DYN Bit Masks
 * ========================================================================= */

#define ST25DV_EH_CTRL_DYN_EH_EN   (0x01U)  /*!< Enable energy harvesting output */
#define ST25DV_EH_CTRL_DYN_EH_ON   (0x02U)  /*!< EH output is currently active (read-only) */
#define ST25DV_EH_CTRL_DYN_FIELD   (0x04U)  /*!< RF field is currently present (read-only) */
#define ST25DV_EH_CTRL_DYN_VCC     (0x08U)  /*!< VCC supply is active (read-only) */

/* =========================================================================
 * IT_STS_DYN / GPO1 Bit Masks (same bit layout in both registers)
 * ========================================================================= */

#define ST25DV_GPO_RFUSERSTATE      (0x01U)  /*!< RF user state change */
#define ST25DV_GPO_RFBUSY           (0x02U)  /*!< RF busy */
#define ST25DV_GPO_RFINTERRUPT      (0x04U)  /*!< RF interrupt */
#define ST25DV_GPO_FIELDFALLING     (0x08U)  /*!< RF field falling edge */
#define ST25DV_GPO_FIELDRISING      (0x10U)  /*!< RF field rising edge */
#define ST25DV_GPO_RFPUTMSG         (0x20U)  /*!< RF has written to mailbox */
#define ST25DV_GPO_RFGETMSG         (0x40U)  /*!< RF has read from mailbox */
#define ST25DV_GPO_RFWRITE          (0x80U)  /*!< RF write to user memory */

/* =========================================================================
 * NDEF / CC file constants
 * ========================================================================= */

#define ST25DV_NDEF_CC_ADDR         ((uint16_t)0x0000) /*!< Capability Container starts at byte 0 */
#define ST25DV_NDEF_MSG_ADDR        ((uint16_t)0x0004) /*!< NDEF messages start at byte 4 */
#define ST25DV_NDEF_CC_MAGIC        ((uint8_t)0xE1)    /*!< CC file magic byte */
#define ST25DV_NDEF_CC_VERSION      ((uint8_t)0x40)    /*!< CC version + access conditions */
#define ST25DV_NDEF_TLV_NDEF_TYPE   ((uint8_t)0x03)    /*!< NDEF TLV type byte */
#define ST25DV_NDEF_TLV_TERMINATOR  ((uint8_t)0xFE)    /*!< TLV stream terminator */

/** @brief Maximum I2C write burst size */
#define ST25DV_MAX_WRITE_BYTES      256
/** @brief Mailbox buffer maximum length */
#define ST25DV_MAX_MAILBOX_LEN      256
/** @brief EEPROM write timeout (ms) — device NACKs during write cycle */
#define ST25DV_WRITE_TIMEOUT_MS     10
/** @brief I2C password length in bytes */
#define ST25DV_PASSWD_LEN           8

/* =========================================================================
 * Types
 * ========================================================================= */

/**
 * @brief Energy Harvesting operating mode
 */
typedef enum {
    ST25DV_EH_MODE_ON_DEMAND    = 0, /*!< EH output enabled only when explicitly requested via EH_CTRL_DYN */
    ST25DV_EH_MODE_AFTER_BOOT   = 1, /*!< EH output activates automatically when RF field is present */
} st25dv_eh_mode_t;

/**
 * @brief Energy Harvesting runtime status
 */
typedef struct {
    bool eh_enabled;   /*!< EH enable bit is set (EH_EN) */
    bool eh_on;        /*!< EH output is currently active (EH_ON, read-only) */
    bool field_on;     /*!< RF field is currently present (FIELD_ON, read-only) */
    bool vcc_on;       /*!< VCC supply is active (VCC_ON, read-only) */
} st25dv_eh_status_t;

/**
 * @brief Callback function type for GPO interrupt events
 *
 * @param arg User argument passed during registration
 */
typedef void (*st25dv_gpo_callback_t)(void *arg);

/**
 * @brief Opaque ST25DV device handle
 */
typedef struct st25dv_dev_t *st25dv_handle_t;

/* =========================================================================
 * Device Lifecycle
 * ========================================================================= */

/**
 * @brief Create and initialize an ST25DV device using the native IDF i2c_master driver.
 *
 * Probes both I2C addresses (0x53 for user data, 0x57 for system config), registers
 * both devices on the bus, and reads the chip's memory size.
 *
 * @param bus_handle Handle to the I2C master bus.
 * @param dev_speed  SCL speed in Hz (up to 400 kHz).
 * @return st25dv_handle_t on success, NULL on failure.
 */
st25dv_handle_t st25dv_device_create(i2c_master_bus_handle_t bus_handle, const uint32_t dev_speed);

/**
 * @brief Delete the ST25DV device and release all resources.
 *
 * Removes both I2C device registrations and frees the handle memory.
 *
 * @param handle Handle returned by st25dv_device_create or st25dv_device_create_i2cbus.
 * @return ESP_OK on success, or an error code.
 */
esp_err_t st25dv_device_delete(st25dv_handle_t handle);

#if __has_include("i2c_bus.h")
/**
 * @brief Create an ST25DV handle from two existing i2c_bus device handles.
 *
 * Use this when your application already uses espressif/i2c_bus to manage the bus.
 * The caller must create two i2c_bus devices: one at address 0x53 (data) and one
 * at address 0x57 (system config).
 *
 * @param data_bus_dev i2c_bus device handle for the data address (0x53).
 * @param syst_bus_dev i2c_bus device handle for the system address (0x57).
 * @return st25dv_handle_t on success, NULL on failure.
 */
st25dv_handle_t st25dv_device_create_i2cbus(i2c_bus_device_handle_t data_bus_dev,
                                             i2c_bus_device_handle_t syst_bus_dev);
#endif

/* =========================================================================
 * Chip Information
 * ========================================================================= */

/**
 * @brief Read the 8-byte ISO 15693 UID from the ST25DV.
 *
 * @param handle   Device handle.
 * @param uid_buf  Buffer of at least 8 bytes to receive the UID (LSB first).
 * @return ESP_OK, or an error code.
 */
esp_err_t st25dv_get_uid(st25dv_handle_t handle, uint8_t uid_buf[8]);

/**
 * @brief Read the 1-byte IC Reference (chip identifier).
 *
 * @param handle Device handle.
 * @param ic_ref Pointer to receive the IC reference byte.
 * @return ESP_OK, or an error code.
 */
esp_err_t st25dv_get_ic_ref(st25dv_handle_t handle, uint8_t *ic_ref);

/**
 * @brief Read and decode the total user memory size in bytes.
 *
 * Reads MEM_SIZE_REG and BLK_SIZE_REG from the system area.
 * Formula: mem_size = (Mem_Size[15:0] + 1) × (BlockSize + 1)
 *
 * @param handle    Device handle.
 * @param mem_bytes Pointer to receive the total memory size in bytes.
 * @return ESP_OK, or an error code.
 */
esp_err_t st25dv_get_mem_size(st25dv_handle_t handle, uint32_t *mem_bytes);

/* =========================================================================
 * EEPROM Read / Write
 * ========================================================================= */

/**
 * @brief Read bytes from the ST25DV user EEPROM.
 *
 * @param handle   Device handle.
 * @param reg_addr 16-bit start address in user EEPROM (0x0000–end of memory).
 * @param data     Buffer to receive the data.
 * @param len      Number of bytes to read.
 * @return ESP_OK, or an error code.
 */
esp_err_t st25dv_read_bytes(st25dv_handle_t handle, uint16_t reg_addr, uint8_t *data, size_t len);

/**
 * @brief Write bytes to the ST25DV user EEPROM.
 *
 * Handles the EEPROM write cycle: sends data and ACK-polls until write completes.
 * Max safe write burst is ST25DV_MAX_WRITE_BYTES (256) bytes staying within one
 * 256-byte sector boundary.
 *
 * @param handle   Device handle.
 * @param reg_addr 16-bit start address in user EEPROM.
 * @param data     Data to write.
 * @param len      Number of bytes to write.
 * @return ESP_OK on completion, ESP_ERR_TIMEOUT if ACK polling exceeds ST25DV_WRITE_TIMEOUT_MS.
 */
esp_err_t st25dv_write_bytes(st25dv_handle_t handle, uint16_t reg_addr, const uint8_t *data, size_t len);

/* =========================================================================
 * Field Detection & GPO
 * ========================================================================= */

/**
 * @brief Check if an RF field is currently present (I2C polling, no GPIO required).
 *
 * Reads the FIELD_ON bit from EH_CTRL_DYN_REG via the data I2C address.
 *
 * @param handle        Device handle.
 * @param field_present Set to true if an RF field is currently active.
 * @return ESP_OK, or an error code.
 */
esp_err_t st25dv_is_field_present(st25dv_handle_t handle, bool *field_present);

/**
 * @brief Read and clear IT interrupt status flags.
 *
 * Reads IT_STS_DYN_REG; reading this register automatically clears all flags.
 * Use ST25DV_GPO_* masks to test individual bits.
 *
 * @param handle     Device handle.
 * @param it_status  Pointer to receive the 1-byte status bitmask.
 * @return ESP_OK, or an error code.
 */
esp_err_t st25dv_read_it_status(st25dv_handle_t handle, uint8_t *it_status);

/**
 * @brief Configure the GPO interrupt sources (writes GPO1_REG in system area).
 *
 * @note Requires security session open (I2C password presented) if LOCK_CFG is set.
 *
 * @param handle     Device handle.
 * @param gpo_config Bitmask of ST25DV_GPO_* flags to enable.
 * @return ESP_OK, or an error code.
 */
esp_err_t st25dv_gpo_configure(st25dv_handle_t handle, uint8_t gpo_config);

/**
 * @brief Initialize a GPIO pin as input for GPO interrupt handling.
 *
 * The ST25DV GPO is an open-drain, active-LOW output. This function configures
 * the given GPIO as input with pull-up and registers an ISR on both edges.
 *
 * @param gpo_gpio    GPIO number connected to ST25DV GPO pin.
 * @param callback    ISR function called on GPO edge (IRAM_ATTR required).
 * @param callback_arg User argument forwarded to the callback.
 * @return ESP_OK, or an error code.
 */
esp_err_t st25dv_init_gpo_gpio(gpio_num_t gpo_gpio, st25dv_gpo_callback_t callback, void *callback_arg);

/**
 * @brief Read the current GPO pin level from GPIO.
 *
 * GPO is active LOW: pin LOW means an event is active.
 *
 * @param gpo_gpio     GPIO number connected to ST25DV GPO pin.
 * @param gpo_asserted Set to true if GPO is asserted (pin LOW).
 * @return ESP_OK, or an error code.
 */
esp_err_t st25dv_read_gpo_pin(gpio_num_t gpo_gpio, bool *gpo_asserted);

/* =========================================================================
 * Energy Harvesting
 * ========================================================================= */

/**
 * @brief Set the energy harvesting operating mode (writes EH_MODE_REG in system area).
 *
 * @note Requires security session open (I2C password presented) if LOCK_CFG is set.
 *
 * @param handle  Device handle.
 * @param mode    ST25DV_EH_MODE_ON_DEMAND or ST25DV_EH_MODE_AFTER_BOOT.
 * @return ESP_OK, or an error code.
 */
esp_err_t st25dv_eh_set_mode(st25dv_handle_t handle, st25dv_eh_mode_t mode);

/**
 * @brief Enable energy harvesting output at runtime (sets EH_EN in EH_CTRL_DYN_REG).
 *
 * Has no effect if EH_MODE is ST25DV_EH_MODE_AFTER_BOOT (EH is automatic in that mode).
 *
 * @param handle Device handle.
 * @return ESP_OK, or an error code.
 */
esp_err_t st25dv_eh_enable(st25dv_handle_t handle);

/**
 * @brief Disable energy harvesting output at runtime (clears EH_EN in EH_CTRL_DYN_REG).
 *
 * @param handle Device handle.
 * @return ESP_OK, or an error code.
 */
esp_err_t st25dv_eh_disable(st25dv_handle_t handle);

/**
 * @brief Read current energy harvesting status flags.
 *
 * @param handle Device handle.
 * @param status Pointer to st25dv_eh_status_t to receive the status.
 * @return ESP_OK, or an error code.
 */
esp_err_t st25dv_eh_get_status(st25dv_handle_t handle, st25dv_eh_status_t *status);

/* =========================================================================
 * NDEF
 * ========================================================================= */

/**
 * @brief Write the 4-byte NDEF Capability Container (CC) at address 0x0000.
 *
 * The CC marks the memory as a valid NDEF Type 5 Tag.
 * CC bytes: {0xE1, 0x40, mlen, 0x00} where mlen = (mem_size_bytes - 4) / 8.
 * Memory size is read from the device automatically.
 *
 * @param handle Device handle.
 * @return ESP_OK, or an error code.
 */
esp_err_t st25dv_ndef_write_cc(st25dv_handle_t handle);

/**
 * @brief Write an NDEF URI record to user memory starting at address 0x0004.
 *
 * Encodes the URI as an NDEF Well-Known Record (type 'U') with identifier
 * byte 0x04 (https:// prefix). Pass the URI without the scheme, e.g.
 * "espressif.com/components" for "https://espressif.com/components".
 *
 * @param handle Device handle.
 * @param uri    Null-terminated URI string (without "https://").
 * @return ESP_OK, or an error code.
 */
esp_err_t st25dv_ndef_write_uri(st25dv_handle_t handle, const char *uri);

/**
 * @brief Write an NDEF Text record to user memory starting at address 0x0004.
 *
 * Encodes the text as an NDEF Well-Known Record (type 'T') with the given
 * language code (e.g. "en").
 *
 * @param handle Device handle.
 * @param text   Null-terminated text string.
 * @param lang   Null-terminated ISO 639 language code (e.g. "en", "pt").
 * @return ESP_OK, or an error code.
 */
esp_err_t st25dv_ndef_write_text(st25dv_handle_t handle, const char *text, const char *lang);

/**
 * @brief Read raw NDEF TLV payload from user memory.
 *
 * Validates the CC magic byte (0xE1), then reads the NDEF TLV block and
 * copies the payload bytes (everything inside the NDEF TLV, after the length
 * field and before the terminator) to the caller's buffer.
 *
 * @param handle   Device handle.
 * @param buf      Buffer to receive the raw NDEF payload bytes.
 * @param buf_len  Length of the buffer.
 * @param out_len  Pointer to receive the actual number of bytes written.
 * @return ESP_OK, ESP_ERR_INVALID_STATE if CC magic is wrong, ESP_ERR_NO_MEM if buffer too small.
 */
esp_err_t st25dv_ndef_read_raw(st25dv_handle_t handle, uint8_t *buf, size_t buf_len, size_t *out_len);

#ifdef __cplusplus
}
#endif
