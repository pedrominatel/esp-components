# I2C Driver Component for the ST25DV NFC/I2C EEPROM

[![Component Registry](https://components.espressif.com/components/pedrominatel/st25dv/badge.svg)](https://components.espressif.com/components/pedrominatel/st25dv)

The ST25DV is an ISO 15693 (Type 5 Tag) NFC/I2C EEPROM family from STMicroelectronics. It provides a dual-interface memory that can be accessed via both 13.56 MHz NFC RF and I2C, with variants from 512 bytes (ST25DV04K) up to 8 KB (ST25DV64K). Key features include a configurable GPO interrupt pin, an energy harvesting output, a 256-byte mailbox for fast RF-to-I2C transfer, and per-zone memory protection.

This driver provides a complete ESP-IDF component for the ST25DV family, including byte-addressed EEPROM read/write, NDEF Type 5 Tag support (CC + URI/Text records), field detection via GPIO interrupt or I2C polling, energy harvesting control, and dual-mode support for both the native `i2c_master` API and the `espressif/i2c_bus` component.

## Features

- **Dual I2C address management** — registers and manages both the user data address (0x53) and system config address (0x57) transparently
- **EEPROM read/write** — byte-addressed access with automatic EEPROM write-cycle ACK polling
- **Chip identification** — read the 8-byte ISO 15693 UID, IC reference, and auto-detected memory size
- **NDEF Type 5 Tag** — write Capability Container, NDEF URI records (with `https://` prefix), NDEF Text records; read raw NDEF payload
- **Field detection** — poll `FIELD_ON` bit via I2C or register a GPIO ISR on the GPO pin; read and clear interrupt status flags
- **GPO interrupt configuration** — enable/disable individual GPO event sources (field rising/falling, RF write, mailbox events, etc.)
- **Energy Harvesting** — set EH mode (on-demand or after-boot), enable/disable EH output at runtime, read EH status flags
- **i2c_bus compatibility** — optional `st25dv_device_create_i2cbus()` for projects using `espressif/i2c_bus`
- **Full Kconfig integration** — configurable I2C pins, speed, and GPO GPIO via `menuconfig`

## Hardware Connection

| ST25DV Pin | ESP32 GPIO        | Description                        |
|------------|-------------------|------------------------------------|
| SDA        | GPIO21 (default)  | I2C data                           |
| SCL        | GPIO22 (default)  | I2C clock                          |
| GPO        | GPIO (configurable) | Interrupt output (optional, active LOW) |
| EH_OUT     | External load     | Energy harvesting output (optional) |
| VCC        | 3.3 V             | Power supply                       |
| GND        | GND               | Ground                             |

> I2C pull-up resistors are enabled internally by default. The GPO pin is optional — set `ST25DV_GPO_PIN = -1` to disable it.

## API Reference

### Device Management

```c
// Create ST25DV device (registers both I2C addresses 0x53 and 0x57)
st25dv_handle_t st25dv_device_create(
    i2c_master_bus_handle_t bus_handle,
    const uint32_t dev_speed   // Up to 400000 Hz
);

// Delete device and release resources
esp_err_t st25dv_device_delete(st25dv_handle_t handle);

// Create from existing i2c_bus device handles (optional, requires espressif/i2c_bus)
// Caller must create one i2c_bus device at 0x53 and one at 0x57
st25dv_handle_t st25dv_device_create_i2cbus(
    i2c_bus_device_handle_t data_bus_dev,  // address 0x53
    i2c_bus_device_handle_t syst_bus_dev   // address 0x57
);
```

### Chip Information

```c
// Read 8-byte ISO 15693 UID (LSB first in buffer)
esp_err_t st25dv_get_uid(st25dv_handle_t handle, uint8_t uid_buf[8]);

// Read IC reference byte (chip identification)
esp_err_t st25dv_get_ic_ref(st25dv_handle_t handle, uint8_t *ic_ref);

// Read total user memory size in bytes (auto-detected from MEM_SIZE and BLK_SIZE registers)
esp_err_t st25dv_get_mem_size(st25dv_handle_t handle, uint32_t *mem_bytes);
```

### EEPROM Read / Write

```c
// Read bytes from user EEPROM (16-bit byte address, up to memory size)
esp_err_t st25dv_read_bytes(
    st25dv_handle_t handle,
    uint16_t reg_addr,    // 0x0000 – end of memory
    uint8_t *data,
    size_t len
);

// Write bytes to user EEPROM (automatically polls for write cycle completion)
esp_err_t st25dv_write_bytes(
    st25dv_handle_t handle,
    uint16_t reg_addr,
    const uint8_t *data,
    size_t len            // Max ST25DV_MAX_WRITE_BYTES (256) per call
);
```

### Field Detection & GPO

```c
// Poll RF field presence via I2C (reads FIELD_ON bit in EH_CTRL_DYN_REG)
esp_err_t st25dv_is_field_present(st25dv_handle_t handle, bool *field_present);

// Read and clear interrupt status flags (IT_STS_DYN_REG)
// Use ST25DV_GPO_FIELDRISING, ST25DV_GPO_FIELDFALLING, etc. to test bits
esp_err_t st25dv_read_it_status(st25dv_handle_t handle, uint8_t *it_status);

// Configure GPO interrupt event sources (writes GPO1_REG in system area)
esp_err_t st25dv_gpo_configure(st25dv_handle_t handle, uint8_t gpo_config);

// Initialize GPIO as input for GPO interrupt (active LOW, both edges)
esp_err_t st25dv_init_gpo_gpio(
    gpio_num_t gpo_gpio,
    st25dv_gpo_callback_t callback,  // IRAM_ATTR required
    void *callback_arg
);

// Read GPO pin state directly from GPIO
esp_err_t st25dv_read_gpo_pin(gpio_num_t gpo_gpio, bool *gpo_asserted);
```

### Energy Harvesting

```c
// Set EH operating mode (writes EH_MODE_REG in system area)
esp_err_t st25dv_eh_set_mode(st25dv_handle_t handle, st25dv_eh_mode_t mode);
//   ST25DV_EH_MODE_ON_DEMAND  — enabled explicitly via st25dv_eh_enable()
//   ST25DV_EH_MODE_AFTER_BOOT — enabled automatically when RF field is present

// Enable EH output at runtime (sets EH_EN bit in EH_CTRL_DYN_REG)
esp_err_t st25dv_eh_enable(st25dv_handle_t handle);

// Disable EH output at runtime
esp_err_t st25dv_eh_disable(st25dv_handle_t handle);

// Read EH status flags
esp_err_t st25dv_eh_get_status(st25dv_handle_t handle, st25dv_eh_status_t *status);
// status.eh_enabled — EH_EN bit is set
// status.eh_on      — EH output is currently active (read-only)
// status.field_on   — RF field is present (read-only)
// status.vcc_on     — VCC supply active (read-only)
```

### NDEF (Type 5 Tag)

```c
// Write 4-byte Capability Container at address 0x0000
// Must be called before writing any NDEF records
esp_err_t st25dv_ndef_write_cc(st25dv_handle_t handle);

// Write NDEF URI record (https:// prefix is added automatically)
// uri = "espressif.com/components" → writes "https://espressif.com/components"
esp_err_t st25dv_ndef_write_uri(st25dv_handle_t handle, const char *uri);

// Write NDEF Text record with language code
esp_err_t st25dv_ndef_write_text(st25dv_handle_t handle, const char *text, const char *lang);

// Read raw NDEF TLV payload (validates CC magic, returns payload bytes)
esp_err_t st25dv_ndef_read_raw(
    st25dv_handle_t handle,
    uint8_t *buf,
    size_t buf_len,
    size_t *out_len
);
```

## Examples

### `st25dv_read`

Demonstrates initializing the ST25DV, reading chip info (UID, IC reference, memory size), writing and reading back a NDEF URI record, monitoring RF field presence via I2C polling and the GPO interrupt, and reading energy harvesting status. See [examples/st25dv_read](examples/st25dv_read).

## Quick Start Guide

### 1. Configure via menuconfig

```
idf.py menuconfig
→ Driver ST25DV NFC/I2C EEPROM → I2C → set SDA, SCL, speed
→ Driver ST25DV NFC/I2C EEPROM → set GPO pin (-1 to disable)
```

### 2. Basic usage

```c
#include "st25dv.h"

// Initialize I2C bus
i2c_master_bus_config_t bus_cfg = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port   = 0,
    .scl_io_num = CONFIG_ST25DV_I2C_SCL,
    .sda_io_num = CONFIG_ST25DV_I2C_SDA,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
};
i2c_master_bus_handle_t bus_handle;
i2c_new_master_bus(&bus_cfg, &bus_handle);

// Create device (probes both 0x53 and 0x57, reads memory size)
st25dv_handle_t st25dv = st25dv_device_create(bus_handle, CONFIG_ST25DV_I2C_CLK_SPEED_HZ);

// Read UID
uint8_t uid[8];
st25dv_get_uid(st25dv, uid);

// Write NDEF URI
st25dv_ndef_write_cc(st25dv);
st25dv_ndef_write_uri(st25dv, "espressif.com/components");

// Poll field presence
bool field_present = false;
st25dv_is_field_present(st25dv, &field_present);

// Cleanup
st25dv_device_delete(st25dv);
i2c_del_master_bus(bus_handle);
```

### 3. Using with espressif/i2c_bus

```c
// Create two i2c_bus devices manually
i2c_bus_device_handle_t data_dev = i2c_bus_device_create(bus, ST25DV_ADDR_DATA, speed);
i2c_bus_device_handle_t syst_dev = i2c_bus_device_create(bus, ST25DV_ADDR_SYST, speed);

st25dv_handle_t st25dv = st25dv_device_create_i2cbus(data_dev, syst_dev);
```
