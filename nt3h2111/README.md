# I2C Driver Component for the NT3H2111 NFC/I2C Bridge

[![Component Registry](https://components.espressif.com/components/pedrominatel/nt3h2111/badge.svg)](https://components.espressif.com/components/pedrominatel/nt3h2111)

The NT3H2111 is an NFC Forum Type 2 Tag compliant IC with an I2C interface. It features 1KB EEPROM memory that can be accessed via both NFC and I2C, making it ideal for NFC-enabled applications with microcontroller interface.

This driver provides a complete ESP-IDF component for the NT3H2111, including support for block-based memory access, field detection via GPIO interrupt, and NDEF parsing for WiFi provisioning.

## Features

- **Dual Interface Access**: Read/Write EEPROM memory via I2C while supporting NFC RF access
- **Memory Management**: 
  - User memory: 888 bytes (blocks 0x01-0x38)
  - Configuration block access
  - SRAM buffer for fast data exchange (blocks 0xF8-0xFB)
  - Session registers for status monitoring
- **Field Detection**: 
  - GPIO-based hardware field detection with interrupt support
  - FD pin monitoring (active LOW when NFC field present)
  - I2C session register reading for field status
- **NDEF Support**: 
  - WiFi credential parsing (WSC/WiFi Simple Configuration)
  - NDEF message parsing for NFC Forum records
- **Configuration**: Full Kconfig integration for GPIO pins and I2C settings

## Hardware Connection

| NT3H2111 Pin | ESP32 GPIO | Description |
|--------------|------------|-------------|
| SDA | GPIO21 (configurable) | I2C Data Line |
| SCL | GPIO22 (configurable) | I2C Clock Line |
| FD | GPIO19 (configurable) | Field Detection (optional, active LOW) |
| VCC | 3.3V | Power Supply |
| GND | GND | Ground |

**Note**: 
- I2C pull-up resistors are enabled internally by default
- FD pin is optional but recommended for instant field detection
- I2C speed up to 400kHz supported

## API Reference

### Device Management

```c
// Create NT3H2111 device handle
nt3h2111_handle_t nt3h2111_device_create(
    i2c_master_bus_handle_t bus_handle, 
    const uint16_t dev_addr,  // Use NT3H2111_I2C_ADDR (0x55)
    const uint32_t dev_speed  // Up to 400000 Hz
);

// Delete device handle
esp_err_t nt3h2111_device_delete(nt3h2111_handle_t dev_handle);
```

### Memory Operations

```c
// Read 16-byte block
esp_err_t nt3h2111_read_block(
    nt3h2111_handle_t dev_handle,
    uint8_t block_addr,  // 0x01-0xFB
    uint8_t *data        // 16-byte buffer
);

// Write 16-byte block (requires 50ms delay after write)
esp_err_t nt3h2111_write_block(
    nt3h2111_handle_t dev_handle,
    uint8_t block_addr,
    const uint8_t *data
);

// Read session registers
esp_err_t nt3h2111_read_session_regs(
    nt3h2111_handle_t dev_handle,
    uint8_t *data
);

// Read/Write configuration block
esp_err_t nt3h2111_read_config(nt3h2111_handle_t dev_handle, uint8_t *config);
esp_err_t nt3h2111_write_config(nt3h2111_handle_t dev_handle, const uint8_t *config);
```

### Field Detection

```c
// Check field via I2C session register (blocked during RF field)
esp_err_t nt3h2111_is_field_present(
    nt3h2111_handle_t dev_handle,
    bool *field_present
);

// Initialize GPIO field detection with interrupt
esp_err_t nt3h2111_init_field_detect_gpio(
    gpio_num_t fd_gpio,
    nt3h2111_fd_callback_t callback,  // ISR callback (can be NULL for polling)
    void *callback_arg
);

// Read FD pin state directly (reliable during RF field)
esp_err_t nt3h2111_read_fd_pin(
    gpio_num_t fd_gpio,
    bool *field_present  // true if FD pin is LOW
);
```

## Examples

### 1. Basic Read/Write Example

Located in `examples/nt3h2111_read/`, this example demonstrates:
- I2C initialization and device creation
- Reading session registers and configuration
- Writing and verifying user memory blocks
- GPIO-based field detection with interrupts
- Proper handling of I2C access during RF field

**Key Learning**: I2C access is blocked when NFC field is active. Use GPIO FD pin for real-time field detection and read I2C registers only when field is removed.

### 2. WiFi Provisioning Example

Located in `examples/nt3h2111_wifi/`, this example shows:
- Reading NDEF-formatted WiFi credentials from NFC tag
- Parsing WiFi Simple Configuration (WSC) records
- Extracting SSID and password from NFC memory
- Automatic WiFi connection using credentials

**Use Case**: Write WiFi credentials using NFC Tools app on your phone, then ESP32 reads and connects automatically.

## Quick Start Guide

### 1. Configure GPIO Pins

Run `idf.py menuconfig` and navigate to **Driver NT3H2111 NFC/I2C Bridge**:
- Set I2C SDA GPIO (default: 21)
- Set I2C SCL GPIO (default: 22)
- Set Field Detection GPIO (default: 19, -1 to disable)
- Set I2C clock speed (default: 100kHz, max: 400kHz)

### 2. Basic Usage

```c
#include "nt3h2111.h"

// Initialize I2C bus
i2c_master_bus_config_t i2c_bus_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = 0,
    .scl_io_num = CONFIG_NT3H2111_I2C_SCL,
    .sda_io_num = CONFIG_NT3H2111_I2C_SDA,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
};

i2c_master_bus_handle_t bus_handle;
i2c_new_master_bus(&i2c_bus_config, &bus_handle);

// Create NT3H2111 device
nt3h2111_handle_t nt3h2111 = nt3h2111_device_create(
    bus_handle, 
    NT3H2111_I2C_ADDR,
    100000  // 100kHz
);

// Read a block
uint8_t data[16];
nt3h2111_read_block(nt3h2111, 0x01, data);

// Write a block
uint8_t write_data[16] = {'H', 'e', 'l', 'l', 'o', ' ', 'N', 'F', 'C', '!', 0, 0, 0, 0, 0, 0};
nt3h2111_write_block(nt3h2111, 0x01, write_data);
vTaskDelay(pdMS_TO_TICKS(50));  // Wait for EEPROM write completion

// Initialize field detection
nt3h2111_init_field_detect_gpio(CONFIG_NT3H2111_FD_PIN, field_callback, NULL);

// Cleanup
nt3h2111_device_delete(nt3h2111);
i2c_del_master_bus(bus_handle);
```

### 3. Field Detection with Interrupt

```c
static void IRAM_ATTR field_detect_isr_handler(void *arg)
{
    // Set flag for main task
    g_field_changed = true;
}

void app_main(void)
{
    // ... initialize I2C and device ...
    
    // Setup field detection interrupt
    nt3h2111_init_field_detect_gpio(GPIO_NUM_19, field_detect_isr_handler, NULL);
    
    while (1) {
        if (g_field_changed) {
            g_field_changed = false;
            
            bool field_present;
            nt3h2111_read_fd_pin(GPIO_NUM_19, &field_present);
            
            if (field_present) {
                ESP_LOGI(TAG, "NFC field detected!");
            } else {
                ESP_LOGI(TAG, "NFC field removed");
                // Now safe to read I2C registers
            }
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

## Important Notes

### I2C Access Arbitration

The NT3H2111 implements arbitration between NFC RF and I2C interfaces:
- **During RF Field**: I2C reads may return all zeros or NACK
- **Solution**: Use GPIO FD pin for field detection, read I2C only when field is removed
- **Write Delay**: After writing to EEPROM, wait 50ms before next I2C operation

### Memory Map

| Block Range | Description | Size | Notes |
|-------------|-------------|------|-------|
| 0x00 | Capability Container | 16 bytes | Read-only via I2C |
| 0x01-0x38 | User Memory | 888 bytes | EEPROM, NFC writable |
| 0x39 | Configuration | 16 bytes | Session/Config registers |
| 0x3A | Configuration | 16 bytes | Settings (NC_REG) |
| 0xF8-0xFB | SRAM | 64 bytes | Fast exchange buffer |
| 0xFE | Session Regs | 16 bytes | Status information |

### Field Detection Pin

- **Active LOW**: FD pin goes LOW (0V) when NFC field present
- **Active HIGH**: FD pin is HIGH (3.3V) when no field
- **Interrupt**: Configure for both edges to catch field changes instantly
- **Reliable**: FD pin works even when I2C is blocked by RF

## WiFi Provisioning with NFC Tools

1. **Write WiFi Credentials**:
   - Install "NFC Tools" app on your phone
   - Tap "Write" → "Add a record" → "WiFi network"
   - Enter SSID and password
   - Tap phone to NT3H2111 tag to write

2. **Read and Connect**:
   - Flash `nt3h2111_wifi` example to ESP32
   - ESP32 automatically reads credentials and connects to WiFi

## Troubleshooting

**I2C NACK errors after write**: Wait 50ms after each EEPROM write operation.

**Field detection not working**: Ensure FD pin is connected to configured GPIO and check pin state with multimeter (should be 3.3V normally, 0V when NFC field present).

**All zeros when reading during RF field**: This is normal. Use GPIO FD pin to detect field removal, then read via I2C.

**WiFi credentials not parsing**: Ensure you used the "WiFi network" record type in NFC Tools, not plain text.

## Resources

- [NT3H2111 Product Page](https://www.nxp.com/products/rfid-nfc/nfc-hf/ntag/nfc-forum-compliant-i2c-bridge-and-ntag:NT3H2111_2211)
- [NT3H2111 Datasheet](https://www.nxp.com/docs/en/data-sheet/NT3H2111_2211.pdf)
- [NFC Forum Type 2 Tag Specification](https://nfc-forum.org/our-work/specification-releases/specifications/type-2-tag-2/)
- [WiFi Simple Configuration Technical Specification](https://www.wi-fi.org/)

## License

Apache License 2.0
