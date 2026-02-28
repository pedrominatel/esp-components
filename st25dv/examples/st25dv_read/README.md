# ST25DV Read Example

This example demonstrates how to use the ST25DV NFC/I2C EEPROM component with an ESP32:

- Reading the chip UID, IC reference, and memory size
- Writing a NDEF Capability Container (CC) and a NDEF URI record
- Reading back the raw NDEF payload
- Monitoring RF field presence via I2C polling and GPO interrupt
- Reading Energy Harvesting status

## Hardware Required

- Any ESP32-series board (ESP32, ESP32-S2, ESP32-S3, ESP32-C3, etc.)
- ST25DV04K, ST25DV16K, or ST25DV64K evaluation board or module
- NFC-capable smartphone or reader (for field detection testing)

## Hardware Connection

| ST25DV Pin | ESP32 GPIO       | Description              |
|------------|------------------|--------------------------|
| SDA        | GPIO21 (default) | I2C data                 |
| SCL        | GPIO22 (default) | I2C clock                |
| GPO        | GPIO19 (default) | Interrupt output (opt.)  |
| VCC        | 3.3 V            | Power supply             |
| GND        | GND              | Ground                   |

> The GPO pin is optional. Set `ST25DV_GPO_PIN = -1` in menuconfig to disable it.

## Configuration

Run `idf.py menuconfig` and navigate to **Driver ST25DV NFC/I2C EEPROM**:

| Config | Default | Description |
|--------|---------|-------------|
| `ST25DV_I2C_SDA` | 21 | I2C SDA GPIO |
| `ST25DV_I2C_SCL` | 22 | I2C SCL GPIO |
| `ST25DV_I2C_CLK_SPEED_HZ` | 100000 | I2C clock speed |
| `ST25DV_GPO_PIN` | -1 | GPO interrupt GPIO (-1 = disabled) |

## Build and Flash

```bash
idf.py build flash monitor
```

## Expected Output

```
I (xxx) ST25DV: ST25DV initialized: data=0x53 syst=0x57 mem=512 B
I (xxx) ST25DV-Example: UID: E0:04:01:XX:XX:XX:XX:XX
I (xxx) ST25DV-Example: IC Reference: 0x24
I (xxx) ST25DV-Example: User memory: 512 bytes
I (xxx) ST25DV-Example: Writing NDEF CC...
I (xxx) ST25DV-Example: Writing NDEF URI: https://espressif.com/components
I (xxx) ST25DV-Example: NDEF payload (30 bytes):
  D1 01 1A 55 04 65 73 70 72 65 73 73 69 66 2E 63
  6F 6D 2F 63 6F 6D 70 6F 6E 65 6E 74 73 FE
I (xxx) ST25DV-Example: Entering monitoring loop...
I (xxx) ST25DV-Example: RF field: absent
I (xxx) ST25DV-Example: EH: eh_on=0  field_on=0  vcc_on=1
```

Bring an NFC-capable phone near the tag to see field detection events and to open the written URI.
