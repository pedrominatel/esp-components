# TV-B-Gone for ESP-IDF

[![Component Registry](https://components.espressif.com/components/pedrominatel/tv-b-gone/badge.svg)](https://components.espressif.com/components/pedrominatel/tv-b-gone)

The Power to Turn TVs OFF, on ESP32.

This component brings the TV-B-Gone concept to ESP-IDF using the RMT TX peripheral. It sends integrated TV power codes, including NA and EU sets plus the Xiaomi power code, so an ESP32 device can perform a full TV-B-Gone style sweep in one-shot or continuous mode.

The original TV-B-Gone project and product line were created by Mitch Altman. For the official project, history, and hardware products, see the TV-B-Gone website: [tvbgone.com](https://www.tvbgone.com/).

This implementation is based on the BruceDevices firmware project (https://github.com/BruceDevices/firmware), which includes code from the original TV-B-Gone project.

## Features

- Send a full TV-B-Gone style IR sweep from an ESP32
- Run a single sweep with `tvbgone_ir_send_once()`
- Run continuous background sweeps with `tvbgone_ir_start()` / `tvbgone_ir_stop()`
- Select regional code coverage with `TVBGONE_IR_MODE_NA`, `TVBGONE_IR_MODE_EU`, or `TVBGONE_IR_MODE_BOTH`
- Include the Xiaomi power code in the same integrated transmit pipeline as the other codes
- Configure the IR TX pin with `CONFIG_TVBGONE_IR_TX_GPIO`

## What It Does

This component transmits bursts of infrared power codes that match many television models, similar to the original TV-B-Gone device. In practice, that means:

- one sweep sends the integrated Xiaomi code first, then the selected regional code sets
- continuous mode repeats the sweep with a configurable gap
- all transmission is handled by ESP-IDF RMT, so timing stays in the driver instead of bit-banging in software

## How to use

### Initialize the component

```c
tvbgone_ir_config_t config = TVBGONE_IR_DEFAULT_CONFIG();
config.code_gap_ms = 205;
config.sweep_gap_ms = 5000;

ESP_ERROR_CHECK(tvbgone_ir_init(&config));
ESP_ERROR_CHECK(tvbgone_ir_set_mode(TVBGONE_IR_MODE_BOTH));
```

### Send one sweep

```c
ESP_ERROR_CHECK(tvbgone_ir_send_once());
```

### Start/stop continuous mode

```c
ESP_ERROR_CHECK(tvbgone_ir_start());
// ...
ESP_ERROR_CHECK(tvbgone_ir_stop(pdMS_TO_TICKS(5000)));
```

### Deinitialize

```c
ESP_ERROR_CHECK(tvbgone_ir_deinit());
```

### Configure IR TX GPIO (Kconfig)

In `idf.py menuconfig`:

- `Component config -> TV-B-Gone IR Configuration -> IR TX GPIO pin`

## Example

See:

- `tv-b-gone/examples/tvbgone_ir_send_once`

This example uses a button on GPIO9 (active LOW) to toggle `tvbgone_ir_start()` and `tvbgone_ir_stop()`, giving you a simple TV-B-Gone style trigger on hardware.

## Resources

- [Official TV-B-Gone Website](https://www.tvbgone.com/)
- [ESP-IDF RMT Peripheral](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/rmt.html)
