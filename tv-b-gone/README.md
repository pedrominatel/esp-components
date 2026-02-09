# IR TV Power-Off Component (TV-B-Gone Style)

[![Component Registry](https://components.espressif.com/components/pedrominatel/tv-b-gone/badge.svg)](https://components.espressif.com/components/pedrominatel/tv-b-gone)

This component transmits IR power-off code sets (NA and EU) using ESP-IDF RMT TX. The Xiaomi raw power code is transmitted by default before regional code sets, and it supports one-shot or continuous sweep mode.

This implementation is based on the BruceDevices firmware project (https://github.com/BruceDevices/firmware), which includes code from the original TV-B-Gone project by Mitch Altman.

## Features

- Send one full sweep of IR power-off codes
- Start/stop continuous sweeping in a background task
- Select region mode: `TVBGONE_IR_MODE_NA`, `TVBGONE_IR_MODE_EU`, or `TVBGONE_IR_MODE_BOTH`
- Xiaomi power code is sent by default before region sweeps
- Kconfig support for IR TX GPIO (`CONFIG_TVBGONE_IR_TX_GPIO`)

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

This example uses a button on GPIO9 (active LOW) to toggle `tvbgone_ir_start()` and `tvbgone_ir_stop()`.

## Resources

- [ESP-IDF RMT Peripheral](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/rmt.html)
