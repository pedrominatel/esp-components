# TV-B-Gone for ESP-IDF

`tv-b-gone` packages the current TV-B-Gone ESP-IDF firmware as a reusable component for ESP-IDF projects while keeping the public C API under `tvbgone_core`.

This project is based on the [TV-B-Gone-kit_V2](https://github.com/maltman23/TV-B-Gone-kit_V2) by Mitch Altman.

## What It Includes

- The current TV-B-Gone IR transmit logic
- The bundled `WORLDcodes.cpp` TV-B-Gone power-code database shared with the Arduino implementation
- A root-level component layout consistent with the rest of this repository

## Core Responsibility

The core configures and drives only the IR output hardware. Button handling,
visible LED signaling, and any board-specific UI behavior belong in the
application or example code.

## Public API

```c
#include "tvbgone_core.h"

tvbgone_core_config_t config;
tvbgone_core_get_default_config(&config);
config.ir_led_gpio = GPIO_NUM_2;

ESP_ERROR_CHECK(tvbgone_core_init(&config));
ESP_ERROR_CHECK(tvbgone_core_send(TVBGONE_CORE_REGION_NA,
                                  TVBGONE_CORE_SEND_MODE_SINGLE));
```

Available send options:

- Regions: `TVBGONE_CORE_REGION_NA`, `TVBGONE_CORE_REGION_EU`, `TVBGONE_CORE_REGION_BOTH`
- Modes: `TVBGONE_CORE_SEND_MODE_SINGLE`, `TVBGONE_CORE_SEND_MODE_CONTINUOUS`

Use `tvbgone_core_stop()` to interrupt an in-flight single sweep or stop a
continuous send loop. Use `tvbgone_core_deinit()` when you need to tear down the
IR hardware before reconfiguration.

## Example

The example in [`examples/tvbgone_esp32c3_supermini`](examples/tvbgone_esp32c3_supermini)
is the reference application for this component. It keeps the board-facing
behavior in example code:

- press NA to transmit the North America database
- press EU to transmit the Europe database
- press either button during transmission to stop and restart from the beginning
- drive the visible status LED outside the core API

## Build The Example

```bash
cd examples/tvbgone_esp32c3_supermini
idf.py set-target esp32c3
idf.py build
```
