# TV-B-Gone for ESP-IDF

`tv-b-gone` packages the current TV-B-Gone ESP-IDF firmware as a reusable component for ESP-IDF projects while keeping the public C API under `tvbgone_core`.

This project is based on the [TV-B-Gone-kit_V2](https://github.com/maltman23/TV-B-Gone-kit_V2) by Mitch Altman.

## What It Includes

- The current TV-B-Gone IR transmit logic
- The bundled `WORLDcodes.cpp` TV-B-Gone power-code database shared with the Arduino implementation
- A root-level component layout consistent with the rest of this repository

## Core Responsibility

The core drives IR transmission and send lifecycle. It can either create and
own its own RMT TX channel or borrow a channel that was already created by a
BSP or another component. Button handling, visible LED signaling, and any
board-specific UI behavior belong in the application or example code.

## Public API

```c
#include "tvbgone_core.h"

tvbgone_core_config_t config;
tvbgone_core_get_default_config(&config);
config.ir_led_gpio = GPIO_NUM_2;
config.rmt_channel_mode = TVBGONE_CORE_RMT_CHANNEL_MODE_INTERNAL;

ESP_ERROR_CHECK(tvbgone_core_init(&config));
tvbgone_core_status_t status;
ESP_ERROR_CHECK(tvbgone_core_get_status(&status));
ESP_ERROR_CHECK(tvbgone_core_send(TVBGONE_CORE_REGION_NA,
                                  TVBGONE_CORE_SEND_MODE_SINGLE));
```

For BSP-owned RMT, pass the pre-created channel handle instead:

```c
rmt_channel_handle_t bsp_ir_channel = /* created and enabled by BSP */;

tvbgone_core_config_t config;
tvbgone_core_get_default_config(&config);
config.rmt_channel_mode = TVBGONE_CORE_RMT_CHANNEL_MODE_BORROWED;
config.external_rmt_channel = bsp_ir_channel;

ESP_ERROR_CHECK(tvbgone_core_init(&config));
```

Available send options:

- Regions: `TVBGONE_CORE_REGION_NA`, `TVBGONE_CORE_REGION_EU`, `TVBGONE_CORE_REGION_BOTH`
- Modes: `TVBGONE_CORE_SEND_MODE_SINGLE`, `TVBGONE_CORE_SEND_MODE_CONTINUOUS`

Use `tvbgone_core_get_status()` to read the current run state, requested region,
active code number, and total code count for the current or most recent send.

Use `tvbgone_core_stop()` to interrupt an in-flight single sweep or stop a
continuous send loop. Use `tvbgone_core_deinit()` when you need to tear down the
IR hardware before reconfiguration.

When using a borrowed BSP-owned channel, `tvbgone_core_deinit()` only releases
resources created by `tvbgone_core` itself. The external channel must already be
created and enabled before `tvbgone_core_init()`. `tvbgone_core_stop()` still
uses the RMT driver recovery path to interrupt an active transmission.

## Example

The example in [`examples/tvbgone_esp32c3_supermini`](examples/tvbgone_esp32c3_supermini)
is the reference application for the internal-channel mode. The
[`examples/defconsg1-badge`](examples/defconsg1-badge) example shows how to pass
in a BSP-owned TX channel.

Both examples keep the board-facing behavior in example code:

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
