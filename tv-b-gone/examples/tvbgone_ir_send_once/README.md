# Example for TV-B-Gone IR (button start/stop)

This example initializes the `tv-b-gone` component and starts/stops continuous IR sweeps using a button.

## What it does

- Initializes TV-B-Gone IR TX on a GPIO pin
- Sets mode to `TVBGONE_IR_MODE_BOTH` (NA + EU)
- Configures a button on GPIO9 (active LOW, internal pull-up enabled)
- Toggles `tvbgone_ir_start()` / `tvbgone_ir_stop()` on each button press

## Hardware

- IR LED connected to the GPIO configured in `Component config -> TV-B-Gone IR Configuration -> IR TX GPIO pin`
- Push button connected to GPIO9 and GND

## Build

From this folder:

```bash
idf.py build
idf.py flash monitor
```
