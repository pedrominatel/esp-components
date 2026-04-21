# TV-B-Gone ESP32-C3 Super Mini Example

This example reproduces the current TV-B-Gone button-driven behavior using the
root-level `tv-b-gone` component from this repository.

This project is based on the [TV-B-Gone-kit_V2](https://github.com/maltman23/TV-B-Gone-kit_V2) by Mitch Altman.

## Wiring

- IR LED amplifier gate: `GPIO2`
- NA button: `GPIO10`
- EU button: `GPIO9`
- Built-in visible LED: `GPIO8` active-low

## Behavior

- Press the NA button to transmit the North America database.
- Press the EU button to transmit the Europe database.
- Press either button during an active transmission to stop and restart from
  the first code of the newly selected region.
- This example uses the default internal RMT-channel mode, where
  `tvbgone_core` creates and owns the TX channel.
- The example owns button polling and visible LED signaling; the core only
  drives the IR LED.

## Build

```bash
idf.py set-target esp32c3
idf.py build
```
