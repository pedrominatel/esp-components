# TV-B-Gone DEFCONSG1 Badge Example

This example packages the root-level `tv-b-gone` component for the DEFCONSG1
badge firmware layout.

This project is based on the [TV-B-Gone-kit_V2](https://github.com/maltman23/TV-B-Gone-kit_V2) by Mitch Altman.

## Wiring

- IR LED amplifier gate: `GPIO19`
- Single trigger button: `GPIO10`

## Behavior

- Press the single button once to transmit both the North America and Europe
  databases in sequence.
- Press the button again during an active transmission to stop it.
- Press the button again after a stopped or completed sweep to start a new one.
- The example simulates a BSP by creating and enabling the RMT TX channel
  itself, then passes that channel into `tvbgone_core`.
- The example owns the button behavior and borrowed-channel setup; the core only
  controls TV-B-Gone transmission.

## Build

```bash
idf.py set-target esp32c6
idf.py build
```
