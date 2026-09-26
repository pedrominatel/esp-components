# Ultraled basic example

This example drives two WS2812B-compatible strips. The strips may have different lengths, GPIOs, brightness values, and byte orders. It demonstrates bulk array loading, fill, clear, per-pixel updates, synchronized `show()` calls, and masked animation updates that leave the other strip untouched.

Configure the GPIOs and lengths with `idf.py menuconfig`, then build for ESP32-P4 revision 3.0 or later:

```sh
idf.py set-target esp32p4
idf.py build flash monitor
```

Connect each configured data GPIO to one strip input. Connect the ESP32-P4 and LED supply grounds together. Use an appropriate logic-level shifter and separately sized LED power supply; do not power a substantial strip from the development board.
