# UltraLED

`ultraled` is an ESP-IDF 6.0+ component that drives one to eight 24-bit addressable RGB LED strips in parallel with the ESP32-P4 PARLIO TX peripheral and BitScrambler. `ultraled_show()` updates every configured strip in lock-step, while the masked show APIs can update any selected subset through the same serialized PARLIO TX unit.

## Requirements and limits

- ESP32-P4 revision 3.0 or newer. `ultraled_new()` checks the actual silicon revision and returns `ESP_ERR_NOT_SUPPORTED` on older chips, whose BitScrambler cannot provide the variable-length output required here.
- ESP-IDF 6.0 or newer.
- One active handle is expected. A handle exclusively owns one PARLIO TX unit and its TX BitScrambler channel until `ultraled_del()`.
- All channels in one handle must use the same LED model and timing profile. Different channels may use different GPIOs, lengths, brightness values, pixel data, and color orders, but models with different clocks or wire timings cannot be mixed. This is a consequence of driving all lanes through one PARLIO TX unit and one BitScrambler program.
- The component supports 24-bit RGB strips only. RGBW, HSV/effects, gamma correction, caller-owned buffers, asynchronous callbacks, and custom timings are outside its scope.
- The API is task-safe through internal framebuffer and transmission mutexes, but no API is ISR-safe. The application must prevent a handle from being used concurrently with or after `ultraled_del()`.

## Supported models

| Model | PARLIO sample clock | Zero / one samples | Default wire order |
|---|---:|---|---|
| WS2812 / WS2812B | 3.2 MHz | `1000` / `1110` | GRB |
| SK6812 RGB | 3.333333 MHz | `1000` / `1100` | GRB |
| APA106 | 2.34 MHz | `1000` / `1110` | RGB |
| SM16703 | 3.333333 MHz | `1000` / `1110` | RGB |

Explicit RGB, RBG, GRB, GBR, BRG, and BGR orders are available on every channel. Pixels always remain canonical `{red, green, blue}` in the framebuffer; brightness and wire ordering are applied non-destructively while staging a frame.

## Basic use

```c
ultraled_config_t config = {
    .led_model = ULTRALED_MODEL_WS2812B,
    .channel_count = 2,
    .channels = {
        { .gpio_num = GPIO_NUM_10, .led_count = 60, .color_order = ULTRALED_COLOR_ORDER_MODEL_DEFAULT },
        { .gpio_num = GPIO_NUM_11, .led_count = 24, .color_order = ULTRALED_COLOR_ORDER_BRG },
    },
};

ultraled_handle_t leds;
ESP_ERROR_CHECK(ultraled_new(&config, &leds));
ESP_ERROR_CHECK(ultraled_fill(leds, 0, (ultraled_rgb_t) { .red = 255 }));
ESP_ERROR_CHECK(ultraled_set_brightness(leds, 0, 128));
ESP_ERROR_CHECK(ultraled_show(leds, 1000));
ESP_ERROR_CHECK(ultraled_del(leds));
```

`ultraled_show(handle, -1)` waits indefinitely. For a finite timeout, `ESP_ERR_TIMEOUT` means output may still be active. The DMA staging memory is deliberately retained; the next `show()` waits for that work before modifying it. Pixel and configuration calls modify the separate canonical framebuffer and remain safe after a timed-out show. Deletion always drains queued output.

For a receiver or render task that must remain responsive, submit without waiting:

```c
esp_err_t error = ultraled_show_async(leds);
if (error == ESP_ERR_INVALID_STATE) {
    // The one-frame DMA staging buffer is busy: coalesce or drop this frame.
} else {
    ESP_ERROR_CHECK(error);
}

// Pixel APIs remain available while PARLIO transmits the staged snapshot.
ESP_ERROR_CHECK(ultraled_wait_done(leds, 1000));
```

`ultraled_show_async()` first reaps a completed transfer, then returns `ESP_ERR_INVALID_STATE` if DMA is still using the staging buffer. `ultraled_wait_done()` is idempotent, supports zero-time polling and `ULTRALED_WAIT_FOREVER`, and returns `ESP_OK` when no transfer is pending. The blocking `ultraled_show()` remains available and shares the same staging buffer.

To update only selected strips, pass a channel bit mask. Unselected GPIOs are held continuously low during the transaction, so their strips receive no LED symbols and retain their previously latched pixels:

```c
// Update channel 1 without retransmitting channel 0.
ESP_ERROR_CHECK(ultraled_set_pixels(leds, 1, 0, pixels, pixel_count));
ESP_ERROR_CHECK(ultraled_show_mask_async(leds, 1U << 1));
```

The mask must be nonzero and cannot contain unconfigured channel bits. Multiple selected channels, such as `(1U << 1) | (1U << 3)`, still advance in lock-step. Transactions remain serialized because the ESP32-P4 has one PARLIO TX unit. The encoded length and wire time are based on the longest selected channel.

Shorter selected strips transmit zero-valued pixels until the longest selected lane ends. Every frame is followed by at least 400 microseconds of low output for reset/latch.

## Memory use

For channel lengths `N0...N7` and `Nmax = max(Ni)`, the principal allocations are:

- canonical framebuffers: `3 * sum(Ni)` bytes;
- compact DMA staging: `24 * Nmax` bytes;
- reset DMA buffer: `ceil(clock_hz * 400 us)`, rounded up to 32 bytes;
- driver descriptors, handle state, and RTOS synchronization overhead.

BitScrambler expands each staging byte into four PARLIO samples while transmitting, avoiding a `96 * Nmax`-byte encoded waveform buffer.

At eight equal lanes, the canonical and staging allocations are each 11,520 bytes for 480 LEDs, or 23,040 bytes for 960 LEDs. WS2812 wire time is about 14.8 ms at 480 LEDs and 29.2 ms at 960 LEDs, including the 400 microsecond reset. The 480-LED configuration therefore has ample wire-time margin for a 30 FPS producer. APA106 cannot reach 30 FPS at 960 LEDs with its configured timing.

The encoder uses cached channel orders, brightness fast paths, and a compact nibble lookup to transpose eight RGB values into PARLIO lane masks. Performance measurements should use `CONFIG_COMPILER_OPTIMIZATION_PERF`; the test application under `test_apps/ultraled_perf` benchmarks 480×8 submission, a 300-frame 30 FPS run, and informational 960×8 submission timing.

The hardware test under `test_apps/ultraled_selective` rotates a masked eight-LED update across all eight reference GPIOs every 20 ms. It is intended for logic-analyzer regression testing: each transaction must contain 192 symbols on exactly one selected lane while all unselected lanes remain continuously low.

## Wiring and power

Channel 0 through channel 7 map directly to PARLIO lane 0 through lane 7 and to the GPIOs passed in the configuration. GPIOs must be distinct and output-capable. Unused lanes are disconnected.

Connect the ESP32-P4 ground and LED supply ground. Many 5 V strips need a 3.3-to-5 V logic-level shifter for reliable signaling. Size and inject external LED power for the maximum expected current; do not power a substantial strip through an ESP development board. A small series resistor on each data line and bulk capacitance at the strip supply are generally advisable.

See [`examples/ultraled_basic`](examples/ultraled_basic) for a two-strip animation.
