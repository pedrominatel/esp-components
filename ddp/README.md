# DDP component

This component implements the transport and wire-level parts of Distributed
Display Protocol version 1. It intentionally has no dependency on NVS, the web
interface, or a particular LED driver.

## Installation

Add the component from the Espressif Component Registry:

```sh
idf.py add-dependency "pedrominatel/ddp^0.1.0"
```

Supported behavior:

- UDP and TCP on the standard DDP port (4048 by default)
- 10-byte headers and optional 4-byte timecodes
- data, Push, Query, and Reply packets
- bounded payloads (1440 bytes by default)
- TCP stream fragmentation and multiple sequential packets
- back-to-back duplicate suppression and sequence-gap statistics
- callback-generated query replies

The application supplies callbacks for data, Push, and Query packets. Callback
data is valid only for the duration of the callback and must be copied if the
application needs to retain it.

The component owns no display framebuffer. Frame assembly, address mapping,
packet-loss policy, and persistent configuration belong to the application.

Use `DDP_SERVER_CONFIG_DEFAULT()` to initialize the server configuration, set
the desired transport flags, populate `ddp_callbacks_t`, and call
`ddp_server_start()`.

```c
static esp_err_t on_data(void *context, const ddp_packet_t *packet)
{
    // Consume or copy packet->data before returning.
    return ESP_OK;
}

static esp_err_t on_push(void *context, const ddp_packet_t *packet)
{
    // Commit the frame assembled by on_data().
    return ESP_OK;
}

ddp_server_config_t config = DDP_SERVER_CONFIG_DEFAULT();
ddp_callbacks_t callbacks = {
    .on_data = on_data,
    .on_push = on_push,
};
ddp_server_handle_t server = NULL;
ESP_ERROR_CHECK(ddp_server_start(&config, &callbacks, NULL, &server));
```

Callbacks execute in the DDP server task and must not retain packet pointers.
They should do bounded work and must not call `ddp_server_stop()` for the same
server.

The component reports malformed, oversized, duplicate, and sequence-gap
statistics through `ddp_server_get_stats()`. Application-specific frame
assembly and display policy should remain outside this component.
