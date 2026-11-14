# DDP server example

This example starts a UDP DDP v1 server on port 4048. It logs data and Push
packets and returns a small JSON document for Query packets.

The example intentionally does not configure Ethernet or Wi-Fi. Add the
network initialization appropriate for your board before expecting remote
packets.

```sh
idf.py set-target esp32
idf.py build flash monitor
```
