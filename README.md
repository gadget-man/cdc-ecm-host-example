| Supported Targets | ESP32-S3 |
| ----------------- | -------- | 

# USB CDC-ECM Host Driver Example

This example shows how to use the CDC-ECM Host Driver to allow an ESP chip to communicate with a USB CDC-ECM device, e.g. a Realkek 8152 usb-to-ethernet dongle.

## How to use example

### Hardware Required

A development boards with USB-OTG support.
A compatible usb-to-ethernet device, e.g. Realtek 8152.  


### Build and Flash

1. Build this project and flash it to the USB host board, then run monitor tool to view serial output:

```bash
idf.py -p PORT flash monitor
```

(Replace PORT with the name of the serial port to use.)

(To exit the serial monitor, type ``Ctrl-]``.)

See the Getting Started Guide for full steps to configure and use ESP-IDF to build projects.

## Example Output

After the flashing, plug in a usb-to-ethernet adapter connected by ethernet cable to your network. You should see the output at idf monitor:

[to be completed - expected output confirms Ethernet connected, then IP address obtained]