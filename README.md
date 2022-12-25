# i2c-to-buttons
Use Atmega8 as I2C port expander for custom PCB with 16 binary inputs

## I2C Register File Layout

```
0x00:       Status byte. Bit 1 triggers readout if 1, bit 2 indicates watchdog reset if 0.
0x01, 0x02: Button data.
0x03:       Checksum, 0x01+0x02.
```

## Building

With CMake should work.
To flash with avrdude on Linux using AVRISP2 connected to USB:

```
avrdude -c avrisp2 -P usb -p m8 -U flash:w:cmake-build-release/i2c_to_buttons
```