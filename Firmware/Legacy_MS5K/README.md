# Legacy Firmware

This folder contains legacy firmware which drives the ICM42607 and MLX90393 independently from a central Teensy 4.0 MCU. While the ICM54207 is retained for the sixth sense, the communication protocol is updated to be SPI instead. MLX is also not in use as the magnatometer anymore, but calibration and sensor fusion can be retained.
