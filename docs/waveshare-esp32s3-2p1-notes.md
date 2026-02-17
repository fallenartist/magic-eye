# Waveshare ESP32-S3 2.1" Round Touch Notes

Reference: https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-2.1

## Important board wiring (from wiki)

- EXIO chip: `TCA9554A` at I2C `0x20`
- EXIO I2C: `SDA=GPIO8`, `SCL=GPIO9`
- LCD reset via EXIO bit `2`
- LCD CS via EXIO bit `3`
- Touch reset via EXIO bit `1`
- Touch INT: `GPIO16`
- Touch I2C bus: `SDA=GPIO15`, `SCL=GPIO7`

## Why this matters

The display control lines are not all directly on ESP32 GPIOs, so a basic
library init often shows a black screen unless EXIO is initialized first.

