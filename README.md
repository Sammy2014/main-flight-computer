# Main Flight Computer

RP2040-based rocket avionics firmware and hardware design files. The firmware
reads barometric, inertial, and GNSS data; records flight data to an SD card;
and sends telemetry over a LoRa radio. It includes a flight-state machine with
barometric apogee detection and a timer-based backup.

> **Safety:** This project controls a deployment output. The firmware defaults
> to `DRY_RUN = True`, which prevents the output from being energized. Keep dry
> run enabled during development and bench testing. Review the hardware,
> firmware, and applicable safety requirements before any use with a live
> deployment system.

## Repository contents

```text
firmware/       MicroPython flight controller and sensor/radio drivers
hardware/
  bom/          Bill of materials
  pcb/          PCB image and Gerber file
  schematic/    Schematic PDF
documentation/  Flight computer datasheet
images/         Board and testing photos
```

## Hardware

The design and firmware target a Raspberry Pi Pico (RP2040). The firmware
interfaces with a BMP388 barometer, MPU6050 IMU, GNSS receiver, RYLR998 LoRa
module, and an SPI SD-card module. The BOM is in
[hardware/bom/Main_FC.csv](hardware/bom/Main_FC.csv); board files are in
[hardware/pcb/](hardware/pcb/) and [hardware/schematic/](hardware/schematic/).

### Pin assignments used by `firmware/main.py`

| Function | Pico GPIO / interface |
| --- | --- |
| I2C0 SDA / SCL (BMP388, MPU6050) | GP0 / GP1 |
| GNSS UART1 TX / RX | GP4 / GP5 |
| LoRa UART0 TX / RX | GP16 / GP17 |
| Deployment output | GP19 |
| Status LED | GP15 |
| Battery voltage ADC | GP26 / ADC0 |
| SD SPI1 SCK / MOSI / MISO | GP10 / GP11 / GP8 |
| SD chip select | GP12 |

Confirm these assignments and module voltage/logic requirements against the
assembled board before connecting peripherals.

## Firmware

The firmware is written for MicroPython on the Pico:

- [main.py](firmware/main.py) initializes the hardware, calibrates sensors,
  manages the PAD / FLIGHT / DESCENT / LANDED states, logs CSV data, and sends
  telemetry.
- [bmp388.py](firmware/bmp388.py) and [mpu6050.py](firmware/mpu6050.py)
  provide barometer and IMU drivers.
- [gps.py](firmware/gps.py) reads NMEA GNSS data without blocking the main loop.
- [lora.py](firmware/lora.py) configures and communicates with the LoRa module.
- [sd_logger.py](firmware/sd_logger.py) is a standalone SD-card check script.

Copy the flight controller and its driver modules to the Pico. The controller
also imports `sdcard`, which must be installed on the Pico separately (for
example, from the MicroPython SD-card driver library). `sd_logger.py` is a
standalone hardware check and is not required by `main.py`.

The source defaults to dry-run mode and has watchdog use disabled. Flight
thresholds, radio settings, and timing values are constants near the top of
`main.py`; verify them for the specific hardware and mission. The current
firmware settings and BOM include component/model differences, so confirm
compatibility before use (including the GNSS and radio modules).

## Hardware and documentation files

- [Datasheet](documentation/MFC_Datasheet.pdf)
- [Schematic](hardware/schematic/backup_mc.pdf)
- [PCB image](hardware/pcb/MAIN_FC.png)
- [PCB Gerber file](hardware/pcb/backup_MAIN_FC-B_Cu.gbr)
- [Board photos](images/)
- [Bill of materials](hardware/bom/Main_FC.csv)
