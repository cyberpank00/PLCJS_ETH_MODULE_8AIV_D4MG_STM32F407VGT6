# PLCJS_ETH_MODULE_8AIV_D4MG_STM32F407VGT6

Firmware for the PLCJS Ethernet 8AIV module: 8 isolated voltage analog inputs
on one `ADS8688`, STM32F407VGT6, KSZ8863 switch, Modbus TCP. The network,
Modbus, discovery (PDP), bootloader contract and LED subsystems are shared with
the rest of the family (8AIC / 8UAI); the analog path is a simple timer-paced
scan. The board also serves as the bring-up platform for the ADS8688 path that
the 8UAI firmware uses.

Schematic: `DOC/SCH_8AIV.pdf`. Image identity: `product_id 0x504C0805`,
`hw 01.01`, module ID (IR125) `0x08A5`. The image runs only through the
bootloader (`BOOTLOADER_PLCJS_ETH_MODULE_STM32F407VGT6`, `scripts/variants.csv`
row `8aiv`).

## Hardware

| Item | Value |
|---|---|
| ADC | ADS8688, internal 4.096 V reference (REFSEL = GND), software RST (RST/PD pulled up) |
| Bus | SPI1 PA5 SCLK / PA6 MISO / PB5 MOSI, CS PA9, through the isolator CA-IS3741LB, 10 MHz, mode 1 |
| Input network | Vin -> 1 kΩ (2512) -> node (120 kΩ to GND, 12 V TVS) -> 1 kΩ -> AIN_nP; AIN_nGND -> 2 kΩ -> GND |
| Channel LEDs | AI0 PB15, AI1 PB14, AI2 PB10, AI3 PE15, AI4 PE14, AI5 PE13, AI6 PE12, AI7 PE11 |
| ADS8688 channel of AI0..AI7 | 5, 4, 3, 2, 0, 1, 7, 6 (`BOARD_ADC_MAP`) |

## Modes

| Mode | Signal | ADS8688 range |
|---|---|---|
| 0 OFF | | |
| 1 | 0..10 V (default) | 0..10.24 V |
| 2 | 2..10 V, below 1.8 V = wire break | 0..10.24 V |
| 3 | 0..5 V | 0..5.12 V |
| 4 | ±10 V | ±10.24 V |
| 5 | ±5 V | ±5.12 V |

The network gain is about 0.9898 (divider plus ADC load); the firmware inverts
the full model, then applies the per-channel calibration
`V = gain · Vin + offset` and the optional engineering scaling.

## Acquisition

TIM2 paces one ADS8688 frame every 39.0625 µs; the 8 inputs are scanned
round robin (each input every 312.5 µs). One window is 2560 slots = 100.000 ms
(320 samples per input, whole periods of 50 and 60 Hz) or 512 slots = 20.000 ms
(64 samples, 50 Hz only). Between windows the engine applies queued range
changes and reads back one range register (rotating), so a lost ADS8688
configuration is detected and healed (ADC_FAIL for 5 s, IR133 counts the
mismatches). During a Flash erase (settings save) the scan is paused and the
window restarted.

## Modbus map

The full map is the header comment of `Application/modbus/modbus_app.h`.

| Registers | Content |
|---|---|
| HR 0..7 / 8..15 / 16..23 | reading mV (RO) / mode / EMA filter ms |
| HR 24..87 | float32 ×8 each: cal gain, cal offset V, scale low, scale high |
| HR 101..119, 130, 131 | family-wide: LED mode, slave id, port, IP, mask, gateway, net mode, SAVE `0xA5A5`, REBOOT `0xB00B` / BOOT `0xB007` / KSZ reset `0x8863`, FACTORY `0xDEAD`, temperature, channel command |
| HR 132..154 | integration window, input-network model, diagnostics (service) |
| IR 300..331 | value and raw (V), float32 ×8 |
| IR 332..411 | status, counter, mean / min / max code, noise RMS, clip counts, state |
| IR 120..134 | firmware, uptime, module ID, temperature, flags, samples, saves, windows, overruns, range errors, bring-up attempts |

Channel status bits: 0 NO_DATA, 1 OVERRANGE, 2 UNDERRANGE, 3 WIRE_BREAK,
4 ADC_CLIP, 5 CONFIG_ERR, 6 ADC_FAIL, 7 OFF.

## Image

| Field | Value |
|---|---|
| `fw_header` product_id | `0x504C0805` |
| hw_revision / fw_version | `0x0101` / `0x0100` |
| Flash origin | `0x08040000`, 256 KB; header at offset `0x200` |
| Settings | sector 10 `0x080C0000`, CRC32, magic `0x08A54A57` |

## Build and tests

```
cmake --preset Debug && cmake --build --preset Debug      # or Release
```

Host tests of the conversion maths (native gcc):

```
cmake -S tests -B build/host-tests -G Ninja -DCMAKE_C_COMPILER=gcc
cmake --build build/host-tests
ctest --test-dir build/host-tests --output-on-failure
```

Programming over SWD alone leaves the board in the bootloader (metadata CRC
check); use OTA or the bootloader's `scripts/service_flash.ps1`.

## Status

Both presets build warning-free and the host tests pass. Nothing has been run on
hardware: SPI timing through the isolator, the ISR load (about 10 % of the CPU),
the real input-network gain / the AIN_nGND resistor effect and the accuracy
still have to be measured on the board.
