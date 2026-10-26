# pico2-dynamixel

Dynamixel Protocol 2.0 for the Raspberry Pi Pico 2 (RP2350) using **one GPIO, one PIO
state machine and two DMA channels**. No buffer chip, no direction pin. The pin and baud rate are
set at runtime, and the PIO, SM and DMA channels are claimed automatically.

## Wiring

```
Pico GPIO n ──── DATA      (RP2350 GPIOs are 5 V tolerant; an internal pull-up is enabled)
Pico GND    ──── GND
PSU +V      ──── VDD       (servo power from its own supply, NOT from the Pico)
```

## Use

```cmake
add_subdirectory(pico2-dynamixel)   # the cloned repo folder
target_link_libraries(my_app pico_stdlib pico2_dynamixel)
```
## Examples

```
cmake -B build -DPICO_SDK_PATH=/usr/share/pico-sdk && cmake --build build
```

Each example prints over USB serial.

| example     | what it does                                               |
|-------------|------------------------------------------------------------|
| `dxl_scan`  | lists every servo at every standard baud rate              |
| `dxl_sweep` | position-mode sweep of ID 1, register table picked from its model |
| `dxl_sync`  | sine wave on IDs 1–3 via sync write, read back via sync read |
| `dxl_bulk`  | three different registers on IDs 1–3, blinks LEDs          |

## Test

```
gcc -Wall -I. test/test_packet.c dxl_packet.c && ./a.out   # packet encode/decode vs e-Manual vectors
```

The protocol follows the ROBOTIS "DYNAMIXEL Protocol 2.0" e-Manual, and `test/test_packet.c`
checks against every example packet on that page.
