/* Bulk read/write: a different register on each servo in one transaction. */
#include <stdio.h>
#include "pico/stdlib.h"
#include "dynamixel.h"

#define DXL_PIN 0
#define DXL_BAUD 57600

static dxl_bus_t bus;

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    dxl_init(&bus, DXL_PIN, DXL_BAUD);

    dxl_entry_t rd[] = {
        {.id = 1, .reg = DXL_XM430.present_position},    /* 4 bytes */
        {.id = 2, .reg = DXL_XM430.present_current},     /* 2 bytes, signed */
        {.id = 3, .reg = DXL_XM430.present_temperature}, /* 1 byte */
    };
    /* start opposite, so the two LEDs alternate */
    dxl_entry_t leds[] = {{.id = 1, .reg = DXL_XM430.led}, {.id = 2, .reg = DXL_XM430.led, .value = 1}};

    for (;;) {
        dxl_bulk_read(&bus, rd, count_of(rd));
        for (size_t i = 0; i < count_of(rd); i++) {
            if (dxl_ok(rd[i].res)) printf("id%d %6ld   ", rd[i].id, (long)rd[i].value);
            else printf("id%d %s   ", rd[i].id, dxl_strerror(rd[i].res));
        }
        putchar('\n');

        leds[0].value ^= 1;
        leds[1].value ^= 1;
        dxl_bulk_write(&bus, leds, count_of(leds));
        sleep_ms(500);
    }
}
