/* Finds every servo on the bus at every standard baud rate. */
#include <stdio.h>
#include "pico/stdlib.h"
#include "dynamixel.h"

#define DXL_PIN 0

static dxl_bus_t bus;

int main(void) {
    stdio_init_all();
    sleep_ms(2000); /* time to open the USB serial monitor */
    if (!dxl_init(&bus, DXL_PIN, 57600)) {
        puts("dxl_init failed");
        return 1;
    }
    bus.timeout_us = 3000; /* short timeout: most IDs won't answer */

    /* most likely first: 57600 is the factory default */
    static const uint32_t bauds[] = {57600, 1000000, 115200, 2000000, 3000000, 4000000, 4500000, 9600};
    for (;;) {
        for (size_t i = 0; i < count_of(bauds); i++) {
            dxl_set_baud(&bus, bauds[i]);
            uint8_t ids[253];
            uint16_t models[253];
            int n = dxl_scan(&bus, ids, models, 253);
            printf("%7lu baud: %d found\n", (unsigned long)bauds[i], n);
            for (int k = 0; k < n; k++)
                printf("    id %3d  model %5u%s\n", ids[k], models[k],
                       dxl_regs(models[k]) ? "" : "  (no register table)");
        }
        puts("--");
        sleep_ms(1000);
    }
}
