/*
 * Sweeps one servo back and forth in position mode. Picks the register table
 * from the model number, so it works with any supported series except XL-320
 * (no operating_mode, 0..1023 range: it would never reach 3072).
 */
#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "dynamixel.h"

#define DXL_PIN 0
#define DXL_BAUD 57600
#define DXL_ID 1

static dxl_bus_t bus;

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    dxl_init(&bus, DXL_PIN, DXL_BAUD);

    uint16_t model;
    dxl_result_t r;
    while (!dxl_ok(r = dxl_ping(&bus, DXL_ID, &model))) {
        printf("ping id %d: %s\n", DXL_ID, dxl_strerror(r));
        sleep_ms(1000);
    }
    const dxl_regs_t *reg = dxl_regs(model);
    if (!reg) {
        printf("model %u has no register table\n", model);
        return 1;
    }
    printf("id %d is model %u\n", DXL_ID, model);

    dxl_write(&bus, DXL_ID, reg->torque_enable, 0); /* mode can only change with torque off */
    dxl_write(&bus, DXL_ID, reg->operating_mode, DXL_MODE_POSITION);
    dxl_write(&bus, DXL_ID, reg->profile_velocity, 100);
    dxl_write(&bus, DXL_ID, reg->torque_enable, 1);

    int32_t goal = 1024;
    for (;;) {
        goal = goal == 1024 ? 3072 : 1024;
        dxl_write(&bus, DXL_ID, reg->goal_position, goal);
        int32_t pos = 0;
        do {
            sleep_ms(20);
            r = dxl_read(&bus, DXL_ID, reg->present_position, &pos);
            if (!dxl_ok(r)) printf("read: %s\n", dxl_strerror(r));
            else printf("goal %ld  pos %ld\n", (long)goal, (long)pos);
            if (r.err & DXL_ALERT) {
                int32_t hw = 0;
                dxl_read(&bus, DXL_ID, reg->hardware_error_status, &hw);
                printf("hardware error 0x%02lx%s%s\n", (long)hw,
                       hw & DXL_HW_OVERLOAD ? " overload" : "",
                       hw & DXL_HW_OVERHEATING ? " overheating" : "");
                return 1; /* fix the cause, then dxl_reboot() to clear it */
            }
        } while (abs(pos - goal) > 20);
        sleep_ms(500);
    }
}
