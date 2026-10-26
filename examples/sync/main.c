/* Moves several servos together with sync write and reads them back with sync read. */
#include <math.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "dynamixel.h"

#define DXL_PIN 0
#define DXL_BAUD 57600
#define REG DXL_XM430 /* register table of the servos on the bus */

static dxl_bus_t bus;
static dxl_entry_t cmd[] = {{.id = 1}, {.id = 2}, {.id = 3}};   /* goals out */
static dxl_entry_t state[] = {{.id = 1}, {.id = 2}, {.id = 3}}; /* positions in */
#define N count_of(cmd)

static void write_all(dxl_reg_t reg, int32_t value) {
    for (size_t i = 0; i < N; i++) cmd[i].value = value;
    dxl_sync_write(&bus, reg, cmd, N);
}

int main(void) {
    stdio_init_all();
    sleep_ms(2000);
    dxl_init(&bus, DXL_PIN, DXL_BAUD);

    write_all(REG.torque_enable, 0);
    write_all(REG.operating_mode, DXL_MODE_POSITION);
    write_all(REG.torque_enable, 1);

    for (float t = 0;; t += 0.02f) {
        /* 2.094 = 2*pi/3: three servos, evenly out of phase */
        for (size_t i = 0; i < N; i++) cmd[i].value = 2048 + (int32_t)(800 * sinf(t + i * 2.094f));
        dxl_sync_write(&bus, REG.goal_position, cmd, N);
        /* dxl_fast_sync_read: one combined reply (X430/540 FW45+, X330 FW46+) */
        if (dxl_sync_read(&bus, REG.present_position, state, N) < (int)N)
            printf("(some failed) ");
        for (size_t i = 0; i < N; i++) {
            if (dxl_ok(state[i].res))
                printf("id%d %5ld/%5ld  ", state[i].id, (long)state[i].value, (long)cmd[i].value);
            else
                printf("id%d %-11s  ", state[i].id, dxl_strerror(state[i].res));
        }
        putchar('\n');
        sleep_ms(20);
    }
}
