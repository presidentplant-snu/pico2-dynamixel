/*
 * Dynamixel Protocol 2.0 master on a single GPIO.
 *
 * One PIO state machine runs a half-duplex UART on the data pin and two DMA
 * channels feed it; all three are claimed by dxl_init(). No buffer chip or
 * direction pin: the SM drives the pin while sending and releases it to the
 * pull-up to receive.
 *
 * Every call blocks (busy-waits) until the reply arrives or the bus has been
 * silent for bus.timeout_us. Calls that get no reply (broadcasts, sync/bulk
 * writes) return as soon as the packet is queued; the next call waits for it
 * to finish going out.
 *
 * There is no locking: use a bus from one core and never from an IRQ.
 *
 * Status Return Level 1 makes servos skip write replies, 0 skips all but ping;
 * calls waiting for those replies time out.
 *
 * Registers: dxl_regs.h. Packets and DXL_ERR_* codes: dxl_packet.h.
 */
#pragma once
#include <stdbool.h>
#include "pico/types.h"
#include "hardware/pio.h"
#include "dxl_packet.h"
#include "dxl_regs.h"

/*
 * Largest packet either way, after byte stuffing. Raise it for sync/bulk
 * transfers to many servos, or fast-read replies (8 + n * (len + 4) bytes).
 */
#define DXL_BUF_SIZE 1024

/* Takes around ~2.5 KB of memory */
typedef struct {
    PIO pio;
    uint sm, offset, pin;
    int tx_dma, rx_dma;
    uint32_t baud;
    /*
     * Bus silence before giving up (default 5000). Keep it above the servos'
     * Return Delay Time (X default 500 us, max 508 us): a reply that comes
     * after the timeout can be taken as the next call's reply.
     */
    uint32_t timeout_us;
    uint32_t rd; /* next ring[] index to read */
    /* RX DMA writes every byte here, forever; aligned for the DMA ring wrap */
    uint8_t ring[256] __attribute__((aligned(256)));
    uint8_t tx[DXL_BUF_SIZE]; /* TX DMA reads it until the packet is out */
    uint8_t rx[DXL_BUF_SIZE]; /* last reply (dxl_status_t.params), also scratch for params */
} dxl_bus_t;

/*
 * One servo's outcome: single-servo calls return it, sync/bulk reads store
 * one per servo in dxl_entry_t.res. comm says whether the exchange worked,
 * err what the servo thought of the instruction.
 */
typedef struct {
    int8_t comm; /* DXL_OK or DXL_ERR_TIMEOUT / CRC / PACKET / ARG (library / bus side) */
    uint8_t err; /* servo status byte: bits 0-6 error number (1..7), bit 7 DXL_ALERT */
} dxl_result_t;

/**
 * dxl_ok() - True if the call worked.
 *
 * DXL_ALERT alone doesn't fail it (data is valid), but the servo has a
 * hardware fault and may have cut torque: read hardware_error_status
 * (Y series: error_code), fix the cause, then dxl_reboot()
 * (Y series: dxl_clear(..., DXL_CLEAR_ERRORS)).
 */
static inline bool dxl_ok(dxl_result_t r) { return r.comm == DXL_OK && !(r.err & 0x7F); }

/* Static string for printing @r, e.g. "timeout" or "servo: access error". */
const char *dxl_strerror(dxl_result_t r);

/**
 * dxl_init() - Claim a PIO SM and two DMA channels and start listening.
 * @b:    zeroed and filled in here
 * @pin:  data pin; gets the internal pull-up, which only has to hold the
 *        idle level (every sender leaves the line high after its stop bit)
 * @baud: what the servos use; X series ship at 57600
 *
 * Return: false if no SM, instruction memory or DMA channel was free.
 * Nothing stays claimed then.
 */
bool dxl_init(dxl_bus_t *b, uint pin, uint32_t baud);

/*
 * Changes the Pico's side only. To move a servo: write its baud_rate register
 * (see dxl_regs.h), then call this. Waits for a queued packet first, so a
 * broadcast baud_rate write still goes out at the old rate.
 */
void dxl_set_baud(dxl_bus_t *b, uint32_t baud);

/* Waits for the last packet to go out, then frees the SM, DMA and pin. */
void dxl_deinit(dxl_bus_t *b);

/*
 * ---- one servo, by register ----
 *
 * reg.len sets the wire size. Reads: 1 byte gives 0..255, 2/4 bytes are
 * signed; mask with 0xFFFF for an unsigned 2-byte register over 32767
 * (PRO model_number; dxl_ping() gets models right). *value is only written
 * on success. Writes that don't fit reg.len, or to a missing register
 * ({0, 0}), give DXL_ERR_ARG.
 */
dxl_result_t dxl_write(dxl_bus_t *b, uint8_t id, dxl_reg_t reg, int32_t value);
dxl_result_t dxl_read(dxl_bus_t *b, uint8_t id, dxl_reg_t reg, int32_t *value);

/*
 * Reg Write stores a write without applying it; Action (often to
 * DXL_BROADCAST) applies all stored writes at once, so servos start together.
 */
dxl_result_t dxl_reg_write(dxl_bus_t *b, uint8_t id, dxl_reg_t reg, int32_t value);
dxl_result_t dxl_action(dxl_bus_t *b, uint8_t id);

/*
 * ---- many servos, by register ----
 *
 * One dxl_entry_t per servo, n <= 253. Sync: one register for all (e[i].reg
 * is ignored on write and set to reg on read). Bulk: e[i].reg per servo, each
 * ID at most once (protocol rule).
 *
 * Reads return how many servos worked (dxl_ok(e[i].res)), so == n means all
 * of them; e[i].value is only written for those. Replies come in list order,
 * and the first comms failure (timeout, CRC, reply from the wrong ID) ends the
 * read: that servo and every later one get the error. A servo's own error in
 * res.err doesn't. A missing servo costs timeout_us.
 *
 * Writes return DXL_OK. Both return DXL_ERR_ARG (< 0) if rejected, and then
 * nothing was sent.
 *
 * Writes get no replies, so a write-only loop won't notice a servo that shut
 * down: read something too.
 */
typedef struct {
    uint8_t id;
    dxl_reg_t reg;
    int32_t value;    /* in for write, out for read (only written if res is ok) */
    dxl_result_t res; /* out for read */
} dxl_entry_t;
int dxl_sync_write(dxl_bus_t *b, dxl_reg_t reg, const dxl_entry_t *e, int n);
int dxl_sync_read(dxl_bus_t *b, dxl_reg_t reg, dxl_entry_t *e, int n);
int dxl_bulk_write(dxl_bus_t *b, const dxl_entry_t *e, int n);
int dxl_bulk_read(dxl_bus_t *b, dxl_entry_t *e, int n);

/*
 * Fast: all servos answer in one combined reply of 8 + n*(len+4) bytes, which
 * must fit DXL_BUF_SIZE. Less overhead than one reply each; fails the same way.
 * Needs X430/540 FW45+, X330 FW46+ or P FW12+.
 */
int dxl_fast_sync_read(dxl_bus_t *b, dxl_reg_t reg, dxl_entry_t *e, int n);
int dxl_fast_bulk_read(dxl_bus_t *b, dxl_entry_t *e, int n);

/*
 * ---- raw bytes: any address and length, e.g. a block of registers ----
 *
 * Little-endian, as on the wire. out is only written on success.
 */
dxl_result_t dxl_read_bytes(dxl_bus_t *b, uint8_t id, uint16_t addr, uint16_t len, uint8_t *out);
dxl_result_t dxl_write_bytes(dxl_bus_t *b, uint8_t id, uint16_t addr, uint16_t len, const uint8_t *data);

/* ---- other instructions ---- */

/*
 * id 0..252, no broadcast: use dxl_scan(). model (may be NULL) is filled even
 * if the servo reports an error.
 */
dxl_result_t dxl_ping(dxl_bus_t *b, uint8_t id, uint16_t *model);

/*
 * Pings IDs 0..252 in turn; returns the count found (alerted servos too).
 * Each empty ID costs timeout_us, so lower it first. models may be NULL.
 */
int dxl_scan(dxl_bus_t *b, uint8_t *ids, uint16_t *models, int max);

/*
 * Clears hardware errors (DXL_ALERT). RAM registers go back to defaults:
 * torque off, goals and gains reset.
 */
dxl_result_t dxl_reboot(dxl_bus_t *b, uint8_t id);

/*
 * mode: 0xFF resets everything (ID 1, factory baud), 0x01 keeps the ID,
 * 0x02 keeps ID + baud. Broadcast 0xFF is DXL_ERR_ARG: servos refuse it, as
 * they'd all end up as ID 1.
 */
dxl_result_t dxl_factory_reset(dxl_bus_t *b, uint8_t id, uint8_t mode);

/*
 * Servo must be stopped. MULTI_TURN brings present_position back within one
 * turn; ERRORS resets a Y series error_code marked "clear" in dxl_regs.h.
 */
enum { DXL_CLEAR_MULTI_TURN = 1, DXL_CLEAR_ERRORS = 2 /* Y series only */ };
dxl_result_t dxl_clear(dxl_bus_t *b, uint8_t id, uint8_t what);

/* Saves (or restores) the control table in the servo. Torque must be off; restore reboots it. */
dxl_result_t dxl_backup(dxl_bus_t *b, uint8_t id, bool restore);
