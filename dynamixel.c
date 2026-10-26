/*
 * Bus driver: the PIO UART from dynamixel.pio plus two DMA channels.
 *
 * TX: the packet is built in b->tx and DMA'd into the SM's TX FIFO.
 * RX: the other channel copies every received byte, forever, into the
 * 256-byte b->ring. The CPU reads from b->rd up to the DMA's write pointer.
 * No interrupts: everything busy-waits.
 */
#include <string.h>
#include "dynamixel.h"
#include "dynamixel.pio.h"
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "pico/time.h"

/* ---- bus ---- */

/* Index in b->ring the RX DMA writes next. */
static uint32_t ring_wr(dxl_bus_t *b) {
    return (dma_hw->ch[b->rx_dma].write_addr - (uintptr_t)b->ring) & 0xFF;
}

/*
 * Blocks until the SM has sent everything and released the bus. DMA done
 * isn't enough (FIFO and OSR still hold bytes); the SM reaching the RX code,
 * which follows the TX code in dynamixel.pio, is.
 */
static void wait_tx_done(dxl_bus_t *b) {
    while (dma_channel_is_busy(b->tx_dma)) tight_loop_contents();
    while (pio_sm_get_pc(b->pio, b->sm) < b->offset + dxl_offset_do_rx) tight_loop_contents();
}

bool dxl_init(dxl_bus_t *b, uint pin, uint32_t baud) {
    memset(b, 0, sizeof *b);
    if (!pio_claim_free_sm_and_add_program_for_gpio_range(&dxl_program, &b->pio, &b->sm,
                                                          &b->offset, pin, 1, true))
        return false;
    b->tx_dma = dma_claim_unused_channel(false);
    b->rx_dma = dma_claim_unused_channel(false);
    if (b->tx_dma < 0 || b->rx_dma < 0) {
        if (b->tx_dma >= 0) dma_channel_unclaim(b->tx_dma);
        pio_remove_program_and_unclaim_sm(&dxl_program, b->pio, b->sm, b->offset);
        return false;
    }
    b->pin = pin;
    b->timeout_us = 5000;

    pio_sm_config c = dxl_program_get_default_config(b->offset);
    sm_config_set_set_pins(&c, pin, 1);
    sm_config_set_out_pins(&c, pin, 1);
    sm_config_set_in_pin_base(&c, pin);
    sm_config_set_sideset_pins(&c, pin);
    sm_config_set_jmp_pin(&c, pin);
    sm_config_set_out_shift(&c, true, false, 32);        /* LSB first */
    sm_config_set_in_shift(&c, true, false, 32);         /* byte lands in ISR[31:24] */
    sm_config_set_mov_status(&c, STATUS_TX_LESSTHAN, 1); /* status = ~0 when TX FIFO empty */
    pio_gpio_init(b->pio, pin);
    gpio_pull_up(pin);
    pio_sm_init(b->pio, b->sm, b->offset + dxl_offset_do_rx, &c);
    pio_sm_exec(b->pio, b->sm, pio_encode_set(pio_pins, 1));    /* drive value high... */
    pio_sm_exec(b->pio, b->sm, pio_encode_set(pio_pindirs, 0)); /* ...but released */
    dxl_set_baud(b, baud);

    dma_channel_config tc = dma_channel_get_default_config(b->tx_dma);
    channel_config_set_transfer_data_size(&tc, DMA_SIZE_8);
    channel_config_set_dreq(&tc, pio_get_dreq(b->pio, b->sm, true));
    /* 8-bit writes fill all 4 byte lanes of the FIFO word; the SM shifts out 8 bits */
    dma_channel_configure(b->tx_dma, &tc, &b->pio->txf[b->sm], b->tx, 0, false);

    dma_channel_config rc = dma_channel_get_default_config(b->rx_dma);
    channel_config_set_transfer_data_size(&rc, DMA_SIZE_8);
    channel_config_set_read_increment(&rc, false);
    channel_config_set_write_increment(&rc, true);
    channel_config_set_ring(&rc, true, 8); /* wrap writes on the 256-byte ring */
    channel_config_set_dreq(&rc, pio_get_dreq(b->pio, b->sm, false));
    /* + 3: the byte sits in bits 31:24 of the FIFO word */
    dma_channel_configure(b->rx_dma, &rc, b->ring, (uint8_t *)&b->pio->rxf[b->sm] + 3,
                          dma_encode_endless_transfer_count(), true);

    pio_sm_set_enabled(b->pio, b->sm, true);
    return true;
}

void dxl_set_baud(dxl_bus_t *b, uint32_t baud) {
    wait_tx_done(b); /* e.g. a broadcast baud_rate write still going out */
    b->baud = baud;
    pio_sm_set_clkdiv(b->pio, b->sm, (float)clock_get_hz(clk_sys) / (8.0f * baud));
}

void dxl_deinit(dxl_bus_t *b) {
    wait_tx_done(b);
    pio_sm_set_enabled(b->pio, b->sm, false);
    dma_channel_abort(b->tx_dma);
    dma_channel_abort(b->rx_dma);
    dma_channel_unclaim(b->tx_dma);
    dma_channel_unclaim(b->rx_dma);
    pio_remove_program_and_unclaim_sm(&dxl_program, b->pio, b->sm, b->offset);
    gpio_deinit(b->pin);
}

/*
 * Builds the packet and starts sending it; doesn't wait for it to go out.
 *
 * The SM is restarted at do_tx each time (~0.25 us) because its RX loop sits
 * in `wait` and can't notice the TX FIFO. Unread RX bytes are dropped so a
 * late reply to an earlier, timed-out request isn't taken for this one's.
 */
static int send(dxl_bus_t *b, uint8_t id, uint8_t inst, const uint8_t *params, size_t n) {
    wait_tx_done(b); /* a reply-less packet (sync/bulk write, broadcast) may still be reading b->tx */
    size_t len = dxl_build(b->tx, sizeof b->tx, id, inst, params, n);
    if (!len) return DXL_ERR_ARG;
    pio_sm_set_enabled(b->pio, b->sm, false);
    pio_sm_clear_fifos(b->pio, b->sm);
    pio_sm_restart(b->pio, b->sm);
    pio_sm_exec(b->pio, b->sm, pio_encode_jmp(b->offset + dxl_offset_do_tx));
    b->rd = ring_wr(b); /* drop anything stale */
    dma_channel_transfer_from_buffer_now(b->tx_dma, b->tx, len);
    pio_sm_set_enabled(b->pio, b->sm, true);
    return DXL_OK;
}

/*
 * Next RX byte, or -1 after timeout_us of silence. Each byte pushes *deadline
 * out again, so a long reply never times out while it's still arriving.
 */
static int getc_timeout(dxl_bus_t *b, absolute_time_t *deadline) {
    while (ring_wr(b) == b->rd)
        if (time_reached(*deadline)) return -1;
    uint8_t c = b->ring[b->rd];
    b->rd = (b->rd + 1) & 0xFF;
    *deadline = make_timeout_time_us(b->timeout_us);
    return c;
}

/*
 * Discards everything until timeout_us of silence. Called after every bad
 * reply except a timeout (the bus is quiet then), so no call returns while a
 * servo is still talking and the next call can't talk over it.
 */
static void drain(dxl_bus_t *b) {
    absolute_time_t dl = make_timeout_time_us(b->timeout_us);
    while (getc_timeout(b, &dl) >= 0) {}
}

/*
 * Receives one status packet into b->rx. Bytes before an FF FF FD 00 header
 * are skipped, which resyncs after noise or a cut-off packet. The timeout
 * starts only once our own packet is out, however long that takes at low baud.
 */
static int recv(dxl_bus_t *b, dxl_status_t *st, bool stuffed) {
    wait_tx_done(b);
    absolute_time_t dl = make_timeout_time_us(b->timeout_us);
    uint32_t hdr = 0;
    int c;
    while (hdr != 0xFFFFFD00) {
        if ((c = getc_timeout(b, &dl)) < 0) return DXL_ERR_TIMEOUT;
        hdr = hdr << 8 | c;
    }
    uint8_t *p = b->rx;
    p[0] = 0xFF; p[1] = 0xFF; p[2] = 0xFD; p[3] = 0x00;
    for (int i = 4; i < 7; i++) {
        if ((c = getc_timeout(b, &dl)) < 0) return DXL_ERR_TIMEOUT;
        p[i] = c;
    }
    size_t n = 7 + (p[5] | p[6] << 8);
    if (n > sizeof b->rx) return DXL_ERR_PACKET;
    for (size_t i = 7; i < n; i++) {
        if ((c = getc_timeout(b, &dl)) < 0) return DXL_ERR_TIMEOUT;
        p[i] = c;
    }
    return dxl_parse(p, n, stuffed, st);
}

static dxl_result_t result(int comm, uint8_t err) {
    return (dxl_result_t){(int8_t)comm, err};
}

/* Send, then (unless broadcast) receive the status from @id; another ID is DXL_ERR_PACKET. */
static dxl_result_t txrx(dxl_bus_t *b, uint8_t id, uint8_t inst, const uint8_t *params, size_t n,
                         dxl_status_t *st) {
    int c = send(b, id, inst, params, n);
    if (c || id == DXL_BROADCAST) return result(c, 0);
    c = recv(b, st, true);
    if (!c && st->id != id) c = DXL_ERR_PACKET;
    if (c && c != DXL_ERR_TIMEOUT) drain(b);
    return result(c, c ? 0 : st->err);
}

const char *dxl_strerror(dxl_result_t r) {
    switch (r.comm) {
    case DXL_OK: break;
    case DXL_ERR_TIMEOUT: return "timeout";
    case DXL_ERR_CRC: return "bad CRC";
    case DXL_ERR_PACKET: return "bad packet";
    case DXL_ERR_ARG: return "bad argument";
    default: return "unknown error";
    }
    switch (r.err & 0x7F) {
    case 0: return r.err & DXL_ALERT ? "ok, but hardware alert" : "ok";
    case 1: return "servo: result fail";
    case 2: return "servo: instruction error";
    case 3: return "servo: CRC error";
    case 4: return "servo: data range error";
    case 5: return "servo: data length error";
    case 6: return "servo: data limit error";
    case 7: return "servo: access error";
    default: return "servo: unknown error";
    }
}

/* ---- encoding ---- */

/* Low @len bytes of @v, little-endian. */
static void put_le(uint8_t *p, int32_t v, uint8_t len) {
    for (int i = 0; i < len; i++) p[i] = (uint32_t)v >> (8 * i);
}

static uint8_t *put16(uint8_t *p, uint16_t v) {
    put_le(p, v, 2);
    return p + 2;
}

/* 1 byte reads unsigned, 2 and 4 bytes signed (see dynamixel.h). */
static int32_t get_le(const uint8_t *p, uint8_t len) {
    switch (len) {
    case 1: return p[0];
    case 2: return (int16_t)(p[0] | p[1] << 8);
    default: return (int32_t)(p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24);
    }
}

static bool reg_ok(dxl_reg_t r) {
    return r.len == 1 || r.len == 2 || r.len == 4;
}

/* Value fits the register as either signed or unsigned (so 255 and -1 both fit 1 byte). */
static bool fits(dxl_reg_t r, int32_t v) {
    if (!reg_ok(r)) return false;
    return r.len == 4 || (v >= -(1 << (8 * r.len - 1)) && v < (1 << (8 * r.len)));
}

/* ---- one servo ---- */

dxl_result_t dxl_ping(dxl_bus_t *b, uint8_t id, uint16_t *model) {
    if (id >= 253) return result(DXL_ERR_ARG, 0);
    dxl_status_t st;
    dxl_result_t r = txrx(b, id, DXL_INST_PING, NULL, 0, &st);
    if (r.comm == DXL_OK && st.n >= 2) {
        if (model) *model = st.params[0] | st.params[1] << 8; /* filled even if err is set */
    } else if (dxl_ok(r)) {
        r.comm = DXL_ERR_PACKET; /* claims success but has no model bytes */
    }
    return r;
}

int dxl_scan(dxl_bus_t *b, uint8_t *ids, uint16_t *models, int max) {
    int n = 0;
    for (int id = 0; id < 253 && n < max; id++) {
        uint16_t m = 0;
        if (dxl_ping(b, id, &m).comm != DXL_OK) continue; /* alerted servos still count */
        ids[n] = id;
        if (models) models[n] = m;
        n++;
    }
    return n;
}

dxl_result_t dxl_read_bytes(dxl_bus_t *b, uint8_t id, uint16_t addr, uint16_t len, uint8_t *out) {
    if (id >= 253) return result(DXL_ERR_ARG, 0); /* broadcast gets no reply, st would be unset */
    uint8_t p[4];
    put16(put16(p, addr), len);
    dxl_status_t st;
    dxl_result_t r = txrx(b, id, DXL_INST_READ, p, 4, &st);
    if (dxl_ok(r) && st.n != len) r.comm = DXL_ERR_PACKET;
    if (dxl_ok(r)) memcpy(out, st.params, len);
    return r;
}

/*
 * Write and Reg Write. Params go in b->rx: dxl_build() can't work in place,
 * and b->rx is free until the reply arrives.
 */
static dxl_result_t write_inst(dxl_bus_t *b, uint8_t inst, uint8_t id, uint16_t addr,
                               uint16_t len, const uint8_t *data) {
    if (len + 2u > sizeof b->rx) return result(DXL_ERR_ARG, 0);
    uint8_t *p = put16(b->rx, addr);
    memcpy(p, data, len);
    dxl_status_t st;
    return txrx(b, id, inst, b->rx, len + 2, &st);
}

dxl_result_t dxl_write_bytes(dxl_bus_t *b, uint8_t id, uint16_t addr, uint16_t len, const uint8_t *data) {
    return write_inst(b, DXL_INST_WRITE, id, addr, len, data);
}

dxl_result_t dxl_write(dxl_bus_t *b, uint8_t id, dxl_reg_t reg, int32_t value) {
    if (!fits(reg, value)) return result(DXL_ERR_ARG, 0);
    uint8_t d[4];
    put_le(d, value, reg.len);
    return write_inst(b, DXL_INST_WRITE, id, reg.addr, reg.len, d);
}

dxl_result_t dxl_reg_write(dxl_bus_t *b, uint8_t id, dxl_reg_t reg, int32_t value) {
    if (!fits(reg, value)) return result(DXL_ERR_ARG, 0);
    uint8_t d[4];
    put_le(d, value, reg.len);
    return write_inst(b, DXL_INST_REG_WRITE, id, reg.addr, reg.len, d);
}

dxl_result_t dxl_read(dxl_bus_t *b, uint8_t id, dxl_reg_t reg, int32_t *value) {
    if (!reg_ok(reg)) return result(DXL_ERR_ARG, 0);
    uint8_t d[4];
    dxl_result_t r = dxl_read_bytes(b, id, reg.addr, reg.len, d);
    if (dxl_ok(r)) *value = get_le(d, reg.len);
    return r;
}

dxl_result_t dxl_action(dxl_bus_t *b, uint8_t id) {
    dxl_status_t st;
    return txrx(b, id, DXL_INST_ACTION, NULL, 0, &st);
}

dxl_result_t dxl_reboot(dxl_bus_t *b, uint8_t id) {
    dxl_status_t st;
    return txrx(b, id, DXL_INST_REBOOT, NULL, 0, &st);
}

dxl_result_t dxl_factory_reset(dxl_bus_t *b, uint8_t id, uint8_t mode) {
    if (id == DXL_BROADCAST && mode == 0xFF) return result(DXL_ERR_ARG, 0); /* servos refuse this */
    dxl_status_t st;
    return txrx(b, id, DXL_INST_FACTORY_RESET, &mode, 1, &st);
}

dxl_result_t dxl_clear(dxl_bus_t *b, uint8_t id, uint8_t what) {
    /* fixed keys from the e-Manual: "DXL\"" and "ERCL" */
    static const uint8_t key[2][4] = {{0x44, 0x58, 0x4C, 0x22}, {0x45, 0x52, 0x43, 0x4C}};
    if (what != DXL_CLEAR_MULTI_TURN && what != DXL_CLEAR_ERRORS) return result(DXL_ERR_ARG, 0);
    uint8_t p[5] = {what};
    memcpy(p + 1, key[what - 1], 4);
    dxl_status_t st;
    return txrx(b, id, DXL_INST_CLEAR, p, 5, &st);
}

dxl_result_t dxl_backup(dxl_bus_t *b, uint8_t id, bool restore) {
    const uint8_t p[5] = {restore ? 0x02 : 0x01, 0x43, 0x54, 0x52, 0x4C}; /* key "CTRL" */
    dxl_status_t st;
    return txrx(b, id, DXL_INST_BACKUP, p, 5, &st);
}

/* ---- many servos ---- */

/* Stores e's result, and its value if it worked. */
static void store(dxl_entry_t *e, dxl_result_t r, const uint8_t *data) {
    e->res = r;
    if (dxl_ok(r)) e->value = get_le(data, e->reg.len);
}

/*
 * Receives the replies to a sync/bulk read that send() just started. They
 * come in list order; the first comms failure ends the read, and that entry
 * and every later one get the error. A servo's own error doesn't.
 * Returns how many servos worked.
 */
static int collect(dxl_bus_t *b, bool fast, dxl_entry_t *e, int n) {
    dxl_status_t st = {0};
    int c = DXL_OK;

    if (fast) {
        /* One combined packet; blocks are found by expected length. */
        c = recv(b, &st, false);
        if (!c && st.id != DXL_BROADCAST) c = DXL_ERR_PACKET;
        if (c && c != DXL_ERR_TIMEOUT) drain(b);
        size_t off = 0;
        for (int i = 0; i < n; i++) {
            uint8_t err = 0;
            const uint8_t *d = NULL;
            if (!c) c = dxl_fast_next(&st, &off, e[i].id, e[i].reg.len, &err, &d);
            store(&e[i], result(c, c ? 0 : err), d);
        }
    } else {
        /*
         * One packet per servo. After a bad packet the servos behind it are
         * still talking: wait them out, or the next call would talk over them.
         */
        for (int i = 0; i < n; i++) {
            if (!c) {
                c = recv(b, &st, true);
                if (!c && st.id != e[i].id) c = DXL_ERR_PACKET;
                if (c && c != DXL_ERR_TIMEOUT) drain(b);
            }
            dxl_result_t r = result(c, c ? 0 : st.err);
            if (dxl_ok(r) && st.n != e[i].reg.len) r.comm = DXL_ERR_PACKET;
            store(&e[i], r, st.params);
        }
    }

    int ok = 0;
    for (int i = 0; i < n; i++) ok += dxl_ok(e[i].res);
    return ok;
}

static int sync_read(dxl_bus_t *b, bool fast, dxl_reg_t reg, dxl_entry_t *e, int n) {
    if (n <= 0 || n > 253 || !reg_ok(reg)) return DXL_ERR_ARG;
    uint8_t *p = put16(put16(b->rx, reg.addr), reg.len);
    for (int i = 0; i < n; i++) {
        e[i].reg = reg; /* so collect() sees one layout for sync and bulk */
        *p++ = e[i].id;
    }
    int c = send(b, DXL_BROADCAST, fast ? DXL_INST_FAST_SYNC_READ : DXL_INST_SYNC_READ, b->rx, n + 4);
    return c ? c : collect(b, fast, e, n);
}

int dxl_sync_read(dxl_bus_t *b, dxl_reg_t reg, dxl_entry_t *e, int n) {
    return sync_read(b, false, reg, e, n);
}

int dxl_fast_sync_read(dxl_bus_t *b, dxl_reg_t reg, dxl_entry_t *e, int n) {
    return sync_read(b, true, reg, e, n);
}

int dxl_sync_write(dxl_bus_t *b, dxl_reg_t reg, const dxl_entry_t *e, int n) {
    if (n <= 0 || 4u + n * (1u + reg.len) > sizeof b->rx) return DXL_ERR_ARG;
    uint8_t *p = put16(put16(b->rx, reg.addr), reg.len);
    for (int i = 0; i < n; i++) {
        if (!fits(reg, e[i].value)) return DXL_ERR_ARG;
        *p++ = e[i].id;
        put_le(p, e[i].value, reg.len);
        p += reg.len;
    }
    return send(b, DXL_BROADCAST, DXL_INST_SYNC_WRITE, b->rx, p - b->rx);
}

static int bulk_read(dxl_bus_t *b, bool fast, dxl_entry_t *e, int n) {
    if (n <= 0 || n > 253 || n * 5u > sizeof b->rx) return DXL_ERR_ARG; /* id, addr, len */
    uint8_t *p = b->rx;
    for (int i = 0; i < n; i++) {
        if (!reg_ok(e[i].reg)) return DXL_ERR_ARG;
        *p++ = e[i].id;
        p = put16(put16(p, e[i].reg.addr), e[i].reg.len);
    }
    int c = send(b, DXL_BROADCAST, fast ? DXL_INST_FAST_BULK_READ : DXL_INST_BULK_READ, b->rx, p - b->rx);
    return c ? c : collect(b, fast, e, n);
}

int dxl_bulk_read(dxl_bus_t *b, dxl_entry_t *e, int n) {
    return bulk_read(b, false, e, n);
}

int dxl_fast_bulk_read(dxl_bus_t *b, dxl_entry_t *e, int n) {
    return bulk_read(b, true, e, n);
}

int dxl_bulk_write(dxl_bus_t *b, const dxl_entry_t *e, int n) {
    if (n <= 0 || n * 9u > sizeof b->rx) return DXL_ERR_ARG; /* id, addr, len, value <= 4 */
    uint8_t *p = b->rx;
    for (int i = 0; i < n; i++) {
        if (!fits(e[i].reg, e[i].value)) return DXL_ERR_ARG;
        *p++ = e[i].id;
        p = put16(put16(p, e[i].reg.addr), e[i].reg.len);
        put_le(p, e[i].value, e[i].reg.len);
        p += e[i].reg.len;
    }
    return send(b, DXL_BROADCAST, DXL_INST_BULK_WRITE, b->rx, p - b->rx);
}
