/*
 * Dynamixel Protocol 2.0 packet encode/decode. Pure C, no hardware deps, so
 * test/test_packet.c runs it on the host.
 *
 * Packet: FF FF FD 00 | id | len (2, LE) | inst | params | crc (2, LE)
 * len counts inst, params and crc. Inside inst..params every FF FF FD gets
 * an extra FD ("byte stuffing") so the header can't appear there.
 */
#pragma once
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define DXL_BROADCAST 0xFE /* all servos act; only ping and sync/bulk reads get replies */

enum {
    DXL_INST_PING = 0x01,
    DXL_INST_READ = 0x02,
    DXL_INST_WRITE = 0x03,
    DXL_INST_REG_WRITE = 0x04,
    DXL_INST_ACTION = 0x05,
    DXL_INST_FACTORY_RESET = 0x06,
    DXL_INST_REBOOT = 0x08,
    DXL_INST_CLEAR = 0x10,
    DXL_INST_BACKUP = 0x20,
    DXL_INST_STATUS = 0x55, /* every reply */
    DXL_INST_SYNC_READ = 0x82,
    DXL_INST_SYNC_WRITE = 0x83,
    DXL_INST_FAST_SYNC_READ = 0x8A,
    DXL_INST_BULK_READ = 0x92,
    DXL_INST_BULK_WRITE = 0x93,
    DXL_INST_FAST_BULK_READ = 0x9A,
};

/* Library/bus errors (dxl_result_t.comm). */
enum {
    DXL_OK = 0,
    DXL_ERR_TIMEOUT = -1, /* no reply: wrong ID or baud, no power, Status Return Level */
    DXL_ERR_CRC = -2,     /* reply corrupted on the wire */
    DXL_ERR_PACKET = -3,  /* reply malformed, from the wrong ID, wrong length, or > DXL_BUF_SIZE */
    DXL_ERR_ARG = -4,     /* rejected before anything was sent */
};

#define DXL_ALERT 0x80 /* status error bit 7: servo has a hardware error (data still valid) */

/* A parsed status packet. */
typedef struct {
    uint8_t id;
    uint8_t err;     /* status error byte (bit 7 = hardware alert) */
    uint8_t *params; /* points into the parsed packet: valid until it's overwritten */
    uint16_t n;
} dxl_status_t;

/* CRC-16/BUYPASS (poly 0x8005, init 0, MSB first) over header..params. */
uint16_t dxl_crc(const uint8_t *p, size_t n);

/**
 * dxl_build() - Build a stuffed instruction packet.
 * @out: must not overlap @params
 * @cap: size of @out; stuffing can add a byte per FF FF FD in @params
 *
 * Return: packet length, or 0 if it doesn't fit in @cap.
 */
size_t dxl_build(uint8_t *out, size_t cap, uint8_t id, uint8_t inst,
                 const uint8_t *params, size_t n);

/**
 * dxl_parse() - Check and parse a whole status packet.
 * @pkt:     header through CRC; params are unstuffed in place
 * @stuffed: false for fast-read replies, which aren't stuffed
 *
 * Return: DXL_OK, DXL_ERR_PACKET (not a status packet) or DXL_ERR_CRC.
 */
int dxl_parse(uint8_t *pkt, size_t n, bool stuffed, dxl_status_t *st);

/**
 * dxl_fast_next() - Step through a fast-read reply, one servo per call.
 * @st:   parsed with stuffed = false
 * @off:  0 before the first call
 * @id:   the servo this block should be from (request order)
 * @len:  its data length
 * @err:  set to its status error byte
 * @data: set to its @len data bytes, inside @st's packet
 *
 * Return: DXL_ERR_PACKET if the block is missing or not from @id.
 */
int dxl_fast_next(const dxl_status_t *st, size_t *off, uint8_t id, uint16_t len,
                  uint8_t *err, const uint8_t **data);
