#include "wireless.h"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <assert.h>
#include <stdio.h>
#include <string.h>

#define IMAGE_SIZE 3701u
#define IMAGE_CRC UINT32_C(2735251618)

static struct {
    uint8_t flash[IMAGE_SIZE];
    uint32_t saved_size;
    uint32_t saved_crc;
    uint32_t saved_off;
    unsigned erases;
    unsigned writes;
    unsigned commits;
    unsigned activations;
    bool fail_save;
    bool corrupt_readback;
    uint8_t tx[LP_WIRE_FRAME_SIZE];
    unsigned transmissions;
} hw;

static uint32_t u32(const uint8_t *p)
{
    return (uint32_t)p[0] | (uint32_t)p[1] << 8 | (uint32_t)p[2] << 16 | (uint32_t)p[3] << 24;
}

static int begin(void *ctx, uint32_t size, uint32_t crc)
{
    (void)ctx;
    (void)crc;
    assert(size == IMAGE_SIZE);
    memset(hw.flash, 0xff, sizeof(hw.flash));
    hw.saved_size = 0;
    ++hw.erases;
    return 0;
}

static int write_chunk(void *ctx, uint32_t off, const uint8_t *data, size_t len)
{
    (void)ctx;
    assert(off + len <= sizeof(hw.flash));
    for (size_t i = 0; i < len; ++i) {
        assert(hw.flash[off + i] == 0xff || hw.flash[off + i] == data[i]);
        hw.flash[off + i] = data[i];
    }
    ++hw.writes;
    return 0;
}

static int read_chunk(void *ctx, uint32_t off, uint8_t *data, size_t len)
{
    (void)ctx;
    assert(off + len <= sizeof(hw.flash));
    memcpy(data, hw.flash + off, len);
    if (hw.corrupt_readback && off == 0) {
        data[0] ^= 1;
    }
    return 0;
}

static int save(void *ctx, uint32_t size, uint32_t crc, uint32_t off)
{
    (void)ctx;
    if (hw.fail_save) {
        return -1;
    }
    hw.saved_size = size;
    hw.saved_crc = crc;
    hw.saved_off = off;
    ++hw.commits;
    return 0;
}

static int load(void *ctx, uint32_t *size, uint32_t *crc, uint32_t *off)
{
    (void)ctx;
    if (hw.saved_size == 0) {
        return -1;
    }
    *size = hw.saved_size;
    *crc = hw.saved_crc;
    *off = hw.saved_off;
    return 0;
}

static int activate(void *ctx, uint32_t size, uint32_t crc)
{
    (void)ctx;
    assert(size == IMAGE_SIZE && crc == IMAGE_CRC);
    ++hw.activations;
    return 0;
}

static int send(void *ctx, const uint8_t *frame, size_t len)
{
    (void)ctx;
    assert(len == sizeof(hw.tx));
    memcpy(hw.tx, frame, len);
    ++hw.transmissions;
    return 0;
}

/* CRC-8/SMBUS using polynomial division, independently of the implementation. */
static void frame_crc(uint8_t *frame)
{
    unsigned remainder = 0;
    for (size_t i = 2; i < 19; ++i) {
        remainder = (remainder ^ frame[i]) << 8;
        for (unsigned bit = 0x8000, divisor = 0x8380; bit >= 0x100; bit >>= 1, divisor >>= 1) {
            if (remainder & bit) {
                remainder ^= divisor;
            }
        }
    }
    frame[19] = (uint8_t)remainder;
}

static void command(struct lp_wire *wire, uint8_t type, uint8_t seq,
                    const uint8_t *data, uint8_t len, uint8_t expect_status)
{
    uint8_t frame[LP_WIRE_FRAME_SIZE] = {0xa5, 0x5a, type, seq, len};

    if (len != 0) {
        memcpy(frame + 5, data, len);
    }
    frame_crc(frame);

    /* UART reads may split a frame anywhere. */
    unsigned before = hw.transmissions;
    lp_wire_rx(wire, frame, 7);
    assert(hw.transmissions == before);
    lp_wire_rx(wire, frame + 7, sizeof(frame) - 7);
    assert(hw.transmissions == before + 1);
    assert(hw.tx[2] == 0x80 && hw.tx[3] == seq && hw.tx[5] == type);
    assert(hw.tx[6] == expect_status);
}

static void start(struct lp_wire *wire, bool force)
{
    uint8_t payload[9] = {
        (uint8_t)IMAGE_SIZE, (uint8_t)(IMAGE_SIZE >> 8), 0, 0,
        (uint8_t)IMAGE_CRC, (uint8_t)(IMAGE_CRC >> 8),
        (uint8_t)(IMAGE_CRC >> 16), (uint8_t)(IMAGE_CRC >> 24), 1,
    };

    command(wire, 0x10, 0, payload, force ? 9 : 8, 0);
    assert(lp_wire_updating(wire));
    assert(u32(hw.tx + 11) == IMAGE_SIZE);
}

static void chunk(struct lp_wire *wire, unsigned off, uint8_t expect_status)
{
    uint8_t payload[LP_WIRE_PAYLOAD_SIZE];
    uint8_t len = IMAGE_SIZE - off < sizeof(payload) ?
                  (uint8_t)(IMAGE_SIZE - off) : sizeof(payload);

    for (uint8_t i = 0; i < len; ++i) {
        payload[i] = (uint8_t)(off + i);
    }
    command(wire, 0x11, (uint8_t)(off / LP_WIRE_PAYLOAD_SIZE), payload, len, expect_status);
}

int main(void)
{
    const struct lp_ota_ops ops = {
        .capacity = IMAGE_SIZE, .begin = begin, .write = write_chunk, .read = read_chunk,
        .save = save, .load = load, .activate = activate,
    };
    struct lp_wire wire;

    lp_wire_init(&wire, &ops, send, NULL);
    assert(lp_wire_identity(&wire, UINT32_C(0x04030201), LP_WIRE_BATTERY_UNKNOWN) == 0);
    assert(hw.tx[2] == 1 && hw.tx[4] == 5 && hw.tx[5] == 1 && hw.tx[9] == 0xff);
    assert(lp_wire_publish(&wire, 100, 1, 1234, 65) == 0);
    assert(hw.tx[2] == 2 && hw.tx[4] == 8 && u32(hw.tx + 5) == 100);
    assert(hw.tx[9] == 1 && hw.tx[10] == 0xd2 && hw.tx[11] == 0x04 && hw.tx[12] == 65);

    unsigned before = hw.transmissions;
    assert(lp_wire_publish(&wire, 599, 1, 1234, 65) == 0);
    assert(hw.transmissions == before);
    const uint8_t invalid_interval[] = {99, 0};
    const uint8_t valid_interval[] = {100, 0};
    command(&wire, 3, 9, invalid_interval, sizeof(invalid_interval), 3);
    command(&wire, 3, 10, valid_interval, sizeof(valid_interval), 0);
    assert(lp_wire_publish(&wire, 200, 1, 1234, 65) == 0 && hw.tx[2] == 2);

    start(&wire, false);
    start(&wire, false);
    assert(hw.erases == 1);
    before = hw.transmissions;
    assert(lp_wire_publish(&wire, 1000, 1, 1234, 65) == 0);
    assert(hw.transmissions == before);
    command(&wire, 0x12, 0, NULL, 0, 2);

    chunk(&wire, 0, 0);
    chunk(&wire, 0, 0);
    assert(hw.writes == 1);
    for (unsigned off = 14; off < 126; off += 14) {
        chunk(&wire, off, 0);
    }
    hw.fail_save = true;
    chunk(&wire, 126, 5);
    assert(hw.saved_off == 0);
    hw.fail_save = false;
    chunk(&wire, 126, 0);
    assert(hw.saved_off == 140);
    chunk(&wire, 140, 0);
    assert(hw.saved_off == 140);

    /* Simulate a reset with one post-checkpoint chunk already programmed. */
    lp_wire_init(&wire, &ops, send, NULL);
    start(&wire, false);
    assert(u32(hw.tx + 7) == 140 && hw.erases == 1);

    for (unsigned i = 0; i < 6; ++i) {
        chunk(&wire, 280, 4);
    }
    assert(!lp_wire_updating(&wire) && hw.saved_off == 140);
    start(&wire, false);
    assert(u32(hw.tx + 7) == 140);
    for (unsigned off = 140; off < IMAGE_SIZE; off += 14) {
        chunk(&wire, off, 0);
    }
    assert(hw.saved_off == IMAGE_SIZE && hw.commits == 28);
    chunk(&wire, 3696, 0); /* ACK loss on the final partial chunk. */

    hw.corrupt_readback = true;
    command(&wire, 0x12, 0, NULL, 0, 6);
    assert(hw.activations == 0 && !lp_wire_updating(&wire));
    hw.corrupt_readback = false;
    start(&wire, false);
    command(&wire, 0x12, 0, NULL, 0, 0);
    command(&wire, 0x12, 0, NULL, 0, 0);
    assert(hw.activations == 1 && !lp_wire_updating(&wire));

    lp_wire_abort(&wire);
    start(&wire, true);
    assert(hw.erases == 2 && hw.saved_off == 0);

    /* Corrupted/truncated packet followed by a valid packet on the same stream. */
    uint8_t damaged[7] = {0xa5, 0x5a, 0x11, 0, 14, 1, 2};
    uint8_t valid[LP_WIRE_FRAME_SIZE] = {0xa5, 0x5a, 0x03, 5, 2, 0xe8, 0x03};
    frame_crc(valid);
    before = hw.transmissions;
    lp_wire_rx(&wire, damaged, sizeof(damaged));
    lp_wire_rx(&wire, valid, sizeof(valid));
    assert(hw.transmissions == before + 2 && hw.tx[3] == 5 && hw.tx[6] == 0);
    assert(wire.interval_ms == 1000);

    puts("wireless protocol tests passed");
    return 0;
}
