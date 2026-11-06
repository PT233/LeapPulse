#include "wireless.h"

#include <string.h>

#define MAGIC_FIRST 0xa5u
#define MAGIC_SECOND 0x5au
#define CHECKPOINT_BYTES (10u * LP_WIRE_PAYLOAD_SIZE)
#define MAX_CONSECUTIVE_ERRORS 5u

enum message {
    MSG_IDENTITY = 0x01,
    MSG_TELEMETRY = 0x02,
    MSG_INTERVAL = 0x03,
    MSG_START = 0x10,
    MSG_DATA = 0x11,
    MSG_FINISH = 0x12,
    MSG_REPLY = 0x80,
};

enum status {
    STATUS_OK = 0,
    STATUS_FRAME = 1,
    STATUS_SESSION = 2,
    STATUS_RANGE = 3,
    STATUS_SEQUENCE = 4,
    STATUS_FLASH = 5,
    STATUS_CHECKSUM = 6,
};

static uint32_t get_u32(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8) |
           ((uint32_t)data[2] << 16) | ((uint32_t)data[3] << 24);
}

static void put_u32(uint8_t *data, uint32_t value)
{
    for (size_t i = 0; i < 4; ++i) {
        data[i] = (uint8_t)(value >> (i * 8));
    }
}

static uint8_t crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0;

    while (len--) {
        crc ^= *data++;
        for (size_t bit = 0; bit < 8; ++bit) {
            crc = (uint8_t)((crc & 0x80u) ? (crc << 1) ^ 0x07 : crc << 1);
        }
    }
    return crc;
}

static uint32_t crc32_update(uint32_t crc, const uint8_t *data, size_t len)
{
    while (len--) {
        crc ^= *data++;
        for (size_t bit = 0; bit < 8; ++bit) {
            crc = (crc >> 1) ^ ((crc & 1u) ? UINT32_C(0xedb88320) : 0u);
        }
    }
    return crc;
}

static int send_frame(struct lp_wire *wire, uint8_t type, uint8_t seq,
                      const uint8_t *payload, uint8_t len)
{
    uint8_t frame[LP_WIRE_FRAME_SIZE] = { MAGIC_FIRST, MAGIC_SECOND, type, seq, len };

    memcpy(frame + 5, payload, len);
    frame[19] = crc8(frame + 2, 17);
    return wire->send(wire->send_ctx, frame, sizeof(frame));
}

static int reply(struct lp_wire *wire, uint8_t type, uint8_t seq, enum status status)
{
    uint8_t payload[10] = { type, (uint8_t)status };

    put_u32(payload + 2, wire->off);
    put_u32(payload + 6, wire->ota.capacity);
    return send_frame(wire, MSG_REPLY, seq, payload, sizeof(payload));
}

void lp_wire_init(struct lp_wire *wire, const struct lp_ota_ops *ops,
                  lp_wire_send_fn send, void *send_ctx)
{
    memset(wire, 0, sizeof(*wire));
    if (ops != NULL) {
        wire->ota = *ops;
    }
    wire->send = send;
    wire->send_ctx = send_ctx;
    wire->interval_ms = 500;
}

void lp_wire_abort(struct lp_wire *wire)
{
    wire->updating = false;
    wire->activated = false;
    wire->rx_len = 0;
    wire->off = 0;
    wire->bad_frames = 0;
    wire->published = false;
}

bool lp_wire_updating(const struct lp_wire *wire)
{
    return wire->updating;
}

static enum status start(struct lp_wire *wire, const uint8_t *data, uint8_t len)
{
    const struct lp_ota_ops *ops = &wire->ota;

    if (ops->begin == NULL || ops->write == NULL || ops->read == NULL ||
        ops->save == NULL || ops->load == NULL || ops->activate == NULL) {
        return STATUS_SESSION;
    }

    if (len != 8 && (len != 9 || data[8] != 1)) {
        return STATUS_RANGE;
    }

    uint32_t size = get_u32(data);
    uint32_t crc32 = get_u32(data + 4);
    bool force = len == 9;

    if (size == 0 || size > ops->capacity) {
        return STATUS_RANGE;
    }
    if (wire->activated) {
        return STATUS_SESSION;
    }
    if (!force && wire->updating && wire->size == size && wire->crc32 == crc32) {
        wire->bad_frames = 0;
        return STATUS_OK;
    }

    lp_wire_abort(wire);
    uint32_t off = 0;
    uint32_t saved_size = 0;
    uint32_t saved_crc32 = 0;
    int rc = force ? -1 : ops->load(ops->ctx, &saved_size, &saved_crc32, &off);
    bool valid = rc == 0 && saved_size == size && saved_crc32 == crc32 &&
                 (off == size || (off < size && off % CHECKPOINT_BYTES == 0));

    if (!valid) {
        off = 0;
        if (ops->begin(ops->ctx, size, crc32) != 0 ||
            ops->save(ops->ctx, size, crc32, 0) != 0) {
            return STATUS_FLASH;
        }
    }

    wire->size = size;
    wire->crc32 = crc32;
    wire->off = off;
    wire->updating = true;
    return STATUS_OK;
}

static void record_bad_frame(struct lp_wire *wire)
{
    if (wire->updating && ++wire->bad_frames > MAX_CONSECUTIVE_ERRORS) {
        lp_wire_abort(wire);
    }
}

static enum status write_chunk(struct lp_wire *wire, uint8_t seq,
                               const uint8_t *data, uint8_t len)
{
    if (!wire->updating) {
        return STATUS_SESSION;
    }

    if (wire->off != 0 && seq == (uint8_t)((wire->off - 1) / LP_WIRE_PAYLOAD_SIZE)) {
        uint8_t previous[LP_WIRE_PAYLOAD_SIZE];
        uint32_t previous_off = (wire->off - 1) / LP_WIRE_PAYLOAD_SIZE * LP_WIRE_PAYLOAD_SIZE;

        if (len != wire->off - previous_off) {
            record_bad_frame(wire);
            return STATUS_RANGE;
        }
        if (wire->ota.read(wire->ota.ctx, previous_off, previous, len) != 0) {
            return STATUS_FLASH;
        }
        if (memcmp(previous, data, len) != 0) {
            record_bad_frame(wire);
            return STATUS_SEQUENCE;
        }
        wire->bad_frames = 0;
        return STATUS_OK;
    }

    uint32_t remaining = wire->size - wire->off;
    uint8_t expected_len = remaining < LP_WIRE_PAYLOAD_SIZE ?
                           (uint8_t)remaining : LP_WIRE_PAYLOAD_SIZE;

    if (seq != (uint8_t)(wire->off / LP_WIRE_PAYLOAD_SIZE) || remaining == 0) {
        record_bad_frame(wire);
        return STATUS_SEQUENCE;
    }
    if (len != expected_len) {
        record_bad_frame(wire);
        return STATUS_RANGE;
    }

    const struct lp_ota_ops *ops = &wire->ota;
    uint32_t next = wire->off + len;

    if (ops->write(ops->ctx, wire->off, data, len) != 0) {
        return STATUS_FLASH;
    }
    if ((next == wire->size || next % CHECKPOINT_BYTES == 0) &&
        ops->save(ops->ctx, wire->size, wire->crc32, next) != 0) {
        return STATUS_FLASH;
    }

    wire->off = next;
    wire->bad_frames = 0;
    return STATUS_OK;
}

static enum status finish(struct lp_wire *wire, uint8_t len)
{
    if (len != 0) {
        return STATUS_RANGE;
    }
    if (wire->activated) {
        return STATUS_OK;
    }
    if (!wire->updating || wire->off != wire->size) {
        return STATUS_SESSION;
    }

    const struct lp_ota_ops *ops = &wire->ota;
    uint8_t buffer[64];
    uint32_t crc = UINT32_MAX;

    for (uint32_t off = 0; off < wire->size;) {
        uint32_t remaining = wire->size - off;
        size_t count = remaining < sizeof(buffer) ? (size_t)remaining : sizeof(buffer);

        if (ops->read(ops->ctx, off, buffer, count) != 0) {
            return STATUS_FLASH;
        }
        crc = crc32_update(crc, buffer, count);
        off += (uint32_t)count;
    }

    if ((crc ^ UINT32_MAX) != wire->crc32) {
        lp_wire_abort(wire);
        return STATUS_CHECKSUM;
    }

    if (ops->activate(ops->ctx, wire->size, wire->crc32) != 0) {
        return STATUS_FLASH;
    }

    wire->updating = false;
    wire->activated = true;
    return STATUS_OK;
}

static enum status set_interval(struct lp_wire *wire, const uint8_t *data, uint8_t len)
{
    if (len != 2) {
        return STATUS_RANGE;
    }
    uint16_t interval = (uint16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));

    if (interval < 100 || interval > 1000) {
        return STATUS_RANGE;
    }
    wire->interval_ms = interval;
    return STATUS_OK;
}

static void handle_frame(struct lp_wire *wire)
{
    uint8_t type = wire->rx[2];
    uint8_t seq = wire->rx[3];
    uint8_t len = wire->rx[4];
    const uint8_t *payload = wire->rx + 5;
    enum status status;

    switch (type) {
    case MSG_INTERVAL:
        status = set_interval(wire, payload, len);
        break;
    case MSG_START:
        status = start(wire, payload, len);
        break;
    case MSG_DATA:
        status = write_chunk(wire, seq, payload, len);
        break;
    case MSG_FINISH:
        status = finish(wire, len);
        break;
    default:
        status = STATUS_FRAME;
        break;
    }

    (void)reply(wire, type, seq, status);
}

static void resync(struct lp_wire *wire)
{
    for (size_t i = 1; i + 1 < sizeof(wire->rx); ++i) {
        if (wire->rx[i] == MAGIC_FIRST && wire->rx[i + 1] == MAGIC_SECOND) {
            wire->rx_len = (uint8_t)(sizeof(wire->rx) - i);
            memmove(wire->rx, wire->rx + i, wire->rx_len);
            return;
        }
    }
    wire->rx_len = 0;
    if (wire->rx[sizeof(wire->rx) - 1] == MAGIC_FIRST) {
        wire->rx[0] = MAGIC_FIRST;
        wire->rx_len = 1;
    }
}

void lp_wire_rx(struct lp_wire *wire, const uint8_t *data, size_t len)
{
    while (len--) {
        uint8_t byte = *data++;

        if (wire->rx_len == 0 && byte != MAGIC_FIRST) {
            continue;
        }
        if (wire->rx_len == 1 && byte != MAGIC_SECOND) {
            wire->rx_len = byte == MAGIC_FIRST ? 1 : 0;
            continue;
        }
        wire->rx[wire->rx_len++] = byte;

        if (wire->rx_len != LP_WIRE_FRAME_SIZE) {
            continue;
        }
        if (wire->rx[4] <= LP_WIRE_PAYLOAD_SIZE && crc8(wire->rx + 2, 17) == wire->rx[19]) {
            wire->rx_len = 0;
            handle_frame(wire);
        } else {
            record_bad_frame(wire);
            (void)reply(wire, wire->rx[2], wire->rx[3], STATUS_FRAME);
            resync(wire);
        }
    }
}

int lp_wire_publish(struct lp_wire *wire, uint32_t timestamp_ms,
                    uint8_t mode, uint16_t count, uint8_t bpm)
{
    if (wire->updating || wire->activated ||
        (wire->published && timestamp_ms - wire->last_publish_ms < wire->interval_ms)) {
        return 0;
    }

    uint8_t payload[8];

    put_u32(payload, timestamp_ms);
    payload[4] = mode;
    payload[5] = (uint8_t)count;
    payload[6] = (uint8_t)(count >> 8);
    payload[7] = bpm;

    wire->published = true;
    wire->last_publish_ms = timestamp_ms;
    return send_frame(wire, MSG_TELEMETRY, 0, payload, sizeof(payload));
}

int lp_wire_identity(struct lp_wire *wire, uint32_t device_id, uint8_t battery)
{
    if (battery > 100 && battery != LP_WIRE_BATTERY_UNKNOWN) {
        return -1;
    }

    uint8_t payload[5];

    put_u32(payload, device_id);
    payload[4] = battery;
    return send_frame(wire, MSG_IDENTITY, 0, payload, sizeof(payload));
}
