#ifndef LEAPPULSE_WIRELESS_H
#define LEAPPULSE_WIRELESS_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define LP_WIRE_FRAME_SIZE 20u
#define LP_WIRE_PAYLOAD_SIZE 14u
#define LP_WIRE_BATTERY_UNKNOWN UINT8_MAX

/* The offsets are relative to the staging slot. Return 0 on success.
 * begin erases staging and the checkpoint journal. write accepts byte-identical
 * replay. save commits a durable checkpoint. load finds the latest valid record.
 * activate sets the boot pending flag. Reset only after the final ACK is sent.
 */
struct lp_ota_ops {
    void *ctx;
    uint32_t capacity;
    int (*begin)(void *ctx, uint32_t size, uint32_t crc32);
    int (*write)(void *ctx, uint32_t off, const uint8_t *data, size_t len);
    int (*read)(void *ctx, uint32_t off, uint8_t *data, size_t len);
    int (*save)(void *ctx, uint32_t size, uint32_t crc32, uint32_t off);
    int (*load)(void *ctx, uint32_t *size, uint32_t *crc32, uint32_t *off);
    int (*activate)(void *ctx, uint32_t size, uint32_t crc32);
};

/* Return 0 on success, negative on failure. The callback consumes data before returning. */
typedef int (*lp_wire_send_fn)(void *ctx, const uint8_t *data, size_t len);

struct lp_wire {
    struct lp_ota_ops ota;
    lp_wire_send_fn send;
    void *send_ctx;
    uint32_t size;
    uint32_t crc32;
    uint32_t off;
    uint32_t last_publish_ms;
    uint16_t interval_ms;
    uint8_t bad_frames;
    uint8_t rx_len;
    uint8_t rx[LP_WIRE_FRAME_SIZE];
    bool updating;
    bool activated;
    bool published;
};

/* Keep calls to each instance serialized in one application task. */
void lp_wire_init(struct lp_wire *wire, const struct lp_ota_ops *ops,
                  lp_wire_send_fn send, void *send_ctx);
void lp_wire_rx(struct lp_wire *wire, const uint8_t *data, size_t len);
void lp_wire_abort(struct lp_wire *wire);
bool lp_wire_updating(const struct lp_wire *wire);
int lp_wire_publish(struct lp_wire *wire, uint32_t timestamp_ms,
                    uint8_t mode, uint16_t count, uint8_t bpm);
int lp_wire_identity(struct lp_wire *wire, uint32_t device_id, uint8_t battery);

#endif
