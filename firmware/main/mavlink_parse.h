/**
 * @file mavlink_parse.h
 * @brief Hand-rolled MAVLink v2 frame parser and builder.
 *
 * Implements the MAVLink v2 wire format from the protocol spec
 * (https://mavlink.io/en/guide/serialization.html) with a byte-at-a-time
 * incremental state machine, so it can sit directly on a UART or UDP
 * socket without buffering whole frames.
 *
 * Frame layout (v2):
 *   [STX=0xFD][LEN][INCOMPAT][COMPAT][SEQ][SYSID][COMPID][MSGID 3xLE]
 *   [PAYLOAD (LEN bytes)][CRC16-X.25 2xLE]
 *
 * i.e. 1 STX byte + 9 header bytes + payload + 2 CRC bytes.
 *
 * The frame checksum is CRC-16/X-25 over the header fields after STX
 * plus the payload, with one extra "magic" byte per message type
 * (CRC_EXTRA) appended to catch cross-version corruption.
 */
#ifndef MAVLINK_PARSE_H
#define MAVLINK_PARSE_H

#include <stdint.h>
#include <stddef.h>

#define MAVLINK_V2_STX          0xFD
#define MAVLINK_HEADER_LEN      9     /* bytes between STX and payload */
#define MAVLINK_MAX_PAYLOAD_LEN 255
#define MAVLINK_MAX_FRAME_LEN   (1 + MAVLINK_HEADER_LEN + \
                                 MAVLINK_MAX_PAYLOAD_LEN + 2)

/* Message IDs we care about for telemetry (subset, spec §"minimal.xml") */
#define MAVLINK_MSG_ID_HEARTBEAT          0u   /* crc_extra 50  */
#define MAVLINK_MSG_ID_SYS_STATUS         1u   /* crc_extra 124 */
#define MAVLINK_MSG_ID_ATTITUDE           30u  /* crc_extra 39  */
#define MAVLINK_MSG_ID_GLOBAL_POSITION_INT 33u /* crc_extra 104 */
#define MAVLINK_MSG_ID_RC_CHANNELS        65u  /* crc_extra 244 */
#define MAVLINK_MSG_ID_BATTERY_STATUS     147u /* crc_extra 217 */

typedef enum {
    MAV_PARSE_NEED_DATA = 0,   /* frame not complete yet */
    MAV_PARSE_COMPLETE,        /* *frame_out holds a verified frame */
    MAV_PARSE_ERROR_BAD_STX,   /* garbage byte between frames */
    MAV_PARSE_ERROR_LENGTH,    /* payload byte count out of range */
    MAV_PARSE_ERROR_OVERFLOW,  /* internal state machine corruption */
    MAV_PARSE_ERROR_CRC,       /* checksum mismatch — frame dropped */
} mav_parse_status_t;

typedef struct {
    uint8_t  payload_len;
    uint8_t  incompat_flags;
    uint8_t  compat_flags;
    uint8_t  seq;
    uint8_t  sysid;
    uint8_t  compid;
    uint32_t msgid;          /* 24-bit, stored wide for convenience */
    uint8_t  payload[MAVLINK_MAX_PAYLOAD_LEN];
    uint16_t checksum;       /* as received on the wire */
} mav_frame_t;

/* Incremental parser: feed one byte at a time, poll after each. */
typedef struct {
    /* private */
    uint8_t  state;
    uint16_t header_idx;
    uint8_t  header[MAVLINK_HEADER_LEN]; /* bytes between STX and payload */
    uint16_t payload_idx;
    uint16_t crc;            /* running CRC over header+payload */
    mav_frame_t frame;
} mav_parser_t;

void mav_parser_init(mav_parser_t *p);

/* Feed one byte. When it returns MAV_PARSE_COMPLETE, *frame_out is
 * populated (if non-NULL) and the parser resets for the next frame.
 * Returns MAV_PARSE_NEED_DATA while mid-frame. */
mav_parse_status_t mav_parser_feed(mav_parser_t *p, uint8_t byte,
                                   mav_frame_t *frame_out);

/* Build a complete v2 frame into out[] (max MAVLINK_MAX_FRAME_LEN).
 * Returns total frame length in bytes, or 0 on invalid input. */
size_t mav_frame_build(uint32_t msgid, uint8_t crc_extra,
                       const uint8_t *payload, uint8_t payload_len,
                       uint8_t seq, uint8_t sysid, uint8_t compid,
                       uint8_t out[MAVLINK_MAX_FRAME_LEN]);

/* CRC-16/X-25 primitives, exported for tests. */
uint16_t mav_crc_init(void);
void     mav_crc_accumulate(uint8_t data, uint16_t *crc);
uint8_t  mav_crc_extra(uint32_t msgid);

#endif /* MAVLINK_PARSE_H */
