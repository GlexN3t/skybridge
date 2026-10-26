#include "mavlink_parse.h"

/* CRC-16/X-25: reflected polynomial 0x8408, init 0xFFFF, final XOR
 * 0xFFFF. This is the checksum MAVLink mandates for v2 frames. */
uint16_t mav_crc_init(void)
{
    return 0xFFFF;
}

void mav_crc_accumulate(uint8_t data, uint16_t *crc)
{
    *crc ^= (uint16_t)data;
    for (int i = 0; i < 8; i++) {
        if (*crc & 1)
            *crc = (uint16_t)((*crc >> 1) ^ 0x8408);
        else
            *crc >>= 1;
    }
}

static uint16_t crc_finalize(uint16_t crc)
{
    return (uint16_t)(crc ^ 0xFFFF);
}

/* Per-message CRC_EXTRA ("magic byte") from the MAVLink XML message
 * definitions. A parser that doesn't know the sender's message set
 * fails the checksum for unknown/changed messages by design. */
uint8_t mav_crc_extra(uint32_t msgid)
{
    switch (msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:           return 50;
    case MAVLINK_MSG_ID_SYS_STATUS:          return 124;
    case MAVLINK_MSG_ID_ATTITUDE:            return 39;
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: return 104;
    case MAVLINK_MSG_ID_RC_CHANNELS:         return 244;
    case MAVLINK_MSG_ID_BATTERY_STATUS:      return 217;
    default:                                 return 0;
    }
}

/* Parser states */
enum {
    ST_IDLE = 0,
    ST_HEADER,      /* collecting the 10 bytes between STX and payload */
    ST_PAYLOAD,
    ST_CRC1,
    ST_CRC2,
};

void mav_parser_init(mav_parser_t *p)
{
    p->state = ST_IDLE;
    p->header_idx = 0;
    p->payload_idx = 0;
    p->crc = mav_crc_init();
}

mav_parse_status_t mav_parser_feed(mav_parser_t *p, uint8_t byte,
                                   mav_frame_t *frame_out)
{
    switch (p->state) {
    case ST_IDLE:
        if (byte != MAVLINK_V2_STX)
            return MAV_PARSE_ERROR_BAD_STX;
        mav_parser_init(p);
        p->state = ST_HEADER;
        return MAV_PARSE_NEED_DATA;

    case ST_HEADER:
        p->header[p->header_idx++] = byte;
        mav_crc_accumulate(byte, &p->crc);
        if (p->header_idx < MAVLINK_HEADER_LEN)
            return MAV_PARSE_NEED_DATA;

        p->frame.payload_len = p->header[0];
        p->frame.incompat_flags = p->header[1];
        p->frame.compat_flags = p->header[2];
        p->frame.seq = p->header[3];
        p->frame.sysid = p->header[4];
        p->frame.compid = p->header[5];
        p->frame.msgid = (uint32_t)p->header[6]
                       | ((uint32_t)p->header[7] << 8)
                       | ((uint32_t)p->header[8] << 16);
        p->payload_idx = 0;
        p->state = (p->frame.payload_len > 0) ? ST_PAYLOAD : ST_CRC1;
        return MAV_PARSE_NEED_DATA;

    case ST_PAYLOAD:
        p->frame.payload[p->payload_idx++] = byte;
        mav_crc_accumulate(byte, &p->crc);
        if (p->payload_idx < p->frame.payload_len)
            return MAV_PARSE_NEED_DATA;
        p->state = ST_CRC1;
        return MAV_PARSE_NEED_DATA;

    case ST_CRC1:
        p->frame.checksum = byte;
        p->state = ST_CRC2;
        return MAV_PARSE_NEED_DATA;

    case ST_CRC2:
        p->frame.checksum |= (uint16_t)byte << 8;
        p->state = ST_IDLE;
        /* Fold in the message's CRC_EXTRA before comparing — the
         * sender computed its checksum the same way. */
        mav_crc_accumulate(mav_crc_extra(p->frame.msgid), &p->crc);
        if (crc_finalize(p->crc) != p->frame.checksum)
            return MAV_PARSE_ERROR_CRC;
        if (frame_out)
            *frame_out = p->frame;
        return MAV_PARSE_COMPLETE;

    default:
        mav_parser_init(p);
        return MAV_PARSE_ERROR_OVERFLOW;
    }
}

size_t mav_frame_build(uint32_t msgid, uint8_t crc_extra,
                       const uint8_t *payload, uint8_t payload_len,
                       uint8_t seq, uint8_t sysid, uint8_t compid,
                       uint8_t out[MAVLINK_MAX_FRAME_LEN])
{
    if (!out || (payload_len > 0 && !payload))
        return 0;

    out[0] = MAVLINK_V2_STX;
    out[1] = payload_len;      /* LEN */
    out[2] = 0;                /* INCOMPAT_FLAGS: no signing */
    out[3] = 0;                /* COMPAT_FLAGS */
    out[4] = seq;
    out[5] = sysid;
    out[6] = compid;
    out[7] = (uint8_t)(msgid & 0xFF);
    out[8] = (uint8_t)((msgid >> 8) & 0xFF);
    out[9] = (uint8_t)((msgid >> 16) & 0xFF);
    for (uint8_t i = 0; i < payload_len; i++)
        out[10 + i] = payload[i];

    uint16_t crc = mav_crc_init();
    for (size_t i = 1; i < (size_t)(MAVLINK_HEADER_LEN + 1 + payload_len); i++)
        mav_crc_accumulate(out[i], &crc);
    mav_crc_accumulate(crc_extra, &crc);
    uint16_t final = crc_finalize(crc);

    size_t total = 10 + payload_len;
    out[total] = (uint8_t)(final & 0xFF);
    out[total + 1] = (uint8_t)(final >> 8);
    return total + 2;
}
