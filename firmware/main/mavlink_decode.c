#include "mavlink_decode.h"

#include <string.h>

/*
 * The wire payload fields are little-endian. We reassemble them from
 * bytes explicitly (no memcpy of wider types, no unaligned access), so
 * this compiles identically on the ESP32 (Xtensa/ little-endian) and
 * any host.
 */

static uint16_t load_u16(const uint8_t *p)
{
    return (uint16_t)p[0] | (uint16_t)((uint16_t)p[1] << 8);
}

static int16_t load_i16(const uint8_t *p)
{
    return (int16_t)load_u16(p);
}

static uint32_t load_u32(const uint8_t *p)
{
    return (uint32_t)p[0]
         | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

static int32_t load_i32(const uint8_t *p)
{
    return (int32_t)load_u32(p);
}

/* IEEE-754 single precision lives in the low 32 bits, little-endian.
 * Going through the integer bit pattern (union) is the standard,
 * strict-aliasing-safe way to reinterpret wire bits as a float. */
static float load_f32(const uint8_t *p)
{
    union {
        uint32_t bits;
        float value;
    } conv;
    conv.bits = load_u32(p);
    return conv.value;
}

int mav_decode_attitude(const mav_frame_t *f, mav_attitude_t *out)
{
    if (f->msgid != MAVLINK_MSG_ID_ATTITUDE || f->payload_len < 28)
        return -1;
    out->time_boot_ms = load_u32(f->payload + 0);
    out->roll_rad = load_f32(f->payload + 4);
    out->pitch_rad = load_f32(f->payload + 8);
    out->yaw_rad = load_f32(f->payload + 12);
    return 0;
}

int mav_decode_global_position(const mav_frame_t *f, mav_position_t *out)
{
    if (f->msgid != MAVLINK_MSG_ID_GLOBAL_POSITION_INT ||
        f->payload_len < 28)
        return -1;
    /* time_boot_ms(0-3), lat(4-7), lon(8-11), alt MSL(12-15),
     * relative_alt(16-19), vx(20-21), vy(22-23), vz(24-25), hdg(26-27) */
    out->lat_e7 = load_i32(f->payload + 4);
    out->lon_e7 = load_i32(f->payload + 8);
    out->rel_alt_mm = load_i32(f->payload + 16);
    return 0;
}

int mav_decode_sys_status(const mav_frame_t *f, mav_sys_status_t *out)
{
    if (f->msgid != MAVLINK_MSG_ID_SYS_STATUS || f->payload_len < 31)
        return -1;
    /* sensors_present(0-3), enabled(4-7), healthy(8-11), load(12-13),
     * voltage_battery(14-15), current_battery(16-17), remaining(18) */
    out->voltage_mv = load_u16(f->payload + 14);
    out->current_ma = load_i16(f->payload + 16);
    out->battery_remaining_pct = (int8_t)f->payload[18];
    return 0;
}
