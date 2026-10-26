/*
 * Host-side test harness for the MAVLink parser.
 *
 * Runs on the build machine with plain gcc (no ESP-IDF needed), so
 * protocol logic can be verified in CI before it ever touches a chip:
 *
 *   gcc -Wall -Wextra -O2 -I firmware/main \
 *       tools/test_mavlink_parse.c firmware/main/mavlink_parse.c \
 *       -o build/test_mavlink_parse && ./build/test_mavlink_parse
 */
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "mavlink_decode.h"
#include "mavlink_parse.h"

#define HEARTBEAT_CRC_EXTRA 50
#define ATTITUDE_CRC_EXTRA  39
#define SYS_STATUS_CRC_EXTRA 124
#define GLOBAL_POSITION_CRC_EXTRA 104

static int tests_run = 0;

/* Build a frame in-process and hand it to a decoder via the real
 * parser — pins the payload field offsets against regressions. */
static mav_frame_t build_and_parse(uint32_t msgid, uint8_t crc_extra,
                                   const uint8_t *payload, uint8_t len)
{
    uint8_t buf[MAVLINK_MAX_FRAME_LEN];
    size_t total = mav_frame_build(msgid, crc_extra, payload, len,
                                   0, 1, 1, buf);
    assert(total > 0);
    mav_parser_t p;
    mav_parser_init(&p);
    mav_frame_t f;
    for (size_t i = 0; i < total; i++)
        assert(mav_parser_feed(&p, buf[i], &f) != MAV_PARSE_ERROR_CRC);
    return f;
}

static void test_decode_sys_status_offsets(void)
{
    /* Official layout: sensors x3 u32, load u16@12, voltage u16@14,
     * current i16@16, remaining i8@18, ... (31 bytes base) */
    uint8_t payload[31] = {0};
    payload[12] = 0x5E; payload[13] = 0x01;          /* load = 350 */
    payload[14] = 0x4E; payload[15] = 0x31;          /* voltage = 12622 mV */
    payload[16] = 0x68; payload[17] = 0x10;          /* current = 4200 mA */
    payload[18] = (uint8_t)(int8_t)-3;               /* remaining = -3% */

    mav_frame_t f = build_and_parse(MAVLINK_MSG_ID_SYS_STATUS,
                                    SYS_STATUS_CRC_EXTRA, payload, 31);
    mav_sys_status_t out;
    assert(mav_decode_sys_status(&f, &out) == 0);
    assert(out.voltage_mv == 12622);
    assert(out.current_ma == 4200);
    assert(out.battery_remaining_pct == -3);
    tests_run++;
}

static void test_decode_global_position_offsets(void)
{
    /* Official layout: time u32@0, lat i32@4, lon i32@8, alt i32@12,
     * relative_alt i32@16, vx/vy/vz i16, hdg u16 (28 bytes) */
    uint8_t payload[28] = {0};
    const int32_t lat = 473977418, lon = 85455938, rel_alt = 50000;
    memcpy(payload + 4, &lat, 4);
    memcpy(payload + 8, &lon, 4);
    memcpy(payload + 16, &rel_alt, 4);

    mav_frame_t f = build_and_parse(MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                                    GLOBAL_POSITION_CRC_EXTRA, payload, 28);
    mav_position_t out;
    assert(mav_decode_global_position(&f, &out) == 0);
    assert(out.lat_e7 == 473977418);   /* ArduPilot SITL home */
    assert(out.lon_e7 == 85455938);
    assert(out.rel_alt_mm == 50000);   /* 50 m */
    tests_run++;
}

static void test_decode_attitude_offsets(void)
{
    /* Official layout: time u32@0, then 6 floats: roll@4, pitch@8,
     * yaw@12 (28 bytes total) */
    uint8_t payload[28] = {0};
    const float roll = 0.2f, pitch = -0.15f, yaw = 1.5f;
    memcpy(payload + 4, &roll, 4);
    memcpy(payload + 8, &pitch, 4);
    memcpy(payload + 12, &yaw, 4);

    mav_frame_t f = build_and_parse(MAVLINK_MSG_ID_ATTITUDE,
                                    ATTITUDE_CRC_EXTRA, payload, 28);
    mav_attitude_t out;
    assert(mav_decode_attitude(&f, &out) == 0);
    assert(out.roll_rad > 0.19f && out.roll_rad < 0.21f);
    assert(out.pitch_rad > -0.16f && out.pitch_rad < -0.14f);
    assert(out.yaw_rad > 1.49f && out.yaw_rad < 1.51f);
    tests_run++;
}

static void test_crc_known_answer(void)
{
    /* CRC-16/X-25 check value for "123456789" per the CRC catalog
     * (reveng.sourceforge.io/crc-catalogue). */
    uint16_t crc = mav_crc_init();
    const char *s = "123456789";
    while (*s)
        mav_crc_accumulate((uint8_t)*s++, &crc);
    assert((uint16_t)(crc ^ 0xFFFF) == 0x906E); /* X-25 final XOR */
    tests_run++;
}

static void test_roundtrip_heartbeat(void)
{
    /* HEARTBEAT payload per minimal.xml: custom_mode u32, type u8,
     * autopilot u8, base_mode u8, system_status u8, ...
     * (we only test framing, not field semantics) */
    uint8_t payload[9] = {0x03, 0, 0, 0, 0x02, 0x03, 0x59, 0x04, 0x00};
    uint8_t buf[MAVLINK_MAX_FRAME_LEN];
    size_t len = mav_frame_build(MAVLINK_MSG_ID_HEARTBEAT,
                                 HEARTBEAT_CRC_EXTRA, payload, sizeof(payload),
                                 42, 1, 1, buf);
    assert(len == 10 + 9 + 2);
    assert(buf[0] == MAVLINK_V2_STX);
    assert(buf[1] == 9);
    assert(buf[4] == 42); /* seq */

    mav_parser_t p;
    mav_parser_init(&p);
    mav_frame_t f;
    mav_parse_status_t st = MAV_PARSE_NEED_DATA;
    for (size_t i = 0; i < len; i++)
        st = mav_parser_feed(&p, buf[i], &f);
    assert(st == MAV_PARSE_COMPLETE);
    assert(f.msgid == MAVLINK_MSG_ID_HEARTBEAT);
    assert(f.seq == 42);
    assert(f.sysid == 1 && f.compid == 1);
    assert(f.payload_len == 9);
    assert(memcmp(f.payload, payload, 9) == 0);
    tests_run++;
}

static void test_crc_rejection(void)
{
    uint8_t payload[30] = {0};
    uint8_t buf[MAVLINK_MAX_FRAME_LEN];
    size_t len = mav_frame_build(MAVLINK_MSG_ID_ATTITUDE,
                                 ATTITUDE_CRC_EXTRA, payload, sizeof(payload),
                                 0, 1, 1, buf);

    /* Corrupt one payload byte mid-frame */
    buf[15] ^= 0xA5;

    mav_parser_t p;
    mav_parser_init(&p);
    mav_parse_status_t st = MAV_PARSE_NEED_DATA;
    for (size_t i = 0; i < len; i++)
        st = mav_parser_feed(&p, buf[i], NULL);
    assert(st == MAV_PARSE_ERROR_CRC);
    tests_run++;
}

static void test_garbage_between_frames(void)
{
    uint8_t payload[9] = {0};
    uint8_t buf[MAVLINK_MAX_FRAME_LEN];
    size_t len = mav_frame_build(MAVLINK_MSG_ID_HEARTBEAT,
                                 HEARTBEAT_CRC_EXTRA, payload, 9,
                                 7, 1, 1, buf);

    uint8_t noise[] = {0x00, 0xFF, 0x42, 0x13};
    mav_parser_t p;
    mav_parser_init(&p);
    mav_frame_t f;

    /* Leading noise must be reported as bad STX without desyncing us */
    for (size_t i = 0; i < sizeof(noise); i++)
        assert(mav_parser_feed(&p, noise[i], NULL)
               == MAV_PARSE_ERROR_BAD_STX);

    mav_parse_status_t st = MAV_PARSE_NEED_DATA;
    for (size_t i = 0; i < len; i++)
        st = mav_parser_feed(&p, buf[i], &f);
    assert(st == MAV_PARSE_COMPLETE);
    assert(f.seq == 7);
    tests_run++;
}

static void test_zero_payload_frame(void)
{
    uint8_t buf[MAVLINK_MAX_FRAME_LEN];
    size_t len = mav_frame_build(0xFFFF, 0x00, NULL, 0, 1, 1, 1, buf);
    assert(len == 12);

    mav_parser_t p;
    mav_parser_init(&p);
    mav_frame_t f;
    mav_parse_status_t st = MAV_PARSE_NEED_DATA;
    for (size_t i = 0; i < len; i++)
        st = mav_parser_feed(&p, buf[i], &f);
    assert(st == MAV_PARSE_COMPLETE);
    assert(f.payload_len == 0);
    assert(f.msgid == 0xFFFF);
    tests_run++;
}

static void test_stream_two_frames_back_to_back(void)
{
    uint8_t pl1[9] = {1}, pl2[30] = {2};
    uint8_t b1[MAVLINK_MAX_FRAME_LEN], b2[MAVLINK_MAX_FRAME_LEN];
    size_t l1 = mav_frame_build(MAVLINK_MSG_ID_HEARTBEAT, HEARTBEAT_CRC_EXTRA,
                                pl1, 9, 1, 1, 1, b1);
    size_t l2 = mav_frame_build(MAVLINK_MSG_ID_ATTITUDE, ATTITUDE_CRC_EXTRA,
                                pl2, 30, 2, 1, 1, b2);

    mav_parser_t p;
    mav_parser_init(&p);
    mav_frame_t f;
    mav_parse_status_t st = MAV_PARSE_NEED_DATA;
    for (size_t i = 0; i < l1; i++)
        st = mav_parser_feed(&p, b1[i], &f);
    assert(st == MAV_PARSE_COMPLETE && f.msgid == MAVLINK_MSG_ID_HEARTBEAT);

    /* Parser must be ready for the next frame immediately — no resync */
    for (size_t i = 0; i < l2; i++)
        st = mav_parser_feed(&p, b2[i], &f);
    assert(st == MAV_PARSE_COMPLETE && f.msgid == MAVLINK_MSG_ID_ATTITUDE);
    tests_run++;
}

int main(void)
{
    test_crc_known_answer();
    test_roundtrip_heartbeat();
    test_crc_rejection();
    test_garbage_between_frames();
    test_zero_payload_frame();
    test_stream_two_frames_back_to_back();
    test_decode_sys_status_offsets();
    test_decode_global_position_offsets();
    test_decode_attitude_offsets();
    printf("OK: %d test(s) passed\n", tests_run);
    return 0;
}
