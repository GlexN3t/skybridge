/**
 * @file mavlink_decode.h
 * @brief Typed decoders for the telemetry message set (payload -> struct).
 *
 * Pure functions, no I/O: takes a parsed mav_frame_t, validates the
 * payload length, and fills a little-endian wire struct. Shared
 * verbatim between the firmware and the host-side bridge_sim, and
 * covered by host unit tests.
 */
#ifndef MAVLINK_DECODE_H
#define MAVLINK_DECODE_H

#include <stdint.h>

#include "mavlink_parse.h"

typedef struct {
    uint32_t time_boot_ms;
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
} mav_attitude_t;

typedef struct {
    int32_t lat_e7;        /* degrees * 1e7 */
    int32_t lon_e7;
    int32_t rel_alt_mm;
} mav_position_t;

typedef struct {
    uint16_t voltage_mv;
    int16_t current_ma;
    int8_t battery_remaining_pct;
} mav_sys_status_t;

/* All return 0 on success, -1 if the payload is shorter than the
 * message definition requires. */
int mav_decode_attitude(const mav_frame_t *f, mav_attitude_t *out);
int mav_decode_global_position(const mav_frame_t *f, mav_position_t *out);
int mav_decode_sys_status(const mav_frame_t *f, mav_sys_status_t *out);

#endif /* MAVLINK_DECODE_H */
