#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "mavlink_parse.h"

/*
 * SkyBridge firmware — ESP32 MAVLink telemetry bridge.
 *
 * Current stage: protocol core online. The MAVLink v2 framing parser
 * is verified by host-side unit tests (tools/test_mavlink_parse.c)
 * and re-checked here on-target at boot, so any toolchain/porting
 * surprise surfaces immediately.
 *
 * Roadmap (see docs/architecture.md):
 *   [ ] WiFi STA bring-up + UDP socket to GCS/SITL
 *   [ ] Heartbeat exchange + telemetry stream request
 *   [ ] On-board WebSocket dashboard
 */

static void run_on_target_self_test(void)
{
    uint8_t payload[9] = {0x03, 0, 0, 0, 0x02, 0x03, 0x59, 0x04, 0x00};
    uint8_t buf[MAVLINK_MAX_FRAME_LEN];
    size_t len = mav_frame_build(MAVLINK_MSG_ID_HEARTBEAT, 50,
                                 payload, sizeof(payload), 1, 1, 1, buf);

    mav_parser_t p;
    mav_parser_init(&p);
    mav_frame_t f;
    mav_parse_status_t st = MAV_PARSE_NEED_DATA;
    for (size_t i = 0; i < len; i++)
        st = mav_parser_feed(&p, buf[i], &f);

    if (st == MAV_PARSE_COMPLETE &&
        f.msgid == MAVLINK_MSG_ID_HEARTBEAT &&
        memcmp(f.payload, payload, sizeof(payload)) == 0) {
        printf("[skybridge] MAVLink self-test PASSED\n");
    } else {
        printf("[skybridge] MAVLink self-test FAILED (st=%d)\n", st);
    }
}

void app_main(void)
{
    printf("[skybridge] boot\n");
    run_on_target_self_test();

    /* TODO(phase-1): WiFi STA + UDP task. The MAVLink parser is
     * byte-at-a-time, so the UDP receive loop will feed payload
     * bytes straight into mav_parser_feed() with no extra copy. */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
