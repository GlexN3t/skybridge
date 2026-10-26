/*
 * bridge_sim — the SkyBridge firmware data path, compiled for the host.
 *
 * Runs the same mavlink_parse.c + mavlink_decode.c that ship on the
 * ESP32, wired to a UDP socket instead of WiFi: sends a HEARTBEAT to
 * announce itself, then prints the incoming telemetry stream live.
 *
 * Purpose: verify the full firmware pipeline — socket bytes -> framing
 * parser -> typed decoders — with zero hardware, and give CI an
 * end-to-end interop test against tools/virtual_drone.py. When the
 * ESP32 arrives, net_link.c replaces the socket glue below; the parser
 * and decoders are unchanged.
 *
 *   gcc -Wall -Wextra -O2 -I firmware/main \
 *       tools/bridge_sim.c firmware/main/mavlink_parse.c \
 *       firmware/main/mavlink_decode.c -o build/bridge_sim
 *   ./build/bridge_sim 127.0.0.1 14550 --duration 5
 *
 * Style note: printf format strings are kept as single string literals
 * (never concatenated across lines) so format/argument checking tools
 * can always verify them.
 */
#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>

#include "mavlink_decode.h"
#include "mavlink_parse.h"

#define HEARTBEAT_CRC_EXTRA 50
#define ANNOUNCE_INTERVAL_S 1

typedef struct {
    unsigned long attitude;
    unsigned long position;
    unsigned long sys_status;
    unsigned long heartbeat;
    unsigned long crc_errors;
    unsigned long noise_bytes;
} bridge_stats_t;

static void print_frame(const mav_frame_t *f, bridge_stats_t *st)
{
    mav_attitude_t att;
    mav_position_t pos;
    mav_sys_status_t bat;

    switch (f->msgid) {
    case MAVLINK_MSG_ID_ATTITUDE:
        if (mav_decode_attitude(f, &att) == 0) {
            printf("ATTITUDE     roll=%+7.2f pitch=%+7.2f yaw=%7.2f\n",
                   (double)att.roll_rad * 57.2958,
                   (double)att.pitch_rad * 57.2958,
                   (double)att.yaw_rad * 57.2958);
            st->attitude++;
        }
        break;
    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
        if (mav_decode_global_position(f, &pos) == 0) {
            printf("POSITION     lat=%.7f lon=%.7f rel_alt=%.1f\n",
                   (double)pos.lat_e7 * 1e-7, (double)pos.lon_e7 * 1e-7,
                   (double)pos.rel_alt_mm / 1000.0);
            st->position++;
        }
        break;
    case MAVLINK_MSG_ID_SYS_STATUS:
        if (mav_decode_sys_status(f, &bat) == 0) {
            printf("SYS_STATUS   battery=%.2f current=%d remaining=%d\n",
                   (double)bat.voltage_mv / 1000.0, bat.current_ma,
                   bat.battery_remaining_pct);
            st->sys_status++;
        }
        break;
    case MAVLINK_MSG_ID_HEARTBEAT:
        st->heartbeat++;
        break;
    default:
        break; /* parsed fine; CRC_EXTRA table lacks this message id */
    }
}

static void send_heartbeat(int fd, const struct sockaddr_in *dst,
                           uint8_t seq)
{
    uint8_t payload[9] = {
        0x03, 0, 0, 0,      /* custom_mode */
        0x06,               /* MAV_TYPE_GCS */
        0x00,               /* MAV_AUTOPILOT_GENERIC */
        0x01,               /* custom mode enabled */
        0x04,               /* MAV_STATE_ACTIVE */
        3,                  /* mavlink version */
    };
    uint8_t buf[MAVLINK_MAX_FRAME_LEN];
    size_t len = mav_frame_build(MAVLINK_MSG_ID_HEARTBEAT,
                                 HEARTBEAT_CRC_EXTRA, payload,
                                 sizeof(payload), seq, 255, 190, buf);
    (void)sendto(fd, buf, len, 0, (const struct sockaddr *)dst,
                 sizeof(*dst));
}

static long parse_long(const char *s, long lo, long hi)
{
    char *end = NULL;
    long v = strtol(s, &end, 10);
    if (end == s || *end != '\0' || v < lo || v > hi)
        return -1;
    return v;
}

int main(int argc, char **argv)
{
    if (argc < 3 || argc > 5) {
        fprintf(stderr, "usage: %s <drone-host> <drone-port> [--duration SEC]\n", argv[0]);
        return 2;
    }
    const char *host = argv[1];
    long port = parse_long(argv[2], 1, 65535);
    long duration_s = 0; /* 0 = run until killed */
    if (port < 0) {
        fprintf(stderr, "bad port: %s\n", argv[2]);
        return 2;
    }
    if (argc == 5) {
        if (strcmp(argv[3], "--duration") != 0) {
            fprintf(stderr, "unknown option: %s\n", argv[3]);
            return 2;
        }
        duration_s = parse_long(argv[4], 1, 86400);
        if (duration_s < 0) {
            fprintf(stderr, "bad duration: %s\n", argv[4]);
            return 2;
        }
    }

    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) {
        perror("socket");
        return 1;
    }

    struct sockaddr_in dst;
    memset(&dst, 0, sizeof(dst));
    dst.sin_family = AF_INET;
    dst.sin_port = htons((uint16_t)port);
    if (inet_pton(AF_INET, host, &dst.sin_addr) != 1) {
        fprintf(stderr, "bad host: %s\n", host);
        close(fd);
        return 2;
    }

    mav_parser_t parser;
    mav_parser_init(&parser);
    bridge_stats_t stats;
    memset(&stats, 0, sizeof(stats));
    uint8_t seq = 0;
    mav_frame_t frame;

    /* Announce ourselves; repeat in case the drone starts after us. */
    send_heartbeat(fd, &dst, seq++);
    time_t last_announce = time(NULL);
    time_t started = last_announce;

    printf("bridge_sim: linked to %s:%ld — waiting for telemetry...\n", host, port);

    uint8_t pkt[2048];
    for (;;) {
        struct pollfd pfd;
        pfd.fd = fd;
        pfd.events = POLLIN;
        int rc = poll(&pfd, 1, 200);
        if (rc < 0) {
            if (errno == EINTR)
                continue;
            perror("poll");
            break;
        }
        if (time(NULL) - last_announce >= ANNOUNCE_INTERVAL_S) {
            send_heartbeat(fd, &dst, seq++);
            last_announce = time(NULL);
        }
        if (duration_s > 0 && time(NULL) - started >= duration_s)
            break;
        if (rc == 0 || !(pfd.revents & POLLIN))
            continue;

        ssize_t n = recvfrom(fd, pkt, sizeof(pkt), 0, NULL, NULL);
        if (n <= 0)
            continue;

        for (ssize_t i = 0; i < n; i++) {
            mav_parse_status_t st = mav_parser_feed(&parser, pkt[i], &frame);
            if (st == MAV_PARSE_COMPLETE)
                print_frame(&frame, &stats);
            else if (st == MAV_PARSE_ERROR_CRC)
                stats.crc_errors++;
            else if (st == MAV_PARSE_ERROR_BAD_STX)
                stats.noise_bytes++;
        }
    }

    printf("bridge_sim: attitude=%lu position=%lu sys_status=%lu heartbeats=%lu crc_errors=%lu noise=%lu\n",
           stats.attitude, stats.position, stats.sys_status,
           stats.heartbeat, stats.crc_errors, stats.noise_bytes);
    close(fd);
    return 0;
}
