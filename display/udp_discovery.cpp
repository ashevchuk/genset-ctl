#include "udp_discovery.h"
#include <WiFiUdp.h>
#include <string.h>

#define UDP_DISCOVERY_SEEN_MAX        8
#define UDP_DISCOVERY_SEEN_TIMEOUT_MS 15000

struct SeenLan {
    bool     used;
    char     name[UDP_DISCOVERY_NAME_LEN + 1];
    char     ip[16];
    uint32_t last_ms;
};

static WiFiUDP s_udp;
static bool    s_inited = false;
static SeenLan s_seen[UDP_DISCOVERY_SEEN_MAX];

void udp_discovery_init(void) {
    if (s_inited) return;
    s_inited = s_udp.begin(UDP_DISCOVERY_PORT);
}

void udp_discovery_request(void) {
    if (!s_inited) udp_discovery_init();
    UdpDiscoveryPkt pkt = {UDP_DISCOVERY_MAGIC, UDP_DISCOVERY_PROTO_VERSION,
                            UDP_DISCOVERY_TYPE_REQUEST, {0}};
    s_udp.beginPacket(IPAddress(255, 255, 255, 255), UDP_DISCOVERY_PORT);
    s_udp.write((const uint8_t *)&pkt, sizeof(pkt));
    s_udp.endPacket();
}

void udp_discovery_poll(void) {
    if (!s_inited) return;
    int n = s_udp.parsePacket();
    if (n != (int)sizeof(UdpDiscoveryPkt)) {
        if (n > 0) s_udp.flush();
        return;
    }
    UdpDiscoveryPkt pkt;
    if (s_udp.read((uint8_t *)&pkt, sizeof(pkt)) != (int)sizeof(pkt)) return;
    if (pkt.magic != UDP_DISCOVERY_MAGIC || pkt.proto_version != UDP_DISCOVERY_PROTO_VERSION ||
        pkt.type != UDP_DISCOVERY_TYPE_REPLY)
        return;

    char name[UDP_DISCOVERY_NAME_LEN + 1];
    strlcpy(name, pkt.name, sizeof(name));
    // Unauthenticated over-the-air text, same treatment as the ESP-NOW
    // beacon name (esp_now_client.cpp) — it ends up in LVGL's '\n'-separated
    // dropdown options.
    for (char *c = name; *c; c++)
        if (*c < 0x20 || *c > 0x7e || *c == '"' || *c == '\\') *c = '?';
    String ipStr = s_udp.remoteIP().toString();

    int slot = -1;
    for (int i = 0; i < UDP_DISCOVERY_SEEN_MAX; i++) {
        if (s_seen[i].used && strcmp(s_seen[i].ip, ipStr.c_str()) == 0) { slot = i; break; }
        if (slot < 0 && !s_seen[i].used) slot = i;
    }
    if (slot < 0) slot = 0;  // 8 slots is far more than one LAN ever needs — plain overwrite, not LRU
    strlcpy(s_seen[slot].name, name, sizeof(s_seen[slot].name));
    strlcpy(s_seen[slot].ip, ipStr.c_str(), sizeof(s_seen[slot].ip));
    s_seen[slot].last_ms = millis();
    s_seen[slot].used = true;
}

int udp_discovery_list(UdpGenInfo *out, int max) {
    int n = 0;
    uint32_t now = millis();
    for (int i = 0; i < UDP_DISCOVERY_SEEN_MAX && n < max; i++) {
        if (!s_seen[i].used || now - s_seen[i].last_ms > UDP_DISCOVERY_SEEN_TIMEOUT_MS) continue;
        strlcpy(out[n].name, s_seen[i].name, sizeof(out[n].name));
        strlcpy(out[n].ip,   s_seen[i].ip,   sizeof(out[n].ip));
        n++;
    }
    return n;
}
