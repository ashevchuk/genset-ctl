#pragma once
#include <Arduino.h>

// LAN discovery counterpart to the ESP-NOW generator list (esp_now_client.h)
// — finds controllers reachable directly over the panel's existing WiFi/IP
// network instead of over the radio link, so it also works for a controller
// with no ESP-NOW key configured at all. Request/reply broadcast UDP; wire
// format must match ../gen_discovery.h byte for byte.
#define UDP_DISCOVERY_PORT          52323
#define UDP_DISCOVERY_MAGIC         0xD7
#define UDP_DISCOVERY_PROTO_VERSION 1
#define UDP_DISCOVERY_TYPE_REQUEST  0
#define UDP_DISCOVERY_TYPE_REPLY    1
#define UDP_DISCOVERY_NAME_LEN      24

struct __attribute__((packed)) UdpDiscoveryPkt {
    uint8_t magic;
    uint8_t proto_version;
    uint8_t type;
    char    name[UDP_DISCOVERY_NAME_LEN];
};

struct UdpGenInfo {
    char name[UDP_DISCOVERY_NAME_LEN + 1];
    char ip[16];
};

// Binds the local UDP socket (idempotent). Call once from setup().
void udp_discovery_init(void);

// Broadcasts one discovery request. Needs an IP already (STA or AP) to go
// anywhere — call from the Config tab's Scan button, alongside the existing
// WiFi.scanNetworks().
void udp_discovery_request(void);

// Drains any pending replies into the seen list. Call every loop().
void udp_discovery_poll(void);

// Controllers heard recently (replies older than the internal timeout are
// dropped). Returns the number of entries written to `out`.
int udp_discovery_list(UdpGenInfo *out, int max);
