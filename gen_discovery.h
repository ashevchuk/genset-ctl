#pragma once
#include <stdint.h>
#include <stddef.h>

// LAN UDP discovery: lets a display panel find this controller's IP on the
// local network without the operator typing it in. Independent of ESP-NOW
// (../ESPNOW.md) — no key, no radio pairing, just request/reply broadcast
// UDP, so it also works for panels that never configured ESP-NOW at all.
// Wire format must match display/udp_discovery.h byte for byte (hand
// duplicated, same convention ESP-NOW's own packet structs already use).
#define GEN_DISCOVERY_UDP_PORT      52323
#define GEN_DISCOVERY_MAGIC         0xD7
#define GEN_DISCOVERY_PROTO_VERSION 1
#define GEN_DISCOVERY_TYPE_REQUEST  0
#define GEN_DISCOVERY_TYPE_REPLY    1
#define GEN_DISCOVERY_NAME_LEN      24

struct __attribute__((packed)) GenDiscoveryPkt {
  uint8_t magic;
  uint8_t proto_version;
  uint8_t type;
  char    name[GEN_DISCOVERY_NAME_LEN];  // reply only; zero on a request
};

// Binds the UDP listening socket. Call once from setup_web_servers().
void gen_discovery_begin(void);

// Answers a pending discovery request, if any, with this controller's
// hostname. Call every WebTask tick (web.cpp) — non-blocking.
void gen_discovery_poll(void);
