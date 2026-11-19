#include "gen_discovery.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <cstring>

extern String wifiHostname;

static WiFiUDP s_udp;
static bool    s_began = false;

void gen_discovery_begin(void) {
  s_began = s_udp.begin(GEN_DISCOVERY_UDP_PORT);
}

void gen_discovery_poll(void) {
  if (!s_began) return;
  int n = s_udp.parsePacket();
  if (n != (int)sizeof(GenDiscoveryPkt)) {
    if (n > 0) s_udp.flush();
    return;
  }
  GenDiscoveryPkt pkt;
  if (s_udp.read((uint8_t *)&pkt, sizeof(pkt)) != (int)sizeof(pkt)) return;
  if (pkt.magic != GEN_DISCOVERY_MAGIC || pkt.proto_version != GEN_DISCOVERY_PROTO_VERSION ||
      pkt.type != GEN_DISCOVERY_TYPE_REQUEST)
    return;

  IPAddress remoteIp   = s_udp.remoteIP();
  uint16_t  remotePort = s_udp.remotePort();

  GenDiscoveryPkt reply = {GEN_DISCOVERY_MAGIC, GEN_DISCOVERY_PROTO_VERSION,
                            GEN_DISCOVERY_TYPE_REPLY, {0}};
  strlcpy(reply.name, wifiHostname.c_str(), sizeof(reply.name));
  s_udp.beginPacket(remoteIp, remotePort);
  s_udp.write((const uint8_t *)&reply, sizeof(reply));
  s_udp.endPacket();
}
