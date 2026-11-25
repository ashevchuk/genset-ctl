#include "gen_discovery.h"
#include <lwip/sockets.h>
#include <cstring>

extern char wifiHostname[33];

static int s_sock = -1;

void gen_discovery_begin(void) {
  int sock = socket(AF_INET, SOCK_DGRAM, 0);
  if (sock < 0) return;
  int yes = 1;
  setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
  struct sockaddr_in addr = {};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(GEN_DISCOVERY_UDP_PORT);
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    close(sock);
    return;
  }
  fcntl(sock, F_SETFL, O_NONBLOCK);
  s_sock = sock;
}

void gen_discovery_poll(void) {
  if (s_sock < 0) return;
  // One byte larger than a valid packet so an oversized datagram is
  // detected by its length instead of being silently truncated to fit.
  uint8_t buf[sizeof(GenDiscoveryPkt) + 1];
  struct sockaddr_in remote = {};
  socklen_t remote_len = sizeof(remote);
  int n = recvfrom(s_sock, buf, sizeof(buf), MSG_DONTWAIT, (struct sockaddr *)&remote, &remote_len);
  if (n != (int)sizeof(GenDiscoveryPkt)) return;

  GenDiscoveryPkt pkt;
  memcpy(&pkt, buf, sizeof(pkt));
  if (pkt.magic != GEN_DISCOVERY_MAGIC || pkt.proto_version != GEN_DISCOVERY_PROTO_VERSION ||
      pkt.type != GEN_DISCOVERY_TYPE_REQUEST)
    return;

  GenDiscoveryPkt reply = {GEN_DISCOVERY_MAGIC, GEN_DISCOVERY_PROTO_VERSION,
                            GEN_DISCOVERY_TYPE_REPLY, {0}};
  strlcpy(reply.name, wifiHostname, sizeof(reply.name));
  sendto(s_sock, &reply, sizeof(reply), 0, (struct sockaddr *)&remote, remote_len);
}
