#include "debug_telnet.h"
#include <lwip/sockets.h>
#include <string.h>

static int s_listen = -1;
static int s_clients[DEBUG_TELNET_MAX_CLIENTS] = {-1, -1, -1, -1, -1};

void debug_telnet_begin(void) {
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) return;
    int yes = 1;
    setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    struct sockaddr_in addr = {};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(DEBUG_TELNET_PORT);
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0 || listen(sock, 1) < 0) {
        close(sock);
        return;
    }
    fcntl(sock, F_SETFL, O_NONBLOCK);
    s_listen = sock;
}

void debug_telnet_poll(void) {
    if (s_listen < 0) return;
    int fd = accept(s_listen, nullptr, nullptr);
    if (fd < 0) return;
    int nodelay = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));
    fcntl(fd, F_SETFL, O_NONBLOCK);
    for (int &c : s_clients) {
        if (c < 0) {
            c = fd;
            return;
        }
    }
    close(fd);  // all slots in use
}

void debug_telnet_send(const char *data, size_t len) {
    for (int &c : s_clients) {
        if (c < 0) continue;
        if (send(c, data, len, MSG_DONTWAIT) < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            close(c);
            c = -1;
        }
    }
}
