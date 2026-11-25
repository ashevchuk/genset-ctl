/*
 * debug_telnet.h — telnet sink (port 23) for SDEBUG() output.
 *
 * Only started in DEBUG builds. Up to DEBUG_TELNET_MAX_CLIENTS clients get
 * every SDEBUG() line; nothing is read from them.
 */
#pragma once

#include <stddef.h>

#define DEBUG_TELNET_PORT        23
#define DEBUG_TELNET_MAX_CLIENTS 5

void debug_telnet_begin(void);
// Accepts new clients; call regularly (loop()).
void debug_telnet_poll(void);
void debug_telnet_send(const char *data, size_t len);
