/*
 * ffat.h — the FAT partition ("ffat" in partitions.csv) on wear levelling.
 *
 * Mounted at FFAT_MOUNT_POINT with the parameters Arduino's FFat.begin(true)
 * used, so the on-flash filesystem is unchanged; files are then plain
 * fopen()/opendir() under that path.
 */
#pragma once

#include <stdint.h>

#define FFAT_MOUNT_POINT "/ffat"

// Formats the partition if it cannot be mounted (first boot).
bool ffat_mount(void);

bool ffat_info(uint64_t *total_bytes, uint64_t *free_bytes);
