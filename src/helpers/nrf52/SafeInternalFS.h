#pragma once

#if defined(NRF52_PLATFORM)

#include <InternalFileSystem.h>

// InternalFileSystem::begin() ERASES the whole filesystem region and reformats
// it whenever a mount attempt fails — identity key, prefs, contacts, all of it.
// That is only the right move on a factory-fresh chip. On a node whose flash
// already holds a filesystem, a transient mount failure (memory pressure, a
// crash mid-write) must NOT become a silent factory reset — a T1000-E lost its
// whole configuration exactly this way (2026-08-21).
//
// This helper only allows the format-on-fail path when the region does not
// look like a littlefs filesystem. If it does look like one but fails to
// mount, we return false with the data left intact, and the caller halts
// loudly instead — a dead-but-recoverable node beats a wiped one.
//
// LFS_FLASH_ADDR is fixed at 0xED000 in the core (see InternalFileSystem.cpp);
// the littlefs2 superblock carries the ASCII magic "littlefs" in its first
// blocks (block size 128 here).
static bool safeInternalFSBegin() {
  const uint8_t* base = (const uint8_t*)0xED000;
  bool looks_lfs = false;
  for (int off = 0; off <= 512 - 8 && !looks_lfs; off++) {
    if (memcmp(base + off, "littlefs", 8) == 0) looks_lfs = true;
  }
  if (!looks_lfs) {
    // Fresh or fully-erased region: the normal path (which may format) is correct
    return InternalFS.begin();
  }
  // Region holds a filesystem: mount only, NEVER auto-format
  return InternalFS.Adafruit_LittleFS::begin();
}

// Halt pattern for a failed mount of an existing filesystem: keep the node
// alive enough to report the fault over serial, never touch the flash.
static void haltFSMountFailed() {
  while (1) {
    Serial.println("FATAL: internal filesystem mount failed - NOT formatting (data preserved). Power-cycle to retry.");
    delay(5000);
  }
}

#endif // NRF52_PLATFORM
