// Firmware version and build identity: the version from CMakeLists.txt,
// the build number (commits in HEAD), the commit and the build time from
// cmake/version.cmake.
#pragma once

// Hidden USB string index of the version string, read by tools/monitor.py;
// format in tools/diag.py.
#define STRID_VERSION 0x25

// "1.04 build 57 ca5ef88" (+ "-dirty" when built with uncommitted changes),
// for the console banner.
const char *version_short(void);

// The version string: "ver=1.04 build=57 git=ca5ef88 dirty=0
// date=2026-10-11T08:12Z sdk=2.3.1 tinyusb=0.21.0".
const char *version_string(void);
