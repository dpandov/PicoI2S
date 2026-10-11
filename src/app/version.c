#include "app/version.h"

#include <stdio.h>
#include "pico/binary_info.h"
#include "pico/version.h"
#include "tusb.h"

#include "picoi2s_build.h"

#if PICOI2S_BUILD_DIRTY
#define DIRTY_SUFFIX "-dirty"
#else
#define DIRTY_SUFFIX ""
#endif

#define VERSION_SHORT PICOI2S_VERSION_TEXT " build " \
    STRINGIFY(PICOI2S_BUILD_NUMBER) " " PICOI2S_BUILD_COMMIT DIRTY_SUFFIX
#define STRINGIFY_(x) #x
#define STRINGIFY(x)  STRINGIFY_(x)

// Shown by `picotool info` for the .uf2 file and the running device (the
// SDK's own build date, of its file, is turned off in CMakeLists.txt).
bi_decl(bi_program_version_string(VERSION_SHORT));
bi_decl(bi_program_build_date_string(PICOI2S_BUILD_DATE));

const char *version_short(void) {
    return VERSION_SHORT;
}

const char *version_string(void) {
    return "ver=" PICOI2S_VERSION_TEXT " build=" STRINGIFY(PICOI2S_BUILD_NUMBER)
           " git=" PICOI2S_BUILD_COMMIT " dirty=" STRINGIFY(PICOI2S_BUILD_DIRTY)
           " date=" PICOI2S_BUILD_DATE " sdk=" PICO_SDK_VERSION_STRING
           " tinyusb=" TUSB_VERSION_STRING;
}
