// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Required Notice: Copyright (c) 2026 nanaisu - NBT-AAD (https://github.com/Sq00ky/Supra-iDrive6-Android-Auto)
//
// nbt-aad-coding: prints the head unit's coding blocks (car/coding.hpp) for the daemon, which runs it once
// (car/coding_read.hpp). Read-only. It knows nothing about what the values mean.
//
// It is a program of its own for two reasons. The settings library waits for its lock with no timeout, so a stuck lock
// must not be able to freeze the daemon. And the library expects its host to provide doFatalError, which firmware
// libraries the daemon loads already define: exported from the daemon, ours would replace theirs for the whole daemon.
//
// Values are read through the firmware's own library, the calls libhmicoding.so makes, so its locking and its reading of
// the settings page stay the firmware's job. Only SysetPageGetSh_attach, _getDataVersion and _getBuffer are ever called.
// Only attach touches anything shared: the rest read the library's private copy, so reading every block adds no risk.

#include "car/coding.hpp"

#include <dlfcn.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

// The library leaves these undefined for its host to provide, which would fail the dlopen. Nothing here reaches the two
// getMappedCb calls. doFatalError is what the library calls when its lock is broken. tools/nbt_aad_coding.syms exports them.
extern "C" {
int SysetPageGet_getMappedCbBuffer() { return 11; }
int SysetPageGet_getMappedCbValue() { return 11; }
void doFatalError() { _exit(nbt::car::kCodingExitFatal); }
}

namespace {

const char* const kLibPaths[] = {
#ifdef NBT_HOST_TEST
    getenv("NBT_CODING_LIB"),
#endif
    "/opt/sys/lib/libsys_data_systemsettings_pagegetsh.cpp-ne.so",
    "/fs/sda0/opt/sys/lib/libsys_data_systemsettings_pagegetsh.cpp-ne.so",
};
const char kPage[] = "/sysetpage";
const char kPageLock[] = "/sysetpage-lock";
const unsigned kHangLimitS = 2;   // the daemon stops waiting and kills this program a little later

typedef int (*AttachFn)(const char* page, const char* lock);
typedef int (*DataVersionFn)(uint32_t key, uint16_t* version);
typedef int (*GetBufferFn)(uint32_t key, uint16_t version, void* buf, uint32_t cap, uint32_t* len);

void onHang(int) { _exit(nbt::car::kCodingExitHang); }

int printBlocks() {
    using namespace nbt::car;
    void* lib = NULL;
    for (size_t i = 0; !lib && i < sizeof(kLibPaths) / sizeof(kLibPaths[0]); ++i)
        if (kLibPaths[i]) lib = dlopen(kLibPaths[i], RTLD_NOW);
    if (!lib) return kCodingExitNoLibrary;
    AttachFn attach = (AttachFn)dlsym(lib, "SysetPageGetSh_attach");
    DataVersionFn dataVersion = (DataVersionFn)dlsym(lib, "SysetPageGetSh_getDataVersion");
    GetBufferFn getBuffer = (GetBufferFn)dlsym(lib, "SysetPageGetSh_getBuffer");
    if (!attach || !dataVersion || !getBuffer) return kCodingExitNoLibrary;
    if (attach(kPage, kPageLock) != 0) return kCodingExitAttach;

    printf("%s\n", kCodingHeader);
    size_t printed = 0;
    for (size_t i = 0; i < kCodingBlockCount; ++i) {
        // The version is the page's own: the library only uses it to refuse a reader built for another major version.
        uint16_t version = 0;
        uint8_t block[1024];
        uint32_t size = 0;
        if (dataVersion(kCodingBlocks[i].key, &version) != 0) continue;
        if (getBuffer(kCodingBlocks[i].key, version, block, sizeof(block), &size) != 0 || size > sizeof(block)) continue;
        printf("%04x %u ", (unsigned)kCodingBlocks[i].block, (unsigned)size);
        for (uint32_t k = 0; k < size; ++k) printf("%02x", block[k]);
        printf("\n");
        ++printed;
    }
    return printed ? kCodingExitOk : kCodingExitNoBlocks;
}

}  // namespace

int main() {
    signal(SIGALRM, onHang);
    alarm(kHangLimitS);
    const int code = printBlocks();
    if (code != nbt::car::kCodingExitOk) fprintf(stderr, "nbt-aad-coding: %s\n", nbt::car::codingExitName(code));
    fflush(NULL);
    _exit(code);   // no library teardown: everything has been said
}
