// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Required Notice: Copyright (c) 2026 nanaisu - NBT-AAD (https://github.com/Sq00ky/Supra-iDrive6-Android-Auto)
//
// Stands in for the firmware's settings library in test_car_coding. Like the real one, it leaves the three host symbols
// undefined, so loading it proves the helper exports them. NBT_FAKE_SYSET picks a failure. Each block's bytes are invented:
// every byte is its offset, except byte 3 of block 0x3004, which is NBT_FAKE_BYTE (a number).
#include "car/coding.hpp"

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

using namespace nbt::car;

extern "C" {
void doFatalError();
int SysetPageGet_getMappedCbBuffer();
int SysetPageGet_getMappedCbValue();

static const uint16_t kVersion = 0x1234;
static const uint32_t kKey3004 = 12;
static const uint32_t kKey3000 = 7;

static bool mode(const char* name) {
    const char* m = getenv("NBT_FAKE_SYSET");
    return m && strcmp(m, name) == 0;
}

static const CodingBlock* blockOfKey(uint32_t key) {
    for (size_t i = 0; i < kCodingBlockCount; ++i)
        if (kCodingBlocks[i].key == key) return &kCodingBlocks[i];
    return NULL;
}

int SysetPageGetSh_attach(const char* page, const char* lock) {
    if (mode("hang")) for (;;) sleep(1);
    if (mode("fatal")) doFatalError();
    if (mode("attach") || !page || !lock) return 5;
    return SysetPageGet_getMappedCbBuffer() == 11 && SysetPageGet_getMappedCbValue() == 11 ? 0 : 11;
}

int SysetPageGetSh_getDataVersion(uint32_t key, uint16_t* version) {
    if (mode("noblocks") || !blockOfKey(key)) return 1;
    if (mode("no3000") && key == kKey3000) return 1;
    *version = kVersion;
    return 0;
}

int SysetPageGetSh_getBuffer(uint32_t key, uint16_t version, void* buf, uint32_t cap, uint32_t* len) {
    const CodingBlock* block = blockOfKey(key);
    if (!block || version != kVersion) return 4;
    if (mode("no3004") && key == kKey3004) return 1;
    const uint32_t size = mode("short") && key == kKey3004 ? 4 : block->size;
    if (cap < size) return 9;
    uint8_t* bytes = static_cast<uint8_t*>(buf);
    for (uint32_t i = 0; i < size; ++i) bytes[i] = (uint8_t)i;
    const char* value = getenv("NBT_FAKE_BYTE");
    if (key == kKey3004 && size > 3) bytes[3] = (uint8_t)(value ? atoi(value) : 0);
    *len = size;
    return 0;
}
}
