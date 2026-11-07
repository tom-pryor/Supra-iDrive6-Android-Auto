// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Required Notice: Copyright (c) 2026 nanaisu - NBT-AAD (https://github.com/Sq00ky/Supra-iDrive6-Android-Auto)
#ifndef CAR_CODING_HPP
#define CAR_CODING_HPP

// The head unit's coding values, as nbt-aad-coding (tools/nbt_aad_coding.cpp) reads them and the daemon looks them up
// (car/coding_read.hpp). The helper only knows blocks, and what a value means is only known here, so a new value needs
// one more constant below and no change to the helper.
//
// A parameter's address comes from the table in libhmicoding.so: its records hold the block, the byte, the mask and the
// name of each value. A value is counted from the lowest bit of its mask.

#include <map>
#include <stddef.h>
#include <stdint.h>
#include <vector>

namespace nbt {
namespace car {

struct CodingParam {
    const char* name;
    uint16_t block;
    uint8_t byte;
    uint8_t mask;
};

// 0 = linkslenker, 1 = rechtslenker. The firmware's own CarPlay setup reads the same bit (ConnStarter).
const CodingParam kTypLenkung = { "TYP_LENKUNG", 0x3004, 3, 0x02 };

// The values the daemon logs once it has read them.
const CodingParam* const kLoggedCodingParams[] = { &kTypLenkung };

// Coding block -> settings key and size, from CDsiBinaryStream::readCodingBlock and getCodingBlockSize in libhmicoding.so.
struct CodingBlock {
    uint16_t block;
    uint32_t key;
    uint32_t size;
};
const CodingBlock kCodingBlocks[] = {
    {0x3000,  7, 150}, {0x3001,  5, 150}, {0x3002,  1, 150}, {0x3003, 13, 149}, {0x3004, 12,  50}, {0x3005,  3, 150},
    {0x3006,  9, 100}, {0x3007,  8,  50}, {0x3008,  6,  80}, {0x3009,  4,  76}, {0x300a, 10,  50}, {0x300b, 15,  30},
    {0x300c, 16,  80}, {0x300d, 17,  79}, {0x300e, 14,   5}, {0x300f,  2,   8}, {0x3010, 11, 132},
};
const size_t kCodingBlockCount = sizeof(kCodingBlocks) / sizeof(kCodingBlocks[0]);

inline const CodingBlock* findCodingBlock(unsigned long block) {
    for (size_t i = 0; i < kCodingBlockCount; ++i)
        if (kCodingBlocks[i].block == block) return &kCodingBlocks[i];
    return NULL;
}

// The helper's output, plain text so it can also be read by hand. The header line, then one line per block it could read:
//   nbt-aad-coding 1
//   3004 50 0000...      block (hex), size (decimal), the bytes (hex)
const char kCodingHeader[] = "nbt-aad-coding 1";

inline int codingHexDigit(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

// The blocks in the helper's output. A line that isn't complete and well formed is left out, and so is a block of another
// size than the table says, since its layout can't be trusted: the values in it read as unknown.
class CodingSnapshot {
public:
    // False if this isn't the helper's output or holds no usable block.
    bool parse(const char* text, size_t size) {
        blocks_.clear();
        const size_t headerSize = sizeof(kCodingHeader) - 1;
        if (size <= headerSize) return false;
        for (size_t i = 0; i < headerSize; ++i)
            if (text[i] != kCodingHeader[i]) return false;
        if (text[headerSize] != '\n') return false;
        size_t pos = headerSize + 1;
        while (pos < size) {
            size_t end = pos;
            while (end < size && text[end] != '\n') ++end;
            if (end == size) break;   // cut off: no newline
            parseLine(text + pos, end - pos);
            pos = end + 1;
        }
        return !blocks_.empty();
    }

    size_t blockCount() const { return blocks_.size(); }

    // The value, or -1 if its block or byte isn't there.
    int value(const CodingParam& param) const {
        const std::map<uint16_t, std::vector<uint8_t> >::const_iterator it = blocks_.find(param.block);
        if (it == blocks_.end() || param.byte >= it->second.size() || param.mask == 0) return -1;
        unsigned mask = param.mask, shift = 0;
        while (!(mask & 1)) { mask >>= 1; ++shift; }
        return (it->second[param.byte] & param.mask) >> shift;
    }

private:
    void parseLine(const char* line, size_t size) {
        enum { kBlockDigits = 4 };
        if (size < kBlockDigits + 1 || line[kBlockDigits] != ' ') return;
        unsigned long block = 0;
        for (size_t i = 0; i < kBlockDigits; ++i) {
            const int digit = codingHexDigit(line[i]);
            if (digit < 0) return;
            block = block * 16 + (unsigned long)digit;
        }
        size_t pos = kBlockDigits + 1;
        unsigned long byteCount = 0;
        const size_t countStart = pos;
        while (pos < size && line[pos] >= '0' && line[pos] <= '9' && pos - countStart < 6)
            byteCount = byteCount * 10 + (unsigned long)(line[pos++] - '0');
        if (pos == countStart || pos >= size || line[pos] != ' ') return;
        ++pos;
        const CodingBlock* known = findCodingBlock(block);
        if (!known || byteCount != known->size || size - pos != byteCount * 2) return;
        std::vector<uint8_t> bytes(byteCount);
        for (size_t i = 0; i < byteCount; ++i) {
            const int high = codingHexDigit(line[pos + i * 2]), low = codingHexDigit(line[pos + i * 2 + 1]);
            if (high < 0 || low < 0) return;
            bytes[i] = (uint8_t)(high * 16 + low);
        }
        blocks_[(uint16_t)block] = bytes;
    }

    std::map<uint16_t, std::vector<uint8_t> > blocks_;
};

// How the helper ended.
enum CodingHelperExit {
    kCodingExitOk = 0,
    kCodingExitNoLibrary = 12,
    kCodingExitAttach = 13,
    kCodingExitNoBlocks = 14,
    kCodingExitHang = 16,
    kCodingExitFatal = 17
};

inline const char* codingExitName(int code) {
    switch (code) {
        case kCodingExitNoLibrary: return "the settings library did not load";
        case kCodingExitAttach: return "could not attach to the settings page";
        case kCodingExitNoBlocks: return "could not read any coding block";
        case kCodingExitHang: return "gave up waiting for the settings library";
        case kCodingExitFatal: return "the settings library reported a fatal error";
        default: return "unexpected exit code";
    }
}

} // namespace car
} // namespace nbt

#endif
