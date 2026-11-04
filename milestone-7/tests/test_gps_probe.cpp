// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Required Notice: Copyright (c) 2026 nanaisu - NBT-AAD (https://github.com/Sq00ky/Supra-iDrive6-Android-Auto)
// Host test for src/gps_probe.hpp (test_gps position finder, FktIDs and registry decoders).
#include "gps_probe.hpp"
#include <stdio.h>
#include <assert.h>
#include <math.h>
using namespace nbt::gpsprobe;

static void be32(uint8_t* p, uint32_t v) { p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = v; }
static void le32(uint8_t* p, uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }

static bool has(const std::vector<GeoCand>& c, int scale, bool be, bool lonFirst, size_t off, double lat, double lon) {
    for (size_t i = 0; i < c.size(); ++i)
        if (c[i].scale == scale && c[i].be == be && c[i].lonFirst == lonFirst && c[i].off == off &&
            fabs(c[i].lat - lat) < 1e-5 && fabs(c[i].lon - lon) < 1e-5 && c[i].match) return true;
    return false;
}

int main() {
    const double lat = 51.501364, lon = -0.141890;   // reference point
    // 1. MOST GPSPosition style: BE, longitude first, 360/2^32 scaling, behind a 3-byte prefix
    {
        uint8_t p[11] = { 0xAA, 0xBB, 0xCC };
        be32(p + 3, (uint32_t)(int32_t)lround(lon / (360.0 / 4294967296.0)));
        be32(p + 7, (uint32_t)(int32_t)lround(lat / (360.0 / 4294967296.0)));
        std::vector<GeoCand> c;
        findGeo(p, sizeof(p), 1, true, lat, lon, 0.5, c);
        assert(has(c, kSemi32, true, true, 3, lat, lon));
    }
    // 2. DSI style: LE, lat first, 1e-7
    {
        uint8_t p[16] = { 0 };
        le32(p + 4, (uint32_t)(int32_t)lround(lat * 1e7));
        le32(p + 8, (uint32_t)(int32_t)lround(lon * 1e7));
        std::vector<GeoCand> c;
        findGeo(p, sizeof(p), 4, true, lat, lon, 0.5, c);
        assert(has(c, kE7, false, false, 4, lat, lon));
    }
    // 3. LE doubles, lat first
    {
        uint8_t p[24] = { 0 };
        double a = lat, b = lon;
        memcpy(p + 4, &a, 8);
        memcpy(p + 12, &b, 8);
        std::vector<GeoCand> c;
        findGeo(p, sizeof(p), 4, true, lat, lon, 0.5, c);
        assert(has(c, kDouble, false, false, 4, lat, lon));
    }
    // 4. invalid filler (0xFFFFFFFF, as NavigationInfo sends without a fix) and zeros are ignored
    {
        uint8_t p[8];
        memset(p, 0xFF, sizeof(p));
        std::vector<GeoCand> c;
        assert(findGeo(p, sizeof(p), 1, false, 0, 0, 1, c) == 0);
        memset(p, 0, sizeof(p));
        assert(findGeo(p, sizeof(p), 1, false, 0, 0, 1, c) == 0);
    }
    // 5. a far-away reference rejects the pair
    {
        uint8_t p[8];
        be32(p, (uint32_t)(int32_t)lround(lon / (360.0 / 4294967296.0)));
        be32(p + 4, (uint32_t)(int32_t)lround(lat / (360.0 / 4294967296.0)));
        std::vector<GeoCand> c;
        findGeo(p, sizeof(p), 1, true, 35.68, 139.76, 1.0, c);
        assert(c.empty());
    }
    // 6. FktIDs: packed 12-bit list 000 001 002 C01 C02 (+ zero padding) and a bitfield
    {
        const uint8_t packed[] = { 0x00, 0x00, 0x01, 0x00, 0x2C, 0x01, 0xC0, 0x20, 0x00 };
        bool isPacked = false;
        std::vector<uint16_t> f = decodeFktIds(packed, sizeof(packed), &isPacked);
        assert(isPacked && f.size() == 5 && f[0] == 0x000 && f[1] == 0x001 && f[2] == 0x002 && f[3] == 0xC01 && f[4] == 0xC02);
        const uint8_t bits[] = { 0xE0, 0x00, 0x01 };    // fkt 000, 001, 002, 017
        f = decodeFktIds(bits, sizeof(bits), &isPacked);
        assert(!isPacked && f.size() == 4 && f[3] == 0x017);
    }
    // 7. registry vector
    {
        const uint8_t r[] = { 2, 0, 0, 0, 0x01, 0x53, 0x01, 0x01, 0x01, 0xE5, 0x40, 0x01 };
        std::vector<RegEntry> e;
        assert(decodeRegistry(r, sizeof(r), e) && e.size() == 2 && e[0].a == 0x5301 && e[0].b == 0x0101 && e[1].a == 0xE501);
        assert(!decodeRegistry(r, 8, e));
    }
    printf("test_gps_probe OK\n");
    return 0;
}
