// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Required Notice: Copyright (c) 2026 nanaisu - NBT-AAD (https://github.com/Sq00ky/Supra-iDrive6-Android-Auto)
#ifndef GPS_PROBE_HPP
#define GPS_PROBE_HPP

// Pure helpers for test_gps (the stand-alone GPS source probe): no I/O, host-testable.
//   - findGeo: scan a payload for a (latitude, longitude) pair under the encodings a head unit is likely to use
//   - decodeFktIds: MOST FBlock.FktIDs status (packed 12-bit list or bitfield)
//   - decodeRegistry: DSYSMOSTServer registry vector (u32 count, then {u16, u16} entries)

#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <string.h>
#include <vector>

namespace nbt {
namespace gpsprobe {

enum Scale {
    kSemi32 = 0,   // signed 32-bit, 360 / 2^32 deg per unit (BMW / "semicircle" style)
    kE5,           // signed 32-bit, 1e-5 deg
    kE6,           // signed 32-bit, 1e-6 deg
    kE7,           // signed 32-bit, 1e-7 deg (Android LocationData style)
    kMas,          // signed 32-bit, milliarcseconds (1 / 3600000 deg)
    kU32Offset,    // unsigned 32-bit: lon = u * 360/2^32 - 180, lat = u * 180/2^32 - 90
    kDouble,       // IEEE double, degrees
    kDoubleRad,    // IEEE double, radians
    kFloat,        // IEEE float, degrees
    kScaleCount
};

inline const char* scaleName(int s) {
    switch (s) {
        case kSemi32: return "i32*360/2^32";
        case kE5: return "i32*1e-5";
        case kE6: return "i32*1e-6";
        case kE7: return "i32*1e-7";
        case kMas: return "i32 mas";
        case kU32Offset: return "u32 offset";
        case kDouble: return "double deg";
        case kDoubleRad: return "double rad";
        case kFloat: return "float deg";
        default: return "?";
    }
}

struct GeoCand {
    size_t off;        // byte offset of the first value
    bool be;           // big-endian
    int scale;         // Scale
    bool lonFirst;     // first value is the longitude (MOST GPSPosition order)
    double lat, lon;
    bool match;        // within tolerance of the reference position
};

inline uint32_t rd32(const uint8_t* p, bool be) {
    return be ? ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3]
              : ((uint32_t)p[3] << 24) | ((uint32_t)p[2] << 16) | ((uint32_t)p[1] << 8) | p[0];
}
inline uint64_t rd64(const uint8_t* p, bool be) {
    uint64_t hi = rd32(be ? p : p + 4, be), lo = rd32(be ? p + 4 : p, be);
    return (hi << 32) | lo;
}
inline double asDouble(uint64_t v) { double d; memcpy(&d, &v, sizeof(d)); return d; }
inline float asFloat(uint32_t v) { float f; memcpy(&f, &v, sizeof(f)); return f; }

// One raw value -> degrees. axis: 0 = latitude, 1 = longitude (only kU32Offset cares).
inline bool toDeg(int scale, uint32_t raw, int axis, double& out) {
    int32_t s = (int32_t)raw;
    switch (scale) {
        case kSemi32: out = s * (360.0 / 4294967296.0); return true;
        case kE5: out = s * 1e-5; return true;
        case kE6: out = s * 1e-6; return true;
        case kE7: out = s * 1e-7; return true;
        case kMas: out = s / 3600000.0; return true;
        case kU32Offset: out = axis ? raw * (360.0 / 4294967296.0) - 180.0 : raw * (180.0 / 4294967296.0) - 90.0; return true;
        case kFloat: { float f = asFloat(raw); if (f != f) return false; out = f; return true; }
        default: return false;
    }
}

// Plausibility without a reference: a real fix is not (0,0), not "invalid" filler, and latitude is not tiny.
inline bool plausible(double lat, double lon) {
    if (!(lat == lat) || !(lon == lon)) return false;
    if (fabs(lat) > 90.0 || fabs(lon) > 180.0) return false;
    if (fabs(lat) < 1.0) return false;            // also rejects the many 0 / small-integer fields
    if (lat == 0.0 && lon == 0.0) return false;
    return true;
}

inline bool nearRef(double lat, double lon, double refLat, double refLon, double tol) {
    double dl = fabs(lon - refLon);
    if (dl > 180.0) dl = 360.0 - dl;
    return fabs(lat - refLat) <= tol && dl <= tol;
}

inline void consider(std::vector<GeoCand>& out, size_t off, bool be, int scale, bool lonFirst, double a, double b,
                     bool haveRef, double refLat, double refLon, double tol) {
    double lat = lonFirst ? b : a, lon = lonFirst ? a : b;
    bool m = haveRef && nearRef(lat, lon, refLat, refLon, tol);
    if (haveRef ? !m : !plausible(lat, lon)) return;
    GeoCand c;
    c.off = off; c.be = be; c.scale = scale; c.lonFirst = lonFirst; c.lat = lat; c.lon = lon; c.match = m;
    out.push_back(c);
}

// Scan for two adjacent values forming a position. With a reference only matches within tol degrees are kept,
// without one every plausible pair is (noisy: use the reference whenever possible). step = offset stride for the
// 32-bit scan (1 for MOST payloads, which are packed big-endian, 4 for DSI payloads, which are aligned).
inline size_t findGeo(const uint8_t* p, size_t n, size_t step, bool haveRef, double refLat, double refLon,
                      double tol, std::vector<GeoCand>& out) {
    size_t before = out.size();
    if (!p || step == 0) return 0;
    for (size_t off = 0; off + 8 <= n; off += step) {
        for (int e = 0; e < 2; ++e) {
            bool be = e == 0;
            uint32_t r0 = rd32(p + off, be), r1 = rd32(p + off + 4, be);
            if (r0 == 0xFFFFFFFFu || r1 == 0xFFFFFFFFu || r0 == 0 || r1 == 0) continue;   // invalid / empty
            for (int s = 0; s < kDouble; ++s) {
                if (s == kFloat) continue;
                for (int lf = 0; lf < 2; ++lf) {
                    double a, b;
                    if (!toDeg(s, r0, lf ? 1 : 0, a) || !toDeg(s, r1, lf ? 0 : 1, b)) continue;
                    consider(out, off, be, s, lf != 0, a, b, haveRef, refLat, refLon, tol);
                }
            }
            for (int lf = 0; lf < 2; ++lf) {
                double a, b;
                if (toDeg(kFloat, r0, 0, a) && toDeg(kFloat, r1, 0, b))
                    consider(out, off, be, kFloat, lf != 0, a, b, haveRef, refLat, refLon, tol);
            }
        }
    }
    // doubles: aligned to 4 in DSI streams, any offset in packed payloads
    for (size_t off = 0; off + 16 <= n; off += step) {
        for (int e = 0; e < 2; ++e) {
            bool be = e == 0;
            double a = asDouble(rd64(p + off, be)), b = asDouble(rd64(p + off + 8, be));
            if (!(a == a) || !(b == b) || a == 0.0 || b == 0.0) continue;
            for (int lf = 0; lf < 2; ++lf) {
                consider(out, off, be, kDouble, lf != 0, a, b, haveRef, refLat, refLon, tol);
                if (fabs(a) <= 3.2 && fabs(b) <= 3.2)
                    consider(out, off, be, kDoubleRad, lf != 0, a * 57.29577951308232, b * 57.29577951308232,
                             haveRef, refLat, refLon, tol);
            }
        }
    }
    return out.size() - before;
}

// MOST FBlock.FktIDs (fkt 0x000) status. Two encodings exist: a packed list of 12-bit function ids (MOST 2.x,
// "00 00 01 00 02 ..." = 0x000, 0x001, 0x002) and a bitfield (bit 7 of byte 0 = fkt 0x000). Picks the packed list
// when it is strictly ascending and starts with 0x000 / 0x001, else the bitfield. isPacked reports the choice.
inline std::vector<uint16_t> decodeFktIds(const uint8_t* p, size_t n, bool* isPacked) {
    std::vector<uint16_t> packed, bits;
    for (size_t i = 0; i + 1 < n; ) {          // 3 bytes = 2 ids; a trailing 2-byte group = 1 id
        uint16_t a = (uint16_t)((p[i] << 4) | (p[i + 1] >> 4));
        packed.push_back(a);
        if (i + 2 < n) packed.push_back((uint16_t)(((p[i + 1] & 0x0F) << 8) | p[i + 2]));
        i += 3;
    }
    bool ok = !packed.empty() && packed[0] <= 1;
    for (size_t i = 1; ok && i < packed.size(); ++i) {
        if (packed[i] <= packed[i - 1]) {
            // allow trailing 0x000 padding only
            for (size_t j = i; j < packed.size(); ++j) if (packed[j] != 0) { ok = false; break; }
            if (ok) { packed.resize(i); }
            break;
        }
    }
    for (size_t i = 0; i < n; ++i)
        for (int b = 7; b >= 0; --b)
            if (p[i] & (1 << b)) bits.push_back((uint16_t)(i * 8 + (7 - b)));
    if (isPacked) *isPacked = ok;
    return ok ? packed : bits;
}

struct RegEntry { uint16_t a, b; };

// DSYSMOSTTypes registry vector as serialised in a DSI stream: u32 count, then count x {u16, u16} (little-endian).
// a = device address, b = FBlock key (fblock << 8 | inst): car log, a=2021 b=5200 is the sender of FB 52/00.
// Returns false if the length does not fit.
inline bool decodeRegistry(const uint8_t* p, size_t n, std::vector<RegEntry>& out) {
    out.clear();
    if (!p || n < 4) return false;
    uint32_t cnt = rd32(p, false);
    if (cnt == 0 || cnt > 1024 || 4 + (size_t)cnt * 4 > n) return false;
    for (uint32_t i = 0; i < cnt; ++i) {
        const uint8_t* e = p + 4 + i * 4;
        RegEntry r;
        r.a = (uint16_t)(e[0] | (e[1] << 8));
        r.b = (uint16_t)(e[2] | (e[3] << 8));
        out.push_back(r);
    }
    return true;
}

} // namespace gpsprobe
} // namespace nbt

#endif
