// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Required Notice: Copyright (c) 2026 nanaisu - NBT-AAD (https://github.com/Sq00ky/Supra-iDrive6-Android-Auto)
// test_gps: stand-alone probe that finds where the head unit's GPS position can be read from, so Android Auto's
// LOCATION sensor (sensor 1) can be fed from the car. Like test_controls it is a separate binary and touches
// nothing in Android Auto. Run it with the car outside (GPS fix) and the engine / head unit fully booted:
//
//   ./test_gps -L 51.501,-0.142 > /mnt/umass01200t7/gps.log 2>&1
//     -L LAT,LON  rough position of the car (from your phone): candidates within -T degrees are flagged MATCH.
//                 Strongly recommended; without it every plausible-looking number pair is reported.
//     -T DEG      match tolerance in degrees (default 1.0)
//     -t N        run for N seconds (default 150)
//     -p          passive: no MOST Get requests (registry + whatever arrives on its own only)
//     -q          quick active mode: FktIDs Get only, no per-function Get sweep
//     -F LIST     MOST FBlock keys to probe (hex FBlock<<8|Inst, e.g. 5301,A101); default: every inst-01 FBlock
//                 of the head unit's own logical devices (address 0x2xxx) from the registry
//     -V LIST     DSI interface versions to try (major numbers, e.g. 1,2,3; default 1-6, minor 0)
//     -X TX,VAR,TAG  MOST send fields: transmission type, TXControl value, header tag (default: learned tx, 0, 0)
//     -s / -m     skip the DSI part / skip the MOST part
//     -r          raw: print every DSI event and MOST message in full
//
// What it does (everything is logged; the only writes are MOST "Get" requests, which are read-only):
//   1. system: running nav / connectivity processes (pidin), /dev/nav entries (stat only: never opened, they belong
//      to NavigationPositioning), QNET nodes, and a scan of /pps for objects that look like position data.
//   2. DSI: NavigationPositioning serves "Pos.DNAVIPos" / "PosReal.DNAVIPos" (positions) and "Sensors.DNAVISensor".
//      Their version is unknown, so one child process per (role, interface, version) tries to attach; an attached
//      child subscribes to every attribute / information id 0xC0000000+n / 0x80000000+n and logs the events.
//   3. MOST: NavNBTEvoController serves the MOST FBlock "NavigationInfo" (inst 0x01) with GPSPosition, GPSPositionExtension
//      and PositionHeadingSpeed; its FBlock and function ids are unknown. The probe registers with
//      MOSTSubSystem.DSYSMOSTServer, logs the central registry, sends "Get FktIDs" to the candidate FBlocks and then
//      "Get" for each function they list, and scans every reply for a position.
//   4. Every payload goes through the position finder (several scalings, both byte orders); the summary at the end
//      lists the candidates by source.
// CDS (navigation.* properties in the iDrive app framework) only exists inside the HMI process: it cannot be probed
// from here.

#include "dsi_client.hpp"
#include "most_msg.hpp"
#include "gps_probe.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <errno.h>
#include <fcntl.h>
#include <dirent.h>
#include <pthread.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <sys/utsname.h>
#include <string>
#include <vector>
#include <map>
#include <set>
#include <algorithm>

using namespace nbt;
using namespace nbt::gpsprobe;

namespace {

// ---- options / shared state ---------------------------------------------------------------------------------------
long g_t0 = 0;
int g_secs = 150;
bool g_raw = false, g_passive = false, g_quick = false, g_skipDsi = false, g_skipMost = false;
bool g_haveRef = false;
double g_refLat = 0, g_refLon = 0, g_tol = 1.0;
std::vector<int> g_versions;
std::vector<uint16_t> g_forceKeys;
bool g_haveTx = false;
uint32_t g_txType = 0, g_txCtl = 0, g_txTag = 0;
const char* g_tag = "main";            // log prefix: "main" or the DSI child's service name

pthread_mutex_t g_lock = PTHREAD_MUTEX_INITIALIZER;       // position candidates, DSI child counters
pthread_mutex_t g_mostLock = PTHREAD_MUTEX_INITIALIZER;   // MOST state (callback thread vs final summary)

long nowMs() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long)(ts.tv_sec * 1000L + ts.tv_nsec / 1000000L);
}
long rel() { return nowMs() - g_t0; }

void hexLine(const char* pre, const uint8_t* p, size_t n, size_t max) {
    std::string s(pre);
    char b[8];
    for (size_t i = 0; p && i < n && i < max; ++i) { snprintf(b, sizeof(b), " %02x", p[i]); s += b; }
    if (n > max) { snprintf(b, sizeof(b), " ..."); s += b; }
    printf("%s (%u B)\n", s.c_str(), (unsigned)n);
}

// ---- position candidates, aggregated by source ---------------------------------------------------------------------
struct Agg { unsigned hits, matches; double lat, lon; long firstMs, lastMs; };
std::map<std::string, Agg> g_geo;

void reportGeo(const std::string& src, const uint8_t* p, size_t n, size_t step, bool print) {
    std::vector<GeoCand> c;
    findGeo(p, n, step, g_haveRef, g_refLat, g_refLon, g_tol, c);
    if (c.empty()) return;
    pthread_mutex_lock(&g_lock);
    for (size_t i = 0; i < c.size(); ++i) {
        char k[256];
        snprintf(k, sizeof(k), "%s off=%u %s %s %s", src.c_str(), (unsigned)c[i].off, c[i].be ? "BE" : "LE",
                 scaleName(c[i].scale), c[i].lonFirst ? "lon,lat" : "lat,lon");
        Agg& a = g_geo[k];
        if (a.hits == 0) a.firstMs = rel();
        a.hits++;
        if (c[i].match) a.matches++;
        a.lat = c[i].lat; a.lon = c[i].lon; a.lastMs = rel();
        if (print && (c[i].match || a.hits <= 2))
            printf("[%6ld] %s   %s position? %s lat=%.6f lon=%.6f\n", rel(), g_tag, c[i].match ? "MATCH" : "     ",
                   k, c[i].lat, c[i].lon);
    }
    pthread_mutex_unlock(&g_lock);
}

void printGeoSummary() {
    pthread_mutex_lock(&g_lock);
    printf("\n==== %s: position candidates (%u sources) ====\n", g_tag, (unsigned)g_geo.size());
    if (g_geo.empty()) printf("   none%s\n", g_haveRef ? " near the -L position" : "");
    // matches first, then by hit count
    std::vector<std::pair<long, std::string> > order;
    for (std::map<std::string, Agg>::iterator it = g_geo.begin(); it != g_geo.end(); ++it)
        order.push_back(std::make_pair(-(long)(it->second.matches * 100000 + it->second.hits), it->first));
    std::sort(order.begin(), order.end());
    for (size_t i = 0; i < order.size() && i < 60; ++i) {
        const Agg& a = g_geo[order[i].second];
        printf("   %s%-90s hits=%u lat=%.6f lon=%.6f  (%lds..%lds)\n", a.matches ? "MATCH " : "      ",
               order[i].second.c_str(), a.hits, a.lat, a.lon, a.firstMs / 1000, a.lastMs / 1000);
    }
    if (order.size() > 60) printf("   ... %u more\n", (unsigned)(order.size() - 60));
    pthread_mutex_unlock(&g_lock);
}

// ---- 1. system facts -----------------------------------------------------------------------------------------------
bool lowerContains(const std::string& hay, const char* const* words) {
    std::string h(hay);
    for (size_t i = 0; i < h.size(); ++i) if (h[i] >= 'A' && h[i] <= 'Z') h[i] = (char)(h[i] - 'A' + 'a');
    for (int i = 0; words[i]; ++i) if (h.find(words[i]) != std::string::npos) return true;
    return false;
}

const char* const kGeoWords[] = { "gps", "gnss", "position", "latitude", "longitude", "\"lat", "\"lon", "lat:",
                                  "lon:", "geo", "heading", "altitude", "location", "navi", "nmea", "ublox", NULL };

void runCmd(const char* cmd, const char* const* filter, unsigned maxLines) {
    printf("$ %s%s\n", cmd, filter ? "   (filtered)" : "");
    FILE* f = popen(cmd, "r");
    if (!f) { printf("   popen failed: %s\n", strerror(errno)); return; }
    char line[512];
    unsigned n = 0, shown = 0;
    while (fgets(line, sizeof(line), f)) {
        ++n;
        if (filter && !lowerContains(line, filter)) continue;
        if (shown++ >= maxLines) continue;
        printf("   %s", line);
        size_t l = strlen(line);
        if (l && line[l - 1] != '\n') printf("\n");
    }
    int rc = pclose(f);
    printf("   (%u lines, %u shown, exit %d)\n", n, shown > maxLines ? maxLines : shown, rc);
}

void listDir(const char* path) {
    DIR* d = opendir(path);
    if (!d) { printf("   %s: %s\n", path, strerror(errno)); return; }
    printf("   %s:", path);
    struct dirent* e;
    unsigned n = 0;
    while ((e = readdir(d)) != NULL) {
        if (!strcmp(e->d_name, ".") || !strcmp(e->d_name, "..")) continue;
        printf(" %s", e->d_name);
        if (++n >= 80) { printf(" ..."); break; }
    }
    closedir(d);
    printf("\n");
}

void statPath(const char* p) {
    struct stat st;
    if (stat(p, &st) != 0) { printf("   %-24s missing (%s)\n", p, strerror(errno)); return; }
    printf("   %-24s present, mode %o, %s\n", p, (unsigned)st.st_mode,
           S_ISCHR(st.st_mode) ? "character device (not opened)" : S_ISDIR(st.st_mode) ? "directory" : "other");
}

unsigned g_ppsObjects = 0, g_ppsShown = 0;

void scanPps(const std::string& dir, int depth) {
    if (depth > 7 || g_ppsObjects > 4000) return;
    DIR* d = opendir(dir.c_str());
    if (!d) return;
    std::vector<std::string> names;
    struct dirent* e;
    while ((e = readdir(d)) != NULL) {
        if (!strcmp(e->d_name, ".") || !strcmp(e->d_name, "..") || e->d_name[0] == '.') continue;
        names.push_back(e->d_name);
    }
    closedir(d);
    for (size_t i = 0; i < names.size(); ++i) {
        std::string p = dir + "/" + names[i];
        struct stat st;
        if (stat(p.c_str(), &st) != 0) continue;
        if (S_ISDIR(st.st_mode)) { scanPps(p, depth + 1); continue; }
        ++g_ppsObjects;
        // A plain read of a PPS object returns its current attributes and changes nothing (no O_CREAT, no ?delta).
        int fd = open(p.c_str(), O_RDONLY | O_NONBLOCK);
        if (fd < 0) continue;
        char buf[4096];
        ssize_t r = read(fd, buf, sizeof(buf) - 1);
        close(fd);
        if (r < 0) r = 0;
        buf[r] = 0;
        bool nameHit = lowerContains(p, kGeoWords);
        bool bodyHit = lowerContains(std::string(buf, (size_t)r), kGeoWords);
        if (!nameHit && !bodyHit) continue;
        if (g_ppsShown++ > 60) continue;
        printf("   PPS %s (%d B)%s\n", p.c_str(), (int)r, nameHit ? "  [name]" : "");
        // print the matching lines (all lines for a name hit, capped)
        unsigned lines = 0;
        for (char* ln = strtok(buf, "\n"); ln && lines < 40; ln = strtok(NULL, "\n")) {
            if (!nameHit && !lowerContains(ln, kGeoWords)) continue;
            printf("      %.200s\n", ln);
            ++lines;
        }
    }
}

void systemFacts() {
    printf("==== 1. system ====\n");
    struct utsname u;
    if (uname(&u) == 0) printf("   uname: %s %s %s %s %s\n", u.sysname, u.nodename, u.release, u.version, u.machine);
    static const char* const procWords[] = { "nav", "csi_nbt", "online", "connectivity", "most", "nbt-aad",
                                             "hmimain", "pos", "gps", "someip", NULL };
    runCmd("pidin arg 2>&1", procWords, 60);
    printf("   nav devices (stat only):\n");
    statPath("/dev/nav");
    statPath("/dev/nav/gps");
    statPath("/dev/nav/sensors");
    statPath("/dev/nav/pps");
    listDir("/dev/nav");
    listDir("/net");
    printf("   scanning /pps for position-like objects ...\n");
    scanPps("/pps", 0);
    printf("   /pps: %u objects read, %u matched the keyword list%s\n", g_ppsObjects, g_ppsShown,
           g_ppsShown > 61 ? " (first 61 shown)" : "");
    fflush(stdout);
}

// ---- 2. DSI child: one (role, interface, version) ------------------------------------------------------------------
struct DsiTry { const char* iface; const char* role; int major; };
std::map<uint32_t, unsigned> g_dsiCount;
std::map<uint32_t, std::pair<unsigned, unsigned> > g_dsiLen;   // min, max
unsigned g_dsiEvents = 0;
unsigned g_subCount = 64;
char g_childTag[96];

void dsiAttached(DsiClient* c, void*) {
    printf("[%6ld] %s ATTACHED: subscribing to 0xC0000000..+%u and 0x80000000..+%u\n", rel(), g_tag,
           g_subCount - 1, g_subCount - 1);
    for (uint32_t i = 0; i < g_subCount; ++i) c->setNotification(0xC0000000u + i);
    for (uint32_t i = 0; i < g_subCount; ++i) c->setNotification(0x80000000u + i);
    fflush(stdout);
}

void dsiData(DsiClient*, uint32_t type, uint32_t id, uint32_t seq, const uint8_t* pl, uint32_t len, void*) {
    unsigned n;
    pthread_mutex_lock(&g_lock);
    n = ++g_dsiCount[id];
    ++g_dsiEvents;
    std::pair<unsigned, unsigned>& mm = g_dsiLen[id];
    if (n == 1) mm = std::make_pair(len, len);
    if (len < mm.first) mm.first = len;
    if (len > mm.second) mm.second = len;
    pthread_mutex_unlock(&g_lock);
    bool print = g_raw || n <= 25 || n % 20 == 0;
    if (print) {
        char pre[160];
        snprintf(pre, sizeof(pre), "[%6ld] %s type=0x%x id=0x%08x seq=%u #%u:", rel(), g_tag, type, id, seq, n);
        hexLine(pre, pl, len, g_raw ? 1024 : 256);
    }
    char src[128];
    snprintf(src, sizeof(src), "DSI %s id=%08x", g_tag, id);
    reportGeo(src, pl, len, 4, print);
}

int runDsiChild(const DsiTry& t, long deadlineMs) {
    snprintf(g_childTag, sizeof(g_childTag), "%s.%s %d.0", t.role, t.iface, t.major);
    g_tag = g_childTag;
    DsiClient cli;
    cli.setQuiet(true);
    cli.setCallbacks(&dsiAttached, &dsiData, NULL);
    if (!cli.open(t.iface, t.role, t.major, 0)) {
        printf("[%6ld] %s open FAILED\n", rel(), g_tag);
        return 2;
    }
    for (int i = 0; i < 80 && !cli.attached(); ++i) usleep(100000);   // 8 s
    if (!cli.attached()) {
        printf("[%6ld] %s no attach within 8 s\n", rel(), g_tag);
        fflush(stdout);
        return 1;
    }
    while (nowMs() < deadlineMs) usleep(200000);
    pthread_mutex_lock(&g_lock);
    printf("\n==== DSI %s: %u events ====\n", g_tag, g_dsiEvents);
    for (std::map<uint32_t, unsigned>::iterator it = g_dsiCount.begin(); it != g_dsiCount.end(); ++it)
        printf("   id 0x%08x: %u events, %u..%u B\n", it->first, it->second, g_dsiLen[it->first].first,
               g_dsiLen[it->first].second);
    pthread_mutex_unlock(&g_lock);
    printGeoSummary();
    fflush(stdout);
    return 0;
}

// ---- 3. MOST -------------------------------------------------------------------------------------------------------
DsiClient g_most;
uint32_t g_client = 0xffffffffu;
uint16_t g_ownDev = 0;
int g_stage = 0;                        // 0 init, 1 name sent, 2 registered, 3 devctrls + FktIDs sent
bool g_haveRegistry = false;
std::vector<RegEntry> g_registry;
std::vector<uint16_t> g_targets;        // FBlock keys we probe
std::map<uint16_t, uint16_t> g_targetAddr;   // key -> device address to send to
std::set<uint16_t> g_swept;
std::map<uint32_t, unsigned> g_mostSeen;     // fblock<<24 | inst<<16 | fkt<<4 | op -> count
std::map<uint16_t, std::vector<uint16_t> > g_fktLists;
unsigned g_mostMsgs = 0, g_sent = 0, g_replies = 0, g_errors = 0;
std::map<uint32_t, unsigned> g_otherIds;

const char* opName(unsigned op) {
    switch (op) {
        case 0x0: return "Set"; case 0x1: return "Get"; case 0x2: return "SetGet"; case 0x3: return "Inc";
        case 0x4: return "Dec"; case 0x5: return "GetIface"; case 0x6: return "StartRes"; case 0x7: return "Abort";
        case 0x8: return "StartResAck"; case 0x9: return "AbortAck"; case 0xA: return "StartAck";
        case 0xB: return "ErrorAck"; case 0xC: return "Status"; case 0xD: return "Interface";
        case 0xE: return "ProcAck"; case 0xF: return "Error"; default: return "?";
    }
}

bool isTarget(uint16_t key) {
    for (size_t i = 0; i < g_targets.size(); ++i) if (g_targets[i] == key) return true;
    return false;
}

void sendGet(DsiClient* c, uint16_t key, uint16_t fkt) {
    MostMsg m;
    m.txType = g_txType;
    m.varTag = 1;                       // eTransmissionTXControl (CHBFunctionBase::sendTransmission)
    m.varVal = g_txCtl;
    m.dst = g_targetAddr.count(key) ? g_targetAddr[key] : g_ownDev;
    m.src = g_ownDev;
    m.fblock = (uint8_t)(key >> 8);
    m.inst = (uint8_t)(key & 0xff);
    m.fkt = fkt;
    m.opType = 0x1;                     // Get: read-only
    m.hdrTag = g_txTag;
    std::vector<uint8_t> p;
    putMostU32(p, g_client);
    std::vector<uint8_t> msg = buildMostMessage(m);
    p.insert(p.end(), msg.begin(), msg.end());
    c->sendRequest(2, p);               // processMessage(u32 client, MessageType)
    ++g_sent;
}

// Pick the FBlocks to probe once the registry (or -F) is known, register as their device controller (copies of
// their status, as test_controls does for the iDrive controller) and ask each for its function list.
void startProbe(DsiClient* c) {
    if (g_stage >= 3) return;
    g_stage = 3;
    if (!g_forceKeys.empty()) {
        g_targets = g_forceKeys;
    } else if (g_haveRegistry) {
        // Registry entry = {a: device address, b: FBlock<<8|Inst} (car log: a=2021 b=5200 is the device that sends
        // FB 52/00, a=101B b=4001 sends FB 40/01). Probe the head unit's own logical devices (0x2xxx) and inst 01.
        for (size_t i = 0; i < g_registry.size(); ++i) {
            uint16_t addr = g_registry[i].a, key = g_registry[i].b;
            if ((addr & 0xF000) != 0x2000 || (key & 0xff) != 0x01) continue;
            if (!isTarget(key) && g_targets.size() < 64) { g_targets.push_back(key); g_targetAddr[key] = addr; }
        }
    }
    printf("[%6ld] main  probe targets (%u):", rel(), (unsigned)g_targets.size());
    for (size_t i = 0; i < g_targets.size(); ++i)
        printf(" %04X@%04X", g_targets[i], g_targetAddr.count(g_targets[i]) ? g_targetAddr[g_targets[i]] : g_ownDev);
    printf("\n");
    std::vector<uint8_t> p = buildRegisterDevCtrls(g_client, g_targets);
    c->sendRequest(1, p);
    if (g_passive) { printf("[%6ld] main  passive (-p): no Get requests\n", rel()); return; }
    printf("[%6ld] main  sending Get FktIDs (fkt 000) to each target: tx=%u ctl=%u tag=%u\n", rel(), g_txType,
           g_txCtl, g_txTag);
    for (size_t i = 0; i < g_targets.size(); ++i) { sendGet(c, g_targets[i], 0x000); usleep(3000); }
    fflush(stdout);
}

void sweep(DsiClient* c, uint16_t key, const std::vector<uint16_t>& fkts) {
    if (g_quick || g_passive || g_swept.count(key)) return;
    g_swept.insert(key);
    unsigned n = 0;
    for (size_t i = 0; i < fkts.size() && n < 96; ++i) {
        uint16_t f = fkts[i];
        if (f <= 0x002) continue;       // FktIDs, Notification, NotificationCheck
        sendGet(c, key, f);
        ++n;
        usleep(3000);
    }
    printf("[%6ld] main  FB %04X: sent Get for %u functions\n", rel(), key, n);
}

void onMost(DsiClient* c, const uint8_t* pl, uint32_t len) {
    MostMsg m;
    if (!parseMostMessage(pl, len, m)) { hexLine("main  MOST (unparsed):", pl, len, 96); return; }
    ++g_mostMsgs;
    if (!g_haveTx) { g_haveTx = true; g_txType = m.txType; }
    uint16_t key = (uint16_t)((m.fblock << 8) | m.inst);
    uint32_t sig = ((uint32_t)m.fblock << 24) | ((uint32_t)m.inst << 16) | ((uint32_t)m.fkt << 4) | m.opType;
    unsigned n = ++g_mostSeen[sig];
    bool target = isTarget(key);
    if (target && (m.opType == 0xC || m.opType == 0xF)) { if (m.opType == 0xC) ++g_replies; else ++g_errors; }
    bool print = g_raw || target || n <= 3;
    if (print) {
        char pre[200];
        snprintf(pre, sizeof(pre), "[%6ld] main  MOST tx=%u var=%u/%u %04X->%04X FB=%02X/%02X fkt=%03X %s(%X) tag=%u #%u:",
                 rel(), m.txType, m.varTag, m.varVal, m.src, m.dst, m.fblock, m.inst, m.fkt, opName(m.opType),
                 m.opType, m.hdrTag, n);
        hexLine(pre, m.data.empty() ? NULL : &m.data[0], m.data.size(), g_raw ? 512 : 64);
    }
    if (m.fkt == 0x000 && m.opType == 0xC && !m.data.empty()) {
        bool packed = false;
        std::vector<uint16_t> f = decodeFktIds(&m.data[0], m.data.size(), &packed);
        g_fktLists[key] = f;
        printf("[%6ld] main  FB %04X FktIDs (%s, %u):", rel(), key, packed ? "packed 12-bit" : "bitfield",
               (unsigned)f.size());
        for (size_t i = 0; i < f.size() && i < 160; ++i) printf(" %03X", f[i]);
        printf("\n");
        if (target) sweep(c, key, f);
    }
    if (!m.data.empty() && m.data.size() <= 96 && (m.opType == 0xC || m.opType == 0x6)) {
        char src[96];
        snprintf(src, sizeof(src), "MOST FB=%02X/%02X fkt=%03X", m.fblock, m.inst, m.fkt);
        reportGeo(src, &m.data[0], m.data.size(), 1, true);
    }
}

void mostAttached(DsiClient* c, void*) {
    printf("[%6ld] main  attached to MOSTSubSystem.DSYSMOSTServer: registering client NBTAAD_GPSPROBE\n", rel());
    std::vector<uint8_t> p;
    DsiClient::putString(p, "NBTAAD_GPSPROBE");
    g_stage = 1;
    c->sendRequest(0xF, p);
    static const uint32_t kAttrs[] = { 0xC0000002u, 0xC0000003u, 0xC0000001u, 0x80000001u, 0x80000002u,
                                       0x80000003u, 0x80000004u, 0x80000005u, 0x80000006u, 0x80000007u };
    for (size_t i = 0; i < sizeof(kAttrs) / sizeof(kAttrs[0]); ++i) c->setNotification(kAttrs[i]);
}

void mostDataLocked(DsiClient* c, uint32_t type, uint32_t id, uint32_t seq, const uint8_t* pl, uint32_t len) {
    if (id == 0x80000001u) { onMost(c, pl, len); return; }
    unsigned n = ++g_otherIds[id];
    if (g_raw || n <= 4 || id == 0xC0000001u) {
        char pre[120];
        snprintf(pre, sizeof(pre), "[%6ld] main  DSI type=0x%x id=0x%08x seq=%u #%u:", rel(), type, id, seq, n);
        hexLine(pre, pl, len, id == 0xC0000001u ? 2048 : 128);
    }
    if (id == 0x80000000u && len >= 6) {
        g_client = mostRdU32(pl);
        g_ownDev = mostRdU16(pl + 4);
        printf("[%6ld] main  registered: clientId=%u deviceId=0x%04X%s\n", rel(), g_client, g_ownDev,
               g_client == 0xffffffffu ? "  (REGISTRATION REFUSED)" : "");
        if (g_client != 0xffffffffu && g_stage < 2) {
            g_stage = 2;
            c->setNotification(0xC0000000u);
            if (!g_forceKeys.empty()) startProbe(c);
        }
        return;
    }
    if (id == 0xC0000001u) {
        std::vector<RegEntry> r;
        if (decodeRegistry(pl, len, r)) {
            g_registry = r;
            g_haveRegistry = true;
            printf("[%6ld] main  registry: %u entries (a = device address, b = FBlock<<8|Inst)\n", rel(),
                   (unsigned)r.size());
            for (size_t i = 0; i < r.size(); ++i)
                printf("   [%3u] a=%04X b=%04X%s\n", (unsigned)i, r[i].a, r[i].b,
                       (r[i].a == g_ownDev || r[i].b == g_ownDev) ? "   (our device id)" : "");
            if (g_stage == 2) startProbe(c);
        } else {
            printf("[%6ld] main  0xC0000001 payload is not a {count, u16 pairs} vector\n", rel());
        }
    }
    // Registry may never arrive as 0xC0000001: start anyway on the next event after a few seconds.
    if (g_stage == 2 && rel() > 12000) startProbe(c);
}

void mostData(DsiClient* c, uint32_t type, uint32_t id, uint32_t seq, const uint8_t* pl, uint32_t len, void*) {
    pthread_mutex_lock(&g_mostLock);
    mostDataLocked(c, type, id, seq, pl, len);
    pthread_mutex_unlock(&g_mostLock);
}

void mostSummary() {
    printf("\n==== MOST summary ====\n");
    printf("   client %d, device id 0x%04X, %u MOST messages, %u Gets sent, %u Status / %u Error replies from targets\n",
           (int)g_client, g_ownDev, g_mostMsgs, g_sent, g_replies, g_errors);
    printf("   other DSI ids:");
    for (std::map<uint32_t, unsigned>::iterator it = g_otherIds.begin(); it != g_otherIds.end(); ++it)
        printf(" %08x x%u", it->first, it->second);
    printf("\n   FktIDs replies: %u FBlocks\n", (unsigned)g_fktLists.size());
    for (std::map<uint16_t, std::vector<uint16_t> >::iterator it = g_fktLists.begin(); it != g_fktLists.end(); ++it) {
        printf("   FB %04X (%u):", it->first, (unsigned)it->second.size());
        for (size_t i = 0; i < it->second.size() && i < 64; ++i) printf(" %03X", it->second[i]);
        printf("\n");
    }
    printf("   message signatures seen (FBlock/inst fkt op: count):\n");
    unsigned shown = 0;
    for (std::map<uint32_t, unsigned>::iterator it = g_mostSeen.begin(); it != g_mostSeen.end() && shown < 300; ++it, ++shown)
        printf("     %02X/%02X %03X %-9s %u\n", it->first >> 24, (it->first >> 16) & 0xff, (it->first >> 4) & 0xfff,
               opName(it->first & 0xf), it->second);
}

// ---- args ----------------------------------------------------------------------------------------------------------
void parseList(const char* s, std::vector<uint16_t>& out) {
    char buf[512];
    strncpy(buf, s, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = 0;
    for (char* t = strtok(buf, ","); t; t = strtok(NULL, ",")) out.push_back((uint16_t)strtoul(t, NULL, 16));
}

void usage(const char* a0) {
    printf("usage: %s [-L lat,lon] [-T deg] [-t secs] [-p|-q] [-F HEX,..] [-V 1,2,..] [-X tx,ctl,tag] [-s] [-m] [-r]\n", a0);
}

void onSig(int) {
    printf("\n[%s interrupted]\n", g_tag);
    fflush(stdout);
    _exit(0);
}

} // namespace

int main(int argc, char** argv) {
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "-L") && i + 1 < argc) {
            if (sscanf(argv[++i], "%lf,%lf", &g_refLat, &g_refLon) == 2) g_haveRef = true;
            else { usage(argv[0]); return 1; }
        }
        else if (!strcmp(argv[i], "-T") && i + 1 < argc) g_tol = atof(argv[++i]);
        else if (!strcmp(argv[i], "-t") && i + 1 < argc) g_secs = atoi(argv[++i]);
        else if (!strcmp(argv[i], "-p")) g_passive = true;
        else if (!strcmp(argv[i], "-q")) g_quick = true;
        else if (!strcmp(argv[i], "-F") && i + 1 < argc) parseList(argv[++i], g_forceKeys);
        else if (!strcmp(argv[i], "-V") && i + 1 < argc) {
            char vb[128];
            strncpy(vb, argv[++i], sizeof(vb) - 1);
            vb[sizeof(vb) - 1] = 0;
            for (char* t = strtok(vb, ","); t; t = strtok(NULL, ",")) g_versions.push_back(atoi(t));
        }
        else if (!strcmp(argv[i], "-X") && i + 1 < argc) {
            unsigned a = 0, b = 0, c = 0;
            if (sscanf(argv[++i], "%u,%u,%u", &a, &b, &c) < 1) { usage(argv[0]); return 1; }
            g_haveTx = true; g_txType = a; g_txCtl = b; g_txTag = c;
        }
        else if (!strcmp(argv[i], "-s")) g_skipDsi = true;
        else if (!strcmp(argv[i], "-m")) g_skipMost = true;
        else if (!strcmp(argv[i], "-r")) g_raw = true;
        else { usage(argv[0]); return 1; }
    }
    if (g_versions.empty()) for (int v = 1; v <= 6; ++v) g_versions.push_back(v);
    if (g_secs < 30) g_secs = 30;

    setvbuf(stdout, NULL, _IOLBF, 0);
    signal(SIGINT, onSig);
    signal(SIGTERM, onSig);
    signal(SIGPIPE, SIG_IGN);
    g_t0 = nowMs();
    long deadline = g_t0 + g_secs * 1000L;
    printf("test_gps: GPS source probe, %d s%s. Reference: %s", g_secs, g_passive ? " (passive)" : "",
           g_haveRef ? "" : "none (use -L lat,lon for MATCH flags)\n");
    if (g_haveRef) printf("%.5f,%.5f +/- %.2f deg\n", g_refLat, g_refLon, g_tol);

    systemFacts();

    // DSI children are forked before any thread exists in this process (fork + threads do not mix).
    std::vector<pid_t> kids;
    if (!g_skipDsi) {
        static const DsiTry kBase[] = {
            { "DNAVIPos", "PosReal", 0 }, { "DNAVIPos", "Pos", 0 },
            { "DNAVISensor", "Sensors", 0 }, { "DNAVISensor", "SensorWrapper", 0 },
        };
        printf("\n==== 2. DSI: %u services x %u versions (one child each; 8 s to attach) ====\n",
               (unsigned)(sizeof(kBase) / sizeof(kBase[0])), (unsigned)g_versions.size());
        fflush(stdout);
        for (size_t b = 0; b < sizeof(kBase) / sizeof(kBase[0]); ++b) {
            for (size_t v = 0; v < g_versions.size(); ++v) {
                DsiTry t = kBase[b];
                t.major = g_versions[v];
                pid_t pid = fork();
                if (pid == 0) _exit(runDsiChild(t, deadline - 3000));
                if (pid > 0) kids.push_back(pid);
                else printf("   fork failed: %s\n", strerror(errno));
            }
        }
    }

    if (!g_skipMost) {
        printf("\n==== 3. MOST: MOSTSubSystem.DSYSMOSTServer 4.5 ====\n");
        g_most.setQuiet(true);
        g_most.setCallbacks(&mostAttached, &mostData, NULL);
        if (!g_most.open("DSYSMOSTServer", "MOSTSubSystem", 4, 5)) printf("   DSI open FAILED\n");
    }

    unsigned attachedKids = 0;
    std::map<pid_t, int> kidStatus;
    bool warned = false;
    while (nowMs() < deadline) {
        usleep(250000);
        for (;;) {
            int st = 0;
            pid_t p = waitpid(-1, &st, WNOHANG);
            if (p <= 0) break;
            kidStatus[p] = WIFEXITED(st) ? WEXITSTATUS(st) : -1;
        }
        long r = rel();
        if (!g_skipMost && !warned && r > 10000 && g_stage == 0) {
            warned = true;
            printf("[%6ld] main  WARNING: MOST server not attached after 10 s\n", r);
        }
    }
    // children print their own summary at deadline - 3 s; give them time to exit
    for (int i = 0; i < 40; ++i) {
        int st = 0;
        pid_t p = waitpid(-1, &st, WNOHANG);
        if (p > 0) { kidStatus[p] = WIFEXITED(st) ? WEXITSTATUS(st) : -1; continue; }
        if (kidStatus.size() >= kids.size()) break;
        usleep(100000);
    }
    for (size_t i = 0; i < kids.size(); ++i)
        if (!kidStatus.count(kids[i])) { kill(kids[i], SIGKILL); kidStatus[kids[i]] = -9; }
    for (std::map<pid_t, int>::iterator it = kidStatus.begin(); it != kidStatus.end(); ++it)
        if (it->second == 0) ++attachedKids;

    printf("\n==== 4. result ====\n");
    printf("   DSI: %u of %u service/version children attached (see their ==== DSI blocks above)\n", attachedKids,
           (unsigned)kids.size());
    if (!g_skipMost) {
        pthread_mutex_lock(&g_mostLock);
        mostSummary();
        pthread_mutex_unlock(&g_mostLock);
    }
    printGeoSummary();
    printf("\nsend this whole log back.\n");
    fflush(stdout);
    _exit(0);   // DSI connection threads are detached and block in the library
}
