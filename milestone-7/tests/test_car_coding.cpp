// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Required Notice: Copyright (c) 2026 nanaisu - NBT-AAD (https://github.com/Sq00ky/Supra-iDrive6-Android-Auto)
// Host test: car/coding.hpp, the helper (tools/nbt_aad_coding.cpp) against a fake settings library, and car/coding_read.cpp
// running the helper or scripts that stand in for it. Every value here is invented. Arguments: the helper, the fake library.
#include "car/coding_read.hpp"

#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

using namespace nbt::car;

static int fails = 0;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); ++fails; } } while (0)

static unsigned long long nowMs() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (unsigned long long)ts.tv_sec * 1000ULL + (unsigned long long)ts.tv_nsec / 1000000ULL;
}

static std::string g_dir;

static std::string tempPath(const char* name) { return g_dir + "/" + name; }

static std::string script(const char* name, const std::string& body) {
    const std::string path = tempPath(name);
    FILE* f = fopen(path.c_str(), "w");
    if (f) {
        fprintf(f, "#!/bin/sh\n%s\n", body.c_str());
        fclose(f);
    }
    chmod(path.c_str(), 0755);
    return path;
}

// How many lines the scripts below have added to their count file, which is how many times they ran.
static int runsOf(const std::string& path) {
    FILE* f = fopen(path.c_str(), "r");
    if (!f) return 0;
    int n = 0, c;
    while ((c = fgetc(f)) != EOF) if (c == '\n') ++n;
    fclose(f);
    return n;
}

// A complete output line for a block of the size the table says, with one byte set.
static std::string blockLine(unsigned block, unsigned byte, unsigned value) {
    const CodingBlock* known = findCodingBlock(block);
    const unsigned size = known ? known->size : 10;
    char head[32];
    snprintf(head, sizeof(head), "%04x %u ", block, size);
    std::string line = head;
    for (unsigned i = 0; i < size; ++i) {
        char hex[3];
        snprintf(hex, sizeof(hex), "%02x", i == byte ? value : 0);
        line += hex;
    }
    return line + "\n";
}

static std::string withHeader(const std::string& lines) { return std::string(kCodingHeader) + "\n" + lines; }

static bool parses(CodingSnapshot& s, const std::string& text) { return s.parse(text.data(), text.size()); }

static void testSnapshot() {
    const CodingParam kBit = { "BIT", 0x3004, 3, 0x02 };
    const CodingParam kField = { "FIELD", 0x3004, 3, 0x30 };
    const CodingParam kWhole = { "WHOLE", 0x3004, 3, 0xff };
    const CodingParam kNoBlock = { "NOBLOCK", 0x3000, 3, 0x02 };
    const CodingParam kPastEnd = { "PASTEND", 0x3004, 50, 0x01 };
    const CodingParam kNoMask = { "NOMASK", 0x3004, 3, 0x00 };

    CodingSnapshot s;
    CHECK(parses(s, withHeader(blockLine(0x3004, 3, 0x26))));
    CHECK(s.blockCount() == 1);
    CHECK(s.value(kBit) == 1);
    CHECK(s.value(kField) == 2);      // counted from the lowest bit of the mask
    CHECK(s.value(kWhole) == 0x26);
    CHECK(s.value(kNoBlock) == -1);
    CHECK(s.value(kPastEnd) == -1);
    CHECK(s.value(kNoMask) == -1);
    CHECK(s.value(kTypLenkung) == 1);

    // The right bit of the right byte of the right block.
    CHECK(parses(s, withHeader(blockLine(0x3004, 3, 0xfd))));
    CHECK(s.value(kTypLenkung) == 0);
    CHECK(parses(s, withHeader(blockLine(0x3004, 4, 0xff))));
    CHECK(s.value(kTypLenkung) == 0);
    CHECK(parses(s, withHeader(blockLine(0x3000, 3, 0xff) + blockLine(0x3004, 3, 0))));
    CHECK(s.blockCount() == 2 && s.value(kTypLenkung) == 0 && s.value(kNoBlock) == 1);

    // Not the helper's output at all.
    CHECK(!parses(s, ""));
    CHECK(!parses(s, kCodingHeader));
    CHECK(!parses(s, withHeader("")));
    CHECK(!parses(s, "nbt-aad-coding 2\n" + blockLine(0x3004, 3, 2)));
    CHECK(!parses(s, blockLine(0x3004, 3, 2)));
    CHECK(s.blockCount() == 0 && s.value(kTypLenkung) == -1);   // a failed parse leaves nothing behind

    // Lines that can't be trusted are left out, and the others stay.
    const std::string good = blockLine(0x3000, 0, 1);
    const std::string complete = blockLine(0x3004, 3, 2);
    const std::string bad[] = {
        complete.substr(0, complete.size() - 1),                // cut off: no newline
        "3004 49 " + complete.substr(8, 98) + "\n",             // another size than the table says
        "3004 50 " + complete.substr(8, 98) + "\n",             // shorter than it says
        "3004 50 " + std::string(99, '0') + "z\n",              // not hex
        "3004 50 " + std::string(100, 'A') + "\n",              // the helper only writes lower case
        "300g 50 " + std::string(100, '0') + "\n",              // bad block number
        "9999 50 " + std::string(100, '0') + "\n",              // unknown block
        "3004  50 " + std::string(100, '0') + "\n",             // stray space
        "3004 99999999999 00\n",                                // absurd size
        "3004\n",
        "\n",
    };
    for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); ++i) {
        CHECK(parses(s, withHeader(good + bad[i])));
        CHECK(s.blockCount() == 1 && s.value(kTypLenkung) == -1);
    }
}

static void testHelper(const char* helper) {
    const unsigned kLimit = 5000;
    const CodingParam kInOtherBlock = { "OTHER", 0x3010, 7, 0xff };   // the fake's bytes are their offsets

    // Every block comes through, in order.
    setenv("NBT_FAKE_SYSET", "", 1);
    const struct { const char* byte; int value; } kValues[] = { { "2", 1 }, { "0", 0 }, { "253", 0 }, { "255", 1 } };
    for (size_t i = 0; i < sizeof(kValues) / sizeof(kValues[0]); ++i) {
        setenv("NBT_FAKE_BYTE", kValues[i].byte, 1);
        const CodingRead r = runCodingHelper(helper, codingHelperEnvironment(), kLimit);
        CHECK(r.ok && r.why.empty() && !r.stuck);
        CHECK(r.snapshot.blockCount() == kCodingBlockCount);
        CHECK(r.snapshot.value(kTypLenkung) == kValues[i].value);
        CHECK(r.snapshot.value(kInOtherBlock) == 7);
    }

    // One bad block costs only that block's values.
    setenv("NBT_FAKE_BYTE", "2", 1);
    setenv("NBT_FAKE_SYSET", "no3004", 1);
    {
        const CodingRead r = runCodingHelper(helper, codingHelperEnvironment(), kLimit);
        CHECK(r.ok && r.snapshot.blockCount() == kCodingBlockCount - 1);
        CHECK(r.snapshot.value(kTypLenkung) == -1 && r.snapshot.value(kInOtherBlock) == 7);
    }
    setenv("NBT_FAKE_SYSET", "no3000", 1);
    {
        const CodingRead r = runCodingHelper(helper, codingHelperEnvironment(), kLimit);
        CHECK(r.ok && r.snapshot.blockCount() == kCodingBlockCount - 1 && r.snapshot.value(kTypLenkung) == 1);
    }
    setenv("NBT_FAKE_SYSET", "short", 1);
    {
        const CodingRead r = runCodingHelper(helper, codingHelperEnvironment(), kLimit);
        CHECK(r.ok && r.snapshot.value(kTypLenkung) == -1 && r.snapshot.value(kInOtherBlock) == 7);
    }

    // Failures, each with its reason.
    const struct { const char* mode; const char* why; bool stuck; } kFailures[] = {
        { "attach", "attach", false }, { "noblocks", "any coding block", false }, { "fatal", "fatal", true },
    };
    for (size_t i = 0; i < sizeof(kFailures) / sizeof(kFailures[0]); ++i) {
        setenv("NBT_FAKE_SYSET", kFailures[i].mode, 1);
        const CodingRead r = runCodingHelper(helper, codingHelperEnvironment(), kLimit);
        CHECK(!r.ok && r.stuck == kFailures[i].stuck && r.why.find(kFailures[i].why) != std::string::npos);
    }

    // Its own time limit ends a hang, ahead of the daemon's, and that counts as stuck just as a kill does.
    setenv("NBT_FAKE_SYSET", "hang", 1);
    {
        const unsigned long long start = nowMs();
        const CodingRead r = runCodingHelper(helper, codingHelperEnvironment(), kLimit);
        CHECK(!r.ok && r.stuck && r.why.find("gave up") != std::string::npos);
        CHECK(nowMs() - start >= 1500 && nowMs() - start < 4000);
    }
    unsetenv("NBT_FAKE_SYSET");
    unsetenv("NBT_FAKE_BYTE");

    // No library, and no helper.
    const char* lib = getenv("NBT_CODING_LIB");
    const std::string libPath = lib ? lib : "";
    setenv("NBT_CODING_LIB", "/nonexistent/lib.so", 1);
    {
        const CodingRead r = runCodingHelper(helper, codingHelperEnvironment(), kLimit);
        CHECK(!r.ok && r.why.find("did not load") != std::string::npos);
    }
    setenv("NBT_CODING_LIB", libPath.c_str(), 1);
    {
        const CodingRead r = runCodingHelper("/nonexistent/nbt-aad-coding", codingHelperEnvironment(), kLimit);
        CHECK(!r.ok && !r.stuck && !r.why.empty());
    }
}

// A script that prints a good answer and then does whatever `then` says.
static std::string answering(const char* name, const std::string& before, const std::string& then) {
    return script(name, before + "\nprintf '%s\\n' '" + kCodingHeader + "'\nprintf '" + blockLine(0x3004, 3, 2).substr(0, 108) + "\\n'\n" + then);
}

static void testRunner() {
    const std::vector<std::string> env = codingHelperEnvironment();
    {
        const CodingRead r = runCodingHelper(answering("good", "", "exit 0").c_str(), env, 2000);
        CHECK(r.ok && r.snapshot.value(kTypLenkung) == 1);
    }
    // Output only counts from a helper that finished well.
    {
        const CodingRead r = runCodingHelper(answering("bad_exit", "", "exit 13").c_str(), env, 2000);
        CHECK(!r.ok && r.why.find("exit code 13") != std::string::npos);
    }
    {
        const CodingRead r = runCodingHelper(answering("died", "", "kill -9 $$").c_str(), env, 2000);
        CHECK(!r.ok && !r.stuck && r.why.find("died") != std::string::npos);
    }
    {
        const CodingRead r = runCodingHelper(script("garbage", "echo hello\nexit 0").c_str(), env, 2000);
        CHECK(!r.ok && r.why.find("no usable") != std::string::npos);
    }
    {
        const CodingRead r = runCodingHelper(script("silent", "exit 0").c_str(), env, 2000);
        CHECK(!r.ok && r.why.find("no usable") != std::string::npos);
    }

    // Output that arrives in pieces, and more than a pipe holds at once.
    {
        const CodingRead r = runCodingHelper(answering("slow", "sleep 0.1", "sleep 0.1\nexit 0").c_str(), env, 2000);
        CHECK(r.ok && r.snapshot.value(kTypLenkung) == 1);
    }
    {
        std::string body = std::string("printf '%s\\n' '") + kCodingHeader + "'\ni=0\nwhile [ $i -lt 300 ]; do printf '%0100d\\n' 0; i=$((i+1)); done\n"
                           "printf '" + blockLine(0x3004, 3, 2).substr(0, 108) + "\\n'\nexit 0";
        const CodingRead r = runCodingHelper(script("long", body).c_str(), env, 2000);
        CHECK(r.ok && r.snapshot.value(kTypLenkung) == 1);
    }
    // Output with no end is stopped.
    {
        const unsigned long long start = nowMs();
        const CodingRead r = runCodingHelper(script("endless", "while :; do echo 0000000000000000000000000000000000000000; done").c_str(), env, 5000);
        CHECK(!r.ok && !r.stuck && r.why.find("too much") != std::string::npos);
        CHECK(nowMs() - start < 4000);
    }

    // The helper's end counts, not the pipe's: a process that kept its stdout must not hold up the answer.
    {
        const unsigned long long start = nowMs();
        const CodingRead r = runCodingHelper(answering("holder", "sleep 3 &", "exit 0").c_str(), env, 2000);
        CHECK(r.ok && r.snapshot.value(kTypLenkung) == 1);
        CHECK(nowMs() - start < 1000);
    }

    // A helper that never answers is killed at the limit, and what it said before that doesn't count.
    {
        const std::string alive = tempPath("alive");
        const unsigned long long start = nowMs();
        const CodingRead r = runCodingHelper(answering("stuck", "echo $$ > " + alive, "exec sleep 30").c_str(), env, 300);
        CHECK(r.stuck && !r.ok);
        CHECK(r.ms >= 300 && nowMs() - start < 1500);
        FILE* f = fopen(alive.c_str(), "r");
        int pid = 0;
        if (f) { if (fscanf(f, "%d", &pid) != 1) pid = 0; fclose(f); }
        CHECK(pid > 0 && kill(pid, 0) != 0);
    }

    // Only LD_PRELOAD is left out of its environment, and stdin and stderr are on /dev/null.
    setenv("LD_PRELOAD", "libnot-a-real-library.so", 1);
    setenv("NBT_CODING_TEST", "kept", 1);
    const std::vector<std::string> cleaned = codingHelperEnvironment();
    unsetenv("LD_PRELOAD");
    unsetenv("NBT_CODING_TEST");
    {
        const CodingRead r = runCodingHelper(answering("env", "[ -z \"$LD_PRELOAD\" ] && [ \"$NBT_CODING_TEST\" = kept ] || exit 12", "exit 0").c_str(), cleaned, 2000);
        CHECK(r.ok);
    }
    {
        const CodingRead r = runCodingHelper(answering("fds", "[ \"$(readlink /proc/self/fd/0)\" = /dev/null ] && [ \"$(readlink /proc/self/fd/2)\" = /dev/null ] || exit 12", "exit 0").c_str(), env, 2000);
        CHECK(r.ok);
    }
}

static void testReader(const char* helper) {
    // Never started: no answer, and no wait. This is what a session test gets.
    {
        CodingReader reader(2000);
        const unsigned long long start = nowMs();
        CHECK(reader.value(kTypLenkung) == -1);
        CHECK(nowMs() - start < 100);
    }

    // An answer is ready for the first caller and kept: one read however many values are asked for.
    {
        const std::string count = tempPath("count_ok");
        const std::string program = answering("ok", "echo run >> " + count + "\nsleep 0.1", "exit 0");
        CodingReader* reader = new CodingReader(2000);   // leaked, like the daemon's
        const CodingParam kOther = { "OTHER", 0x3004, 3, 0x20 };
        const unsigned long long start = nowMs();
        reader->start(program.c_str());
        CHECK(nowMs() - start < 50);   // start() doesn't wait for it
        CHECK(reader->value(kTypLenkung) == 1);
        reader->start(program.c_str());
        CHECK(reader->value(kTypLenkung) == 1);
        CHECK(reader->value(kOther) == 0);
        CHECK(runsOf(count) == 1);
    }

    // A failed read gives this caller no answer and is tried again for the next.
    {
        const std::string count = tempPath("count_retry");
        const std::string marker = tempPath("retry_marker");
        const std::string before = "echo run >> " + count + "\n[ -e " + marker + " ] || { touch " + marker + "; exit 13; }";
        const std::string program = answering("retry", before, "exit 0");
        CodingReader* reader = new CodingReader(2000);
        reader->start(program.c_str());
        CHECK(reader->value(kTypLenkung) == -1);
        CHECK(reader->value(kTypLenkung) == 1);
        CHECK(reader->value(kTypLenkung) == 1);
        CHECK(runsOf(count) == 2);
    }

    // A helper that had to be killed is not run again, and later callers don't wait.
    {
        const std::string count = tempPath("count_stuck");
        const std::string program = script("stuck2", "echo run >> " + count + "\nexec sleep 30");
        CodingReader* reader = new CodingReader(200);
        reader->start(program.c_str());
        unsigned long long start = nowMs();
        CHECK(reader->value(kTypLenkung) == -1);
        CHECK(nowMs() - start < 1200);
        start = nowMs();
        CHECK(reader->value(kTypLenkung) == -1);
        CHECK(nowMs() - start < 100);
        usleep(100000);
        CHECK(runsOf(count) == 1);
    }

    // So does a helper that gave up by itself or found the lock broken: the helper's own limit ends first, so this is how
    // a stuck lock normally shows up.
    const int kStuckExits[] = { kCodingExitHang, kCodingExitFatal };
    for (size_t i = 0; i < sizeof(kStuckExits) / sizeof(kStuckExits[0]); ++i) {
        char name[32], body[64];
        snprintf(name, sizeof(name), "count_exit_%d", kStuckExits[i]);
        const std::string count = tempPath(name);
        snprintf(name, sizeof(name), "exit_%d", kStuckExits[i]);
        snprintf(body, sizeof(body), "\nexit %d", kStuckExits[i]);
        const std::string program = script(name, "echo run >> " + count + body);
        CodingReader* reader = new CodingReader(2000);
        reader->start(program.c_str());
        CHECK(reader->value(kTypLenkung) == -1);
        CHECK(reader->value(kTypLenkung) == -1);
        usleep(100000);
        CHECK(reader->value(kTypLenkung) == -1);
        CHECK(runsOf(count) == 1);
    }

    // The real thing: the helper against a library that hangs. One wait for its limit, and none after that.
    {
        setenv("NBT_FAKE_SYSET", "hang", 1);
        CodingReader* reader = new CodingReader(kCodingReadLimitMs);
        reader->start(helper);
        unsetenv("NBT_FAKE_SYSET");
        unsigned long long start = nowMs();
        CHECK(reader->value(kTypLenkung) == -1);
        CHECK(nowMs() - start >= 1500 && nowMs() - start < 3000);
        for (int i = 0; i < 3; ++i) {
            start = nowMs();
            CHECK(reader->value(kTypLenkung) == -1);
            CHECK(nowMs() - start < 100);
        }
    }
}

int main(int argc, char** argv) {
    if (argc != 3) {
        printf("usage: test_car_coding HELPER FAKE_LIBRARY\n");
        return 2;
    }
    char dir[] = "/tmp/nbt_coding_XXXXXX";
    if (!mkdtemp(dir)) {
        perror("mkdtemp");
        return 2;
    }
    g_dir = dir;
    setenv("NBT_CODING_LIB", argv[2], 1);

    testSnapshot();
    testHelper(argv[1]);
    testRunner();
    testReader(argv[1]);

    const std::string cleanup = "rm -rf " + g_dir;
    if (system(cleanup.c_str()) != 0) printf("could not remove %s\n", g_dir.c_str());
    printf(fails ? "test_car_coding FAILED (%d)\n" : "test_car_coding OK\n", fails);
    return fails ? 1 : 0;
}
