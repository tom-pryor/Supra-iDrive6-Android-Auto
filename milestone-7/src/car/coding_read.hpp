// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Required Notice: Copyright (c) 2026 nanaisu - NBT-AAD (https://github.com/Sq00ky/Supra-iDrive6-Android-Auto)
#ifndef CAR_CODING_READ_HPP
#define CAR_CODING_READ_HPP

// The head unit's coding values for the daemon, read by a helper program (tools/nbt_aad_coding.cpp) in the background, so
// a stuck settings lock can cost at most a short wait for a value, never the daemon.

#include "car/coding.hpp"

#include <pthread.h>
#include <string>
#include <vector>

namespace nbt {
namespace car {

// One run of the helper.
struct CodingRead {
    bool ok;                  // snapshot holds at least one block
    CodingSnapshot snapshot;
    bool stuck;               // the library's lock looks stuck or broken: the helper hung, was killed or said so
    unsigned ms;              // how long it took
    std::string why;          // why there is no answer
    CodingRead() : ok(false), stuck(false), ms(0) {}
};

// The environment for the helper: ours without LD_PRELOAD, which comes from the HMI and which it has no use for.
std::vector<std::string> codingHelperEnvironment();

// Runs `program` for at most limitMs, with stdin and stderr on /dev/null, stdout on a pipe to us, and nothing else open.
CodingRead runCodingHelper(const char* program, const std::vector<std::string>& environment, unsigned limitMs);

// The coding, read once. start() returns at once and the read goes on in a thread of its own. An answer is kept for good.
// A failed read is tried again on the next value(). A stuck one (CodingRead::stuck) is not: each try would hold up whoever
// asks, and attach to a lock that is already in trouble.
class CodingReader {
public:
    explicit CodingReader(unsigned limitMs);
    void start(const char* program);

    // Waits for a read in progress (bounded), then returns the value, or -1 if there is none yet, in which case a later
    // call may have it. Returns at once if start() was never called.
    int value(const CodingParam& param);

private:
    enum State { kIdle, kReading, kKnown, kFailed, kGaveUp };
    static void* threadMain(void* self);
    void readOnce();
    void launch();    // mu_ held

    pthread_mutex_t mu_;
    const unsigned limitMs_;
    State state_;
    CodingSnapshot snapshot_;
    std::string program_;
    std::vector<std::string> environment_;
};

// The daemon's copy, for at most kCodingReadLimitMs. The helper is looked for beside this program, then in /opt/aa.
enum { kCodingReadLimitMs = 3000 };
void startCodingRead();
int codingValue(const CodingParam& param);

} // namespace car
} // namespace nbt

#endif
