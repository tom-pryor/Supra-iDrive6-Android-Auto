// SPDX-License-Identifier: PolyForm-Noncommercial-1.0.0
// Required Notice: Copyright (c) 2026 nanaisu - NBT-AAD (https://github.com/Sq00ky/Supra-iDrive6-Android-Auto)
#include "car/coding_read.hpp"

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <signal.h>
#include <spawn.h>
#include <stdio.h>
#include <string.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

extern char** environ;

namespace nbt {
namespace car {

namespace {

const char kHelperName[] = "nbt-aad-coding";
const char kInstalledHelper[] = "/opt/aa/nbt-aad-coding";
const unsigned kPollMs = 10;
const unsigned kReapMs = 200;            // how long a killed helper gets to disappear
const unsigned kWaitSlackMs = 500;       // on top of the limit, so the read's own time limit always ends a wait first
const size_t kMaxOutput = 64 * 1024;     // far more than every block, as hex, takes

unsigned long long nowMs() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (unsigned long long)ts.tv_sec * 1000ULL + (unsigned long long)ts.tv_nsec / 1000000ULL;
}

pid_t startHelper(const char* program, const std::vector<std::string>& environment, int stdoutFd, int readFd, std::string& why) {
    int nul = open("/dev/null", O_RDWR);
    if (nul < 0) {
        why = std::string("cannot open /dev/null: ") + strerror(errno);
        return -1;
    }
    std::vector<char*> envp;
    for (size_t i = 0; i < environment.size(); ++i) envp.push_back(const_cast<char*>(environment[i].c_str()));
    envp.push_back(NULL);
    char* const argv[] = { const_cast<char*>(program), NULL };

    pid_t pid = -1;
    int err = 0;
#ifdef __QNX__
    // The HU's posix_spawn refused every path on the car, so QNX spawn() with an fd map, as for mm-renderer. Fds not in
    // the map, like our USB and TCP handles, are not inherited. Every signal back to SIG_DFL (we ignore SIGPIPE), none
    // blocked, and a session of its own so nothing aimed at the shell that started the daemon reaches it.
    (void)readFd;
    int map[3] = { nul, stdoutFd, nul };
    struct inheritance inh;
    memset(&inh, 0, sizeof(inh));
    inh.flags = SPAWN_SETSID | SPAWN_SETSIGDEF | SPAWN_SETSIGMASK;
    sigfillset(&inh.sigdefault);
    sigemptyset(&inh.sigmask);
    pid = spawn(program, 3, map, &inh, argv, &envp[0]);
    if (pid < 0) err = errno;
#else
    posix_spawn_file_actions_t fa;
    posix_spawn_file_actions_init(&fa);
    posix_spawn_file_actions_adddup2(&fa, nul, 0);
    posix_spawn_file_actions_adddup2(&fa, stdoutFd, 1);
    posix_spawn_file_actions_adddup2(&fa, nul, 2);
    posix_spawn_file_actions_addclose(&fa, readFd);
    err = posix_spawn(&pid, program, &fa, NULL, argv, &envp[0]);
    posix_spawn_file_actions_destroy(&fa);
    if (err != 0) pid = -1;
#endif
    close(nul);
    if (pid < 0) why = std::string("cannot start ") + program + ": " + strerror(err);
    return pid;
}

void killHelper(pid_t pid) {
    int status = 0;
    kill(pid, SIGKILL);
    for (unsigned waited = 0; waited < kReapMs && waitpid(pid, &status, WNOHANG) == 0; waited += kPollMs)
        usleep(kPollMs * 1000);
}

std::string helperPath() {
    char self[PATH_MAX];
    ssize_t n = -1;
#ifdef __QNX__
    char file[48];
    snprintf(file, sizeof(file), "/proc/%d/exefile", (int)getpid());
    int fd = open(file, O_RDONLY);
    if (fd >= 0) {
        n = read(fd, self, sizeof(self) - 1);
        close(fd);
    }
#else
    n = readlink("/proc/self/exe", self, sizeof(self) - 1);
#endif
    if (n > 0) {
        self[n] = '\0';
        char* slash = strrchr(self, '/');
        if (slash) {
            const std::string beside = std::string(self, slash + 1 - self) + kHelperName;
            if (access(beside.c_str(), X_OK) == 0) return beside;
        }
    }
    return kInstalledHelper;
}

CodingReader& daemonReader() {
    static CodingReader* reader = new CodingReader(kCodingReadLimitMs);   // never destroyed: its thread may outlive main
    return *reader;
}

}  // namespace

std::vector<std::string> codingHelperEnvironment() {
    std::vector<std::string> out;
    for (char** e = environ; e && *e; ++e)
        if (strncmp(*e, "LD_PRELOAD=", 11) != 0) out.push_back(*e);
    return out;
}

CodingRead runCodingHelper(const char* program, const std::vector<std::string>& environment, unsigned limitMs) {
    CodingRead r;
    const unsigned long long start = nowMs();
    int fds[2];
    if (pipe(fds) != 0) {
        r.why = std::string("cannot make a pipe: ") + strerror(errno);
        return r;
    }
    // Close-on-exec so that nothing else this process starts holds the pipe. Non-blocking: if something does anyway, its
    // end never closes, so the end of the helper is what counts, not the end of the pipe.
    fcntl(fds[0], F_SETFD, FD_CLOEXEC);
    fcntl(fds[1], F_SETFD, FD_CLOEXEC);
    fcntl(fds[0], F_SETFL, O_NONBLOCK);
    const pid_t pid = startHelper(program, environment, fds[1], fds[0], r.why);
    close(fds[1]);
    if (pid < 0) {
        close(fds[0]);
        return r;
    }

    std::string output;
    int status = 0;
    bool ended = false;
    for (;;) {
        char buf[1024];
        for (;;) {
            const ssize_t got = read(fds[0], buf, sizeof(buf));
            if (got > 0) output.append(buf, (size_t)got);
            else if (got < 0 && errno == EINTR) continue;
            if (got <= 0 || output.size() > kMaxOutput) break;
        }
        if (output.size() > kMaxOutput) {
            killHelper(pid);
            r.why = "too much output from the helper";
            break;
        }
        if (ended) break;   // everything it wrote before it ended has been read

        const pid_t waited = waitpid(pid, &status, WNOHANG);
        if (waited == pid) {
            ended = true;
        } else if (waited < 0 && errno != EINTR) {
            r.why = std::string("lost track of the helper: ") + strerror(errno);
            break;
        } else if (nowMs() - start >= limitMs) {
            killHelper(pid);
            char text[64];
            snprintf(text, sizeof(text), "no answer within %u ms", limitMs);
            r.why = text;
            r.stuck = true;
            break;
        } else {
            usleep(kPollMs * 1000);
        }
    }
    close(fds[0]);
    r.ms = (unsigned)(nowMs() - start);
    if (!ended) return r;

    if (!WIFEXITED(status)) {
        char text[48];
        snprintf(text, sizeof(text), "the helper died (status %#x)", (unsigned)status);
        r.why = text;
    } else if (WEXITSTATUS(status) != kCodingExitOk) {
        char text[96];
        snprintf(text, sizeof(text), "%s (exit code %d)", codingExitName(WEXITSTATUS(status)), WEXITSTATUS(status));
        r.why = text;
        // The helper's own time limit ends before ours, so this is how a stuck lock normally shows up.
        r.stuck = WEXITSTATUS(status) == kCodingExitHang || WEXITSTATUS(status) == kCodingExitFatal;
    } else if (r.snapshot.parse(output.data(), output.size())) {
        r.ok = true;
    } else {
        r.why = "no usable coding block in the helper's output";
    }
    return r;
}

CodingReader::CodingReader(unsigned limitMs) : limitMs_(limitMs), state_(kIdle) {
    pthread_mutex_init(&mu_, NULL);
}

void CodingReader::start(const char* program) {
    pthread_mutex_lock(&mu_);
    if (state_ == kIdle) {
        program_ = program;
        environment_ = codingHelperEnvironment();   // here, in the caller's thread: nothing else reads or changes the copy
        launch();
    }
    pthread_mutex_unlock(&mu_);
}

void CodingReader::launch() {
    pthread_t thread;
    pthread_attr_t attr;
    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
    state_ = kReading;
    const int rc = pthread_create(&thread, &attr, &CodingReader::threadMain, this);
    pthread_attr_destroy(&attr);
    if (rc != 0) {
        state_ = kFailed;
        printf("[Coding] not read: cannot start a thread: %s\n", strerror(rc));
    }
}

void* CodingReader::threadMain(void* self) {
    static_cast<CodingReader*>(self)->readOnce();
    return NULL;
}

void CodingReader::readOnce() {
    const CodingRead r = runCodingHelper(program_.c_str(), environment_, limitMs_);
    if (r.ok) {
        printf("[Coding] read %u of %u blocks in %u ms\n", (unsigned)r.snapshot.blockCount(), (unsigned)kCodingBlockCount, r.ms);
        for (size_t i = 0; i < sizeof(kLoggedCodingParams) / sizeof(kLoggedCodingParams[0]); ++i)
            printf("[Coding] %s = %d\n", kLoggedCodingParams[i]->name, r.snapshot.value(*kLoggedCodingParams[i]));
    } else {
        printf("[Coding] not read after %u ms: %s%s\n", r.ms, r.why.c_str(), r.stuck ? " (not trying again)" : "");
    }
    pthread_mutex_lock(&mu_);
    if (r.ok) snapshot_ = r.snapshot;
    state_ = r.ok ? kKnown : r.stuck ? kGaveUp : kFailed;
    pthread_mutex_unlock(&mu_);
}

int CodingReader::value(const CodingParam& param) {
    const unsigned long long start = nowMs();
    pthread_mutex_lock(&mu_);
    while (state_ == kReading && nowMs() - start < limitMs_ + kWaitSlackMs) {
        pthread_mutex_unlock(&mu_);
        usleep(kPollMs * 1000);
        pthread_mutex_lock(&mu_);
    }
    const int value = state_ == kKnown ? snapshot_.value(param) : -1;
    if (state_ == kFailed) launch();   // for a later call
    pthread_mutex_unlock(&mu_);
    return value;
}

void startCodingRead() {
    const std::string program = helperPath();
    daemonReader().start(program.c_str());
}

int codingValue(const CodingParam& param) { return daemonReader().value(param); }

} // namespace car
} // namespace nbt
