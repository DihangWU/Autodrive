#pragma once
#include <chrono>
#include <cstdio>
#include <cstdlib>

// This header is shared only by project-built SUMO GUI translation units.
// No GUI library dependency is introduced into the original SUMO installation.
namespace autodrive_gui {
struct LockContext {
    void* owner;
    void (*release)(void*);
    void (*acquire)(void*);
};

inline LockContext*& currentLock() {
    static thread_local LockContext* context = nullptr;
    return context;
}

class SimulationLockScope {
    LockContext context;
    LockContext* previous;
public:
    SimulationLockScope(void* owner, void (*release)(void*), void (*acquire)(void*))
        : context{owner, release, acquire}, previous(currentLock()) {
        currentLock() = &context;
    }
    ~SimulationLockScope() { currentLock() = previous; }
    SimulationLockScope(const SimulationLockScope&) = delete;
    SimulationLockScope& operator=(const SimulationLockScope&) = delete;
};

class SocketReadScope {
    LockContext* context;
public:
    SocketReadScope() : context(currentLock()) {
        // Diagnostic A/B switch: normal runs always release the idle lock.
        const char* disabled = std::getenv("AUTODRIVE_SUMO_GUI_DISABLE_READ_UNLOCK");
        if (disabled != nullptr && disabled[0] == '1') context = nullptr;
        if (context != nullptr) context->release(context->owner);
    }
    ~SocketReadScope() {
        // Also reacquire on socket exceptions before unwinding the outer lock.
        if (context != nullptr) context->acquire(context->owner);
    }
    SocketReadScope(const SocketReadScope&) = delete;
    SocketReadScope& operator=(const SocketReadScope&) = delete;
};

inline void traceCompletedPaint() {
    if (std::getenv("AUTODRIVE_SUMO_GUI_PAINT_TRACE") == nullptr) return;
    const auto now = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
    std::fprintf(stderr, "AUTODRIVE_GUI_PAINT %lld\n", static_cast<long long>(now));
}
}
