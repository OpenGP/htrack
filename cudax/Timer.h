/// @note this measures wall clock times, not GPU times!!
/// You'd need to sync the GPU (at the cost of a stall) to get
/// the real times
#pragma once

#include <ctime>
#include <string>
#include <cstdarg>
#include <cstdio>

//== NAMESPACE ================================================================
namespace cudax {
//=============================================================================

struct Timer {
    clock_t start;
    clock_t end;
    static const size_t buffer_length=256;
    char buffer[buffer_length];

    Timer(void) {
        restart("");
        //std::cout << "!!! WARNING: obsolete, why are you using it?" << std::endl;
    }

    void restart(const char* format, ...) {
        va_list args;
        va_start (args, format);
        /// @note http://www.tin.org/bin/man.cgi?section=3&topic=vsnprintf
        vsnprintf (buffer, buffer_length, format, args);
        va_end (args);
        start = clock();
    }

    double elapsed(void) {
        end = clock();
        return static_cast<double>(end - start) / static_cast<double>(CLOCKS_PER_SEC);
    }

    double epsilon(void) {
        return 1.0 / static_cast<double>(CLOCKS_PER_SEC);
    }

    void display() {
        #ifndef DISABLE_TIMER_PRINTF
        printf("[%s]: %.2fms\n",buffer,1e3 * elapsed());
        fflush(stdout);
        #endif
    }
};

//=============================================================================
} // namespace cudax
//=============================================================================
