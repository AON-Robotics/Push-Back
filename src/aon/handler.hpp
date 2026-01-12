#pragma once

#include <queue>
#include <functional>
#include <atomic>

#include "pros/rtos.hpp"              // pros::Mutex
#include "commparser/CommandParser.hpp"

namespace aon {

    // Estado/debug
    extern std::atomic<bool> commandBusy;
    
    // Public API (para mantener main clean)
    void init();                 // arranca el worker
    void poll();                 // lee una línea "live" (edge-trigger) y la procesa
    void submit(const Command&); // encola acciones según cmd (no-cero)
    
    // (si quieres ver/usar el queue en otros lados)
    extern std::queue<std::function<void()>> instructionQueue;
    extern pros::Mutex instructionQueueMtx;
    
    } // namespace aon
