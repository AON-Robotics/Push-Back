#include "handler.hpp"

#include "api.h"
#include "pros/rtos.hpp"
#include <cstdio>
#include <unordered_map>
#include <string_view>

namespace aon {

// ===== Shared state =====
std::queue<std::function<void()>> instructionQueue;
pros::Mutex instructionQueueMtx;
std::atomic<bool> commandBusy{false};

// ===== RX buffer (para poll) =====
static constexpr int BUFSZ = 128;
static char rxBuf[BUFSZ];
static int rxIdx = 0;

// ===== Reflection (escalable a N commands) =====
struct FieldRef {
  const char* name;
  float Command::* member;
};

static constexpr FieldRef kFields[] = {
  {"Move", &Command::Move},
  {"Turn", &Command::Turn},
  // {"Lift", &Command::Lift},
  // ...
};

// ===== Debug stubs (reemplazas luego por tus funciones reales) =====
static void move_debug(float v) {
  commandBusy.store(true);
  pros::lcd::print(2, "EXEC: Move %.2f", v);
  printf("EXEC: Move %.2f\n", v);
  pros::delay(5000);
  pros::lcd::print(2, "DONE: Move %.2f", v);
  printf("DONE: Move %.2f\n", v);
  commandBusy.store(false);
}

static void turn_debug(float v) {
  commandBusy.store(true);
  pros::lcd::print(2, "EXEC: Turn %.2f", v);
  printf("EXEC: Turn %.2f\n", v);
  pros::delay(5000);
  pros::lcd::print(2, "DONE: Turn %.2f", v);
  printf("DONE: Turn %.2f\n", v);
  commandBusy.store(false);
}

// ===== Handlers: name -> enqueue job =====
static const std::unordered_map<std::string_view, std::function<void(float)>> handlers = {
  {"Move", [](float v) {
      if (instructionQueueMtx.take(20)) {
        instructionQueue.push([=]{ move_debug(v); });
        instructionQueueMtx.give();
      } else {
        printf("WARN: mutex timeout enqueue Move\n");
      }
  }},
  {"Turn", [](float v) {
      if (instructionQueueMtx.take(20)) {
        instructionQueue.push([=]{ turn_debug(v); });
        instructionQueueMtx.give();
      } else {
        printf("WARN: mutex timeout enqueue Turn\n");
      }
  }},
};

// ===== Worker: ejecuta secuencialmente =====
static void worker_fn(void*) {
  while (true) {
    std::function<void()> job;

    if (instructionQueueMtx.take(20)) {
      if (!instructionQueue.empty()) {
        job = std::move(instructionQueue.front());
        instructionQueue.pop();
      }
      instructionQueueMtx.give();
    } else {
      printf("WARN: mutex timeout worker\n");
    }

    if (job) job();
    else pros::delay(10);
  }
}

void init() {
  static pros::Task worker(worker_fn, nullptr, "cmd_worker");
}

// ===== Submit: edge-trigger (1 Command por línea) =====
void submit(const Command& cmd) {
  int enqueued = 0;

  for (const auto& f : kFields) {
    const float v = cmd.*(f.member);
    if (v == 0.0f) continue;

    auto it = handlers.find(f.name);
    if (it != handlers.end()) {
      it->second(v);
      enqueued++;
    } else {
      pros::lcd::print(5, "No handler: %s", f.name);
      printf("No handler: %s v=%.2f\n", f.name, v);
    }
  }

  if (instructionQueueMtx.take(20)) {
    pros::lcd::print(3, "Q:%d (+%d)", (int)instructionQueue.size(), enqueued);
    instructionQueueMtx.give();
  }
}

// ===== Poll: lee serial y cuando hay línea completa => parse => submit =====
void poll() {
  int c = getchar();              // USB serial stdin
  if (c == EOF) return;
  if (c == '\r') return;

  if (c == '\n') {
    rxBuf[rxIdx] = '\0';
    rxIdx = 0;

    pros::lcd::print(1, "RX: %s", rxBuf);
    printf("RX: %s\n", rxBuf);

    Command cmd = parseCommandKV(std::string(rxBuf));
    submit(cmd);

    pros::lcd::print(4, "BUSY:%d", (int)commandBusy.load());
    return;
  }

  if (rxIdx < BUFSZ - 1) {
    rxBuf[rxIdx++] = (char)c;
  } else {
    // overflow: resetea
    rxIdx = 0;
    pros::lcd::set_text(1, "RX overflow");
    printf("RX overflow\n");
  }
}

} // namespace aon
