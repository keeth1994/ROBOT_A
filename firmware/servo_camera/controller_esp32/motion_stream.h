#pragma once
#include <cstdint>

struct MotionStream {
  bool active = false;
  uint32_t session = 0, sequence = 0, lastFrame = 0;
  void stop() { active = false; session = 0; }
  void begin(uint32_t id, uint32_t now) {
    session = id ? id : 1; sequence = 0; lastFrame = now; active = true;
  }
  bool accepts(uint32_t id, uint32_t next) const {
    return active && id == session && next > sequence;
  }
  void applied(uint32_t next, uint32_t now) { sequence = next; lastFrame = now; }
  bool expired(uint32_t now) const { return active && uint32_t(now - lastFrame) > 300; }
};
