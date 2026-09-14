#include "slamcpp/calibration/core/Logging.hpp"

#include <chrono>
#include <cstdio>
#include <iomanip>
#include <mutex>
#include <ostream>

namespace slamcpp {
namespace calib {
namespace logging {
namespace {

std::mutex& sinkMutex() {
  static std::mutex m;
  return m;
}

Sink& sinkSlot() {
  static Sink sink;
  return sink;
}

std::atomic<Level>& levelSlot() {
  static std::atomic<Level> l{Level::Info};
  return l;
}

std::chrono::steady_clock::time_point startTime() {
  static const auto t0 = std::chrono::steady_clock::now();
  return t0;
}

// The built-in sink. One line per event: elapsed time, level, optional stream
// name, message, then the source location for anything at Warn or above.
void defaultSink(const Event& e) {
  std::stringstream ss;
  ss << '[' << std::fixed << std::setprecision(3) << e.wallSeconds << "] "
     << std::setw(7) << levelName(e.level);
  if (e.stream != nullptr && e.stream[0] != '\0') {
    ss << " (" << e.stream << ')';
  }
  ss << ' ' << e.message;
  if (e.level >= Level::Warn) {
    ss << "  [" << e.file << ':' << e.line << ": " << e.function << "()]";
  }
  ss << '\n';
  const std::string line = ss.str();
  std::fwrite(line.data(), 1, line.size(), stderr);
  std::fflush(stderr);
}

}  // namespace

const char* levelName(Level level) {
  switch (level) {
    case Level::All: return "ALL";
    case Level::Finest: return "FINEST";
    case Level::Verbose: return "VERBOSE";
    case Level::Finer: return "FINER";
    case Level::Trace: return "TRACE";
    case Level::Fine: return "FINE";
    case Level::Debug: return "DEBUG";
    case Level::Info: return "INFO";
    case Level::Warn: return "WARN";
    case Level::Error: return "ERROR";
    case Level::Fatal: return "FATAL";
    case Level::Count: break;
  }
  return "UNKNOWN";
}

void setLevel(Level l) { levelSlot().store(l, std::memory_order_relaxed); }

Level level() { return levelSlot().load(std::memory_order_relaxed); }

void setSink(Sink sink) {
  std::lock_guard<std::mutex> lock(sinkMutex());
  sinkSlot() = std::move(sink);
}

double now() {
  const auto d = std::chrono::steady_clock::now() - startTime();
  return std::chrono::duration<double>(d).count();
}

void log(const char* stream, Level level, const char* file, int line,
         const char* function, std::string message) {
  Event e{stream, level, file, line, function, std::move(message), now()};
  std::lock_guard<std::mutex> lock(sinkMutex());
  if (sinkSlot()) {
    sinkSlot()(e);
  } else {
    defaultSink(e);
  }
}

}  // namespace logging
}  // namespace calib
}  // namespace slamcpp
