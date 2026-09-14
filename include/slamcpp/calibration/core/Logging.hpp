// A small logger, replacing sm_logging.
//
// Upstream is ~2,300 lines, most of it a generated cross product of macro
// variants (plain/stream x named/cond/once/throttle) over eleven severity
// levels, plus a pluggable Logger hierarchy and a printf-style Formatter with
// its own token parser. This keeps the parts that ported call sites actually
// use and drops the rest:
//
//   Kept    the eleven levels and their ordering, named streams, the stream
//           macros, and the COND / ONCE / THROTTLE variants, because upstream
//           calibration code uses all of them.
//   Kept    compile-time severity elision via CALIB_LOG_MIN_SEVERITY, so a
//           release build pays nothing for verbose logging.
//   Dropped the Logger class hierarchy, replaced by one std::function sink.
//   Dropped the printf-style forms and the Formatter token language. Every
//           upstream printf call site has a stream equivalent.
//
// The sink is called under a mutex, so it need not be thread-safe itself.
#ifndef CALIB_CORE_LOGGING_HPP
#define CALIB_CORE_LOGGING_HPP

#include <atomic>
#include <cstdint>
#include <functional>
#include <sstream>
#include <string>

#define CALIB_SEVERITY_ALL 0
#define CALIB_SEVERITY_FINEST 1
#define CALIB_SEVERITY_VERBOSE 2
#define CALIB_SEVERITY_FINER 3
#define CALIB_SEVERITY_TRACE 4
#define CALIB_SEVERITY_FINE 5
#define CALIB_SEVERITY_DEBUG 6
#define CALIB_SEVERITY_INFO 7
#define CALIB_SEVERITY_WARN 8
#define CALIB_SEVERITY_ERROR 9
#define CALIB_SEVERITY_FATAL 10
#define CALIB_SEVERITY_NONE 11

// Everything below this is removed by the preprocessor. Debug builds default to
// keeping all levels; release builds keep Info and above.
#ifndef CALIB_LOG_MIN_SEVERITY
#ifdef NDEBUG
#define CALIB_LOG_MIN_SEVERITY CALIB_SEVERITY_INFO
#else
#define CALIB_LOG_MIN_SEVERITY CALIB_SEVERITY_ALL
#endif
#endif

namespace slamcpp {
namespace calib {
namespace logging {

// Same ordering as sm::logging::levels::Level. Ported code that compares or
// stores levels keeps working.
enum class Level {
  All = 0,
  Finest,
  Verbose,
  Finer,
  Trace,
  Fine,
  Debug,
  Info,
  Warn,
  Error,
  Fatal,
  Count
};

struct Event {
  const char* stream;
  Level level;
  const char* file;
  int line;
  const char* function;
  std::string message;
  double wallSeconds;
};

using Sink = std::function<void(const Event&)>;

const char* levelName(Level level);

// Messages below this level are dropped at run time. Defaults to Info.
void setLevel(Level level);
Level level();

// Replace the sink. Passing nullptr restores the built-in one, which writes a
// single formatted line per event to stderr.
void setSink(Sink sink);

// True when a message at this level would be emitted. The macros test this
// before building the stringstream, so a suppressed call costs one comparison.
inline bool enabled(Level l) { return l >= level(); }

// Monotonic seconds since process start, used by the THROTTLE macros.
double now();

void log(const char* stream, Level level, const char* file, int line,
         const char* function, std::string message);

}  // namespace logging
}  // namespace calib
}  // namespace slamcpp

// ---------------------------------------------------------------------------
// Macro plumbing.
// ---------------------------------------------------------------------------
#define CALIB_LOG_COND_NAMED(cond, level, name, args)                    \
  do {                                                                    \
    if ((cond) && ::slamcpp::calib::logging::enabled(level)) {                    \
      std::stringstream calib_log_ss;                                    \
      calib_log_ss << args;                                              \
      ::slamcpp::calib::logging::log(name, level, __FILE__, __LINE__,             \
                             __FUNCTION__, calib_log_ss.str());          \
    }                                                                     \
  } while (0)

#define CALIB_LOG_ONCE_NAMED(level, name, args)                          \
  do {                                                                    \
    static std::atomic<bool> calib_log_done{false};                      \
    if (!calib_log_done.exchange(true)) {                                \
      CALIB_LOG_COND_NAMED(true, level, name, args);                     \
    }                                                                     \
  } while (0)

#define CALIB_LOG_THROTTLE_NAMED(period, level, name, args)              \
  do {                                                                    \
    static double calib_log_last = -1.0;                                 \
    const double calib_log_now = ::slamcpp::calib::logging::now();               \
    if (calib_log_last < 0.0 ||                                          \
        calib_log_now - calib_log_last >= (period)) {                   \
      calib_log_last = calib_log_now;                                   \
      CALIB_LOG_COND_NAMED(true, level, name, args);                     \
    }                                                                     \
  } while (0)

// ---------------------------------------------------------------------------
// Per-level macros. Eight variants per level, matching the upstream spelling
// so ported call sites need only the SM_ -> CALIB_ rename. Levels below
// CALIB_LOG_MIN_SEVERITY expand to nothing, and the arguments are never
// evaluated, exactly as upstream's generated macros behave.
// ---------------------------------------------------------------------------

#if CALIB_LOG_MIN_SEVERITY <= CALIB_SEVERITY_FINEST
#define CALIB_FINEST_STREAM_COND_NAMED(cond, name, args) \
  CALIB_LOG_COND_NAMED(cond, ::slamcpp::calib::logging::Level::Finest, name, args)
#define CALIB_FINEST_STREAM_ONCE_NAMED(name, args) \
  CALIB_LOG_ONCE_NAMED(::slamcpp::calib::logging::Level::Finest, name, args)
#define CALIB_FINEST_STREAM_THROTTLE_NAMED(period, name, args) \
  CALIB_LOG_THROTTLE_NAMED(period, ::slamcpp::calib::logging::Level::Finest, name, args)
#else
#define CALIB_FINEST_STREAM_COND_NAMED(cond, name, args) static_cast<void>(0)
#define CALIB_FINEST_STREAM_ONCE_NAMED(name, args) static_cast<void>(0)
#define CALIB_FINEST_STREAM_THROTTLE_NAMED(period, name, args) static_cast<void>(0)
#endif
#define CALIB_FINEST_STREAM(args) \
  CALIB_FINEST_STREAM_COND_NAMED(true, "", args)
#define CALIB_FINEST_STREAM_NAMED(name, args) \
  CALIB_FINEST_STREAM_COND_NAMED(true, name, args)
#define CALIB_FINEST_STREAM_COND(cond, args) \
  CALIB_FINEST_STREAM_COND_NAMED(cond, "", args)
#define CALIB_FINEST_STREAM_ONCE(args) \
  CALIB_FINEST_STREAM_ONCE_NAMED("", args)
#define CALIB_FINEST_STREAM_THROTTLE(period, args) \
  CALIB_FINEST_STREAM_THROTTLE_NAMED(period, "", args)

#if CALIB_LOG_MIN_SEVERITY <= CALIB_SEVERITY_VERBOSE
#define CALIB_VERBOSE_STREAM_COND_NAMED(cond, name, args) \
  CALIB_LOG_COND_NAMED(cond, ::slamcpp::calib::logging::Level::Verbose, name, args)
#define CALIB_VERBOSE_STREAM_ONCE_NAMED(name, args) \
  CALIB_LOG_ONCE_NAMED(::slamcpp::calib::logging::Level::Verbose, name, args)
#define CALIB_VERBOSE_STREAM_THROTTLE_NAMED(period, name, args) \
  CALIB_LOG_THROTTLE_NAMED(period, ::slamcpp::calib::logging::Level::Verbose, name, args)
#else
#define CALIB_VERBOSE_STREAM_COND_NAMED(cond, name, args) static_cast<void>(0)
#define CALIB_VERBOSE_STREAM_ONCE_NAMED(name, args) static_cast<void>(0)
#define CALIB_VERBOSE_STREAM_THROTTLE_NAMED(period, name, args) static_cast<void>(0)
#endif
#define CALIB_VERBOSE_STREAM(args) \
  CALIB_VERBOSE_STREAM_COND_NAMED(true, "", args)
#define CALIB_VERBOSE_STREAM_NAMED(name, args) \
  CALIB_VERBOSE_STREAM_COND_NAMED(true, name, args)
#define CALIB_VERBOSE_STREAM_COND(cond, args) \
  CALIB_VERBOSE_STREAM_COND_NAMED(cond, "", args)
#define CALIB_VERBOSE_STREAM_ONCE(args) \
  CALIB_VERBOSE_STREAM_ONCE_NAMED("", args)
#define CALIB_VERBOSE_STREAM_THROTTLE(period, args) \
  CALIB_VERBOSE_STREAM_THROTTLE_NAMED(period, "", args)

#if CALIB_LOG_MIN_SEVERITY <= CALIB_SEVERITY_FINER
#define CALIB_FINER_STREAM_COND_NAMED(cond, name, args) \
  CALIB_LOG_COND_NAMED(cond, ::slamcpp::calib::logging::Level::Finer, name, args)
#define CALIB_FINER_STREAM_ONCE_NAMED(name, args) \
  CALIB_LOG_ONCE_NAMED(::slamcpp::calib::logging::Level::Finer, name, args)
#define CALIB_FINER_STREAM_THROTTLE_NAMED(period, name, args) \
  CALIB_LOG_THROTTLE_NAMED(period, ::slamcpp::calib::logging::Level::Finer, name, args)
#else
#define CALIB_FINER_STREAM_COND_NAMED(cond, name, args) static_cast<void>(0)
#define CALIB_FINER_STREAM_ONCE_NAMED(name, args) static_cast<void>(0)
#define CALIB_FINER_STREAM_THROTTLE_NAMED(period, name, args) static_cast<void>(0)
#endif
#define CALIB_FINER_STREAM(args) \
  CALIB_FINER_STREAM_COND_NAMED(true, "", args)
#define CALIB_FINER_STREAM_NAMED(name, args) \
  CALIB_FINER_STREAM_COND_NAMED(true, name, args)
#define CALIB_FINER_STREAM_COND(cond, args) \
  CALIB_FINER_STREAM_COND_NAMED(cond, "", args)
#define CALIB_FINER_STREAM_ONCE(args) \
  CALIB_FINER_STREAM_ONCE_NAMED("", args)
#define CALIB_FINER_STREAM_THROTTLE(period, args) \
  CALIB_FINER_STREAM_THROTTLE_NAMED(period, "", args)

#if CALIB_LOG_MIN_SEVERITY <= CALIB_SEVERITY_TRACE
#define CALIB_TRACE_STREAM_COND_NAMED(cond, name, args) \
  CALIB_LOG_COND_NAMED(cond, ::slamcpp::calib::logging::Level::Trace, name, args)
#define CALIB_TRACE_STREAM_ONCE_NAMED(name, args) \
  CALIB_LOG_ONCE_NAMED(::slamcpp::calib::logging::Level::Trace, name, args)
#define CALIB_TRACE_STREAM_THROTTLE_NAMED(period, name, args) \
  CALIB_LOG_THROTTLE_NAMED(period, ::slamcpp::calib::logging::Level::Trace, name, args)
#else
#define CALIB_TRACE_STREAM_COND_NAMED(cond, name, args) static_cast<void>(0)
#define CALIB_TRACE_STREAM_ONCE_NAMED(name, args) static_cast<void>(0)
#define CALIB_TRACE_STREAM_THROTTLE_NAMED(period, name, args) static_cast<void>(0)
#endif
#define CALIB_TRACE_STREAM(args) \
  CALIB_TRACE_STREAM_COND_NAMED(true, "", args)
#define CALIB_TRACE_STREAM_NAMED(name, args) \
  CALIB_TRACE_STREAM_COND_NAMED(true, name, args)
#define CALIB_TRACE_STREAM_COND(cond, args) \
  CALIB_TRACE_STREAM_COND_NAMED(cond, "", args)
#define CALIB_TRACE_STREAM_ONCE(args) \
  CALIB_TRACE_STREAM_ONCE_NAMED("", args)
#define CALIB_TRACE_STREAM_THROTTLE(period, args) \
  CALIB_TRACE_STREAM_THROTTLE_NAMED(period, "", args)

#if CALIB_LOG_MIN_SEVERITY <= CALIB_SEVERITY_FINE
#define CALIB_FINE_STREAM_COND_NAMED(cond, name, args) \
  CALIB_LOG_COND_NAMED(cond, ::slamcpp::calib::logging::Level::Fine, name, args)
#define CALIB_FINE_STREAM_ONCE_NAMED(name, args) \
  CALIB_LOG_ONCE_NAMED(::slamcpp::calib::logging::Level::Fine, name, args)
#define CALIB_FINE_STREAM_THROTTLE_NAMED(period, name, args) \
  CALIB_LOG_THROTTLE_NAMED(period, ::slamcpp::calib::logging::Level::Fine, name, args)
#else
#define CALIB_FINE_STREAM_COND_NAMED(cond, name, args) static_cast<void>(0)
#define CALIB_FINE_STREAM_ONCE_NAMED(name, args) static_cast<void>(0)
#define CALIB_FINE_STREAM_THROTTLE_NAMED(period, name, args) static_cast<void>(0)
#endif
#define CALIB_FINE_STREAM(args) \
  CALIB_FINE_STREAM_COND_NAMED(true, "", args)
#define CALIB_FINE_STREAM_NAMED(name, args) \
  CALIB_FINE_STREAM_COND_NAMED(true, name, args)
#define CALIB_FINE_STREAM_COND(cond, args) \
  CALIB_FINE_STREAM_COND_NAMED(cond, "", args)
#define CALIB_FINE_STREAM_ONCE(args) \
  CALIB_FINE_STREAM_ONCE_NAMED("", args)
#define CALIB_FINE_STREAM_THROTTLE(period, args) \
  CALIB_FINE_STREAM_THROTTLE_NAMED(period, "", args)

#if CALIB_LOG_MIN_SEVERITY <= CALIB_SEVERITY_DEBUG
#define CALIB_DEBUG_STREAM_COND_NAMED(cond, name, args) \
  CALIB_LOG_COND_NAMED(cond, ::slamcpp::calib::logging::Level::Debug, name, args)
#define CALIB_DEBUG_STREAM_ONCE_NAMED(name, args) \
  CALIB_LOG_ONCE_NAMED(::slamcpp::calib::logging::Level::Debug, name, args)
#define CALIB_DEBUG_STREAM_THROTTLE_NAMED(period, name, args) \
  CALIB_LOG_THROTTLE_NAMED(period, ::slamcpp::calib::logging::Level::Debug, name, args)
#else
#define CALIB_DEBUG_STREAM_COND_NAMED(cond, name, args) static_cast<void>(0)
#define CALIB_DEBUG_STREAM_ONCE_NAMED(name, args) static_cast<void>(0)
#define CALIB_DEBUG_STREAM_THROTTLE_NAMED(period, name, args) static_cast<void>(0)
#endif
#define CALIB_DEBUG_STREAM(args) \
  CALIB_DEBUG_STREAM_COND_NAMED(true, "", args)
#define CALIB_DEBUG_STREAM_NAMED(name, args) \
  CALIB_DEBUG_STREAM_COND_NAMED(true, name, args)
#define CALIB_DEBUG_STREAM_COND(cond, args) \
  CALIB_DEBUG_STREAM_COND_NAMED(cond, "", args)
#define CALIB_DEBUG_STREAM_ONCE(args) \
  CALIB_DEBUG_STREAM_ONCE_NAMED("", args)
#define CALIB_DEBUG_STREAM_THROTTLE(period, args) \
  CALIB_DEBUG_STREAM_THROTTLE_NAMED(period, "", args)

#if CALIB_LOG_MIN_SEVERITY <= CALIB_SEVERITY_INFO
#define CALIB_INFO_STREAM_COND_NAMED(cond, name, args) \
  CALIB_LOG_COND_NAMED(cond, ::slamcpp::calib::logging::Level::Info, name, args)
#define CALIB_INFO_STREAM_ONCE_NAMED(name, args) \
  CALIB_LOG_ONCE_NAMED(::slamcpp::calib::logging::Level::Info, name, args)
#define CALIB_INFO_STREAM_THROTTLE_NAMED(period, name, args) \
  CALIB_LOG_THROTTLE_NAMED(period, ::slamcpp::calib::logging::Level::Info, name, args)
#else
#define CALIB_INFO_STREAM_COND_NAMED(cond, name, args) static_cast<void>(0)
#define CALIB_INFO_STREAM_ONCE_NAMED(name, args) static_cast<void>(0)
#define CALIB_INFO_STREAM_THROTTLE_NAMED(period, name, args) static_cast<void>(0)
#endif
#define CALIB_INFO_STREAM(args) \
  CALIB_INFO_STREAM_COND_NAMED(true, "", args)
#define CALIB_INFO_STREAM_NAMED(name, args) \
  CALIB_INFO_STREAM_COND_NAMED(true, name, args)
#define CALIB_INFO_STREAM_COND(cond, args) \
  CALIB_INFO_STREAM_COND_NAMED(cond, "", args)
#define CALIB_INFO_STREAM_ONCE(args) \
  CALIB_INFO_STREAM_ONCE_NAMED("", args)
#define CALIB_INFO_STREAM_THROTTLE(period, args) \
  CALIB_INFO_STREAM_THROTTLE_NAMED(period, "", args)

#if CALIB_LOG_MIN_SEVERITY <= CALIB_SEVERITY_WARN
#define CALIB_WARN_STREAM_COND_NAMED(cond, name, args) \
  CALIB_LOG_COND_NAMED(cond, ::slamcpp::calib::logging::Level::Warn, name, args)
#define CALIB_WARN_STREAM_ONCE_NAMED(name, args) \
  CALIB_LOG_ONCE_NAMED(::slamcpp::calib::logging::Level::Warn, name, args)
#define CALIB_WARN_STREAM_THROTTLE_NAMED(period, name, args) \
  CALIB_LOG_THROTTLE_NAMED(period, ::slamcpp::calib::logging::Level::Warn, name, args)
#else
#define CALIB_WARN_STREAM_COND_NAMED(cond, name, args) static_cast<void>(0)
#define CALIB_WARN_STREAM_ONCE_NAMED(name, args) static_cast<void>(0)
#define CALIB_WARN_STREAM_THROTTLE_NAMED(period, name, args) static_cast<void>(0)
#endif
#define CALIB_WARN_STREAM(args) \
  CALIB_WARN_STREAM_COND_NAMED(true, "", args)
#define CALIB_WARN_STREAM_NAMED(name, args) \
  CALIB_WARN_STREAM_COND_NAMED(true, name, args)
#define CALIB_WARN_STREAM_COND(cond, args) \
  CALIB_WARN_STREAM_COND_NAMED(cond, "", args)
#define CALIB_WARN_STREAM_ONCE(args) \
  CALIB_WARN_STREAM_ONCE_NAMED("", args)
#define CALIB_WARN_STREAM_THROTTLE(period, args) \
  CALIB_WARN_STREAM_THROTTLE_NAMED(period, "", args)

#if CALIB_LOG_MIN_SEVERITY <= CALIB_SEVERITY_ERROR
#define CALIB_ERROR_STREAM_COND_NAMED(cond, name, args) \
  CALIB_LOG_COND_NAMED(cond, ::slamcpp::calib::logging::Level::Error, name, args)
#define CALIB_ERROR_STREAM_ONCE_NAMED(name, args) \
  CALIB_LOG_ONCE_NAMED(::slamcpp::calib::logging::Level::Error, name, args)
#define CALIB_ERROR_STREAM_THROTTLE_NAMED(period, name, args) \
  CALIB_LOG_THROTTLE_NAMED(period, ::slamcpp::calib::logging::Level::Error, name, args)
#else
#define CALIB_ERROR_STREAM_COND_NAMED(cond, name, args) static_cast<void>(0)
#define CALIB_ERROR_STREAM_ONCE_NAMED(name, args) static_cast<void>(0)
#define CALIB_ERROR_STREAM_THROTTLE_NAMED(period, name, args) static_cast<void>(0)
#endif
#define CALIB_ERROR_STREAM(args) \
  CALIB_ERROR_STREAM_COND_NAMED(true, "", args)
#define CALIB_ERROR_STREAM_NAMED(name, args) \
  CALIB_ERROR_STREAM_COND_NAMED(true, name, args)
#define CALIB_ERROR_STREAM_COND(cond, args) \
  CALIB_ERROR_STREAM_COND_NAMED(cond, "", args)
#define CALIB_ERROR_STREAM_ONCE(args) \
  CALIB_ERROR_STREAM_ONCE_NAMED("", args)
#define CALIB_ERROR_STREAM_THROTTLE(period, args) \
  CALIB_ERROR_STREAM_THROTTLE_NAMED(period, "", args)

#if CALIB_LOG_MIN_SEVERITY <= CALIB_SEVERITY_FATAL
#define CALIB_FATAL_STREAM_COND_NAMED(cond, name, args) \
  CALIB_LOG_COND_NAMED(cond, ::slamcpp::calib::logging::Level::Fatal, name, args)
#define CALIB_FATAL_STREAM_ONCE_NAMED(name, args) \
  CALIB_LOG_ONCE_NAMED(::slamcpp::calib::logging::Level::Fatal, name, args)
#define CALIB_FATAL_STREAM_THROTTLE_NAMED(period, name, args) \
  CALIB_LOG_THROTTLE_NAMED(period, ::slamcpp::calib::logging::Level::Fatal, name, args)
#else
#define CALIB_FATAL_STREAM_COND_NAMED(cond, name, args) static_cast<void>(0)
#define CALIB_FATAL_STREAM_ONCE_NAMED(name, args) static_cast<void>(0)
#define CALIB_FATAL_STREAM_THROTTLE_NAMED(period, name, args) static_cast<void>(0)
#endif
#define CALIB_FATAL_STREAM(args) \
  CALIB_FATAL_STREAM_COND_NAMED(true, "", args)
#define CALIB_FATAL_STREAM_NAMED(name, args) \
  CALIB_FATAL_STREAM_COND_NAMED(true, name, args)
#define CALIB_FATAL_STREAM_COND(cond, args) \
  CALIB_FATAL_STREAM_COND_NAMED(cond, "", args)
#define CALIB_FATAL_STREAM_ONCE(args) \
  CALIB_FATAL_STREAM_ONCE_NAMED("", args)
#define CALIB_FATAL_STREAM_THROTTLE(period, args) \
  CALIB_FATAL_STREAM_THROTTLE_NAMED(period, "", args)

#endif  // CALIB_CORE_LOGGING_HPP
