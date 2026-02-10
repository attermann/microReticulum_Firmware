#undef LOG_ERROR
#undef LOG_WARN
#undef LOG_INFO
#undef LOG_DEBUG
#undef LOG_TRACE
#undef ASSERT
#undef ASSERTM

#define LOG_SHORT_FILENAME (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)

// C pre-proc Token Concatenation: https://wiki.sei.cmu.edu/confluence/display/c/PRE05-C.+Understand+macro+replacement+when+concatenating+tokens+or+performing+stringification
#define LOG_MACRO_APPEND_STR(s) LOG_HELPER_MACRO_APPEND_STR(s)
#define LOG_HELPER_MACRO_APPEND_STR(s) #s

#ifndef LOG_PREAMBLE
  #define LOG_PREAMBLE LOG_SHORT_FILENAME, LOG_MACRO_APPEND_STR(L.__LINE__), __func__, ":"
#endif

#if defined(DEBUGLOG_DEFAULT_LOG_LEVEL_ERROR)
  #define LOG_ERROR(...) DebugLog::Manager::get().log(arx::debug::LogLevel::LVL_ERROR, LOG_PREAMBLE, __VA_ARGS__)
  #define  LOG_WARN(...)
  #define  LOG_INFO(...)
  #define LOG_DEBUG(...)
  #define LOG_TRACE(...)
#elif defined(DEBUGLOG_DEFAULT_LOG_LEVEL_WARN)
  #define LOG_ERROR(...) DebugLog::Manager::get().log(arx::debug::LogLevel::LVL_ERROR, LOG_PREAMBLE, __VA_ARGS__)
  #define  LOG_WARN(...) DebugLog::Manager::get().log(arx::debug::LogLevel::LVL_WARN, LOG_PREAMBLE, __VA_ARGS__)
  #define  LOG_INFO(...)
  #define LOG_DEBUG(...)
  #define LOG_TRACE(...)
#elif defined(DEBUGLOG_DEFAULT_LOG_LEVEL_INFO)
  #define LOG_ERROR(...) DebugLog::Manager::get().log(arx::debug::LogLevel::LVL_ERROR, LOG_PREAMBLE, __VA_ARGS__)
  #define  LOG_WARN(...) DebugLog::Manager::get().log(arx::debug::LogLevel::LVL_WARN, LOG_PREAMBLE, __VA_ARGS__)
  #define  LOG_INFO(...) DebugLog::Manager::get().log(arx::debug::LogLevel::LVL_INFO, LOG_PREAMBLE, __VA_ARGS__)
  #define LOG_DEBUG(...)
  #define LOG_TRACE(...)
#elif defined(DEBUGLOG_DEFAULT_LOG_LEVEL_DEBUG)
  #define LOG_ERROR(...) DebugLog::Manager::get().log(arx::debug::LogLevel::LVL_ERROR, LOG_PREAMBLE, __VA_ARGS__)
  #define  LOG_WARN(...) DebugLog::Manager::get().log(arx::debug::LogLevel::LVL_WARN, LOG_PREAMBLE, __VA_ARGS__)
  #define  LOG_INFO(...) DebugLog::Manager::get().log(arx::debug::LogLevel::LVL_INFO, LOG_PREAMBLE, __VA_ARGS__)
  #define LOG_DEBUG(...) DebugLog::Manager::get().log(arx::debug::LogLevel::LVL_DEBUG, LOG_PREAMBLE, __VA_ARGS__)
  #define LOG_TRACE(...)
#elif defined(DEBUGLOG_DEFAULT_LOG_LEVEL_TRACE)
  #define LOG_ERROR(...) DebugLog::Manager::get().log(arx::debug::LogLevel::LVL_ERROR, LOG_PREAMBLE, __VA_ARGS__)
  #define  LOG_WARN(...) DebugLog::Manager::get().log(arx::debug::LogLevel::LVL_WARN, LOG_PREAMBLE, __VA_ARGS__)
  #define  LOG_INFO(...) DebugLog::Manager::get().log(arx::debug::LogLevel::LVL_INFO, LOG_PREAMBLE, __VA_ARGS__)
  #define LOG_DEBUG(...) DebugLog::Manager::get().log(arx::debug::LogLevel::LVL_DEBUG, LOG_PREAMBLE, __VA_ARGS__)
  #define LOG_TRACE(...) DebugLog::Manager::get().log(arx::debug::LogLevel::LVL_TRACE, LOG_PREAMBLE, __VA_ARGS__)
#else
  #warning "Defaulting to a log level of: DEBUGLOG_DEFAULT_LOG_LEVEL_TRACE"
  #define LOG_ERROR(...) DebugLog::Manager::get().log(arx::debug::LogLevel::LVL_ERROR, LOG_PREAMBLE, __VA_ARGS__)
  #define  LOG_WARN(...) DebugLog::Manager::get().log(arx::debug::LogLevel::LVL_WARN, LOG_PREAMBLE, __VA_ARGS__)
  #define  LOG_INFO(...) DebugLog::Manager::get().log(arx::debug::LogLevel::LVL_INFO, LOG_PREAMBLE, __VA_ARGS__)
  #define LOG_DEBUG(...) DebugLog::Manager::get().log(arx::debug::LogLevel::LVL_DEBUG, LOG_PREAMBLE, __VA_ARGS__)
  #define LOG_TRACE(...) DebugLog::Manager::get().log(arx::debug::LogLevel::LVL_TRACE, LOG_PREAMBLE, __VA_ARGS__)
#endif

#ifdef ARDUINO
#define ASSERT(b) DebugLog::Manager::get().assertion((b), LOG_SHORT_FILENAME, __LINE__, __func__, #b)
#define ASSERTM(b, msg) DebugLog::Manager::get().assertion((b), LOG_SHORT_FILENAME, __LINE__, __func__, #b, msg)
#else  // ARDUINO
#include <cassert>
#define ASSERT(b) assert(b)
#define ASSERTM(b, msg) assert((msg, b))
#endif  // ARDUINO
