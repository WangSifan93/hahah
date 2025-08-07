/******************************************************************************
 * Copyright 2018 The zpilot Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @log
 */

#ifndef CYBER_COMMON_LOG_H_
#define CYBER_COMMON_LOG_H_

#include <cstdarg>
#include <string>

#include "cheryos/zos/zlog/zlog.h"
// #include "apps/planning/src/common/macros.h"
#include "glog/logging.h"
#include "glog/raw_logging.h"

#define CL_RED "\e[1;31m"
#define CL_GREEN "\e[0;32m"
#define CL_YELLOW "\e[1;33m"
#define CL_BLUE "\e[1;34m"
#define CL_PURPLE "\e[1;35m"
#define CL_CYAN "\e[0;36m"
#define CL_BRED "\e[1;91m"
#define CL_RESET "\e[0m"

#define BOLD_ON "\e[1m"
#define ITALIC_ON "\e[3m"

#define CHECK_NULLPTR(val) CHECK_NOTNULL(val)

#ifndef INIT_ZLOG
#define INIT_ZLOG zpilot::zlog::init_zlog()
#endif

#ifndef DEINIT_ZLOG
#define DEINIT_ZLOG zpilot::zlog::deinit_zlog();
#endif

#ifndef UNUSED
#define UNUSED(x) (void)x
#endif

#ifndef ZLOG_DEBUG
#define ZLOG_DEBUG ZSLOG_DEBUG
#define ZLOG_INFO ZSLOG_INFO
#define ZLOG_WARNING ZSLOG_WARNING
#define ZLOG_ERROR ZSLOG_ERROR
#define ZLOG_FATAL ZSLOG_FATAL
#endif

#ifndef ZSLOG_EVERY_N
#define ZSLOG_EVERY_N(severity, interval) LOG_EVERY_N(severity, interval)
#define ZSLOG_INFO_EVERY_N(interval) ZSLOG_EVERY_N(INFO, interval)
#endif

#define DEBUG_VPRINT(var) std::cout << #var << ": " << var << std::endl;

#ifndef PROHIBIT_MOVE_AND_COPY

#define PROHIBIT_MOVE_AND_COPY(classname)           \
  classname(const classname &) = delete;            \
  classname &operator=(const classname &) = delete; \
  classname(classname &&) = delete;                 \
  classname &operator=(classname &&) = delete;

#endif

#define ZLOG_MODULE

#define LEFT_BRACKET "["
#define RIGHT_BRACKET "]"

#ifndef MODULE_NAME
// #define MODULE_NAME zark::cyber::binary::GetName().c_str()  //todo cyber
#define MODULE_NAME "planning"
#endif

#define ADEBUG ADEBUG_MODULE(MODULE_NAME)
#define AINFO ALOG_MODULE(MODULE_NAME, INFO)
#define AWARN ALOG_MODULE(MODULE_NAME, WARN)
#define AERROR ALOG_MODULE(MODULE_NAME, ERROR)
#define AFATAL ALOG_MODULE(MODULE_NAME, FATAL)

#ifndef ALOG_MODULE_STREAM
#define ALOG_MODULE_STREAM(log_severity) ALOG_MODULE_STREAM_##log_severity
#endif

#ifndef ALOG_MODULE
#define ALOG_MODULE(module, log_severity) \
  ALOG_MODULE_STREAM(log_severity)(module)
#endif

#ifdef ZLOG_MODULE
#define ADEBUG_MODULE(module) ZLOG_DEBUG
#define ALOG_MODULE_STREAM_INFO(module) ZLOG_INFO
#define ALOG_MODULE_STREAM_WARN(module) ZLOG_WARNING
#define ALOG_MODULE_STREAM_ERROR(module) ZLOG_ERROR
#define ALOG_MODULE_STREAM_FATAL(module) ZLOG_FATAL
#define ALOG_IF(severity, cond, module) \
  !(cond) ? 0 : ALOG_MODULE(module, severity)
#else
#define ADEBUG_MODULE(module) \
  VLOG(4) << LEFT_BRACKET << module << RIGHT_BRACKET << "[DEBUG] "
#define ALOG_MODULE_STREAM_INFO(module)                         \
  google::LogMessage(__FILE__, __LINE__, google::INFO).stream() \
      << LEFT_BRACKET << module << RIGHT_BRACKET

#define ALOG_MODULE_STREAM_WARN(module)                            \
  google::LogMessage(__FILE__, __LINE__, google::WARNING).stream() \
      << LEFT_BRACKET << module << RIGHT_BRACKET

#define ALOG_MODULE_STREAM_ERROR(module)                         \
  google::LogMessage(__FILE__, __LINE__, google::ERROR).stream() \
      << LEFT_BRACKET << module << RIGHT_BRACKET

#define ALOG_MODULE_STREAM_FATAL(module)                         \
  google::LogMessage(__FILE__, __LINE__, google::FATAL).stream() \
      << LEFT_BRACKET << module << RIGHT_BRACKET
#define ALOG_IF(severity, cond, module) \
  !(cond) ? (void)0                     \
          : google::LogMessageVoidify() & ALOG_MODULE(module, severity)
#endif

#define AINFO_IF(cond) ALOG_IF(INFO, cond, MODULE_NAME)
#define AWARN_IF(cond) ALOG_IF(WARN, cond, MODULE_NAME)
#define AERROR_IF(cond) ALOG_IF(ERROR, cond, MODULE_NAME)
#define AFATAL_IF(cond) ALOG_IF(FATAL, cond, MODULE_NAME)

#define ACHECK(cond) CHECK(cond) << LEFT_BRACKET << MODULE_NAME << RIGHT_BRACKET

#define AINFO_EVERY(freq) \
  LOG_EVERY_N(INFO, freq) << LEFT_BRACKET << MODULE_NAME << RIGHT_BRACKET
#define AWARN_EVERY(freq) \
  LOG_EVERY_N(WARNING, freq) << LEFT_BRACKET << MODULE_NAME << RIGHT_BRACKET
#define AERROR_EVERY(freq) \
  LOG_EVERY_N(ERROR, freq) << LEFT_BRACKET << MODULE_NAME << RIGHT_BRACKET

#if !defined(RETURN_IF_NULL)
#define RETURN_IF_NULL(ptr)          \
  if (ptr == nullptr) {              \
    AWARN << #ptr << " is nullptr."; \
    return;                          \
  }
#endif

#if !defined(RETURN_VAL_IF_NULL)
#define RETURN_VAL_IF_NULL(ptr, val) \
  if (ptr == nullptr) {              \
    AWARN << #ptr << " is nullptr."; \
    return val;                      \
  }
#endif

#if !defined(RETURN_IF)
#define RETURN_IF(condition)           \
  if (condition) {                     \
    AWARN << #condition << " is met."; \
    return;                            \
  }
#endif

#if !defined(RETURN_VAL_IF)
#define RETURN_VAL_IF(condition, val)  \
  if (condition) {                     \
    AWARN << #condition << " is met."; \
    return val;                        \
  }
#endif

#if !defined(_RETURN_VAL_IF_NULL2__)
#define _RETURN_VAL_IF_NULL2__
#define RETURN_VAL_IF_NULL2(ptr, val) \
  if (ptr == nullptr) {               \
    return (val);                     \
  }
#endif

#if !defined(_RETURN_VAL_IF2__)
#define _RETURN_VAL_IF2__
#define RETURN_VAL_IF2(condition, val) \
  if (condition) {                     \
    return (val);                      \
  }
#endif

#if !defined(_RETURN_IF2__)
#define _RETURN_IF2__
#define RETURN_IF2(condition) \
  if (condition) {            \
    return;                   \
  }
#endif

#define ZCHECK(cond) ZCheck(cond, #cond, __FILE__, __LINE__)
#define ZCHECK_EQ(val1, val2) \
  ZCheckEq(val1, val2, #val1, #val2, __FILE__, __LINE__)
#define ZCHECK_NE(val1, val2) \
  ZCheckNe(val1, val2, #val1, #val2, __FILE__, __LINE__)
#define ZCHECK_LE(val1, val2) \
  ZCheckLe(val1, val2, #val1, #val2, __FILE__, __LINE__)
#define ZCHECK_LT(val1, val2) \
  ZCheckLt(val1, val2, #val1, #val2, __FILE__, __LINE__)
#define ZCHECK_GE(val1, val2) \
  ZCheckGe(val1, val2, #val1, #val2, __FILE__, __LINE__)
#define ZCHECK_GT(val1, val2) \
  ZCheckGt(val1, val2, #val1, #val2, __FILE__, __LINE__)
#define ZCHECK_NOTNULL(val) ZCheckNotNull(val, #val, __FILE__, __LINE__)

template <typename T>
inline bool ZCheck(T condition, const char *name, const char *file, int line) {
  if (!(condition)) {
    AERROR << file << ":" << line << ": "
           << "Check failed: " << name << " !!!!!";
    return false;
  }
  return true;
}

template <typename T1, typename T2>
inline bool ZCheckEq(T1 val1, T2 val2, const char *name1, const char *name2,
                     const char *file, int line) {
  if (!(val1 == val2)) {
    AERROR << file << ":" << line << ": "
           << "Check failed: " << name1 << " == " << name2 << " !!!!!";
    ;
    return false;
  }
  return true;
}

template <typename T1, typename T2>
inline bool ZCheckNe(T1 val1, T2 val2, const char *name1, const char *name2,
                     const char *file, int line) {
  if (!(val1 != val2)) {
    AERROR << file << ":" << line << ": "
           << "Check failed: " << name1 << " != " << name2 << " !!!!!";
    ;
    return false;
  }
  return true;
}

template <typename T1, typename T2>
inline bool ZCheckLe(T1 val1, T2 val2, const char *name1, const char *name2,
                     const char *file, int line) {
  if (!(val1 <= val2)) {
    AERROR << file << ":" << line << ": "
           << "Check failed: " << name1 << " <= " << name2 << " !!!!!";
    ;
    return false;
  }
  return true;
}

template <typename T1, typename T2>
inline bool ZCheckLt(T1 val1, T2 val2, const char *name1, const char *name2,
                     const char *file, int line) {
  if (!(val1 < val2)) {
    AERROR << file << ":" << line << ": "
           << "Check failed: " << name1 << " < " << name2 << " !!!!!";
    ;
    return false;
  }
  return true;
}

template <typename T1, typename T2>
inline bool ZCheckGe(T1 val1, T2 val2, const char *name1, const char *name2,
                     const char *file, int line) {
  if (!(val1 >= val2)) {
    AERROR << file << ":" << line << ": "
           << "Check failed: " << name1 << " >= " << name2 << " !!!!!";
    ;
    return false;
  }
  return true;
}

template <typename T1, typename T2>
inline bool ZCheckGt(T1 val1, T2 val2, const char *name1, const char *name2,
                     const char *file, int line) {
  if (!(val1 > val2)) {
    AERROR << file << ":" << line << ": "
           << "Check failed: " << name1 << " > " << name2 << " !!!!!";
    ;
    return false;
  }
  return true;
}

template <typename T>
inline bool ZCheckNotNull(T val, const char *name, const char *file, int line) {
  if (!val) {
    AERROR << file << ":" << line << ": "
           << "Check failed: " << name << " must be not null !!!!! ";
    return false;
  }
  return true;
}

#endif  // CYBER_COMMON_LOG_H_
