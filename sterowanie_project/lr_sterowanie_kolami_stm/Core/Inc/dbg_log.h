/**
  * @file    dbg_log.h
  * @brief   Debug log interface. The USB-C debug path is not known yet, so
  *          the implementation is a stub.
  */
#ifndef DBG_LOG_H
#define DBG_LOG_H

void dbg_log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

#endif /* DBG_LOG_H */
