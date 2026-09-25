#ifndef HOST_ESP_LOG_STUB_H
#define HOST_ESP_LOG_STUB_H 1
/** @file esp_log.h  @brief Host stand-in: there is no IDF log to silence. */
typedef enum { ESP_LOG_NONE = 0 } esp_log_level_t;
inline void esp_log_level_set(const char *, esp_log_level_t) {}
#endif
