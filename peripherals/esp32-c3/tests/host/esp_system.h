#ifndef HOST_ESP_SYSTEM_STUB_H
#define HOST_ESP_SYSTEM_STUB_H 1
/** @file esp_system.h  @brief Host stand-in: every host boot is a power-on. */
typedef enum { ESP_RST_POWERON = 1 } esp_reset_reason_t;
inline esp_reset_reason_t esp_reset_reason() { return ESP_RST_POWERON; }
#endif
