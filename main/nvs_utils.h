#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t nvs_initialize(void);

/* store an arbitrary zero-terminated string */
esp_err_t nvs_save_string(const char *ns, const char *key, const char *value);

/* read a string back; buffer length is provided in maxlen */
esp_err_t nvs_load_string(const char *ns, const char *key, char *out, size_t maxlen);

#ifdef __cplusplus
}
#endif
