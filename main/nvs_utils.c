#include "nvs_utils.h"

#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"

static const char *TAG = "nvs_utils";

esp_err_t nvs_initialize(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "erasing NVS flash (err=%#x)", err);
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    return err;
}

esp_err_t nvs_save_string(const char *ns, const char *key, const char *value)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(ns, NVS_READWRITE, &handle);
    if (err != ESP_OK) return err;
    err = nvs_set_str(handle, key, value);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

esp_err_t nvs_load_string(const char *ns, const char *key, char *out, size_t maxlen)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(ns, NVS_READONLY, &handle);
    if (err != ESP_OK) return err;
    err = nvs_get_str(handle, key, out, &maxlen);
    nvs_close(handle);
    return err;
}
