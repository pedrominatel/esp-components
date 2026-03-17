#include "esp_err.h"
#include "esp_log.h"
#include "template.h"

static const char *TAG = "example_template";

void app_main(void)
{
    template_handle_t handle = template_create();
    if (handle == NULL) {
        ESP_LOGE(TAG, "Failed to create template handle");
        return;
    }

    ESP_LOGI(TAG, "Template handle created successfully");

    esp_err_t ret = template_delete(handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to delete template handle: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "Template handle deleted successfully");
}
