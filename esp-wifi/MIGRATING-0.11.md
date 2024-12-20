# Migration Guide from 0.11.x to v0.12.x

## Crate configuration changes

To prevent ambiguity between configurations, we had to change the naming format of configuration
keys. Before, we used `{prefix}_{key}`, which meant that esp-hal and esp-hal-* configuration keys
were impossible to tell apart. To fix this issue, we are changing the separator from one to two
underscore characters. This also means that users will have to change their `config.toml`
configurations to match the new format.

```diff
 [env]
-ESP_WIFI_RX_QUEUE_SIZE = "16"
-ESP_WIFI_STATIC_RX_BUF_NUM = "32"
-ESP_WIFI_DYNAMIC_RX_BUF_NUM = "16"
+ESP_WIFI_CONFIG_RX_QUEUE_SIZE = "16"
+ESP_WIFI_CONFIG_STATIC_RX_BUF_NUM = "32"
+ESP_WIFI_CONFIG_DYNAMIC_RX_BUF_NUM = "16"
```
