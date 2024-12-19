# Migration Guide from 0.5.x to v0.6.x

## Crate configuration changes

To prevent ambiguity between configurations, we had to change the naming format of configuration
keys. Before, we used `{prefix}_{key}`, which meant that esp-hal and esp-hal-* configuration keys
were impossible to tell apart. To fix this issue, we are changing the separator from one to two
underscore characters. This also means that users will have to change their `config.toml`
configurations to match the new format.

```diff
 [env]
-ESP_HAL_EMBASSY_LOW_POWER_WAIT="false"
+ESP_HAL_EMBASSY__LOW_POWER_WAIT="false"
```
