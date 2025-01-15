# Migration Guide from 0.5.x to v0.6.x

## Crate configuration changes

To prevent ambiguity between configurations, we had to change the naming format of configuration
keys. Before, we used `{prefix}_{key}`, which meant that esp-hal and esp-hal-* configuration keys
were impossible to tell apart. To fix this issue, we are changing the separator from one underscore
character to `_CONFIG_`. This also means that users will have to change their `config.toml`
configurations to match the new format.

```diff
 [env]
-ESP_HAL_EMBASSY_LOW_POWER_WAIT="false"
+ESP_HAL_EMBASSY_CONFIG_LOW_POWER_WAIT="false"
```

### Removal of `integrated-timers`

The `integrated-timers` feature has been replaced by two new configuration options:

- `ESP_HAL_EMBASSY_CONFIG_TIMER_QUEUE`: selects a timer queue implementation, which allows
  tuning based on a few requirements. Possible options:
  - `generic`: a generic queue implementation is used. This option does not depend on
    embassy-executor, uses a global critical section to access the timer queue, and its
    capacity determines how many tasks can wait for timers at the same time. You need to pass only a
    single timer to `esp_hal_embassy::init`.
  - `single-integrated`: This option requires embassy-executor and the `executors` feature, uses a
    global critical section to access the timer queue. You need to pass only a single timer to
    `esp_hal_embassy::init`. This is slightly faster than `generic` and does not need a capacity
    configuration. This is the default option when the `executors` feature is enabled.
  - `multiple-integrated`: This option requires embassy-executor and the `executors` feature, uses
    more granular locks to access the timer queue. You need to pass one timer for each embassy
    executor to `esp_hal_embassy::init` and the option does not need a capacity configuration.
    This is the most performant option.
- `ESP_HAL_EMBASSY_CONFIG_GENERIC_QUEUE_SIZE`: determines the capacity of the timer queue when using
    the `generic` option. `esp_hal_embassy` ignores the `embassy-time/generic-queue-*` features.
