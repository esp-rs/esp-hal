/* The scheduler and the OS adapter that the Wi-Fi driver calls for each frame. Each function has its
   own section, named after its mangled symbol. A Rust symbol contains each path segment with its
   length, for example `10os_adapter`. */

/* esp-rtos: the scheduler, timers and synchronization primitives. The functions that the Wi-Fi
   driver calls directly have C names, such as `esp_rtos_semaphore_take`. */
*(.literal.*esp_rtos* .text.*esp_rtos*)

/* The OS adapter of esp-radio and its queues and semaphores. */
*(.literal.*21esp_radio_rtos_driver* .text.*21esp_radio_rtos_driver*)
*(.literal.*9esp_radio6compat* .text.*9esp_radio6compat*)
*(.literal.*10os_adapter* .text.*10os_adapter*)
*(.literal.*14common_adapter* .text.*14common_adapter*)
