/* The heap. The Wi-Fi driver allocates a buffer for each frame through the allocation functions of
   the esp-radio OS adapter, for example `wifi_malloc`. */
*(.literal.*10os_adapter*alloc* .text.*10os_adapter*alloc*)
*(.literal.*10os_adapter*free* .text.*10os_adapter*free*)
*(.literal.*8esp_rtos7syscall* .text.*8esp_rtos7syscall*)
*(.literal.*9esp_alloc* .text.*9esp_alloc*)
*(.literal.malloc .text.malloc .literal.free .text.free)
*(.literal.calloc .text.calloc .literal.realloc .text.realloc)
*(.literal.malloc_internal .text.malloc_internal .literal.free_internal .text.free_internal)
*(.literal.calloc_internal .text.calloc_internal .literal.realloc_internal .text.realloc_internal)
