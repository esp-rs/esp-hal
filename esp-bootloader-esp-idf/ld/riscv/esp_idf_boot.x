/* The 2nd-stage bootloader loads all segments and jumps straight to the runtime entry. */
ENTRY(_esp_bootloader_entry)
EXTERN(_start)
_esp_bootloader_entry = _start;

/* don't init data - expect the bootloader to do it */
__sdata = 0;
__edata = 0;
__sidata = 0;
