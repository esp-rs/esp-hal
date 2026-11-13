/* The 2nd-stage bootloader loads all segments and jumps straight to the runtime entry. */
ENTRY(_esp_bootloader_entry)
EXTERN(Reset)
_esp_bootloader_entry = Reset;
