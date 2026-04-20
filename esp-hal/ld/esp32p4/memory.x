/* ESP32-P4X (chip revision v3.x incl. v3.2 / eco7) Memory Layout
 *
 * Source: esp-idf soc.h, TRM v0.5, Chip Revision v3.x User Guide v1.0,
 *         esp-idf 6.1 memory.ld (generated), esptool esp32p4.py
 * Target: ESP32-P4NRW16X / ESP32-P4NRW32X only (NOT NRND variants)
 *
 * WARNING: Chip revision v3.x changed L2MEM cached region mapping
 * from top-down to bottom-up (vs v1.x). This linker script is
 * specifically for v3.x silicon.
 *
 * Memory map verified against esp-idf:
 *   SOC_IRAM_LOW   = 0x4FF00000
 *   SOC_IRAM_HIGH  = 0x4FFC0000  (768 KB total L2MEM)
 *   SOC_DIRAM_ROM_RESERVE_HIGH = 0x4FF40000  (ROM uses first 256 KB, low end)
 *   SOC_RTC_IRAM_LOW  = 0x50108000
 *   SOC_RTC_IRAM_HIGH = 0x50110000  (32 KB LP SRAM)
 *   SOC_EXTRAM_LOW = 0x48000000  (PSRAM, up to 64 MB via cache)
 *
 * CRITICAL (v3.x): ROM ALSO reserves the TOP of L2MEM for BSS/stack.
 *   esptool: BSS_UART_DEV_ADDR = 0x4FFBFEB0 for chip revision >= v3.0
 *   esp-idf sram_seg ends at 0x4FFAEFC0 (leaves ~68KB at top for ROM)
 *   Writing into this region during Rust startup corrupts ROM UART
 *   state and the bootloader immediately prints "user code done"
 *   because the UART_DEV struct was trashed by _start's stack push.
 */

MEMORY
{
    /* 768 KB HP L2MEM (on-chip SRAM)
     * Full range: 0x4FF00000 - 0x4FFC0000
     * Low reserve  (ROM bootloader):      0x4FF00000 - 0x4FF40000 (256 KB)
     * High reserve (ROM BSS/stack, v3+):  0x4FFAE000 - 0x4FFC0000 (~72 KB)
     * Usable for application:             0x4FF40000 - 0x4FFAE000 (440 KB)
     */
    RAM : ORIGIN = 0x4FF40000, LENGTH = 0x6E000

    /* External flash (XIP via cache)
     * Mapped at 0x40000000, up to 64 MB
     * +0x20 offset to skip flash header
     */
    ROM : ORIGIN = 0x40000000 + 0x20, LENGTH = 0x400000 - 0x20

    /* LP SRAM (32 KB, persists over deep sleep)
     * Used by LP core and RTC fast memory
     */
    RTC_FAST : ORIGIN = 0x50108000, LENGTH = 32K
}
