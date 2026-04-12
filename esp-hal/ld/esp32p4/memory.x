/* ESP32-P4X (chip revision v3.x / eco5) Memory Layout
 *
 * Source: esp-idf soc.h, TRM v0.5, Chip Revision v3.x User Guide v1.0
 * Target: ESP32-P4NRW16X / ESP32-P4NRW32X only (NOT NRND variants)
 *
 * WARNING: Chip revision v3.x changed L2MEM cached region mapping
 * from top-down to bottom-up (vs v1.x). This linker script is
 * specifically for v3.x silicon.
 *
 * Memory map verified against esp-idf:
 *   SOC_IRAM_LOW   = 0x4FF00000
 *   SOC_IRAM_HIGH  = 0x4FFC0000  (768 KB total L2MEM)
 *   SOC_DIRAM_ROM_RESERVE_HIGH = 0x4FF40000  (ROM uses first 256 KB)
 *   SOC_RTC_IRAM_LOW  = 0x50108000
 *   SOC_RTC_IRAM_HIGH = 0x50110000  (32 KB LP SRAM)
 *   SOC_EXTRAM_LOW = 0x48000000  (PSRAM, up to 64 MB via cache)
 */

MEMORY
{
    /* 768 KB HP L2MEM (on-chip SRAM)
     * Full range: 0x4FF00000 - 0x4FFC0000
     * ROM bootloader reserves 0x4FF00000 - 0x4FF40000 (256 KB)
     * Usable for application: 0x4FF40000 - 0x4FFC0000 (512 KB)
     *
     * NOTE: For bare-metal without ROM bootloader, the full 768 KB
     * may be available. Adjust ORIGIN/LENGTH accordingly.
     * Current setting: conservative, avoids ROM reserved region.
     */
    RAM : ORIGIN = 0x4FF40000, LENGTH = 512K

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
