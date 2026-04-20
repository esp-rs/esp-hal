ENTRY(_start)

PROVIDE(_stext = ORIGIN(ROTEXT));
PROVIDE(_max_hart_id = 1);

PROVIDE(UserSoft = DefaultHandler);
PROVIDE(SupervisorSoft = DefaultHandler);
PROVIDE(MachineSoft = DefaultHandler);
PROVIDE(UserTimer = DefaultHandler);
PROVIDE(SupervisorTimer = DefaultHandler);
PROVIDE(MachineTimer = DefaultHandler);
PROVIDE(UserExternal = DefaultHandler);
PROVIDE(SupervisorExternal = DefaultHandler);
PROVIDE(MachineExternal = DefaultHandler);

PROVIDE(ExceptionHandler = DefaultExceptionHandler);

/* The ESP32-C2 and ESP32-C3 have interrupt IDs 1-31, while the ESP32-C6,
   ESP32-H2, and ESP32-P4 have IDs 0-31, so we much define the handler for the
   one additional interrupt ID: */
PROVIDE(interrupt0 = DefaultHandler);

/* ESP32-P4 peripheral interrupt handlers (99 sources from PAC).
   Each maps to DefaultHandler unless overridden by the application.
   Ref: esp32p4 PAC lib.rs __EXTERNAL_INTERRUPTS vector table
        TRM v0.5 Ch 14 (Interrupt Matrix) */
PROVIDE(LP_WDT = DefaultHandler);
PROVIDE(LP_TIMER0 = DefaultHandler);
PROVIDE(LP_TIMER1 = DefaultHandler);
PROVIDE(PMU0 = DefaultHandler);
PROVIDE(PMU1 = DefaultHandler);
PROVIDE(LP_ANA = DefaultHandler);
PROVIDE(LP_ADC = DefaultHandler);
PROVIDE(LP_GPIO = DefaultHandler);
PROVIDE(LP_I2C0 = DefaultHandler);
PROVIDE(LP_I2S0 = DefaultHandler);
PROVIDE(LP_TOUCH = DefaultHandler);
PROVIDE(LP_TSENS = DefaultHandler);
PROVIDE(LP_UART = DefaultHandler);
PROVIDE(LP_SYS = DefaultHandler);
PROVIDE(LP_HUK = DefaultHandler);
PROVIDE(USB_DEVICE = DefaultHandler);
PROVIDE(DMA = DefaultHandler);
PROVIDE(SPI2 = DefaultHandler);
PROVIDE(SPI3 = DefaultHandler);
PROVIDE(I2S0 = DefaultHandler);
PROVIDE(I2S1 = DefaultHandler);
PROVIDE(I2S2 = DefaultHandler);
PROVIDE(UHCI0 = DefaultHandler);
PROVIDE(UART0 = DefaultHandler);
PROVIDE(UART1 = DefaultHandler);
PROVIDE(UART2 = DefaultHandler);
PROVIDE(UART3 = DefaultHandler);
PROVIDE(UART4 = DefaultHandler);
PROVIDE(PWM0 = DefaultHandler);
PROVIDE(PWM1 = DefaultHandler);
PROVIDE(TWAI0 = DefaultHandler);
PROVIDE(TWAI1 = DefaultHandler);
PROVIDE(TWAI2 = DefaultHandler);
PROVIDE(RMT = DefaultHandler);
PROVIDE(I2C0 = DefaultHandler);
PROVIDE(I2C1 = DefaultHandler);
PROVIDE(TG0_T0_LEVEL = DefaultHandler);
PROVIDE(TG0_T1_LEVEL = DefaultHandler);
PROVIDE(TG0_WDT_LEVEL = DefaultHandler);
PROVIDE(TG1_T0_LEVEL = DefaultHandler);
PROVIDE(TG1_T1_LEVEL = DefaultHandler);
PROVIDE(TG1_WDT_LEVEL = DefaultHandler);
PROVIDE(LEDC = DefaultHandler);
PROVIDE(SYSTIMER_TARGET0 = DefaultHandler);
PROVIDE(SYSTIMER_TARGET1 = DefaultHandler);
PROVIDE(SYSTIMER_TARGET2 = DefaultHandler);
PROVIDE(AHB_PDMA_IN_CH0 = DefaultHandler);
PROVIDE(AHB_PDMA_IN_CH1 = DefaultHandler);
PROVIDE(AHB_PDMA_IN_CH2 = DefaultHandler);
PROVIDE(AHB_PDMA_OUT_CH0 = DefaultHandler);
PROVIDE(AHB_PDMA_OUT_CH1 = DefaultHandler);
PROVIDE(AHB_PDMA_OUT_CH2 = DefaultHandler);
PROVIDE(AXI_PDMA_IN_CH0 = DefaultHandler);
PROVIDE(AXI_PDMA_IN_CH1 = DefaultHandler);
PROVIDE(AXI_PDMA_IN_CH2 = DefaultHandler);
PROVIDE(AXI_PDMA_OUT_CH0 = DefaultHandler);
PROVIDE(AXI_PDMA_OUT_CH1 = DefaultHandler);
PROVIDE(AXI_PDMA_OUT_CH2 = DefaultHandler);
PROVIDE(RSA = DefaultHandler);
PROVIDE(AES = DefaultHandler);
PROVIDE(SHA = DefaultHandler);
PROVIDE(ECC = DefaultHandler);
PROVIDE(GPIO = DefaultHandler);
PROVIDE(GPIO_INT1 = DefaultHandler);
PROVIDE(GPIO_INT2 = DefaultHandler);
PROVIDE(GPIO_INT3 = DefaultHandler);
PROVIDE(GPIO_PAD_COMP = DefaultHandler);
PROVIDE(FROM_CPU_INTR0 = DefaultHandler);
PROVIDE(FROM_CPU_INTR1 = DefaultHandler);
PROVIDE(FROM_CPU_INTR2 = DefaultHandler);
PROVIDE(FROM_CPU_INTR3 = DefaultHandler);
PROVIDE(CACHE = DefaultHandler);
PROVIDE(CSI_BRIDGE = DefaultHandler);
PROVIDE(DSI_BRIDGE = DefaultHandler);
PROVIDE(CSI = DefaultHandler);
PROVIDE(DSI = DefaultHandler);
PROVIDE(JPEG = DefaultHandler);
PROVIDE(PPA = DefaultHandler);
PROVIDE(ISP = DefaultHandler);
PROVIDE(I3C = DefaultHandler);
PROVIDE(I3C_SLV = DefaultHandler);
PROVIDE(HP_SYS = DefaultHandler);
PROVIDE(PCNT = DefaultHandler);
PROVIDE(PAU = DefaultHandler);
PROVIDE(PARLIO_RX = DefaultHandler);
PROVIDE(PARLIO_TX = DefaultHandler);
PROVIDE(H264_DMA2D_OUT_CH0 = DefaultHandler);
PROVIDE(H264_DMA2D_OUT_CH1 = DefaultHandler);
PROVIDE(H264_DMA2D_OUT_CH2 = DefaultHandler);
PROVIDE(H264_DMA2D_OUT_CH3 = DefaultHandler);
PROVIDE(H264_DMA2D_OUT_CH4 = DefaultHandler);
PROVIDE(H264_DMA2D_IN_CH0 = DefaultHandler);
PROVIDE(H264_DMA2D_IN_CH1 = DefaultHandler);
PROVIDE(H264_DMA2D_IN_CH2 = DefaultHandler);
PROVIDE(H264_DMA2D_IN_CH3 = DefaultHandler);
PROVIDE(H264_DMA2D_IN_CH4 = DefaultHandler);
PROVIDE(H264_DMA2D_IN_CH5 = DefaultHandler);
PROVIDE(H264_REG = DefaultHandler);
PROVIDE(ASSIST_DEBUG = DefaultHandler);

PROVIDE(__post_init = default_post_init);

/* A PAC/HAL defined routine that should initialize custom interrupt controller if needed. */
PROVIDE(_setup_interrupts = default_setup_interrupts);

/* # Multi-processing hook function
   fn _mp_hook() -> bool;
   This function is called from all the harts and must return true only for one hart,
   which will perform memory initialization. For other harts it must return false
   and implement wake-up in platform-dependent way (e.g. after waiting for a user interrupt).
*/
PROVIDE(_mp_hook = default_mp_hook);

/* # Start trap function override
  By default uses the riscv crates default trap handler
  but by providing the `_start_trap` symbol external crates can override.
*/
PROVIDE(_start_trap = _default_start_trap);

/* Must be called __global_pointer$ for linker relaxations to work. */
PROVIDE(__global_pointer$ = _data_start + 0x800);

/* NOTE: .trap section is generated by build.rs in rwtext.x, not duplicated here. */

SECTIONS {
  /**
   * Bootloader really wants to have separate segments for ROTEXT and RODATA
   * It also needs to be located in a separate 64k flash segment.
   */
  .text_gap (NOLOAD): {
    . = ALIGN(0x10000) + 0x20;
  } > ROM
}
INSERT BEFORE .rodata;

/* Shared sections - ordering matters.
   rwtext.x and rwdata.x contain bare section directives that need a SECTIONS context.
   We wrap them in SECTIONS {} blocks. */
SECTIONS {
  INCLUDE "rwtext.x"
}

SECTIONS {
  INCLUDE "rwdata.x"
}

INCLUDE "rodata.x"
INCLUDE "text.x"
INCLUDE "rtc_fast.x"
INCLUDE "stack.x"
INCLUDE "metadata.x"
INCLUDE "eh_frame.x"
/* End of Shared sections */

_dram_data_start = ORIGIN(RAM) + SIZEOF(.trap) + SIZEOF(.rwtext);