//! Interrupt handling for ESP32-S2 & ESP32-S3 RISCV ULP cores.
//! Uses custom R-type instructions for ESP32-S2 & ESP32-S3 RISCV ULP cores.
use crate::pac;

// #[cfg(any(esp32s2, esp32s3))]
// pub use pac::interrupt as sens_interrupt;
// use crate::gpio::MAX_GPIO_PIN;

/// Argument passed to GpioInterrupt handler.
pub type GpioInterruptPin = u32;
/// Argument passed to SensInterrupt handler.
pub use pac::Interrupt as SensInterruptStatus;

const STATUS_WORDS: usize = 1;

/// Representation of peripheral-interrupt status bits.
#[doc(hidden)]
#[derive(Clone, Copy, Default, Debug)]
pub struct InterruptStatus {
    status: [u32; STATUS_WORDS],
}

impl InterruptStatus {
    /// Is the given interrupt bit set
    pub fn is_set(&self, interrupt: u8) -> bool {
        (self.status[interrupt as usize / 32] & (1 << (interrupt % 32))) != 0
    }

    /// Set the given interrupt status bit
    pub fn set(&mut self, interrupt: u8) {
        self.status[interrupt as usize / 32] |= 1 << (interrupt % 32);
    }

    /// Return an iterator over the set interrupt status bits
    pub fn iterator(&self) -> InterruptStatusIterator {
        InterruptStatusIterator {
            status: *self,
            idx: 0,
        }
    }
}

impl From<u32> for InterruptStatus {
    fn from(value: u32) -> Self {
        Self { status: [value] }
    }
}

/// Iterator over set interrupt status bits
#[doc(hidden)]
#[derive(Debug, Clone)]
pub struct InterruptStatusIterator {
    status: InterruptStatus,
    idx: usize,
}

impl Iterator for InterruptStatusIterator {
    type Item = u8;

    fn next(&mut self) -> Option<Self::Item> {
        for i in self.idx..STATUS_WORDS {
            if self.status.status[i] != 0 {
                let bit = self.status.status[i].trailing_zeros();
                self.status.status[i] ^= 1 << bit;
                self.idx = i;
                return Some((bit + 32 * i as u32) as u8);
            }
        }
        self.idx = usize::MAX;
        None
    }
}

/// TODO: Move custom ULP instructions into their own module,
///       and cleanup the assembly files at the same time.
/// Disable all interrupts
#[cfg(any(esp32s2, esp32s3))]
#[inline(always)]
pub fn disable() {
    // Enter a critical section by disabling all interrupts
    // This inline assembly construct uses the t0 register and is equivalent to:
    // > li t0, 0x80000007
    // > maskirq_insn(zero, t0) // Mask all interrupt bits
    // The mask 0x80000007 represents:
    //   Bit 31 - RTC peripheral interrupt
    //   Bit 2  - Bus error
    //   Bit 1  - Ebreak / Ecall / Illegal Instruction
    //   Bit 0  - Internal Timer
    //
    unsafe {
        core::arch::asm!("li t0, 0x80000007", ".word 0x0602e00b");
    }
}

/// Enable all interrupts
#[cfg(any(esp32s2, esp32s3))]
#[inline(always)]
pub fn enable() {
    // Exit a critical section by enabling all interrupts
    // This inline assembly construct is equivalent to:
    // > maskirq_insn(zero, zero)
    unsafe {
        core::arch::asm!(".word 0x0600600b");
    }
}

/// Wait for any (masked or unmasked) interrupt
#[cfg(any(esp32s2, esp32s3))]
pub fn waitirq() -> u32 {
    // Wait for pending interrupt, return pending interrupt mask
    // waitirq a0
    let result: u32;
    unsafe {
        core::arch::asm!(".word 0x0800400B", out("a0") result);
    }
    result
}

/// TODO: Write store_trap / load_trap functions, to generate the context saving assembly code,
//        which is currently hand-written in ulp_riscv_vectors.S
/// Registers saved in trap handler
#[doc(hidden)]
#[cfg(any(esp32s2, esp32s3))]
#[repr(C)]
#[derive(Debug)]
pub struct TrapFrame {
    /// `x1`: return address, stores the address to return to after a function call or interrupt.
    pub ra: usize,
    /// `x5`: temporary register `t0`, used for intermediate values.
    pub t0: usize,
    /// `x6`: temporary register `t1`, used for intermediate values.
    pub t1: usize,
    /// `x7`: temporary register `t2`, used for intermediate values.
    pub t2: usize,
    /// `x28`: temporary register `t3`, used for intermediate values.
    pub t3: usize,
    /// `x29`: temporary register `t4`, used for intermediate values.
    pub t4: usize,
    /// `x30`: temporary register `t5`, used for intermediate values.
    pub t5: usize,
    /// `x31`: temporary register `t6`, used for intermediate values.
    pub t6: usize,
    /// `x10`: argument register `a0`. Used to pass the first argument to a function.
    pub a0: usize,
    /// `x11`: argument register `a1`. Used to pass the second argument to a function.
    pub a1: usize,
    /// `x12`: argument register `a2`. Used to pass the third argument to a function.
    pub a2: usize,
    /// `x13`: argument register `a3`. Used to pass the fourth argument to a function.
    pub a3: usize,
    /// `x14`: argument register `a4`. Used to pass the fifth argument to a function.
    pub a4: usize,
    /// `x15`: argument register `a5`. Used to pass the sixth argument to a function.
    pub a5: usize,
    /// `x16`: argument register `a6`. Used to pass the seventh argument to a function.
    pub a6: usize,
    /// `x17`: argument register `a7`. Used to pass the eighth argument to a function.
    pub a7: usize,
}

/// Trap entry point rust (_start_trap_rust)
/// `irqs` is a bitmask of IRQs to handle.
#[cfg(any(esp32s2, esp32s3))]
#[doc(hidden)]
#[unsafe(link_section = ".trap.rust")]
#[unsafe(export_name = "_start_trap_rust")]
pub extern "C" fn ulp_start_trap_rust(trap_frame: *const TrapFrame, irqs: u32) {
    unsafe extern "C" {
        fn trap_handler(regs: &TrapFrame, pending_irqs: u32);
    }

    unsafe {
        // dispatch trap to handler
        trap_handler(&*trap_frame, irqs);
    }
}

/// Creates the trap_handler() function.
/// This is macro is used later in this file.
/// This style of macro is usually provided
/// for users to hook their own handlers, but here it's just used
/// as a quick way to generate the bit-mask code :)
#[cfg(any(esp32s2, esp32s3))]
macro_rules! build_trap_handler {
    (@interrupt ($n:literal, $pending_irqs:expr, $regs:expr, $handler:ident)) => {
        if $pending_irqs & (1 << $n) != 0 {
            #[allow(unused_unsafe)]
            unsafe { $handler($regs); }
        }
    };
    ( $( $irq:literal : $handler:ident ),* ) => {
        /// Called by _start_trap_rust, this trap handler will call other interrupt handling
        /// functions depending on the bits set in pending_irqs.
        #[doc(hidden)]
        #[unsafe(no_mangle)]
        pub extern "C" fn trap_handler(regs: *const TrapFrame, pending_irqs: u32) {
            let regs = unsafe { regs.as_ref().unwrap() };
            $(
                build_trap_handler!(@interrupt($irq, pending_irqs, regs, $handler));
            )*
        }
    };
}

/// Default interrupt handler, does nothing.
#[cfg(any(esp32s2, esp32s3))]
#[allow(dead_code)]
#[doc(hidden)]
#[allow(non_snake_case)]
#[unsafe(no_mangle)]
pub fn DefaultHandler() {}

/// Default illegal instruction or bus error exception handler.
/// This handler is dispatched by the trap_handler() function.
#[cfg(any(esp32s2, esp32s3))]
#[allow(dead_code)]
#[allow(non_snake_case)]
#[doc(hidden)]
#[unsafe(no_mangle)]
pub fn DefaultExceptionHandler(_regs: &TrapFrame) {
    panic!("Unhandled exception!");
}

// Create the trap_handler function
#[cfg(any(esp32s2, esp32s3))]
build_trap_handler!(
    1: DefaultExceptionHandler,
    2: DefaultExceptionHandler,
    31: dispatch_peripheral_interrupt
);

/// Peripheral interrupt handler for the IRQ bit 31.
/// Checks the SENS and RTC_IO interrup status, and dispatches further interrupt handlers as
/// appropriate.
#[cfg(any(esp32s2, esp32s3))]
#[doc(hidden)]
#[unsafe(no_mangle)]
fn dispatch_peripheral_interrupt(_regs: &TrapFrame) {
    // This function is based on the ESP-IDF implementation found here:
    // https://github.com/espressif/esp-idf/blob/12f36a021f511cd4de41d3fffff146c5336ac1e7/components/ulp/ulp_riscv/ulp_core/ulp_riscv_interrupt.c#L110
    unsafe extern "Rust" {
        fn SensInterrupt(status: SensInterruptStatus);
        fn GpioInterrupt(pin: GpioInterruptPin);
    }

    // Iterate for any SENS interrupt flags
    let cocpu_int_st_bits =
        InterruptStatus::from(unsafe { &*pac::SENS::PTR }.sar_cocpu_int_st().read().bits());
    // Iterate over the 1 bit positions
    for bit in cocpu_int_st_bits.iterator() {
        // Convert into the named interrupt enumeration
        match SensInterruptStatus::try_from(bit) {
            Ok(stat) => {
                // Call handler, and clear the interrupt bit.
                unsafe { SensInterrupt(stat) };

                unsafe { &*pac::SENS::PTR }
                    .sar_cocpu_int_clr()
                    .write(|w| unsafe { w.bits(1 << bit) });
            }
            Err(_) => {}
        }
    }

    // RTC IO interrupts.
    let rtcio_int_st_bits = unsafe { &*pac::RTC_IO::PTR }.status().read().bits();
    let rtcio_int_st = InterruptStatus::from(rtcio_int_st_bits);
    // Iterate over the 1 bit positions
    for bit in rtcio_int_st.iterator() {
        // Call handler, and clear the interrupt bit.
        // Pin must have 10 subtracted from it, due to register offset.
        unsafe { GpioInterrupt((bit - 10) as u32) };

        unsafe { &*pac::RTC_IO::PTR }
            .status_w1tc()
            .write(|w| unsafe { w.bits(1 << bit) });
    }
}

// Macros bind user functions to SensInterrupt and GpioInterrupt.

#[macro_export]
#[doc = r" Assigns a handler to GpioInterrupt"]
#[doc = r""]
#[doc = r" This macro takes one argument: path to the function that"]
#[doc = r" will be used as the handler of that interrupt. The function"]
#[doc = r" must have signature `fn(pin : GpioInterruptPin)`."]
#[doc = r""]
#[doc = r" # Example"]
#[doc = r""]
#[doc = r" ``` ignore"]
#[doc = r" gpio_interrupt!(buttons_handler);"]
#[doc = r""]
#[doc = r" fn buttons_handler(_pin : GpioInterruptPin) {"]
#[doc = r#"     print!("A GPIO pin was pressed!");"#]
#[doc = r" }"]
#[doc = r""]
macro_rules! gpio_interrupt {
    ($ path : path) => {
        #[allow(non_snake_case)]
        #[unsafe(no_mangle)]
        pub extern "C" fn GpioInterrupt(pin: GpioInterruptPin) {
            let f: fn(pin: GpioInterruptPin) = $path;
            f(pin);
        }
    };
}

#[macro_export]
#[doc = r" Assigns a handler to SensInterrupt"]
macro_rules! sens_interrupt {
    ($ path : path) => {
        #[allow(non_snake_case)]
        #[unsafe(no_mangle)]
        pub extern "C" fn SensInterrupt(status: SensInterruptStatus) {
            let f: fn(status: SensInterruptStatus) = $path;
            f(status);
        }
    };
}

#[allow(unused)]
pub use gpio_interrupt;
#[allow(unused)]
pub use sens_interrupt;
