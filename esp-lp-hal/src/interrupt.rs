//! Interrupt handling for ESP32-S2 & ESP32-S3 RISCV ULP cores.
//! Uses custom R-type instructions for ESP32-S2 & ESP32-S3 RISCV ULP cores.
use core::ptr::NonNull;

/// Argument passed to SensInterrupt handler.
pub use pac::Interrupt as SensInterruptStatus;
use portable_atomic::{AtomicPtr, Ordering};

// use procmacros::handler;
use crate::pac;
/// Argument passed to GpioInterrupt handler.
pub type GpioInterruptPin = u32;

// Interrupt vector table
// TODO: Merge back into esp-pacs
// ALT-TODO: Setup an __INTERRUPTS table in here, which contains a superset of
// __EXTERNAL_INTERRUPTS,           and other peripheral interrupts not captured by the PAC.
//           e.g. bits 0..=31 would be for 'external interrupts',
//           and bits 32..=N would be for 'peripheral interrupts', or similar.
//           This might allow the interrupt handling code in here to be more portable, maybe...
unsafe extern "C" {
    fn GPIO_INT();
    fn SENS_INT();
}

/// Default interrupt handler, does nothing.
#[cfg(any(esp32s2, esp32s3))]
#[allow(dead_code)]
#[doc(hidden)]
#[allow(non_snake_case)]
#[unsafe(no_mangle)]
pub fn DefaultHandler() {}

#[doc(hidden)]
#[repr(C)]
pub union Vector {
    pub _handler: unsafe extern "C" fn(),
    pub _reserved: usize,
}

#[doc(hidden)]
#[unsafe(link_section = ".rwtext")]
#[unsafe(no_mangle)]
pub static __INTERRUPTS: [Vector; Interrupt::len()] =
    [Vector { _handler: SENS_INT }, Vector { _handler: GPIO_INT }];

#[doc = r"Enumeration of all the interrupts."]
#[derive(Copy, Clone, PartialEq, Eq)]
#[repr(u16)]
pub enum Interrupt {
    #[doc = "0 - SENS"]
    SENS = 0,
    #[doc = "1 - GPIO"]
    GPIO = 1,
    #[doc(hidden)]
    __LAST__, // Marker to auto-detect enum length.
}

#[doc = r" TryFromInterruptError"]
#[derive(Copy, Clone)]
pub struct TryFromInterruptError(());

impl Interrupt {
    #[doc = r"Attempt to convert a given value into an `Interrupt`"]
    #[inline]
    pub fn try_from(value: u8) -> Result<Self, TryFromInterruptError> {
        match value {
            0 => Ok(Interrupt::SENS),
            1 => Ok(Interrupt::GPIO),
            _ => Err(TryFromInterruptError(())),
        }
    }

    #[doc = r"Get the total number of interrupts."]
    #[inline]
    pub const fn len() -> usize {
        Self::__LAST__ as usize
    }
}

const STATUS_WORDS: usize = 1;

/// Representation of peripheral-interrupt status bits.
#[doc(hidden)]
#[derive(Clone, Copy, Default, Debug)]
pub struct InterruptStatus {
    status: [u32; STATUS_WORDS],
}

impl InterruptStatus {
    /// Get status of peripheral interrupts
    pub fn current() -> Self {
        let mut status_bits = 0b00;
        // Check SENS status
        status_bits |=
            ((unsafe { &*pac::SENS::PTR }.sar_cocpu_int_st().read().bits() != 0) as u32) << 0;
        // Check GPIO status
        status_bits |= ((unsafe { &*pac::RTC_IO::PTR }.status().read().bits() != 0) as u32) << 1;
        InterruptStatus::from(status_bits)
    }

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

pub(super) struct CFnPtr(AtomicPtr<()>);
impl CFnPtr {
    pub const fn new() -> Self {
        Self(AtomicPtr::new(core::ptr::null_mut()))
    }

    pub fn store(&self, f: extern "C" fn()) {
        self.0.store(f as *mut (), Ordering::Relaxed);
    }

    pub fn call(&self) {
        let ptr = self.0.load(Ordering::Relaxed);
        if !ptr.is_null() {
            unsafe { (core::mem::transmute::<*mut (), extern "C" fn()>(ptr))() };
        }
    }
}

// /// If no handler is allocated, panic.
// #[unsafe(no_mangle)]
// extern "C" fn EspDefaultHandler() {
//     panic!("Unhandled interrupt");
// }
// /// Default (unhandled) interrupt handler
// pub const DEFAULT_INTERRUPT_HANDLER: InterruptHandler = InterruptHandler::new(
//     {
//         unsafe extern "C" {
//             fn EspDefaultHandler();
//         }

//         unsafe {
//             core::mem::transmute::<unsafe extern "C" fn(), extern "C" fn()>(EspDefaultHandler)
//         }
//     },
//     Priority::min(),
// );

// Peripheral interrupt API.

fn vector_entry(interrupt: Interrupt) -> &'static Vector {
    &__INTERRUPTS[interrupt as usize]
}

/// Returns the currently bound interrupt handler.
pub fn bound_handler(interrupt: Interrupt) -> Option<IsrCallback> {
    unsafe {
        let vector = vector_entry(interrupt);

        let addr = vector._handler;
        if addr as usize == 0 {
            return None;
        }

        Some(IsrCallback::new(core::mem::transmute::<
            unsafe extern "C" fn(),
            extern "C" fn(),
        >(addr)))
    }
}

/// Binds the given handler to a peripheral interrupt.
///
/// The interrupt handler will be enabled at the specified priority level.
///
/// The interrupt handler will be called on the core where it is registered.
/// Only one interrupt handler can be bound to a peripheral interrupt.
pub fn bind_handler(interrupt: Interrupt, handler: InterruptHandler) {
    unsafe {
        let vector = vector_entry(interrupt);
        let ptr = (&raw const vector._handler).cast::<usize>().cast_mut();
        ptr.write_volatile(handler.handler().address());
    }
    // ULP unused
    // enable(interrupt, handler.priority());
}

/// Trait implemented by drivers which allow the user to set an
/// [InterruptHandler]
pub trait InterruptConfigurable {
    #[doc = "Registers an interrupt handler for the peripheral."]
    #[doc = ""]
    /// Note that this will replace any previously registered interrupt
    /// handlers. Some peripherals offer a shared interrupt handler for
    /// multiple purposes. It's the users duty to honor this.
    ///
    /// You can restore the default/unhandled interrupt handler by using
    /// [DEFAULT_INTERRUPT_HANDLER]
    fn set_interrupt_handler(&mut self, handler: InterruptHandler);
}

/// Represents an ISR callback function
#[derive(Copy, Clone)]
pub struct IsrCallback {
    f: extern "C" fn(),
}

impl IsrCallback {
    /// Construct a new callback from the callback function.
    pub fn new(f: extern "C" fn()) -> Self {
        // a valid fn pointer is non zero
        Self { f }
    }

    /// Returns the address of the callback.
    pub fn address(self) -> usize {
        self.f as usize
    }

    /// The callback function.
    ///
    /// This is aligned and can be called.
    pub fn callback(self) -> extern "C" fn() {
        self.f
    }
}

impl PartialEq for IsrCallback {
    fn eq(&self, other: &Self) -> bool {
        core::ptr::fn_addr_eq(self.callback(), other.callback())
    }
}

/// Interrupt priority levels.
/// For esp-lp-hal there is only one priority level, and this is only included
/// so that the API is compatible with esp-hal handler macro.
#[derive(Copy, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
#[repr(u8)]
#[non_exhaustive]
pub enum Priority {
    /// Priority level 1.
    Priority1 = 1,
}

impl Priority {
    /// Maximum interrupt priority
    pub const fn max() -> Priority {
        Priority::Priority1
    }

    /// Minimum interrupt priority
    pub const fn min() -> Priority {
        Priority::Priority1
    }
}

/// An interrupt handler
#[derive(Copy, Clone)]
pub struct InterruptHandler {
    f: extern "C" fn(),
    prio: Priority,
}

impl InterruptHandler {
    /// Creates a new [InterruptHandler] which will call the given function
    pub const fn new(f: extern "C" fn(), prio: Priority) -> Self {
        Self { f, prio }
    }

    /// The Isr callback.
    #[inline]
    pub fn handler(&self) -> IsrCallback {
        IsrCallback::new(self.f)
    }

    /// Priority to be used when registering the interrupt
    #[inline]
    pub fn priority(&self) -> Priority {
        self.prio
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

/// Setup interrupt handlers, including any default ones
#[cfg(any(esp32s2, esp32s3))]
#[inline(always)]
pub fn setup_interrupts() {
    crate::gpio::bind_default_interrupt_handler();
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
pub extern "C" fn ulp_start_trap_rust(trap_frame: *const u32, irqs: u32) {
    unsafe extern "C" {
        fn trap_handler(regs: &TrapFrame, pending_irqs: u32);
    }

    unsafe {
        // 'trap_frame' pointer safety:
        // _start_trap must place a valid address in a0, prior to calling _start_trap_rust.
        trap_handler(
            NonNull::new_unchecked(trap_frame as *mut TrapFrame).as_ref(),
            irqs,
        );
    }
}

/// Called by _start_trap_rust, this trap handler will call other interrupt handling
/// functions depending on the bits set in pending_irqs.
#[cfg(any(esp32s2, esp32s3))]
#[doc(hidden)]
#[unsafe(no_mangle)]
pub extern "C" fn trap_handler(_regs: &TrapFrame, pending_irqs: u32) {
    // Dispatch peripheral interrupt
    if pending_irqs & (1 << 31) != 0 {
        let status = InterruptStatus::current();

        // Iterate the active interrupts, fetch their handler, and call it if set.
        for interrupt_nr in status.iterator() {
            // New, null-ptr-checking code.
            if let Ok(i) = Interrupt::try_from(interrupt_nr) {
                if let Some(handler) = bound_handler(i) {
                    handler.callback()();
                }
            }

            // Original code
            // unsafe {
            //     let handler = crate::interrupt::__INTERRUPTS[interrupt_nr as usize]._handler;
            //     handler();
            // }
        }
    }
}

// /// Creates the trap_handler() function.
// /// This is macro is used later in this file.
// /// This style of macro is usually provided
// /// for users to hook their own handlers, but here it's just used
// /// as a quick way to generate the bit-mask code :)
// #[cfg(any(esp32s2, esp32s3))]
// macro_rules! build_trap_handler {
//     (@interrupt ($n:literal, $pending_irqs:expr, $regs:expr, $handler:ident)) => {
//         if $pending_irqs & (1 << $n) != 0 {
//             #[allow(unused_unsafe)]
//             unsafe { $handler($regs); }
//         }
//     };
//     ( $( $irq:literal : $handler:ident ),* ) => {
//         /// Called by _start_trap_rust, this trap handler will call other interrupt handling
//         /// functions depending on the bits set in pending_irqs.
//         #[doc(hidden)]
//         #[unsafe(no_mangle)]
//         pub extern "C" fn trap_handler(regs: &TrapFrame, pending_irqs: u32) {
//             $(
//                 build_trap_handler!(@interrupt($irq, pending_irqs, regs, $handler));
//             )*
//         }
//     };
// }

// /// Default interrupt handler, does nothing.
// #[cfg(any(esp32s2, esp32s3))]
// #[allow(dead_code)]
// #[doc(hidden)]
// #[allow(non_snake_case)]
// #[unsafe(no_mangle)]
// pub fn DefaultHandler() {}

// /// Default illegal instruction or bus error exception handler.
// /// This handler is dispatched by the trap_handler() function.
// #[cfg(any(esp32s2, esp32s3))]
// #[allow(dead_code)]
// #[allow(non_snake_case)]
// #[doc(hidden)]
// #[unsafe(no_mangle)]
// pub fn DefaultExceptionHandler(_regs: &TrapFrame) {
//     panic!("Unhandled exception!");
// }

// // Create the trap_handler function
// #[cfg(any(esp32s2, esp32s3))]
// build_trap_handler!(
//     // 1: DefaultExceptionHandler,
//     // 2: DefaultExceptionHandler,
//     31: dispatch_peripheral_interrupt
// );

// /// Peripheral interrupt handler for the IRQ bit 31.
// /// Checks the SENS and RTC_IO interrup status, and dispatches further interrupt handlers as
// /// appropriate.
// #[cfg(any(esp32s2, esp32s3))]
// #[doc(hidden)]
// #[unsafe(no_mangle)]
// fn dispatch_peripheral_interrupt(_regs: &TrapFrame) {
//     // This function is based on the ESP-IDF implementation found here:
//     // https://github.com/espressif/esp-idf/blob/12f36a021f511cd4de41d3fffff146c5336ac1e7/components/ulp/ulp_riscv/ulp_core/ulp_riscv_interrupt.c#L110
//     unsafe extern "Rust" {
//         fn SensInterrupt(status: SensInterruptStatus);
//         fn GpioInterrupt(pin: GpioInterruptPin);
//     }

//     // Iterate for any SENS interrupt flags
//     let cocpu_int_st_bits = InterruptStatus::from(unsafe { &*pac::SENS::PTR
// }.sar_cocpu_int_st().read().bits());     // Iterate over the 1 bit positions
//     for bit in cocpu_int_st_bits.iterator() {
//         // Convert into the named interrupt enumeration
//         if let Ok(stat) = SensInterruptStatus::try_from(bit) {
//             // Call handler, and clear the interrupt bit.
//             unsafe { SensInterrupt(stat) };

//             unsafe { &*pac::SENS::PTR }
//                 .sar_cocpu_int_clr()
//                 .write(|w| unsafe { w.bits(1 << bit) });
//         }
//     }

//     // RTC IO interrupts.
//     let rtcio_int_st_bits = unsafe { &*pac::RTC_IO::PTR }.status().read().bits();
//     let rtcio_int_st = InterruptStatus::from(rtcio_int_st_bits);
//     // Iterate over the 1 bit positions
//     for bit in rtcio_int_st.iterator() {
//         // Call handler, and clear the interrupt bit.
//         // Pin must have 10 subtracted from it, due to register offset.
//         unsafe { GpioInterrupt((bit - 10) as u32) };

//         unsafe { &*pac::RTC_IO::PTR }
//             .status_w1tc()
//             .write(|w| unsafe { w.bits(1 << bit) });
//     }
// }

// // Macros bind user functions to SensInterrupt and GpioInterrupt.

// #[macro_export]
// #[doc = r" Assigns a handler to GpioInterrupt"]
// #[doc = r""]
// #[doc = r" This macro takes one argument: path to the function that"]
// #[doc = r" will be used as the handler of that interrupt. The function"]
// #[doc = r" must have signature `fn(pin : GpioInterruptPin)`."]
// #[doc = r""]
// #[doc = r" # Example"]
// #[doc = r""]
// #[doc = r" ``` ignore"]
// #[doc = r" gpio_interrupt!(buttons_handler);"]
// #[doc = r""]
// #[doc = r" fn buttons_handler(_pin : GpioInterruptPin) {"]
// #[doc = r#"     print!("A GPIO pin was pressed!");"#]
// #[doc = r" }"]
// #[doc = r""]
// macro_rules! gpio_interrupt {
//     ($ path : path) => {
//         #[allow(non_snake_case)]
//         #[unsafe(no_mangle)]
//         pub extern "C" fn GpioInterrupt(pin: GpioInterruptPin) {
//             let f: fn(pin: GpioInterruptPin) = $path;
//             f(pin);
//         }
//     };
// }

// #[macro_export]
// #[doc = r" Assigns a handler to SensInterrupt"]
// macro_rules! sens_interrupt {
//     ($ path : path) => {
//         #[allow(non_snake_case)]
//         #[unsafe(no_mangle)]
//         pub extern "C" fn SensInterrupt(status: SensInterruptStatus) {
//             let f: fn(status: SensInterruptStatus) = $path;
//             f(status);
//         }
//     };
// }

// #[allow(unused)]
// pub use gpio_interrupt;
// #[allow(unused)]
// pub use sens_interrupt;
