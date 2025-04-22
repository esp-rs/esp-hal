use super::ExceptionCause;

/// State of the CPU saved when entering exception or interrupt
///
/// Must be aligned with assembly frame format in asm.rs
#[repr(C)]
#[allow(non_snake_case)]
#[derive(Debug, Clone, Copy)]
pub struct Context {
    pub PC: u32,
    pub PS: u32,

    pub A0: u32,
    pub A1: u32,
    pub A2: u32,
    pub A3: u32,
    pub A4: u32,
    pub A5: u32,
    pub A6: u32,
    pub A7: u32,
    pub A8: u32,
    pub A9: u32,
    pub A10: u32,
    pub A11: u32,
    pub A12: u32,
    pub A13: u32,
    pub A14: u32,
    pub A15: u32,
    pub SAR: u32,
    pub EXCCAUSE: u32,
    pub EXCVADDR: u32,
    pub LBEG: u32,
    pub LEND: u32,
    pub LCOUNT: u32,
    pub THREADPTR: u32,
    pub SCOMPARE1: u32,
    pub BR: u32,
    pub ACCLO: u32,
    pub ACCHI: u32,
    pub M0: u32,
    pub M1: u32,
    pub M2: u32,
    pub M3: u32,
    #[cfg(XCHAL_HAVE_CP)]
    // Either F64R_LO or CPENABLE depending on `float-save-restore`.
    pub F64R_LO_CPENABLE: u32,
    // F64R_HI is only meaningful with cfg!(XCHAL_HAVE_DFP_ACCEL) but it's present in the stack
    // frame unconditionally
    #[cfg(feature = "float-save-restore")]
    pub F64R_HI: u32,
    // F64S is only meaningful with cfg!(XCHAL_HAVE_DFP_ACCEL) but it's present in the stack frame
    // unconditionally
    #[cfg(feature = "float-save-restore")]
    pub F64S: u32,
    #[cfg(all(feature = "float-save-restore", XCHAL_HAVE_FP))]
    pub FCR: u32,
    #[cfg(all(feature = "float-save-restore", XCHAL_HAVE_FP))]
    pub FSR: u32,
    #[cfg(all(feature = "float-save-restore", XCHAL_HAVE_FP))]
    pub F0: u32,
    #[cfg(all(feature = "float-save-restore", XCHAL_HAVE_FP))]
    pub F1: u32,
    #[cfg(all(feature = "float-save-restore", XCHAL_HAVE_FP))]
    pub F2: u32,
    #[cfg(all(feature = "float-save-restore", XCHAL_HAVE_FP))]
    pub F3: u32,
    #[cfg(all(feature = "float-save-restore", XCHAL_HAVE_FP))]
    pub F4: u32,
    #[cfg(all(feature = "float-save-restore", XCHAL_HAVE_FP))]
    pub F5: u32,
    #[cfg(all(feature = "float-save-restore", XCHAL_HAVE_FP))]
    pub F6: u32,
    #[cfg(all(feature = "float-save-restore", XCHAL_HAVE_FP))]
    pub F7: u32,
    #[cfg(all(feature = "float-save-restore", XCHAL_HAVE_FP))]
    pub F8: u32,
    #[cfg(all(feature = "float-save-restore", XCHAL_HAVE_FP))]
    pub F9: u32,
    #[cfg(all(feature = "float-save-restore", XCHAL_HAVE_FP))]
    pub F10: u32,
    #[cfg(all(feature = "float-save-restore", XCHAL_HAVE_FP))]
    pub F11: u32,
    #[cfg(all(feature = "float-save-restore", XCHAL_HAVE_FP))]
    pub F12: u32,
    #[cfg(all(feature = "float-save-restore", XCHAL_HAVE_FP))]
    pub F13: u32,
    #[cfg(all(feature = "float-save-restore", XCHAL_HAVE_FP))]
    pub F14: u32,
    #[cfg(all(feature = "float-save-restore", XCHAL_HAVE_FP))]
    pub F15: u32,
}

impl Default for Context {
    fn default() -> Self {
        Self::new()
    }
}

impl Context {
    /// Creates a new, zeroed out context.
    pub const fn new() -> Self {
        unsafe { core::mem::zeroed() }
    }
}

unsafe extern "Rust" {
    /// The exception assembly jumps here once registers have been spilled
    fn __exception(cause: ExceptionCause, save_frame: &mut Context);
    /// This symbol will be provided by the user via `#[exception]`
    fn __user_exception(cause: ExceptionCause, save_frame: &mut Context);
    /// No attribute is supplied for this symbol as the double exception can
    /// hardly occur
    fn __double_exception(cause: ExceptionCause, save_frame: &mut Context);

    /// This symbol will be provided by the user via `#[interrupt(1)]`
    fn __level_1_interrupt(save_frame: &mut Context);
    /// This symbol will be provided by the user via `#[interrupt(2)]`
    fn __level_2_interrupt(save_frame: &mut Context);
    /// This symbol will be provided by the user via `#[interrupt(3)]`
    fn __level_3_interrupt(save_frame: &mut Context);
    /// This symbol will be provided by the user via `#[interrupt(4)]`
    fn __level_4_interrupt(save_frame: &mut Context);
    /// This symbol will be provided by the user via `#[interrupt(5)]`
    fn __level_5_interrupt(save_frame: &mut Context);
    /// This symbol will be provided by the user via `#[interrupt(6)]`
    fn __level_6_interrupt(save_frame: &mut Context);
    /// This symbol will be provided by the user via `#[interrupt(7)]`
    fn __level_7_interrupt(save_frame: &mut Context);
}

#[unsafe(no_mangle)]
#[unsafe(link_section = ".rwtext")]
unsafe extern "C" fn __default_exception(cause: ExceptionCause, save_frame: &mut Context) {
    unsafe { __user_exception(cause, save_frame) }
}

#[unsafe(no_mangle)]
#[unsafe(link_section = ".rwtext")]
extern "C" fn __default_user_exception(cause: ExceptionCause, save_frame: &Context) {
    #[cfg(any(feature = "esp32", feature = "esp32s3"))]
    if cause == ExceptionCause::Cp0Disabled {
        panic!(
            "Access to the floating point coprocessor is not allowed. You may want to enable the `float-save-restore` feature of the `xtensa-lx-rt` crate. {:08x?}",
            save_frame
        )
    }

    panic!("Exception: {:?}, {:08x?}", cause, save_frame)
}

#[unsafe(no_mangle)]
#[unsafe(link_section = ".rwtext")]
extern "C" fn __default_interrupt(level: u32, save_frame: &Context) {
    panic!("Interrupt: {:?}, {:08x?}", level, save_frame)
}

#[unsafe(no_mangle)]
#[unsafe(link_section = ".rwtext")]
extern "C" fn __default_double_exception(cause: ExceptionCause, save_frame: &Context) {
    panic!("Double Exception: {:?}, {:08x?}", cause, save_frame)
}
