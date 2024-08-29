use core::arch::asm;

use super::ExceptionCause;

/// State of the CPU saved when entering exception or interrupt
///
/// Must be aligned with assembly frame format in assembly_esp32
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
    #[cfg(all(feature = "float-save-restore", XCHAL_HAVE_DFP_ACCEL))]
    pub F64R_LO: u32,
    #[cfg(all(feature = "float-save-restore", XCHAL_HAVE_DFP_ACCEL))]
    pub F64R_HI: u32,
    #[cfg(all(feature = "float-save-restore", XCHAL_HAVE_DFP_ACCEL))]
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

extern "Rust" {
    /// The exception assembly jumps here once registers have been spilled
    fn __exception(cause: ExceptionCause, save_frame: &mut Context);
    /// This symbol will be provided by the user via `#[exception]`
    fn __user_exception(cause: ExceptionCause, save_frame: &mut Context);
    /// No attribute is supplied for this symbol as the double exception can
    /// hardly occur
    fn __double_exception(cause: ExceptionCause, save_frame: &mut Context);

    /// This symbol will be provided by the user via `#[interrupt(1)]`
    fn __level_1_interrupt(level: u32, save_frame: &mut Context);
    /// This symbol will be provided by the user via `#[interrupt(2)]`
    fn __level_2_interrupt(level: u32, save_frame: &mut Context);
    /// This symbol will be provided by the user via `#[interrupt(3)]`
    fn __level_3_interrupt(level: u32, save_frame: &mut Context);
    /// This symbol will be provided by the user via `#[interrupt(4)]`
    fn __level_4_interrupt(level: u32, save_frame: &mut Context);
    /// This symbol will be provided by the user via `#[interrupt(5)]`
    fn __level_5_interrupt(level: u32, save_frame: &mut Context);
    /// This symbol will be provided by the user via `#[interrupt(6)]`
    fn __level_6_interrupt(level: u32, save_frame: &mut Context);
    /// This symbol will be provided by the user via `#[interrupt(7)]`
    fn __level_7_interrupt(level: u32, save_frame: &mut Context);
}

#[no_mangle]
#[link_section = ".rwtext"]
unsafe extern "C" fn __default_exception(cause: ExceptionCause, save_frame: &mut Context) {
    __user_exception(cause, save_frame)
}

#[no_mangle]
#[link_section = ".rwtext"]
extern "C" fn __default_user_exception(cause: ExceptionCause, save_frame: &Context) {
    panic!("Exception: {:?}, {:08x?}", cause, save_frame)
}

#[no_mangle]
#[link_section = ".rwtext"]
extern "C" fn __default_interrupt(level: u32, save_frame: &Context) {
    panic!("Interrupt: {:?}, {:08x?}", level, save_frame)
}

#[no_mangle]
#[link_section = ".rwtext"]
extern "C" fn __default_double_exception(cause: ExceptionCause, save_frame: &Context) {
    panic!("Double Exception: {:?}, {:08x?}", cause, save_frame)
}

// Raw vector handlers
//
// The interrupt handlers all use special return instructions.
// rust still generates a ret.w instruction, which will never be reached.
// generation of the ret.w can be prevented by using
// core::intrinsics::unreachable, but then a break 15,1 will be generated (which
// takes 3 bytes instead of 2) or a 'loop {}', but then a jump to own address
// will be generated which is also 3 bytes. No way found yet to prevent this
// generation altogether.

#[naked]
#[no_mangle]
#[link_section = ".KernelExceptionVector.text"]
unsafe extern "C" fn _KernelExceptionVector() {
    asm!(
        "
        wsr a0, EXCSAVE1 // preserve a0
        rsr a0, EXCCAUSE // get exception cause

        beqi a0, 5, .AllocAException

        call0 __naked_kernel_exception
        ",
        options(noreturn)
    );
}

#[naked]
#[no_mangle]
#[link_section = ".UserExceptionVector.text"]
unsafe extern "C" fn _UserExceptionVector() {
    asm!(
        "
        wsr a0, EXCSAVE1 // preserve a0
        rsr a0, EXCCAUSE // get exception cause

        beqi a0, 5, .AllocAException

        call0 __naked_user_exception

        .AllocAException:
        call0  _AllocAException
        ",
        options(noreturn)
    );
}

#[naked]
#[no_mangle]
#[link_section = ".DoubleExceptionVector.text"]
unsafe extern "C" fn _DoubleExceptionVector() {
    asm!(
        "
    wsr a0, EXCSAVE1                   // preserve a0 (EXCSAVE1 can be reused as long as there
                                       // is no double exception in the first exception until
                                       // EXCSAVE1 is stored to the stack.)
    call0 __naked_double_exception     // used as long jump
    ",
        options(noreturn)
    );
}

#[naked]
#[no_mangle]
#[link_section = ".Level2InterruptVector.text"]
unsafe extern "C" fn _Level2InterruptVector() {
    asm!(
        "
    wsr a0, EXCSAVE2 // preserve a0
    call0 __naked_level_2_interrupt     // used as long jump
    ",
        options(noreturn)
    );
}

#[naked]
#[no_mangle]
#[link_section = ".Level3InterruptVector.text"]
unsafe extern "C" fn _Level3InterruptVector() {
    asm!(
        "
    wsr a0, EXCSAVE3 // preserve a0
    call0 __naked_level_3_interrupt     // used as long jump
    ",
        options(noreturn)
    );
}

#[naked]
#[no_mangle]
#[link_section = ".Level4InterruptVector.text"]
unsafe extern "C" fn _Level4InterruptVector() {
    asm!(
        "
    wsr a0, EXCSAVE4 // preserve a0
    call0 __naked_level_4_interrupt     // used as long jump
    ",
        options(noreturn)
    );
}

#[naked]
#[no_mangle]
#[link_section = ".Level5InterruptVector.text"]
unsafe extern "C" fn _Level5InterruptVector() {
    asm!(
        "
    wsr a0, EXCSAVE5 // preserve a0
    call0 __naked_level_5_interrupt     // used as long jump
    ",
        options(noreturn)
    );
}

#[naked]
#[no_mangle]
#[link_section = ".DebugExceptionVector.text"]
unsafe extern "C" fn _Level6InterruptVector() {
    asm!(
        "
    wsr a0, EXCSAVE6 // preserve a0
    call0 __naked_level_6_interrupt     // used as long jump
    ",
        options(noreturn)
    );
}

#[naked]
#[no_mangle]
#[link_section = ".NMIExceptionVector.text"]
unsafe extern "C" fn _Level7InterruptVector() {
    asm!(
        "
    wsr a0, EXCSAVE7 // preserve a0
    call0 __naked_level_7_interrupt     // used as long jump
    ",
        options(noreturn)
    );
}

#[naked]
#[no_mangle]
#[link_section = ".WindowOverflow4.text"]
unsafe extern "C" fn _WindowOverflow4() {
    asm!(
        "
        s32e    a0, a5, -16
        s32e    a1, a5, -12
        s32e    a2, a5,  -8
        s32e    a3, a5,  -4
        rfwo
    ",
        options(noreturn)
    );
}

#[naked]
#[no_mangle]
#[link_section = ".WindowUnderflow4.text"]
unsafe extern "C" fn _WindowUnderflow4() {
    asm!(
        "
        l32e    a0, a5, -16
        l32e    a1, a5, -12
        l32e    a2, a5,  -8
        l32e    a3, a5,  -4
        rfwu

        // inline the _AllocAException saves on the ret.w for WindowUnderflow4
        // this makes that it just fits, which is needed for the bbci instructions

        .align 4
        _AllocAException:
        rsr     a0, WINDOWBASE  // grab WINDOWBASE before rotw changes it
        rotw    -1              // WINDOWBASE goes to a4, new a0-a3 are scratch
        rsr     a2, PS
        extui   a3, a2, 8, 4    // XCHAL_PS_OWB_SHIFT, XCHAL_PS_OWB_BITS
        xor     a3, a3, a4      // bits changed from old to current windowbase
        rsr     a4, EXCSAVE1    // restore original a0 (now in a4)
        slli    a3, a3, 8       // XCHAL_PS_OWB_SHIFT
        xor     a2, a2, a3      // flip changed bits in old window base
        wsr     a2, PS          // update PS.OWB to new window base
        rsync

        bbci    a4, 31, _WindowUnderflow4
        rotw    -1              // original a0 goes to a8
        bbci    a8, 30, _WindowUnderflow8
        rotw    -1
        j               _WindowUnderflow12
        ",
        options(noreturn)
    );
}

#[naked]
#[no_mangle]
#[link_section = ".WindowOverflow8.text"]
unsafe extern "C" fn _WindowOverflow8() {
    asm!(
        "
        s32e    a0, a9, -16
        l32e    a0, a1, -12

        s32e    a1, a9, -12
        s32e    a2, a9,  -8
        s32e    a3, a9,  -4
        s32e    a4, a0, -32
        s32e    a5, a0, -28
        s32e    a6, a0, -24
        s32e    a7, a0, -20
        rfwo
    ",
        options(noreturn)
    );
}

#[naked]
#[no_mangle]
#[link_section = ".WindowUnderflow8.text"]
unsafe extern "C" fn _WindowUnderflow8() {
    asm!(
        "
        l32e    a0, a9, -16
        l32e    a1, a9, -12
        l32e    a2, a9,  -8
        l32e    a7, a1, -12

        l32e    a3, a9,  -4
        l32e    a4, a7, -32
        l32e    a5, a7, -28
        l32e    a6, a7, -24
        l32e    a7, a7, -20
        rfwu
    ",
        options(noreturn)
    );
}

#[naked]
#[no_mangle]
#[link_section = ".WindowOverflow12.text"]
unsafe extern "C" fn _WindowOverflow12() {
    asm!(
        "
        s32e    a0,  a13, -16
        l32e    a0,  a1,  -12

        s32e    a1,  a13, -12
        s32e    a2,  a13,  -8
        s32e    a3,  a13,  -4
        s32e    a4,  a0,  -48
        s32e    a5,  a0,  -44
        s32e    a6,  a0,  -40
        s32e    a7,  a0,  -36
        s32e    a8,  a0,  -32
        s32e    a9,  a0,  -28
        s32e    a10, a0,  -24
        s32e    a11, a0,  -20
        rfwo
    ",
        options(noreturn)
    );
}

#[naked]
#[no_mangle]
#[link_section = ".WindowUnderflow12.text"]
unsafe extern "C" fn _WindowUnderflow12() {
    asm!(
        "
        l32e    a0,  a13, -16
        l32e    a1,  a13, -12
        l32e    a2,  a13,  -8
        l32e    a11, a1,  -12

        l32e    a3,  a13,  -4
        l32e    a4,  a11, -48
        l32e    a5,  a11, -44
        l32e    a6,  a11, -40
        l32e    a7,  a11, -36
        l32e    a8,  a11, -32
        l32e    a9,  a11, -28
        l32e    a10, a11, -24
        l32e    a11, a11, -20
        rfwu
    ",
        options(noreturn)
    );
}
