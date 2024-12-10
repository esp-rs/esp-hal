use core::arch::global_asm;

use crate::cfg_global_asm;

// We could cfg symbols away and reduce frame size depending on features enabled
// i.e the frame size is a fixed size based on all the features right now
// we know at compile time if a target has loops for example, if it doesn't
// we could cut that memory usage.
// However in order to conveniently use `addmi` we need 256-byte alignment
// anyway so wasting a bit more stack space seems to be the better option.
//
// The only exception is XT_STK_F64R_LO_CPENABLE which combines two register
// values depending on `float-save-restore` feature. This is done so that their
// position stays the same in `Context` without creating a large hole in the
// struct.
//
// Additionally there is a chunk of memory reserved for spilled registers.
//
// With the exception of XT_STK_TMP, the fields must be aligned with the
// `Context` struct in context.rs
global_asm!(
    "
    .set XT_STK_PC,             0
    .set XT_STK_PS,              4
    .set XT_STK_A0,              8
    .equ XT_STK_A1,             12
    .set XT_STK_A2,             16
    .set XT_STK_A3,             20
    .set XT_STK_A4,             24
    .set XT_STK_A5,             28
    .set XT_STK_A6,             32
    .set XT_STK_A7,             36
    .set XT_STK_A8,             40
    .set XT_STK_A9,             44
    .set XT_STK_A10,            48
    .set XT_STK_A11,            52
    .set XT_STK_A12,            56
    .set XT_STK_A13,            60
    .set XT_STK_A14,            64
    .set XT_STK_A15,            68
    .set XT_STK_SAR,            72
    .set XT_STK_EXCCAUSE,       76
    .set XT_STK_EXCVADDR,       80
    .set XT_STK_LBEG,           84 // Registers for Loop Option
    .set XT_STK_LEND,           88
    .set XT_STK_LCOUNT,         92
    .set XT_STK_THREADPTR,      96 // freely usable 32-bit register intended for TLS
    .set XT_STK_SCOMPARE1,     100 // Register for s32ci instruction
    .set XT_STK_BR,            104 // Register for Boolean Option
    .set XT_STK_ACCLO,         108 // Registers for MAC16 option
    .set XT_STK_ACCHI,         112
    .set XT_STK_M0,            116
    .set XT_STK_M1,            120
    .set XT_STK_M2,            124
    .set XT_STK_M3,            128
    // if `float-save-restore` is enabled: Registers for double support option.
    // Otherwise, the saved value of CPENABLE
    .set XT_STK_F64R_LO_CPENABLE, 132
    .set XT_STK_F64R_HI,       136
    .set XT_STK_F64S,          140
    .set XT_STK_FCR,           144 // Registers for floating point coprocessor
    .set XT_STK_FSR,           148
    .set XT_STK_F0,            152
    .set XT_STK_F1,            156
    .set XT_STK_F2,            160
    .set XT_STK_F3,            164
    .set XT_STK_F4,            168
    .set XT_STK_F5,            172
    .set XT_STK_F6,            176
    .set XT_STK_F7,            180
    .set XT_STK_F8,            184
    .set XT_STK_F9,            188
    .set XT_STK_F10,           192
    .set XT_STK_F11,           196
    .set XT_STK_F12,           200
    .set XT_STK_F13,           204
    .set XT_STK_F14,           208
    .set XT_STK_F15,           212
    .set XT_STK_TMP,           216

    .set XT_STK_FRMSZ,         256      // needs to be multiple of 16 and enough additional free space
                                        // for the registers spilled to the stack (max 8 registers / 0x20 bytes)
                                        // multiple of 256 allows use of addmi instruction



    .set PS_INTLEVEL_EXCM, 3	        // interrupt handlers above this level shouldn't be written in high level languages
    .set PS_INTLEVEL_MASK, 0x0000000f
    .set PS_EXCM,          0x00000010
    .set PS_UM,            0x00000020
    .set PS_WOE,           0x00040000

    // Spills all active windowed registers (i.e. registers not visible as
    // A0-A15) to their ABI-defined spill regions on the stack.
    // It will spill registers to their reserved locations in previous frames.
    //
    // Unlike the Xtensa HAL implementation, this code requires that the
    // EXCM and WOE bit be enabled in PS, and relies on repeated hardware
    // exception handling to do the register spills.  The trick is to do a
    // noop write to the high registers, which the hardware will trap
    // (into an overflow exception) in the case where those registers are
    // already used by an existing call frame.  Then it rotates the window
    // and repeats until all but the A0-A3 registers of the original frame
    // are guaranteed to be spilled, eventually rotating back around into
    // the original frame.  Advantages:
    //
    // - Vastly smaller code size
    //
    // - More easily maintained if changes are needed to window over/underflow
    //   exception handling.
    //
    // - Requires no scratch registers to do its work, so can be used safely in any
    //   context.
    //
    // - If the WOE bit is not enabled (for example, in code written for
    //   the CALL0 ABI), this becomes a silent noop and operates compatbily.
    //
    // - Hilariously it's ACTUALLY FASTER than the HAL routine.  And not
    //   just a little bit, it's MUCH faster.  With a mostly full register
    //   file on an LX6 core (ESP-32) I'm measuring 145 cycles to spill
    //   registers with this vs. 279 (!) to do it with
    //   xthal_spill_windows().

    .macro SPILL_REGISTERS
    and a12, a12, a12
    rotw 3
    and a12, a12, a12
    rotw 3
    and a12, a12, a12
    rotw 3
    and a12, a12, a12
    rotw 3
    and a12, a12, a12
    rotw 4
    .endm

    .macro SAVE_CONTEXT level:req
    mov     a0, a1                     // save a1/sp
    addmi   sp, sp, -XT_STK_FRMSZ      // only allow multiple of 256

    s32i    a0, sp, +XT_STK_A1         // save interruptee's A1/SP
    s32e    a0, sp, -12                // for debug backtrace

    .ifc \\level,1
    rsr     a0, PS
    s32i    a0, sp, +XT_STK_PS         // save interruptee's PS

    rsr     a0, EXCCAUSE
    s32i    a0, sp, +XT_STK_EXCCAUSE
    rsr     a0, EXCVADDR
    s32i    a0, sp, +XT_STK_EXCVADDR
    .else
    rsr     a0, EPS\\level
    s32i    a0, sp, +XT_STK_PS         // save interruptee's PS
    .endif

    rsr     a0, EPC\\level
    s32i    a0, sp, +XT_STK_PC         // save interruptee's PC
    s32e    a0, sp, -16                // for debug backtrace

    rsr     a0, EXCSAVE\\level
    s32i    a0, sp, +XT_STK_A0         // save interruptee's A0

    call0   save_context

    .endm

    .macro RESTORE_CONTEXT level:req

    // Restore context and return
    call0   restore_context

    .ifc \\level,1
    l32i    a0, sp, +XT_STK_PS        // retrieve interruptee's PS
    wsr     a0, PS
    l32i    a0, sp, +XT_STK_PC        // retrieve interruptee's PC
    wsr     a0, EPC\\level
    .else
    l32i    a0, sp, +XT_STK_PS        // retrieve interruptee's PS
    wsr     a0, EPS\\level
    l32i    a0, sp, +XT_STK_PC        // retrieve interruptee's PC
    wsr     a0, EPC\\level
    .endif

    l32i    a0, sp, +XT_STK_A0        // retrieve interruptee's A0
    l32i    sp, sp, +XT_STK_A1        // remove exception frame
    rsync                             // ensure PS and EPC written

    .endm

    .macro HANDLE_INTERRUPT_LEVEL level
    SAVE_CONTEXT \\level

    movi    a0, (\\level | PS_WOE)
    wsr     a0, PS
    rsync

    movi    a6, \\level                     // put interrupt level in a6 = a2 in callee
    mov     a7, sp                         // put address of save frame in a7=a3 in callee
    call4   __level_\\level\\()_interrupt    // call handler <= actual call!

    RESTORE_CONTEXT \\level
    rfi \\level

    .endm
    "
);

cfg_global_asm!(
    "
    // Save processor state to stack.
    //
    // *Must only be called with call0.*
    // *For spill all window registers to work WOE must be enabled on entry
    //
    // Saves all registers except PC, PS, A0, A1
    //
    // Inputs:
    //     A0 is the return address
    //     A1 is the stack pointers
    //     Exceptions are disabled (PS.EXCM = 1)
    //
    // Output:
    //     A0 is the return address
    //     A1 is the stack pointer
    //     A3, A9 are used as scratch registers
    //     EPC1 is changed
    .section .rwtext,\"ax\",@progbits
    .global save_context
    .p2align 2
    .type save_context,@function
save_context:
.Lsave_context_start:
    s32i    a2,  sp, +XT_STK_A2
    s32i    a3,  sp, +XT_STK_A3
    s32i    a4,  sp, +XT_STK_A4
    s32i    a5,  sp, +XT_STK_A5
    s32i    a6,  sp, +XT_STK_A6
    s32i    a7,  sp, +XT_STK_A7
    s32i    a8,  sp, +XT_STK_A8
    s32i    a9,  sp, +XT_STK_A9
    s32i    a10, sp, +XT_STK_A10
    s32i    a11, sp, +XT_STK_A11
    s32i    a12, sp, +XT_STK_A12
    s32i    a13, sp, +XT_STK_A13
    s32i    a14, sp, +XT_STK_A14
    s32i    a15, sp, +XT_STK_A15

    rsr     a3,  SAR
    s32i    a3,  sp, +XT_STK_SAR
    ",
    #[cfg(all(XCHAL_HAVE_CP, not(feature = "float-save-restore")))]
    "
    /* Disable coprocessor, any use of floats in ISRs will cause an exception unless float-save-restore feature is enabled */
    rsr     a3, CPENABLE
    s32i    a3, sp, +XT_STK_F64R_LO_CPENABLE
    movi    a3,  0
    wsr     a3,  CPENABLE
    rsync
    ",
    #[cfg(XCHAL_HAVE_LOOPS)]
    "
    // Loop Option
    rsr     a3,  LBEG
    s32i    a3,  sp, +XT_STK_LBEG
    rsr     a3,  LEND
    s32i    a3,  sp, +XT_STK_LEND
    rsr     a3,  LCOUNT
    s32i    a3,  sp, +XT_STK_LCOUNT
    ",
    #[cfg(XCHAL_HAVE_THREADPTR)]
    "
    // Thread Pointer Option
    rur     a3, threadptr
    s32i    a3, sp, +XT_STK_THREADPTR
    ",
    #[cfg(XCHAL_HAVE_S32C1I)]
    "
    // Conditional Store Option
    rsr     a3, scompare1
    s32i    a3, sp, +XT_STK_SCOMPARE1
    ",
    #[cfg(XCHAL_HAVE_BOOLEANS)]
    "
    // Boolean Option
    rsr     a3, br
    s32i    a3, sp, +XT_STK_BR
    ",
    #[cfg(XCHAL_HAVE_MAC16)]
    "
    // MAC16 Option
    rsr     a3, acclo
    s32i    a3, sp, +XT_STK_ACCLO
    rsr     a3, acchi
    s32i    a3, sp, +XT_STK_ACCHI
    rsr     a3, m0
    s32i    a3, sp, +XT_STK_M0
    rsr     a3, m1
    s32i    a3, sp, +XT_STK_M1
    rsr     a3, m2
    s32i    a3, sp, +XT_STK_M2
    rsr     a3, m3
    s32i    a3, sp, +XT_STK_M3
    ",
    #[cfg(all(feature = "float-save-restore", XCHAL_HAVE_DFP_ACCEL))]
    "
    // Double Precision Accelerator Option
    rur     a3, f64r_lo
    s32i    a3, sp, +XT_STK_F64R_LO_CPENABLE
    rur     a3, f64r_hi
    s32i    a3, sp, +XT_STK_F64R_HI
    rur     a3, f64s
    s32i    a3, sp, +XT_STK_F64S
    ",
    #[cfg(all(feature = "float-save-restore", XCHAL_HAVE_FP))]
    "
    // Coprocessor Option
    rur     a3, fcr
    s32i    a3, sp, +XT_STK_FCR
    rur     a3, fsr
    s32i    a3, sp, +XT_STK_FSR
    ssi     f0, sp, +XT_STK_F0
    ssi     f1, sp, +XT_STK_F1
    ssi     f2, sp, +XT_STK_F2
    ssi     f3, sp, +XT_STK_F3
    ssi     f4, sp, +XT_STK_F4
    ssi     f5, sp, +XT_STK_F5
    ssi     f6, sp, +XT_STK_F6
    ssi     f7, sp, +XT_STK_F7
    ssi     f8, sp, +XT_STK_F8
    ssi     f9, sp, +XT_STK_F9
    ssi     f10, sp, +XT_STK_F10
    ssi     f11, sp, +XT_STK_F11
    ssi     f12, sp, +XT_STK_F12
    ssi     f13, sp, +XT_STK_F13
    ssi     f14, sp, +XT_STK_F14
    ssi     f15, sp, +XT_STK_F15
    ",
    #[cfg(XCHAL_HAVE_WINDOWED)]
    "
    s32i    a0, sp, +XT_STK_TMP        // keep return address on the stack

    // SPILL_REGISTERS macro requires window overflow exceptions to be enabled,
    // i.e. PS.EXCM cleared and PS.WOE set.
    // Since we are going to clear PS.EXCM, we also need to increase INTLEVEL
    // at least to XCHAL_EXCM_LEVEL. This matches that value of effective INTLEVEL
    // at entry (CINTLEVEL=max(PS.INTLEVEL, XCHAL_EXCM_LEVEL) when PS.EXCM is set.
    // Since WindowOverflow exceptions will trigger inside SPILL_REGISTERS,
    // need to save/restore EPC1 as well.
    // Note: even though a4-a15 are saved into the exception frame, we should not
    // clobber them until after SPILL_REGISTERS. This is because these registers
    // may contain live windows belonging to previous frames in the call stack.
    // These frames will be spilled by SPILL_REGISTERS, and if the register was
    // used as a temporary by this code, the temporary value would get stored
    // onto the stack, instead of the real value.
    //

    rsr     a2, PS                     // to be restored after SPILL_REGISTERS
    movi    a0, PS_INTLEVEL_MASK
    and     a3, a2, a0                 // get the current INTLEVEL
    bgeui   a3, +PS_INTLEVEL_EXCM, 1f  // calculate max(INTLEVEL, XCHAL_EXCM_LEVEL) - 3 = XCHAL_EXCM_LEVEL
    movi    a3, PS_INTLEVEL_EXCM
    1:
    movi    a0, PS_WOE       // clear EXCM, enable window overflow, set new INTLEVEL
    or      a3, a3, a0
    wsr     a3, ps
    rsr     a0, EPC1

    addmi   sp,  sp, +XT_STK_FRMSZ   // go back to spill register region
    SPILL_REGISTERS
    addmi   sp,  sp, -XT_STK_FRMSZ   // return the current stack pointer

    wsr     a2, PS                   //  restore to the value at entry
    rsync
    wsr     a0, EPC1

    l32i    a0,  sp, +XT_STK_TMP
    ",
    "
    ret
.Lsave_context_end:
    .size .Lsave_context_start, .Lsave_context_end

    .section .rwtext,\"ax\",@progbits
    .global restore_context
    .p2align 2
    .type restore_context,@function
restore_context:
.Lrestore_context_start:
    l32i    a3,  sp, +XT_STK_SAR
    wsr     a3,  SAR
    ",
    #[cfg(XCHAL_HAVE_LOOPS)]
    "
    // Loop Option
    l32i    a3,  sp, +XT_STK_LBEG
    wsr     a3,  LBEG
    l32i    a3,  sp, +XT_STK_LEND
    wsr     a3,  LEND
    l32i    a3,  sp, +XT_STK_LCOUNT
    wsr     a3,  LCOUNT
    ",
    #[cfg(XCHAL_HAVE_THREADPTR)]
    "
    // Thread Pointer Option
    l32i    a3, sp, +XT_STK_THREADPTR
    wur     a3, threadptr
    ",
    #[cfg(XCHAL_HAVE_S32C1I)]
    "
    // Conditional Store Option
    l32i    a3, sp, +XT_STK_SCOMPARE1
    wsr     a3, scompare1
    ",
    #[cfg(XCHAL_HAVE_BOOLEANS)]
    "
    // Boolean Option
    l32i    a3, sp, +XT_STK_BR
    wsr     a3, br
    ",
    #[cfg(XCHAL_HAVE_MAC16)]
    "
    // MAC16 Option
    l32i    a3, sp, +XT_STK_ACCLO
    wsr     a3, acclo
    l32i    a3, sp, +XT_STK_ACCHI
    wsr     a3, acchi
    l32i    a3, sp, +XT_STK_M0
    wsr     a3, m0
    l32i    a3, sp, +XT_STK_M1
    wsr     a3, m1
    l32i    a3, sp, +XT_STK_M2
    wsr     a3, m2
    l32i    a3, sp, +XT_STK_M3
    wsr     a3, m3
    ",
    #[cfg(all(feature = "float-save-restore", XCHAL_HAVE_DFP_ACCEL))]
    "
    // Double Precision Accelerator Option
    l32i    a3, sp, +XT_STK_F64R_LO_CPENABLE
    wur     a3, f64r_lo
    l32i    a3, sp, +XT_STK_F64R_HI
    wur     a3, f64r_hi
    l32i    a3, sp, +XT_STK_F64S
    wur     a3, f64s
    ",
    #[cfg(all(feature = "float-save-restore", XCHAL_HAVE_FP))]
    "
    // Coprocessor Option
    l32i    a3, sp, +XT_STK_FCR
    wur     a3, fcr
    l32i    a3, sp, +XT_STK_FSR
    wur     a3, fsr
    lsi     f0, sp, +XT_STK_F0
    lsi     f1, sp, +XT_STK_F1
    lsi     f2, sp, +XT_STK_F2
    lsi     f3, sp, +XT_STK_F3
    lsi     f4, sp, +XT_STK_F4
    lsi     f5, sp, +XT_STK_F5
    lsi     f6, sp, +XT_STK_F6
    lsi     f7, sp, +XT_STK_F7
    lsi     f8, sp, +XT_STK_F8
    lsi     f9, sp, +XT_STK_F9
    lsi     f10, sp, +XT_STK_F10
    lsi     f11, sp, +XT_STK_F11
    lsi     f12, sp, +XT_STK_F12
    lsi     f13, sp, +XT_STK_F13
    lsi     f14, sp, +XT_STK_F14
    lsi     f15, sp, +XT_STK_F15
    ",
    #[cfg(all(XCHAL_HAVE_CP, not(feature = "float-save-restore")))]
    "
    /* Restore coprocessor state after ISR */
    l32i    a3, sp, +XT_STK_F64R_LO_CPENABLE
    wsr     a3,  CPENABLE
    rsync
    ",
    "
    // general registers
    l32i    a2,  sp, +XT_STK_A2
    l32i    a3,  sp, +XT_STK_A3
    l32i    a4,  sp, +XT_STK_A4
    l32i    a5,  sp, +XT_STK_A5
    l32i    a6,  sp, +XT_STK_A6
    l32i    a7,  sp, +XT_STK_A7
    l32i    a8,  sp, +XT_STK_A8
    l32i    a9,  sp, +XT_STK_A9
    l32i    a10, sp, +XT_STK_A10
    l32i    a11, sp, +XT_STK_A11
    l32i    a12, sp, +XT_STK_A12
    l32i    a13, sp, +XT_STK_A13
    l32i    a14, sp, +XT_STK_A14
    l32i    a15, sp, +XT_STK_A15
    ret
.Lrestore_context_end:
    .size .Lrestore_context_start, .Lrestore_context_end
    ",
);

global_asm!(
    "
    // Handle Other Exceptions or Level 1 interrupt by storing full context and
    // then calling regular function
    //
    // # Input:
    //    * A0 stored in EXCSAVE1

    .section .rwtext,\"ax\",@progbits
    .global __default_naked_exception
    .p2align 2
    .type __default_naked_exception,@function
__default_naked_exception:
.Ldefault_naked_exception_start:
    SAVE_CONTEXT 1

    movi    a0, (PS_INTLEVEL_EXCM | PS_WOE)
    wsr     a0, PS
    rsync

    l32i    a6, sp, +XT_STK_EXCCAUSE  // put cause in a6 = a2 in callee
    beqi    a6, 4, .Level1Interrupt

    mov     a7, sp                    // put address of save frame in a7=a3 in callee
    call4   __exception               // call handler <= actual call!

    j       .RestoreContext

.Level1Interrupt:
    movi    a0, (1 | PS_WOE)          // set PS.INTLEVEL accordingly
    wsr     a0, PS
    rsync

    movi    a6, 1                     // put interrupt level in a6 = a2 in callee
    mov     a7, sp                    // put address of save frame in a7=a3 in callee
    call4   __level_1_interrupt       // call handler <= actual call!

.RestoreContext:
    RESTORE_CONTEXT 1

    rfe                               // PS.EXCM is cleared
.Ldefault_naked_exception_end:
    .size .Ldefault_naked_exception_start, .Ldefault_naked_exception_end

    // Handle Double Exceptions by storing full context and then calling regular
    // function Double exceptions are not a normal occurrence. They indicate a bug
    // of some kind.
    //
    // # Input:
    //    * A0 stored in EXCSAVE1
    .section .rwtext,\"ax\",@progbits
    .global __default_naked_double_exception
    .p2align 2
    .type __default_naked_double_exception,@function
__default_naked_double_exception:
.Ldefault_double_naked_exception_start:
    mov     a0, a1                     // save a1/sp
    addmi   sp, sp, -XT_STK_FRMSZ      // only allow multiple of 256

    s32i    a0, sp, +XT_STK_A1         // save interruptee's A1/SP
    s32e    a0, sp, -12                // for debug backtrace

    rsr     a0, PS
    s32i    a0, sp, +XT_STK_PS         // save interruptee's PS

    rsr     a0, EXCCAUSE
    s32i    a0, sp, +XT_STK_EXCCAUSE
    rsr     a0, EXCVADDR
    s32i    a0, sp, +XT_STK_EXCVADDR

    rsr     a0, DEPC
    s32i    a0, sp, +XT_STK_PC         // save interruptee's PC
    s32e    a0, sp, -16                // for debug backtrace

    rsr     a0, EXCSAVE7               // ok to reuse EXCSAVE7 for double exception as long as
                                        // double exception is not in first couple of instructions
                                        // of level 7 handler
    s32i    a0, sp, +XT_STK_A0         // save interruptee's A0

    call0   save_context

    l32i    a6, sp, +XT_STK_EXCCAUSE  // put cause in a6 = a2 in callee
    mov     a7, sp                    // put address of save frame in a7=a3 in callee
    call4   __exception               // call handler <= actual call!

    // Restore context and return
    call0   restore_context

    l32i    a0, sp, +XT_STK_PS        // retrieve interruptee's PS
    wsr     a0, PS
    l32i    a0, sp, +XT_STK_PC        // retrieve interruptee's PC
    wsr     a0, EPC1

    l32i    a0, sp, +XT_STK_A0        // retrieve interruptee's A0
    l32i    sp, sp, +XT_STK_A1        // remove exception frame
    rsync                             // ensure PS and EPC written

    rfde
.Ldefault_double_naked_exception_end:
    .size .Ldefault_double_naked_exception_start, .Ldefault_double_naked_exception_end

    // Handle Level 2 Interrupt by storing full context and then calling regular
    // function
    //
    // # Input:
    //    * A0 stored in EXCSAVE2
    .section .rwtext,\"ax\",@progbits
    .global __default_naked_level_2_interrupt
    .p2align 2
    .type __default_naked_level_2_interrupt,@function
__default_naked_level_2_interrupt:
.Ldefault_naked_level_2_interrupt_start:
    HANDLE_INTERRUPT_LEVEL 2
.Ldefault_naked_level_2_interrupt_end:
    .size .Ldefault_naked_level_2_interrupt_start, .Ldefault_naked_level_2_interrupt_end

    // Handle Level 3 Interrupt by storing full context and then calling regular
    // function
    //
    // # Input:
    //    * A0 stored in EXCSAVE3
    .section .rwtext,\"ax\",@progbits
    .global __default_naked_level_3_interrupt
    .p2align 2
    .type __default_naked_level_3_interrupt,@function
__default_naked_level_3_interrupt:
.Ldefault_naked_level_3_interrupt_start:
    HANDLE_INTERRUPT_LEVEL 3
.Ldefault_naked_level_3_interrupt_end:
    .size .Ldefault_naked_level_3_interrupt_start, .Ldefault_naked_level_3_interrupt_end

    // Handle Level 4 Interrupt by storing full context and then calling regular
    // function
    //
    // # Input:
    //    * A0 stored in EXCSAVE4
    .section .rwtext,\"ax\",@progbits
    .global __default_naked_level_4_interrupt
    .p2align 2
    .type __default_naked_level_4_interrupt,@function
__default_naked_level_4_interrupt:
.Ldefault_naked_level_4_interrupt_start:
    HANDLE_INTERRUPT_LEVEL 4
.Ldefault_naked_level_4_interrupt_end:
    .size .Ldefault_naked_level_4_interrupt_start, .Ldefault_naked_level_4_interrupt_end

    // Handle Level 5 Interrupt by storing full context and then calling regular
    // function
    //
    // # Input:
    //    * A0 stored in EXCSAVE5
    .section .rwtext,\"ax\",@progbits
    .global __default_naked_level_5_interrupt
    .p2align 2
    .type __default_naked_level_5_interrupt,@function
__default_naked_level_5_interrupt:
.Ldefault_naked_level_5_interrupt_start:
    HANDLE_INTERRUPT_LEVEL 5
.Ldefault_naked_level_5_interrupt_end:
    .size .Ldefault_naked_level_5_interrupt_start, .Ldefault_naked_level_5_interrupt_end

    // Handle Level 6 (=Debug) Interrupt by storing full context and then calling
    // regular function
    //
    // # Input:
    //    * A0 stored in EXCSAVE6
    .section .rwtext,\"ax\",@progbits
    .global __default_naked_level_6_interrupt
    .p2align 2
    .type __default_naked_level_6_interrupt,@function
__default_naked_level_6_interrupt:
.Ldefault_naked_level_6_interrupt_start:
    HANDLE_INTERRUPT_LEVEL 6
.Ldefault_naked_level_6_interrupt_end:
    .size .Ldefault_naked_level_6_interrupt_start, .Ldefault_naked_level_6_interrupt_end

    // Handle Level 7 (=NMI) Interrupt by storing full context and then calling
    // regular function
    //
    // # Input:
    //    * A0 stored in EXCSAVE7
    .section .rwtext,\"ax\",@progbits
    .global __default_naked_level_7_interrupt
    .p2align 2
    .type __default_naked_level_7_interrupt,@function
__default_naked_level_7_interrupt:
.Ldefault_naked_level_7_interrupt_start:
    HANDLE_INTERRUPT_LEVEL 7
.Ldefault_naked_level_7_interrupt_end:
    .size .Ldefault_naked_level_7_interrupt_start, .Ldefault_naked_level_7_interrupt_end
"
);

// Raw vector handlers
//
// The interrupt handlers all use special return instructions.
// rust still generates a ret.w instruction, which will never be reached.
// generation of the ret.w can be prevented by using
// core::intrinsics::unreachable, but then a break 15,1 will be generated (which
// takes 3 bytes instead of 2) or a 'loop {}', but then a jump to own address
// will be generated which is also 3 bytes. No way found yet to prevent this
// generation altogether.
global_asm!(
    "
    .section .WindowOverflow8.text,\"ax\",@progbits
    .global _WindowOverflow8
    .p2align 2
    .type _WindowOverflow8,@function
_WindowOverflow8:
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
    
    .section .WindowUnderflow8.text,\"ax\",@progbits
    .global _WindowUnderflow8
    .p2align 2
    .type _WindowUnderflow8,@function
_WindowUnderflow8:
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

    .section .WindowOverflow12.text,\"ax\",@progbits
    .global _WindowOverflow12
    .p2align 2
    .type _WindowOverflow12,@function
_WindowOverflow12:
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

    .section .WindowUnderflow12.text,\"ax\",@progbits
    .global _WindowUnderflow12
    .p2align 2
    .type _WindowUnderflow12,@function
_WindowUnderflow12:
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

    .section .WindowOverflow4.text,\"ax\",@progbits
    .global _WindowOverflow4
    .p2align 2
    .type _WindowOverflow4,@function
_WindowOverflow4:
        s32e    a0, a5, -16
        s32e    a1, a5, -12
        s32e    a2, a5,  -8
        s32e    a3, a5,  -4
        rfwo

    .section .WindowUnderflow4.text,\"ax\",@progbits
    .global _WindowUnderflow4
    .p2align 2
    .type _WindowUnderflow4,@function
_WindowUnderflow4:
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

    .section .KernelExceptionVector.text,\"ax\",@progbits
    .global _KernelExceptionVector
    .p2align 2
    .type _KernelExceptionVector,@function
_KernelExceptionVector:
        wsr a0, EXCSAVE1 // preserve a0
        rsr a0, EXCCAUSE // get exception cause

        beqi a0, 5, .AllocAException

        call0 __naked_kernel_exception

    .section .UserExceptionVector.text,\"ax\",@progbits
    .global _UserExceptionVector
    .p2align 2
    .type _UserExceptionVector,@function
_UserExceptionVector:
        wsr a0, EXCSAVE1 // preserve a0
        rsr a0, EXCCAUSE // get exception cause

        beqi a0, 5, .AllocAException

        call0 __naked_user_exception

.AllocAException:
        call0  _AllocAException

    .section .DoubleExceptionVector.text,\"ax\",@progbits
    .global _DoubleExceptionVector
    .p2align 2
    .type _DoubleExceptionVector,@function
_DoubleExceptionVector:
        wsr a0, EXCSAVE1                // preserve a0 (EXCSAVE1 can be reused as long as there
                                        // is no double exception in the first exception until
                                        // EXCSAVE1 is stored to the stack.)
        call0 __naked_double_exception  // used as long jump

    .section .Level2InterruptVector.text,\"ax\",@progbits
    .global _Level2InterruptVector
    .p2align 2
    .type _Level2InterruptVector,@function
_Level2InterruptVector:
        wsr a0, EXCSAVE2 // preserve a0
        call0 __naked_level_2_interrupt     // used as long jump

    .section .Level3InterruptVector.text,\"ax\",@progbits
    .global _Level3InterruptVector
    .p2align 2
    .type _Level3InterruptVector,@function
_Level3InterruptVector:
        wsr a0, EXCSAVE3 // preserve a0
        call0 __naked_level_3_interrupt     // used as long jump

    .section .Level4InterruptVector.text,\"ax\",@progbits
    .global _Level4InterruptVector
    .p2align 2
    .type _Level4InterruptVector,@function
_Level4InterruptVector:
        wsr a0, EXCSAVE4 // preserve a0
        call0 __naked_level_4_interrupt     // used as long jump

    .section .Level5InterruptVector.text,\"ax\",@progbits
    .global _Level5InterruptVector
    .p2align 2
    .type _Level5InterruptVector,@function
_Level5InterruptVector:
        wsr a0, EXCSAVE5 // preserve a0
        call0 __naked_level_5_interrupt     // used as long jump

    .section .DebugExceptionVector.text,\"ax\",@progbits
    .global _Level6InterruptVector
    .p2align 2
    .type _Level6InterruptVector,@function
_Level6InterruptVector:
        wsr a0, EXCSAVE6 // preserve a0
        call0 __naked_level_6_interrupt     // used as long jump

    .section .NMIExceptionVector.text,\"ax\",@progbits
    .global _Level7InterruptVector
    .p2align 2
    .type _Level7InterruptVector,@function
_Level7InterruptVector:
        wsr a0, EXCSAVE7 // preserve a0
        call0 __naked_level_7_interrupt     // used as long jump
    "
);
