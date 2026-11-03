//! FPU tests

/// Fills `state` with a pattern that depends on `seed`, and sets the control registers.
#[cfg(float_save_restore)]
fn fpu_pattern<const N: usize>(seed: u32, control: &[u32]) -> [u32; N] {
    let mut state = [0; N];
    let (registers, control_registers) = state.split_at_mut(N - control.len());
    for (i, value) in registers.iter_mut().enumerate() {
        *value = seed.wrapping_mul(0x9e37_79b9) ^ (i as u32).wrapping_mul(0x0101_0101);
    }
    control_registers.copy_from_slice(control);
    state
}

#[cfg(all(float_save_restore, riscv))]
mod fpu_state {
    /// The values of `f0..f31` and `fcsr`.
    pub type FpuState = [u32; 33];

    /// RTZ rounding mode, NV and OF flags.
    pub const CONTROL_A: &[u32] = &[0x34];
    /// RDN rounding mode, DZ and UF flags.
    pub const CONTROL_B: &[u32] = &[0x4a];

    /// Loads `state` into the FPU and returns the FPU state after `spin` loop iterations.
    ///
    /// Interrupts are enabled only during the loop. `raise` runs with interrupts disabled, so an
    /// interrupt that `raise` triggers runs during the loop.
    pub fn hold_fpu_state(state: &FpuState, spin: u32, raise: impl FnOnce()) -> FpuState {
        let mut result = [0; 33];
        let mstatus: usize;
        unsafe { core::arch::asm!("csrrci {0}, mstatus, 8", out(reg) mstatus) };
        raise();
        unsafe {
            core::arch::asm!(
                "frcsr {fcsr}",
                "lw {tmp}, 128({state})",
                "fscsr {tmp}",
                ".irp i, 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31",
                r"flw f\i, \i*4({state})",
                ".endr",
                "csrsi mstatus, 8",
                "1:",
                "addi {spin}, {spin}, -1",
                "bnez {spin}, 1b",
                "csrci mstatus, 8",
                ".irp i, 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31",
                r"fsw f\i, \i*4({result})",
                ".endr",
                "frcsr {tmp}",
                "sw {tmp}, 128({result})",
                "fscsr {fcsr}",
                state = in(reg) state.as_ptr(),
                result = in(reg) result.as_mut_ptr(),
                spin = inout(reg) spin.max(1) => _,
                fcsr = out(reg) _,
                tmp = out(reg) _,
                out("f0") _, out("f1") _, out("f2") _, out("f3") _,
                out("f4") _, out("f5") _, out("f6") _, out("f7") _,
                out("f8") _, out("f9") _, out("f10") _, out("f11") _,
                out("f12") _, out("f13") _, out("f14") _, out("f15") _,
                out("f16") _, out("f17") _, out("f18") _, out("f19") _,
                out("f20") _, out("f21") _, out("f22") _, out("f23") _,
                out("f24") _, out("f25") _, out("f26") _, out("f27") _,
                out("f28") _, out("f29") _, out("f30") _, out("f31") _,
            );
            if mstatus & 8 != 0 {
                core::arch::asm!("csrsi mstatus, 8");
            }
        }
        result
    }

    /// Changes the caller-saved FPU registers and sets the NX flag in `fcsr`.
    pub fn clobber_fpu_state() {
        unsafe {
            core::arch::asm!(
                ".irp i, 0,1,2,3,4,5,6,7,10,11,12,13,14,15,16,17,28,29,30,31",
                r"fmv.w.x f\i, {value}",
                ".endr",
                value = in(reg) 0xdead_beef_u32,
                out("f0") _, out("f1") _, out("f2") _, out("f3") _,
                out("f4") _, out("f5") _, out("f6") _, out("f7") _,
                out("f10") _, out("f11") _, out("f12") _, out("f13") _,
                out("f14") _, out("f15") _, out("f16") _, out("f17") _,
                out("f28") _, out("f29") _, out("f30") _, out("f31") _,
            );
        }
        core::hint::black_box(core::hint::black_box(1.0_f32) / 3.0);
    }
}

#[cfg(all(float_save_restore, xtensa))]
mod fpu_state {
    /// The values of `f0..f15`, `fcr` and `fsr`.
    pub type FpuState = [u32; 18];

    /// Round toward zero, no flags.
    pub const CONTROL_A: &[u32] = &[1, 0];
    /// Round toward positive infinity, no flags.
    pub const CONTROL_B: &[u32] = &[2, 0];

    /// The interrupt level that masks all interrupts that `esp-hal` handlers use.
    const MASK_LEVEL: u32 = 3;

    /// Loads `state` into the FPU and returns the FPU state after `spin` loop iterations.
    ///
    /// Interrupts are enabled only during the loop. `raise` runs with interrupts disabled, so an
    /// interrupt that `raise` triggers runs during the loop.
    pub fn hold_fpu_state(state: &FpuState, spin: u32, raise: impl FnOnce()) -> FpuState {
        let mut result = [0; 18];
        let ps: u32;
        unsafe { core::arch::asm!("rsil {0}, {level}", out(reg) ps, level = const MASK_LEVEL) };
        raise();
        unsafe {
            core::arch::asm!(
                "rur {fcr}, fcr",
                "rur {fsr}, fsr",
                "l32i {tmp}, {state}, 64",
                "wur {tmp}, fcr",
                "l32i {tmp}, {state}, 68",
                "wur {tmp}, fsr",
                ".irp i, 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15",
                r"lsi f\i, {state}, \i*4",
                ".endr",
                "wsr {ps}, ps",
                "rsync",
                "1:",
                "addi {spin}, {spin}, -1",
                "bnez {spin}, 1b",
                "rsil {tmp}, {level}",
                ".irp i, 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15",
                r"ssi f\i, {result}, \i*4",
                ".endr",
                "rur {tmp}, fcr",
                "s32i {tmp}, {result}, 64",
                "rur {tmp}, fsr",
                "s32i {tmp}, {result}, 68",
                "wur {fcr}, fcr",
                "wur {fsr}, fsr",
                state = in(reg) state.as_ptr(),
                result = in(reg) result.as_mut_ptr(),
                ps = in(reg) ps,
                spin = inout(reg) spin.max(1) => _,
                level = const MASK_LEVEL,
                fcr = out(reg) _,
                fsr = out(reg) _,
                tmp = out(reg) _,
                out("f0") _, out("f1") _, out("f2") _, out("f3") _,
                out("f4") _, out("f5") _, out("f6") _, out("f7") _,
                out("f8") _, out("f9") _, out("f10") _, out("f11") _,
                out("f12") _, out("f13") _, out("f14") _, out("f15") _,
            );
            core::arch::asm!("wsr {0}, ps", "rsync", in(reg) ps);
        }
        result
    }

    /// Changes the FPU registers and sets the inexact flag in `fsr`.
    pub fn clobber_fpu_state() {
        unsafe {
            core::arch::asm!(
                ".irp i, 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15",
                r"wfr f\i, {value}",
                ".endr",
                value = in(reg) 0xdead_beef_u32,
                out("f0") _, out("f1") _, out("f2") _, out("f3") _,
                out("f4") _, out("f5") _, out("f6") _, out("f7") _,
                out("f8") _, out("f9") _, out("f10") _, out("f11") _,
                out("f12") _, out("f13") _, out("f14") _, out("f15") _,
            );
        }
        core::hint::black_box(core::hint::black_box(1.0_f32) / 3.0);
    }
}

#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use esp_hal::peripherals::Peripherals;
    #[cfg(multi_core)]
    use esp_hal::system::CpuControl;
    #[cfg(float_save_restore)]
    use esp_hal::{
        handler,
        interrupt::{Priority, software::SoftwareInterrupt},
        peripherals::{FROM_CPU_INTR2, FROM_CPU_INTR3},
    };
    #[cfg(float_save_restore)]
    use portable_atomic::{AtomicBool, Ordering};

    #[cfg(float_save_restore)]
    use super::{fpu_pattern, fpu_state::*};

    #[inline(never)]
    fn run_float_calc(x: f32) -> f32 {
        let result = core::hint::black_box(x) * 2.0;
        defmt::info!("{}", defmt::Display2Format(&result));
        result
    }

    #[init]
    fn init() -> Peripherals {
        crate::init_heap();

        let config = esp_hal::Config::default();
        esp_hal::init(config)
    }

    #[test]
    fn fpu_is_enabled() {
        let result = run_float_calc(2.0);
        assert_eq!(result, 4.0);
    }

    #[cfg(multi_core)]
    #[test]
    fn fpu_is_enabled_on_core1(peripherals: Peripherals) {
        use core::sync::atomic::{AtomicBool, Ordering};

        static DONE: AtomicBool = AtomicBool::new(false);

        let mut cpu_control = CpuControl::new(peripherals.CPU_CTRL);

        let guard = cpu_control
            .start_app_core(
                #[allow(static_mut_refs)]
                unsafe {
                    &mut crate::APP_CORE_STACK
                },
                || {
                    let result = run_float_calc(2.0);
                    assert_eq!(result, 4.0);
                    DONE.store(true, Ordering::Relaxed);
                    loop {}
                },
            )
            .unwrap();

        while !DONE.load(Ordering::Relaxed) {}
        let result = run_float_calc(2.0);
        assert_eq!(result, 4.0);

        core::mem::drop(guard);
    }

    #[cfg(multi_core)]
    #[test]
    fn fpu_is_enabled_on_core1_with_preempt(p: Peripherals) {
        use core::sync::atomic::{AtomicBool, Ordering};

        use esp_hal::timer::timg::TimerGroup;

        static DONE: AtomicBool = AtomicBool::new(false);

        let timg0 = TimerGroup::new(p.TIMG0);
        esp_rtos::start(timg0.timer0);

        esp_rtos::start_second_core::<8192>(
            p.CPU_CTRL,
            #[allow(static_mut_refs)]
            unsafe {
                &mut crate::APP_CORE_STACK
            },
            || {
                esp_radio_rtos_driver::usleep(10);

                let result = run_float_calc(2.0);
                assert_eq!(result, 4.0);
                DONE.store(true, Ordering::Relaxed);
                loop {}
            },
        );

        while !DONE.load(Ordering::Relaxed) {}
        let result = run_float_calc(2.0);
        assert_eq!(result, 4.0);
    }

    #[cfg(float_save_restore)]
    #[test]
    fn interrupt_handler_keeps_fpu_state(p: Peripherals) {
        static HANDLER_RAN: AtomicBool = AtomicBool::new(false);

        #[handler]
        fn handler() {
            SoftwareInterrupt::new(unsafe { FROM_CPU_INTR2::steal() }).reset();
            clobber_fpu_state();
            HANDLER_RAN.store(true, Ordering::Relaxed);
        }

        let mut interrupt = SoftwareInterrupt::new(p.FROM_CPU_INTR2);
        interrupt.set_interrupt_handler(handler);

        let state: FpuState = fpu_pattern(1, CONTROL_A);
        let result = hold_fpu_state(&state, 1000, || interrupt.raise());

        assert!(HANDLER_RAN.load(Ordering::Relaxed));
        assert_eq!(result, state);
    }

    #[cfg(float_save_restore)]
    #[test]
    fn nested_interrupt_handler_keeps_fpu_state(p: Peripherals) {
        static HIGH_RAN: AtomicBool = AtomicBool::new(false);
        static LOW_KEPT_STATE: AtomicBool = AtomicBool::new(false);

        #[handler(priority = Priority::max())]
        fn high_priority() {
            SoftwareInterrupt::new(unsafe { FROM_CPU_INTR3::steal() }).reset();
            clobber_fpu_state();
            HIGH_RAN.store(true, Ordering::Relaxed);
        }

        #[handler(priority = Priority::min())]
        fn low_priority() {
            SoftwareInterrupt::new(unsafe { FROM_CPU_INTR2::steal() }).reset();

            let state: FpuState = fpu_pattern(2, CONTROL_B);
            let result = hold_fpu_state(&state, 1000, || {
                SoftwareInterrupt::new(unsafe { FROM_CPU_INTR3::steal() }).raise()
            });
            LOW_KEPT_STATE.store(result == state, Ordering::Relaxed);
        }

        let mut low = SoftwareInterrupt::new(p.FROM_CPU_INTR2);
        let mut high = SoftwareInterrupt::new(p.FROM_CPU_INTR3);
        low.set_interrupt_handler(low_priority);
        high.set_interrupt_handler(high_priority);

        low.raise();

        assert!(HIGH_RAN.load(Ordering::Relaxed));
        assert!(LOW_KEPT_STATE.load(Ordering::Relaxed));
    }

    #[cfg(float_save_restore)]
    #[test]
    fn task_switch_keeps_fpu_state(p: Peripherals) {
        use esp_hal::{time::Instant, timer::timg::TimerGroup};
        use esp_rtos::thread::ThreadSpawner;

        fn worker(seed: u32, control: &[u32]) -> (bool, Instant, Instant) {
            let state: FpuState = fpu_pattern(seed, control);
            let start = Instant::now();
            let result = hold_fpu_state(&state, 10_000_000, || {});
            (result == state, start, Instant::now())
        }

        let timg0 = TimerGroup::new(p.TIMG0);
        esp_rtos::start(timg0.timer0);

        let first = ThreadSpawner::new(4096).spawn(|| worker(3, CONTROL_A));
        let second = ThreadSpawner::new(4096).spawn(|| worker(4, CONTROL_B));
        let ((first_kept_state, first_start, first_end), _) = first.join();
        let ((second_kept_state, second_start, second_end), _) = second.join();

        assert!(first_kept_state);
        assert!(second_kept_state);
        // The threads overlap in time, so the scheduler switched between them.
        assert!(first_start < second_end && second_start < first_end);
    }
}
