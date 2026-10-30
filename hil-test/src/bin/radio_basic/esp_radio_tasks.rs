//! Tests for the threads that `esp-radio` creates without a core.
//!
//! The threads that `esp-radio` creates run on the first core. `esp-rtos` implements this in the
//! scheduler that it provides to `esp-radio-rtos-driver`, so these tests create threads through
//! that driver API instead of starting a radio driver.

#[embedded_test::tests(default_timeout = 3, executor = esp_rtos::embassy::Executor::new())]
mod tests {
    use core::ffi::c_void;

    use esp_hal::{clock::CpuClock, peripherals::CPU_CTRL, system::Cpu, timer::timg::TimerGroup};
    use esp_radio_rtos_driver::{
        self as preempt,
        semaphore::{SemaphoreHandle, SemaphoreKind},
    };
    use portable_atomic::{AtomicBool, Ordering};

    struct Context {
        cpu_cntl: CPU_CTRL<'static>,
    }

    #[init]
    fn init() -> Context {
        crate::init_heap();

        let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
        let p = esp_hal::init(config);

        let timg0 = TimerGroup::new(p.TIMG0);
        esp_rtos::start(timg0.timer0);

        Context {
            cpu_cntl: p.CPU_CTRL,
        }
    }

    #[test]
    fn thread_runs_on_the_first_core(ctx: Context) {
        struct TestContext {
            ran_on_first_core: AtomicBool,
            done: AtomicBool,
            ran: SemaphoreHandle,
            creator_done: SemaphoreHandle,
        }

        let test_context = TestContext {
            ran_on_first_core: AtomicBool::new(false),
            done: AtomicBool::new(false),
            ran: SemaphoreHandle::new(SemaphoreKind::Counting { initial: 0, max: 1 }),
            creator_done: SemaphoreHandle::new(SemaphoreKind::Counting { initial: 0, max: 1 }),
        };

        extern "C" fn radio_thread(context: *mut c_void) {
            let context = unsafe { &*(context as *const TestContext) };
            context
                .ran_on_first_core
                .store(Cpu::current() == Cpu::ProCpu, Ordering::SeqCst);
            context.done.store(true, Ordering::SeqCst);
            context.ran.give();
        }

        // The scheduler sends a task that names no core to the core that runs the least important
        // task. This thread keeps the first core busy at a higher priority than anything runs on
        // the second core, so a task that is not pinned is sent to the second core.
        extern "C" fn creator(context: *mut c_void) {
            unsafe {
                preempt::task_create("radio_thread", radio_thread, context, 2, None, 4096);
            }

            let context = unsafe { &*(context as *const TestContext) };
            while !context.done.load(Ordering::SeqCst) {
                core::hint::spin_loop();
            }
            context.creator_done.give();
        }

        esp_rtos::start_second_core(
            unsafe { ctx.cpu_cntl.clone_unchecked() },
            #[allow(static_mut_refs)]
            unsafe {
                &mut crate::APP_CORE_STACK
            },
            || {},
        );

        unsafe {
            preempt::task_create(
                "creator",
                creator,
                (&raw const test_context).cast::<c_void>().cast_mut(),
                1,
                Some(0),
                4096,
            );
        }

        test_context.ran.take(None);
        test_context.creator_done.take(None);

        hil_test::assert!(test_context.ran_on_first_core.load(Ordering::SeqCst));

        unsafe {
            // Park the second core, we don't need it anymore
            esp_hal::system::CpuControl::new(ctx.cpu_cntl).park_core(Cpu::AppCpu);
        }
    }
}
