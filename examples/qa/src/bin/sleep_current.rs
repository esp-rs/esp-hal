//! Walks CPU idle and sleep states so a current meter can record each path.
//!
//! The program holds every case for [`HOLD`], then moves to the next. The console
//! prints the case name before the hold, so you can align the meter with the
//! UART log. Use a UART port. USB Serial/JTAG stops across sleep.
//!
//! Active cases combine CPU clock (default and maximum), core count, and either a
//! busy loop or WAITI/WFI. Sleep cases are light sleep without CPU power-down,
//! light sleep with CPU power-down, the cache tag-memory subcase of that path,
//! and deep sleep. The only wakeup source is the LP timer.
//!
//! A chip whose default CPU clock is already the maximum skips the duplicate
//! maximum-clock active cases. Dual-core cases run only on multi-core chips.
//! CPU power-down and tag-memory cases run only where the hardware supports them.

//% CHIP_FILTER: sleep_driver_supported

#![no_std]
#![no_main]
#![allow(static_mut_refs)]

use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    delay::Delay,
    interrupt,
    main,
    rtc_cntl::{
        reset_reason,
        sleep::{LowPower, RtcSleepConfig},
        wakeup_cause,
    },
    system::Cpu,
    time::{Duration, Instant},
    timer::{PeriodicTimer, timg::TimerGroup},
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

#[cfg(multi_core)]
use core::ptr::addr_of_mut;

#[cfg(multi_core)]
use esp_hal::{interrupt::Priority, peripherals::Interrupt};

/// Time the program stays in each case, so a meter can settle and record.
const HOLD: Duration = Duration::from_secs(3);

/// Interrupt period that lets WAITI/WFI return and check the hold deadline.
const WAITI_TICK: Duration = Duration::from_millis(250);

#[cfg(not(any(feature = "esp32c2", feature = "esp32c61")))]
#[esp_hal::ram(unstable(rtc_fast, persistent))]
static mut PHASE: u32 = 0;

fn load_phase() -> u32 {
    cfg_select! {
        feature = "esp32c2" => esp_hal::peripherals::LPWR::regs().store6().read().bits(),
        feature = "esp32c61" => esp_hal::peripherals::LP_AON::regs().store6().read().bits(),
        _ => unsafe { PHASE },
    }
}

fn store_phase(value: u32) {
    cfg_select! {
        feature = "esp32c2" => {
            esp_hal::peripherals::LPWR::regs()
                .store6()
                .write(|w| unsafe { w.bits(value) });
        }
        feature = "esp32c61" => {
            esp_hal::peripherals::LP_AON::regs()
                .store6()
                .write(|w| unsafe { w.bits(value) });
        }
        _ => unsafe { PHASE = value },
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Case {
    Active {
        max_clock: bool,
        two_cores: bool,
        waiti: bool,
    },
    LightSleep {
        two_cores: bool,
        cpu_pd: bool,
        tagmem: bool,
    },
    DeepSleep,
}

const CASES: &[Case] = &[
    Case::Active {
        max_clock: false,
        two_cores: false,
        waiti: false,
    },
    Case::Active {
        max_clock: false,
        two_cores: false,
        waiti: true,
    },
    #[cfg(multi_core)]
    Case::Active {
        max_clock: false,
        two_cores: true,
        waiti: false,
    },
    #[cfg(multi_core)]
    Case::Active {
        max_clock: false,
        two_cores: true,
        waiti: true,
    },
    Case::Active {
        max_clock: true,
        two_cores: false,
        waiti: false,
    },
    Case::Active {
        max_clock: true,
        two_cores: false,
        waiti: true,
    },
    #[cfg(multi_core)]
    Case::Active {
        max_clock: true,
        two_cores: true,
        waiti: false,
    },
    #[cfg(multi_core)]
    Case::Active {
        max_clock: true,
        two_cores: true,
        waiti: true,
    },
    Case::LightSleep {
        two_cores: false,
        cpu_pd: false,
        tagmem: false,
    },
    #[cfg(multi_core)]
    Case::LightSleep {
        two_cores: true,
        cpu_pd: false,
        tagmem: false,
    },
    #[cfg(supports_cpu_power_down)]
    Case::LightSleep {
        two_cores: false,
        cpu_pd: true,
        tagmem: false,
    },
    #[cfg(all(supports_cpu_power_down, multi_core))]
    Case::LightSleep {
        two_cores: true,
        cpu_pd: true,
        tagmem: false,
    },
    #[cfg(supports_tagmem_power_down)]
    Case::LightSleep {
        two_cores: false,
        cpu_pd: true,
        tagmem: true,
    },
    #[cfg(all(supports_tagmem_power_down, multi_core))]
    Case::LightSleep {
        two_cores: true,
        cpu_pd: true,
        tagmem: true,
    },
    Case::DeepSleep,
];

fn case_wants_max_clock(case: Case) -> Option<bool> {
    match case {
        Case::Active { max_clock, .. } => Some(max_clock),
        _ => None,
    }
}

fn skip_duplicate_max_clock(case: Case) -> bool {
    matches!(
        case,
        Case::Active {
            max_clock: true,
            ..
        }
    ) && CpuClock::default() == CpuClock::max()
}

fn print_case(index: usize, case: Case) {
    match case {
        Case::Active {
            max_clock,
            two_cores,
            waiti,
        } => {
            println!(
                "=== [{}/{}] active CPU={} cores={} idle={} ({} s) ===",
                index + 1,
                CASES.len(),
                if max_clock { "max" } else { "default" },
                if two_cores { 2 } else { 1 },
                if waiti { "WAITI/WFI" } else { "busy" },
                HOLD.as_secs(),
            );
        }
        Case::LightSleep {
            two_cores,
            cpu_pd,
            tagmem,
        } => {
            println!(
                "=== [{}/{}] light sleep cores={} cpu_pd={} tagmem={} ({} s) ===",
                index + 1,
                CASES.len(),
                if two_cores { 2 } else { 1 },
                cpu_pd,
                tagmem,
                HOLD.as_secs(),
            );
        }
        Case::DeepSleep => {
            println!(
                "=== [{}/{}] deep sleep ({} s) ===",
                index + 1,
                CASES.len(),
                HOLD.as_secs(),
            );
        }
    }
}

static mut TIMER: *mut PeriodicTimer<'static, esp_hal::Blocking> = core::ptr::null_mut();

#[cfg(multi_core)]
static mut APP_WAITI: bool = true;

#[cfg(multi_core)]
fn set_app_waiti(waiti: bool) {
    unsafe { core::ptr::write_volatile(addr_of_mut!(APP_WAITI), waiti) };
}

#[cfg(multi_core)]
fn app_waiti() -> bool {
    unsafe { core::ptr::read_volatile(&raw const APP_WAITI) }
}

#[esp_hal::handler]
fn timer_handler() {
    unsafe {
        if let Some(timer) = TIMER.as_mut() {
            timer.clear_interrupt();
        }
    }
}

#[cfg(multi_core)]
fn app_core_entry() -> ! {
    cfg_select! {
        xtensa => unsafe {
            let _ = esp_hal::xtensa_lx::interrupt::enable();
        },
        riscv => unsafe {
            let _ = esp_hal::riscv::interrupt::enable();
        },
    }
    interrupt::enable(Interrupt::TG0_T0_LEVEL, Priority::min());

    loop {
        if app_waiti() {
            interrupt::wait_for_interrupt();
        } else {
            core::hint::spin_loop();
        }
    }
}

fn hold_active(waiti: bool) {
    let end = Instant::now() + HOLD;
    if waiti {
        while Instant::now() < end {
            interrupt::wait_for_interrupt();
        }
    } else {
        while Instant::now() < end {
            core::hint::spin_loop();
        }
    }
}

fn deep_sleep_config() -> RtcSleepConfig {
    cfg_select! {
        any(esp32, esp32s2, esp32s3, esp32c3) => {
            let mut config = RtcSleepConfig::deep();
            config.set_rtc_fastmem_pd_en(false);
            config
        }
        _ => RtcSleepConfig::deep(),
    }
}

#[cfg(supports_cpu_power_down)]
#[esp_hal::ram(reclaimed, unstable(zeroed))]
static CPU_RETENTION_MEMORY: esp_hal::rtc_cntl::CpuRetentionStorage =
    esp_hal::rtc_cntl::CpuRetentionStorage::new();

#[cfg(supports_tagmem_power_down)]
#[esp_hal::ram(reclaimed, unstable(zeroed))]
static CACHE_TAGMEM: esp_hal::rtc_cntl::CacheTagRetentionStorage =
    esp_hal::rtc_cntl::CacheTagRetentionStorage::new();

#[main]
fn main() -> ! {
    let mut phase = (load_phase() as usize) % CASES.len();
    while skip_duplicate_max_clock(CASES[phase]) {
        phase = (phase + 1) % CASES.len();
    }
    store_phase(phase as u32);

    let max_clock = case_wants_max_clock(CASES[phase]).unwrap_or(false);
    let mut hal_config = esp_hal::Config::default();
    if max_clock {
        hal_config = hal_config.with_cpu_clock(CpuClock::max());
    }
    let peripherals = esp_hal::init(hal_config);

    let delay = Delay::new();
    let mut lpwr = LowPower::new(peripherals.LPWR);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let mut periodic = PeriodicTimer::new(timg0.timer0);
    periodic.set_interrupt_handler(timer_handler);
    periodic.listen();
    periodic.start(WAITI_TICK).unwrap();
    unsafe {
        TIMER = (&raw mut periodic).cast();
    }

    #[cfg(multi_core)]
    let mut cpu_control = {
        use esp_hal::system::{CpuControl, Stack};

        static mut APP_CORE_STACK: Stack<8192> = Stack::new();
        let mut cpu_control = CpuControl::new(peripherals.CPU_CTRL);
        let guard = cpu_control
            .start_app_core(unsafe { &mut *addr_of_mut!(APP_CORE_STACK) }, || {
                app_core_entry()
            })
            .unwrap();
        core::mem::forget(guard);
        unsafe { cpu_control.park_core(Cpu::AppCpu) };
        cpu_control
    };

    #[cfg(supports_cpu_power_down)]
    let mut cpu_pd_installed = false;
    #[cfg(supports_tagmem_power_down)]
    let mut tagmem_installed = false;

    println!(
        "sleep_current: reset={:?} wake={:?} CPU={:?} phase={}/{}",
        reset_reason(Cpu::ProCpu),
        wakeup_cause(),
        if max_clock {
            CpuClock::max()
        } else {
            CpuClock::default()
        },
        phase + 1,
        CASES.len(),
    );

    loop {
        while skip_duplicate_max_clock(CASES[phase]) {
            phase = (phase + 1) % CASES.len();
            store_phase(phase as u32);
        }

        if let Some(wants_max) = case_wants_max_clock(CASES[phase])
            && wants_max != max_clock
        {
            println!(
                "reset to {} CPU clock",
                if wants_max { "max" } else { "default" }
            );
            delay.delay_millis(200);
            store_phase(phase as u32);
            esp_hal::system::software_reset();
        }

        let case = CASES[phase];
        print_case(phase, case);
        delay.delay_millis(200);

        match case {
            Case::Active {
                two_cores, waiti, ..
            } => {
                #[cfg(multi_core)]
                {
                    set_app_waiti(waiti);
                    if two_cores {
                        cpu_control.unpark_core(Cpu::AppCpu);
                    } else {
                        unsafe { cpu_control.park_core(Cpu::AppCpu) };
                    }
                    delay.delay_millis(50);
                }
                #[cfg(not(multi_core))]
                let _ = two_cores;

                hold_active(waiti);
            }
            Case::LightSleep {
                two_cores,
                cpu_pd,
                tagmem,
            } => {
                periodic.unlisten();
                let _ = periodic.cancel();

                #[cfg(multi_core)]
                {
                    set_app_waiti(true);
                    if two_cores {
                        cpu_control.unpark_core(Cpu::AppCpu);
                    } else {
                        unsafe { cpu_control.park_core(Cpu::AppCpu) };
                    }
                    delay.delay_millis(50);
                }
                #[cfg(not(multi_core))]
                let _ = two_cores;

                #[cfg(supports_cpu_power_down)]
                if cpu_pd && !cpu_pd_installed {
                    lpwr.install_cpu_retention_memory(CPU_RETENTION_MEMORY.take())
                        .unwrap();
                    cpu_pd_installed = true;
                    println!("installed CPU retention memory");
                }
                #[cfg(not(supports_cpu_power_down))]
                let _ = cpu_pd;

                #[cfg(supports_tagmem_power_down)]
                if tagmem && !tagmem_installed {
                    lpwr.install_cache_tag_retention_memory(CACHE_TAGMEM.take())
                        .unwrap();
                    tagmem_installed = true;
                    println!("installed cache tag retention memory");
                }
                #[cfg(not(supports_tagmem_power_down))]
                let _ = tagmem;

                delay.delay_millis(100);
                lpwr.set_wakeup_deadline(Instant::now() + HOLD);
                lpwr.sleep_light(RtcSleepConfig::default());
                println!("woke from light sleep, cause={:?}", wakeup_cause());
            }
            Case::DeepSleep => {
                periodic.unlisten();
                let _ = periodic.cancel();

                #[cfg(multi_core)]
                unsafe {
                    cpu_control.park_core(Cpu::AppCpu);
                }

                let next = ((phase + 1) % CASES.len()) as u32;
                store_phase(next);
                delay.delay_millis(100);
                lpwr.set_wakeup_deadline(Instant::now() + HOLD);
                lpwr.sleep_deep(deep_sleep_config());
            }
        }

        phase = (phase + 1) % CASES.len();
        store_phase(phase as u32);
    }
}
