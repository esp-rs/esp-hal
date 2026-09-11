//! Retains both CPU cores across a light sleep, and checks that the state comes back.
//!
//! Core 0 starts core 1, then both cores seed the same two CSRs and sleep in a loop. Each line of
//! output names the core that printed it.

//% CHIP_FILTER: supports_cpu_power_down && cpu_retention == "software" && multi_core
//% FEATURES: unstable

#![no_std]
#![no_main]

use core::ptr::addr_of_mut;

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    interrupt,
    main,
    ram,
    rtc_cntl::{
        CpuRetentionStorage,
        sleep::{LowPower, RtcSleepConfig},
    },
    system::{Cpu, CpuControl, Stack},
    time::{Duration, Instant},
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

#[ram(reclaimed, unstable(zeroed))]
static RETENTION: CpuRetentionStorage = CpuRetentionStorage::new();

/// An unused PMA entry address, which the non-critical frame holds.
const PMAADDR15: u32 = 0xbdf;
/// User-mode counter access, which no frame holds. The reset value is 0.
const MCOUNTEREN: u32 = 0x306;

macro_rules! read_csr {
    ($csr:expr) => {{
        let value: u32;
        // SAFETY: the read has no side effect, and no other code in this program uses the CSR.
        unsafe { core::arch::asm!("csrr {0}, {1}", out(reg) value, const $csr) };
        value
    }};
}

macro_rules! write_csr {
    ($csr:expr, $value:expr) => {{
        // SAFETY: no other code in this program uses the CSR, and neither one changes how the
        // program runs: the PMA entry stays disabled, and the program never leaves machine mode.
        unsafe { core::arch::asm!("csrw {0}, {1}", const $csr, in(reg) $value) };
    }};
}

/// Lets this core take interrupts.
///
/// Core 1 must answer the doorbell of the rendezvous, and the start of core 1 leaves
/// `mstatus.MIE` clear.
fn enable_interrupts() {
    // SAFETY: bit 3 of `mstatus` is `MIE`, and this program installs no handler that needs a
    // context of its own.
    unsafe { core::arch::asm!("csrsi mstatus, 8") };
}

fn seed_probes(round: u32) -> u32 {
    let seed = 0x0dd0_0000 | round;
    write_csr!(PMAADDR15, seed);
    write_csr!(MCOUNTEREN, 1);
    read_csr!(PMAADDR15)
}

fn check_probes(core: Cpu, round: u32, wanted: u32) {
    let pma = read_csr!(PMAADDR15);
    let counteren = read_csr!(MCOUNTEREN);
    let core = match core {
        Cpu::ProCpu => "core 0",
        Cpu::AppCpu => "core 1",
    };
    println!(
        "{core} round {round}: pma {}, cpu domain {}",
        if pma == wanted { "restored" } else { "LOST" },
        if counteren == 0 {
            "powered down"
        } else {
            "STAYED ON"
        },
    );
}

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let delay = Delay::new();
    let mut lpwr = LowPower::new(peripherals.LPWR);

    match lpwr.install_cpu_retention_memory(RETENTION.take()) {
        Ok(()) => println!("core 0: retention memory installed"),
        Err(error) => {
            println!("core 0: retention memory refused: {:?}", error);
            loop {}
        }
    }

    static mut APP_CORE_STACK: Stack<8192> = Stack::new();
    let mut cpu_control = CpuControl::new(peripherals.CPU_CTRL);
    let _guard = cpu_control
        .start_app_core(unsafe { &mut *addr_of_mut!(APP_CORE_STACK) }, || {
            println!("core 1: started");
            enable_interrupts();
            let mut round: u32 = 0;
            loop {
                round += 1;
                let wanted = seed_probes(round);
                interrupt::wait_for_interrupt();
                check_probes(Cpu::AppCpu, round, wanted);
            }
        })
        .unwrap();

    delay.delay_millis(50);

    let mut round: u32 = 0;
    loop {
        round += 1;
        let wanted = seed_probes(round);

        println!("core 0 round {round}: sleeping");
        delay.delay_millis(50);

        lpwr.set_wakeup_deadline(Instant::now() + Duration::from_millis(500));
        lpwr.sleep_light(RtcSleepConfig::default());

        check_probes(Cpu::ProCpu, round, wanted);

        delay.delay_millis(200);
    }
}
