//! Retains the CPU across a light sleep, and checks that the state comes back.
//!
//! Each round seeds two CSRs and sleeps. `pmaaddr15` is in the non-critical frame, so it must come
//! back. `mcounteren` is in no frame, so a real power-down must clear it: that is what tells a
//! working restore apart from a sleep that never powered the CPU domain down. The rounds repeat,
//! because a partial restore often survives one sleep and not the next.
//!
//! Run this on a chip that powers the CPU domain down in software. Both CSRs are ones that no
//! driver reads, which matters: the trap entry of `esp-riscv-rt` uses `mscratch`, so that CSR
//! cannot report anything about the sleep.

//% CHIP_FILTER: supports_cpu_power_down && cpu_retention == "software"
//% FEATURES: unstable

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    delay::Delay,
    main,
    ram,
    rtc_cntl::{
        CpuRetentionStorage,
        sleep::{LowPower, RtcSleepConfig},
    },
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

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    let delay = Delay::new();
    let mut lpwr = LowPower::new(peripherals.LPWR);

    match lpwr.install_cpu_retention_memory(RETENTION.take()) {
        Ok(()) => println!("retention memory installed"),
        Err(error) => {
            println!("retention memory refused: {:?}", error);
            loop {}
        }
    }

    let mut round: u32 = 0;
    loop {
        round += 1;
        let seed = 0x0dd0_0000 | round;
        write_csr!(PMAADDR15, seed);
        write_csr!(MCOUNTEREN, 1);
        // The PMA address register is WARL: it normalises what it takes. Compare against what it
        // holds, not against what the write asked for.
        let wanted = read_csr!(PMAADDR15);

        println!("round {round}: sleeping");
        delay.delay_millis(50);

        lpwr.set_wakeup_deadline(Instant::now() + Duration::from_millis(500));
        lpwr.sleep_light(RtcSleepConfig::default());

        let pma = read_csr!(PMAADDR15);
        let counteren = read_csr!(MCOUNTEREN);

        println!(
            "round {round}: pma {}, cpu domain {}",
            if pma == wanted { "restored" } else { "LOST" },
            if counteren == 0 {
                "powered down"
            } else {
                "STAYED ON"
            },
        );

        delay.delay_millis(200);
    }
}
