#![no_std]

pub fn cycles() -> u64 {
    #[cfg(feature = "esp32")]
    {
        esp_hal::xtensa_lx::timer::get_cycle_count() as u64
    }

    #[cfg(not(feature = "esp32"))]
    {
        esp_hal::systimer::SystemTimer::now()
    }
}
