#![no_std]

pub use esp_hal_procmacros::ram;

pub mod system {
    #[derive(Debug, Clone, Copy, PartialEq, Eq)]
    pub enum Cpu {
        ProCpu = 0,
        AppCpu = 1,
    }

    impl Cpu {
        pub const COUNT: usize = 2;

        pub fn all() -> impl Iterator<Item = Self> {
            [Cpu::ProCpu, Cpu::AppCpu].into_iter()
        }
    }
}
