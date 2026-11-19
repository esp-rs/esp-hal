//! This shows debug-assist
//!
//! Uncomment the functionality you want to test

//% CHIP_FILTER: assist_debug_driver_supported

#![no_std]
#![no_main]

use core::cell::RefCell;

use critical_section::Mutex;
use esp_backtrace as _;
use esp_hal::{assist_debug::DebugAssist, handler, main};

esp_bootloader_esp_idf::esp_app_desc!();

static DA: Mutex<RefCell<Option<DebugAssist>>> = Mutex::new(RefCell::new(None));

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());

    // Log on UART0's default TX pin.
    let log_tx = cfg_select! {
        feature = "esp32c2" => peripherals.GPIO20,
        feature = "esp32c3" => peripherals.GPIO21,
        any(feature = "esp32c5", feature = "esp32c61") => peripherals.GPIO11,
        feature = "esp32c6" => peripherals.GPIO16,
        feature = "esp32h2" => peripherals.GPIO24,
        feature = "esp32s3" => peripherals.GPIO43,
    };
    let log_writer = esp_hal::uart::UartTx::new(peripherals.UART0, Default::default())
        .unwrap()
        .with_tx(log_tx);
    static LOG_WRITER: static_cell::StaticCell<esp_hal::uart::UartTx<'static, esp_hal::Blocking>> =
        static_cell::StaticCell::new();
    embedded_log::init(LOG_WRITER.init(log_writer));

    let mut da = DebugAssist::new(peripherals.ASSIST_DEBUG);
    da.set_interrupt_handler(interrupt_handler);

    cfg_select! {
        not(feature = "esp32s3") => {
            use core::ptr::addr_of_mut;

            unsafe extern "C" {
                // top of stack
                static mut _stack_start: u32;
                // bottom of stack
                static mut _stack_end: u32;
            }

            #[allow(unused_unsafe)]
            let stack_top = unsafe { addr_of_mut!(_stack_start) } as *mut _ as u32;
            #[allow(unused_unsafe)]
            let stack_bottom = unsafe { addr_of_mut!(_stack_end) } as *mut _ as u32;

            let size = 4096;
        }
        _ => {
            let stack_bottom = 0x3fcce000;

            let size = 256;
        }
    }

    // Monitor the SP so as to prevent stack overflow or erroneous push/pop.
    // When the SP exceeds the minimum or maximum threshold, the module will
    // record the PC pointer and generate an interrupt.
    #[cfg(not(feature = "esp32s3"))]
    da.enable_sp_monitor(stack_bottom + size, stack_top);

    // Monitor reads/writes performed by the CPU over data bus and peripheral
    // bus in a certain address space, i.e., memory region. Whenever the bus
    // reads or writes in the specified address space, an interrupt will be
    // triggered.
    #[cfg(not(feature = "esp32c2"))]
    {
        da.enable_region0_monitor(stack_bottom, stack_bottom + size, true, true);
        da.enable_region1_monitor(stack_bottom, stack_bottom + size, true, true);
    }

    critical_section::with(|cs| DA.borrow_ref_mut(cs).replace(da));

    eat_up_stack(0);

    loop {}
}

#[allow(unconditional_recursion)]
fn eat_up_stack(v: u32) {
    log::info!("Iteration {v}");
    eat_up_stack(v + 1);
}

#[handler(priority = esp_hal::interrupt::Priority::min())]
fn interrupt_handler() {
    critical_section::with(|cs| {
        log::info!("\n\nDEBUG_ASSIST interrupt");
        let mut da = DA.borrow_ref_mut(cs);
        let da = da.as_mut().unwrap();

        #[cfg(not(feature = "esp32s3"))]
        if da.is_sp_monitor_interrupt_set() {
            log::info!("SP MONITOR TRIGGERED");
            da.clear_sp_monitor_interrupt();
            let pc = da.sp_monitor_pc();
            log::info!("PC = 0x{:x}", pc);
        }

        #[cfg(not(feature = "esp32c2"))]
        if da.is_region0_monitor_interrupt_set() {
            log::info!("REGION0 MONITOR TRIGGERED");
            da.clear_region0_monitor_interrupt();
            let pc = da.region_monitor_pc();
            log::info!("PC = 0x{:x}", pc);
        }

        #[cfg(not(feature = "esp32c2"))]
        if da.is_region1_monitor_interrupt_set() {
            log::info!("REGION1 MONITOR TRIGGERED");
            da.clear_region1_monitor_interrupt();
            let pc = da.region_monitor_pc();
            log::info!("PC = 0x{:x}", pc);
        }

        loop {}
    });
}
