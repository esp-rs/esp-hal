//! Sends one packet to an ESP32-C6 SDIO slave and reads the echoed packet.
//! See the shared SDIO README for the reference board, wiring and companion firmware.

//% CHIP_FILTER: esp32p4

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    Blocking,
    delay::Delay,
    dma::aligned::DmaAlignedMut,
    gpio::{Level, Output, OutputConfig},
    main,
    sdmmc::{
        BusWidth,
        CommandFlags,
        Config,
        ResponseLen as HalResponseLen,
        SdHostController,
        Slot,
        SlotConfig,
    },
    uart::{Config as UartConfig, Uart},
};
use esp_println::println;
use sdio::{Command, Response, ResponseLen};

const PACKET_LENGTH: usize = 64;

mod voltage;

esp_bootloader_esp_idf::esp_app_desc!();

type Host<'d> = Slot<'d, 1, Blocking>;
type Result<T> = core::result::Result<T, &'static str>;

/// The P4 cache line is 64 bytes; both ends of this allocation are aligned.
#[repr(C, align(64))]
struct Packet([u8; PACKET_LENGTH]);

fn command<'a, C: Command + 'a>(host: &mut Host<'_>, cmd: C) -> Result<C::Resp<'a>> {
    let response_len = match <C::Resp<'a> as Response>::LEN {
        ResponseLen::Zero => HalResponseLen::None,
        ResponseLen::R48 => HalResponseLen::Short,
        ResponseLen::R136 => HalResponseLen::Long,
    };
    let flags = CommandFlags {
        wait_complete: true,
        stop_abort: false,
        busy: <C::Resp<'a> as Response>::BUSY,
    };
    let words = host
        .command_blocking(
            C::INDEX,
            cmd.arg(),
            response_len,
            <C::Resp<'a> as Response>::CRC,
            flags,
        )
        .map_err(|error| {
            println!("CMD{} failed: {:?}", C::INDEX, error);
            "command failed"
        })?;
    Ok(<C::Resp<'a> as Response>::from_words(&words))
}

fn register(host: &mut Host<'_>, function: u8, address: u32, write: Option<u8>) -> Result<u8> {
    let response = command(
        host,
        sdio::sdio::Cmd52 {
            write: write.is_some(),
            function,
            raw: false,
            addr: address,
            data: write.unwrap_or(0),
        },
    )?;
    if response.flags & sdio::SdioError::ALL != 0 {
        return Err("CMD52 R5 error");
    }
    Ok(response.data)
}

fn register_u32(host: &mut Host<'_>, address: u32) -> Result<u32> {
    let mut bytes = [0; 4];
    for (i, byte) in bytes.iter_mut().enumerate() {
        *byte = register(host, 1, address + i as u32, None)?;
    }
    Ok(u32::from_le_bytes(bytes))
}

/// Reads a function-1 word for polling. The slave withdraws `IOR1` when it completes a transfer
/// and refuses function-1 accesses with FUNCTION_NUMBER until it declares readiness again, so
/// that refusal alone is `None`; any other R5 error stays an error.
fn poll_u32(host: &mut Host<'_>, address: u32) -> Result<Option<u32>> {
    let mut bytes = [0; 4];
    for (i, byte) in bytes.iter_mut().enumerate() {
        let response = command(
            host,
            sdio::sdio::Cmd52 {
                write: false,
                function: 1,
                raw: false,
                addr: address + i as u32,
                data: 0,
            },
        )?;
        match response.flags & sdio::SdioError::ALL {
            0 => *byte = response.data,
            sdio::SdioError::FLAG_FUNCTION_NUMBER => return Ok(None),
            _ => return Err("CMD52 R5 error"),
        }
    }
    Ok(Some(u32::from_le_bytes(bytes)))
}

fn wait_ready(host: &mut Host<'_>, delay: &Delay) -> Result<()> {
    for _ in 0..200 {
        if register(host, 0, 3, None)? & 2 != 0 {
            return Ok(());
        }
        delay.delay_millis(10);
    }
    Err("function 1 readiness timed out")
}

fn enumerate(host: &mut Host<'_>, delay: &Delay) -> Result<()> {
    // The SD physical layer needs at least 74 clocks before the first command.
    host.send_init_sequence().map_err(|error| {
        println!("Init clocks failed: {:?}", error);
        "init clocks failed"
    })?;
    command(host, sdio::common::idle())?;
    let inquiry = command(host, sdio::sdio::io_send_op_cond(false, 0))?;
    if inquiry.ocr == u32::MAX || (inquiry.ocr >> 28) & 7 == 0 {
        return Err("card does not announce an I/O function");
    }
    // OCR bits 20 and 21 describe the 3.2..3.4 V windows. No 1.8 V switch.
    let window = voltage::voltage_window(inquiry.ocr);
    if window == 0 {
        return Err("no common 3.3 V supply window");
    }
    let mut ready = false;
    for _ in 0..40 {
        if command(host, sdio::sdio::io_send_op_cond(false, window))?.ocr & (1 << 31) != 0 {
            ready = true;
            break;
        }
        delay.delay_millis(50);
    }
    if !ready {
        return Err("card initialization timed out");
    }
    let relative = command(host, sdio::sd::Cmd3)?;
    if relative.rca == 0 || relative.status & 0xe000 != 0 {
        return Err("invalid RCA or R6 status");
    }
    command(host, sdio::common::select_card(relative.rca))?;
    register(host, 0, 2, Some(2))?;
    wait_ready(host, delay)
}

fn exchange(host: &mut Host<'_>, delay: &Delay) -> Result<()> {
    enumerate(host, delay)?;
    // This example starts from a reset C6 and consumes just one receive credit.
    if (register_u32(host, 0x44)? >> 16) & 0xfff == 0 {
        return Err("no receive credit");
    }
    let expected = core::array::from_fn::<_, PACKET_LENGTH, _>(|i| i as u8);
    let mut packet = Packet(expected);
    let address = 0x1_f800 - PACKET_LENGTH as u32;
    let read_argument = (1 << 28) | (1 << 26) | (address << 9) | PACKET_LENGTH as u32;
    let write_argument = read_argument | (1 << 31);
    let dma = DmaAlignedMut::new(&mut packet.0)
        .map_err(|_| "unaligned buffer")?
        .unsize();
    let response = host
        .write_blocks_blocking(53, write_argument, dma, PACKET_LENGTH as u16, 1)
        .map_err(|error| {
            println!("CMD53 write failed: {:?}", error);
            "write failed"
        })?;
    if ((response[0] >> 8) as u8) & sdio::SdioError::ALL != 0 {
        return Err("CMD53 write R5 error");
    }
    println!("Sent {} bytes; waiting for the C6 echo", PACKET_LENGTH);

    // PKT_LEN is cumulative modulo 2^20. This fresh host session has consumed 0 bytes.
    // The slave declares IOR1 again once the echo is armed: wait for it before polling.
    wait_ready(host, delay)?;
    let mut available = 0;
    for _ in 0..200 {
        if let Some(word) = poll_u32(host, 0x60)? {
            available = word & 0x000f_ffff;
            if available != 0 {
                break;
            }
        }
        delay.delay_millis(10);
    }
    if available != PACKET_LENGTH as u32 {
        return Err("unexpected echo length or timeout");
    }
    packet.0.fill(0xff);
    let dma = DmaAlignedMut::new(&mut packet.0)
        .map_err(|_| "unaligned buffer")?
        .unsize();
    let response = host
        .read_blocks_blocking(53, read_argument, dma, PACKET_LENGTH as u16, 1)
        .map_err(|error| {
            println!("CMD53 read failed: {:?}", error);
            "read failed"
        })?;
    if ((response[0] >> 8) as u8) & sdio::SdioError::ALL != 0 {
        return Err("CMD53 read R5 error");
    }
    if packet.0 != expected {
        return Err("echo payload mismatch");
    }
    println!("P4 received and checked all {} echoed bytes", PACKET_LENGTH);
    Ok(())
}

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    let delay = Delay::new();
    // The reference board routes this pin to C6 EN. Keep the C6 in reset during setup.
    let mut c6_enable = Output::new(peripherals.GPIO54, Level::Low, OutputConfig::default());
    // Supports the UART fallback of esp-println's automatic console selection.
    let _console = Uart::new(
        peripherals.UART0,
        UartConfig::default().with_baudrate(115_200),
    )
    .unwrap()
    .with_tx(peripherals.GPIO37);
    let controller = SdHostController::new(peripherals.SDHOST, Config::default()).unwrap();
    let mut host = controller
        .slot::<1>(SlotConfig::default())
        .unwrap()
        .with_clk(peripherals.GPIO18)
        .with_cmd(peripherals.GPIO19)
        .with_data0(peripherals.GPIO14)
        .with_data1(peripherals.GPIO15)
        .with_data2(peripherals.GPIO16)
        .with_data3(peripherals.GPIO17);
    host.set_bus_low_level(BusWidth::Bit1, 400_000).unwrap();
    delay.delay_millis(100);
    c6_enable.set_high();
    delay.delay_millis(2000);
    if let Err(error) = exchange(&mut host, &delay) {
        println!("SDIO exchange failed: {}", error);
    }
    // No retry or further reset: a failed transfer requires a fresh session.
    loop {
        delay.delay_millis(1000);
    }
}
