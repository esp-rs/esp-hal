//! Identify the SC2336 camera sensor over SCCB/I2C on ESP32-P4 Function EV Board.
//!
//! This validates sensor communication and chip ID only. It does not start CSI capture.
//% CHIP_FILTER: mipi_csi_driver_supported && i2c_master_driver_supported

#![no_std]
#![no_main]

use esp_backtrace as _;
use esp_hal::{
    Blocking,
    delay::Delay,
    i2c::{
        self,
        master::{Config as I2cConfig, I2c},
    },
    main,
    peripherals::PMU,
    time::Rate,
};
use esp_println::println;

esp_bootloader_esp_idf::esp_app_desc!();

// SC2336 constants from esp-video-components:
// - SC2336_SCCB_ADDR = 0x30
// - SC2336_REG_SENSOR_ID_H = 0x3107
// - SC2336_REG_SENSOR_ID_L = 0x3108
// - SC2336_PID = 0xCB3A
const SC2336_SCCB_ADDR: u8 = 0x30;
const SC2336_REG_SENSOR_ID_H: u16 = 0x3107;
const SC2336_REG_SENSOR_ID_L: u16 = 0x3108;
const SC2336_EXPECTED_PID: u16 = 0xCB3A;

// PMU VO3 (ext_ldo_p0_0p2a) configured to 2.5 V for VDD_MIPI_DPHY.
const MIPI_DPHY_LDO_CTRL: u32 = 0x4020_0180;
const MIPI_DPHY_LDO_ANA_2P5V: u32 = (9u32 << 28) | (6u32 << 23);

fn enable_mipi_dphy_power(delay: &Delay) {
    PMU::regs()
        .ext_ldo_p0_0p2a()
        .write(|w| unsafe { w.bits(MIPI_DPHY_LDO_CTRL) });
    PMU::regs()
        .ext_ldo_p0_0p2a_ana()
        .write(|w| unsafe { w.bits(MIPI_DPHY_LDO_ANA_2P5V) });

    // Let the CSI PHY rail settle before sensor checks.
    delay.delay_micros(500);
}

fn read_reg_u8(i2c: &mut I2c<'_, Blocking>, reg: u16) -> Result<u8, i2c::master::Error> {
    let mut value = [0u8; 1];
    i2c.write_read(SC2336_SCCB_ADDR, &reg.to_be_bytes(), &mut value)?;
    Ok(value[0])
}

#[main]
fn main() -> ! {
    let peripherals = esp_hal::init(esp_hal::Config::default());
    esp_println::logger::init_logger_from_env();
    let delay = Delay::new();

    println!("SC2336 ID check start");
    println!("Board assumptions: SCCB SCL=GPIO8, SDA=GPIO7, addr=0x30");
    println!("Board assumptions: reset/pwdn/xclk are not GPIO-controlled on this board");

    enable_mipi_dphy_power(&delay);
    println!("MIPI D-PHY 2.5V rail enabled");

    // Sub-board reset/clock generation is hardware-managed. Wait for rails + oscillator.
    delay.delay_millis(20);

    let mut i2c = I2c::new(
        peripherals.I2C0,
        I2cConfig::default().with_frequency(Rate::from_khz(100)),
    )
    .unwrap()
    .with_sda(peripherals.GPIO7)
    .with_scl(peripherals.GPIO8);

    let pid_h = match read_reg_u8(&mut i2c, SC2336_REG_SENSOR_ID_H) {
        Ok(v) => v,
        Err(err) => {
            println!(
                "[FAIL] SCCB read failed: addr=0x{:02x} reg=0x{:04x} err={:?}",
                SC2336_SCCB_ADDR, SC2336_REG_SENSOR_ID_H, err
            );
            panic!("SC2336 ID read failed");
        }
    };
    let pid_l = match read_reg_u8(&mut i2c, SC2336_REG_SENSOR_ID_L) {
        Ok(v) => v,
        Err(err) => {
            println!(
                "[FAIL] SCCB read failed: addr=0x{:02x} reg=0x{:04x} err={:?}",
                SC2336_SCCB_ADDR, SC2336_REG_SENSOR_ID_L, err
            );
            panic!("SC2336 ID read failed");
        }
    };

    let observed_pid = ((pid_h as u16) << 8) | pid_l as u16;

    println!(
        "SC2336 ID regs: [0x{:04x}]=0x{:02x}, [0x{:04x}]=0x{:02x}",
        SC2336_REG_SENSOR_ID_H, pid_h, SC2336_REG_SENSOR_ID_L, pid_l
    );
    println!(
        "SC2336 PID observed=0x{:04x}, expected=0x{:04x}",
        observed_pid, SC2336_EXPECTED_PID
    );

    if observed_pid == SC2336_EXPECTED_PID {
        println!("SC2336 ID RESULT: PASS");
    } else {
        println!("SC2336 ID RESULT: FAIL");
        panic!("SC2336 ID mismatch");
    }

    loop {}
}
