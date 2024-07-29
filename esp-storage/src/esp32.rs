use crate::maybe_with_critical_section;

const ESP_ROM_SPIFLASH_READ: u32 = 0x40062ed8;
const ESP_ROM_SPIFLASH_ERASE_SECTOR: u32 = 0x40062ccc;
const SPI_READ_STATUS_HIGH: u32 = 0x40062448;
const SPI_READ_STATUS: u32 = 0x4006226c;
const SPI_WRITE_STATUS: u32 = 0x400622f0;

const CACHE_FLUSH_ROM: u32 = 0x40009a14;
const CACHE_READ_ENABLE_ROM: u32 = 0x40009a84;

const SPI_BASE_REG: u32 = 0x3ff42000; // SPI peripheral 1, used for SPI flash
const SPI0_BASE_REG: u32 = 0x3ff43000; // SPI peripheral 0, inner state machine
const SPI_EXT2_REG: u32 = SPI_BASE_REG + 0xF8;
const SPI0_EXT2_REG: u32 = SPI0_BASE_REG + 0xF8;
const SPI_RD_STATUS_REG: u32 = SPI_BASE_REG + 0x10;
#[allow(clippy::identity_op)]
const SPI_CMD_REG: u32 = SPI_BASE_REG + 0x00;
const SPI_CTRL_REG: u32 = SPI_BASE_REG + 0x08;
const SPI_USER_REG: u32 = SPI_BASE_REG + 0x1c;
const SPI_USER1_REG: u32 = SPI_BASE_REG + 0x20;
const SPI_ADDR_REG: u32 = SPI_BASE_REG + 4;
const SPI_W0_REG: u32 = SPI_BASE_REG + 0x80;
const SPI_ST: u32 = 0x7;
const SPI_USR_DUMMY: u32 = 1 << 29;
const ESP_ROM_SPIFLASH_W_SIO_ADDR_BITSLEN: u32 = 23;
const SPI_USR_ADDR_BITLEN_M: u32 = 0x3f << 26;
const SPI_USR_ADDR_BITLEN_S: u32 = 26;
const SPI_FLASH_WREN: u32 = 1 << 30;
const STATUS_WIP_BIT: u32 = 1 << 0;
const STATUS_QIE_BIT: u32 = 1 << 9; // Quad Enable
const SPI_WRSR_2B: u32 = 1 << 22;

const FLASH_CHIP_ADDR: u32 = 0x3ffae270;
const FLASH_DUMMY_LEN_PLUS_ADDR: u32 = 0x3ffae290;

#[inline(always)]
#[link_section = ".rwtext"]
pub(crate) fn cache_flush_rom(cpu_num: u32) {
    unsafe {
        let cache_flush_rom: unsafe extern "C" fn(u32) = core::mem::transmute(CACHE_FLUSH_ROM);
        cache_flush_rom(cpu_num)
    }
}

#[inline(always)]
#[link_section = ".rwtext"]
pub(crate) fn cache_read_enable_rom(cpu_num: u32) {
    unsafe {
        let cache_read_enable_rom: unsafe extern "C" fn(u32) =
            core::mem::transmute(CACHE_READ_ENABLE_ROM);
        cache_read_enable_rom(cpu_num)
    }
}

#[inline(always)]
#[link_section = ".rwtext"]
pub(crate) fn spi_read_status_high(
    flash_chip: *const EspRomSpiflashChipT,
    status: &mut u32,
) -> i32 {
    unsafe {
        let spi_read_status_high: unsafe extern "C" fn(
            *const EspRomSpiflashChipT,
            *mut u32,
        ) -> i32 = core::mem::transmute(SPI_READ_STATUS_HIGH);
        spi_read_status_high(flash_chip, status as *mut u32)
    }
}

#[inline(always)]
#[link_section = ".rwtext"]
pub(crate) fn spi_read_status(flash_chip: *const EspRomSpiflashChipT, status: &mut u32) -> i32 {
    unsafe {
        let spi_read_status: unsafe extern "C" fn(*const EspRomSpiflashChipT, *mut u32) -> i32 =
            core::mem::transmute(SPI_READ_STATUS);
        spi_read_status(flash_chip, status as *mut u32)
    }
}

#[inline(always)]
#[link_section = ".rwtext"]
pub(crate) fn spi_write_status(flash_chip: *const EspRomSpiflashChipT, status_value: u32) -> i32 {
    unsafe {
        let spi_write_status: unsafe extern "C" fn(*const EspRomSpiflashChipT, u32) -> i32 =
            core::mem::transmute(SPI_WRITE_STATUS);
        spi_write_status(flash_chip, status_value)
    }
}

#[inline(always)]
#[link_section = ".rwtext"]
fn begin() {
    // on some chips disabling cache access caused issues - we don't really need
    // it
}

#[inline(always)]
#[link_section = ".rwtext"]
fn end() {
    cache_flush_rom(0);
    cache_flush_rom(1);
    cache_read_enable_rom(0);
    cache_read_enable_rom(1);
}

#[derive(Debug)]
#[repr(C)]
pub struct EspRomSpiflashChipT {
    device_id: u32,
    chip_size: u32, // chip size in bytes
    block_size: u32,
    sector_size: u32,
    page_size: u32,
    status_mask: u32,
}

#[inline(never)]
#[link_section = ".rwtext"]
pub(crate) fn esp_rom_spiflash_read(src_addr: u32, data: *const u32, len: u32) -> i32 {
    maybe_with_critical_section(|| {
        spiflash_wait_for_ready();
        unsafe {
            let esp_rom_spiflash_read: unsafe extern "C" fn(u32, *const u32, u32) -> i32 =
                core::mem::transmute(ESP_ROM_SPIFLASH_READ);
            esp_rom_spiflash_read(src_addr, data, len)
        }
    })
}

#[inline(never)]
#[link_section = ".rwtext"]
pub(crate) fn esp_rom_spiflash_erase_sector(sector_number: u32) -> i32 {
    maybe_with_critical_section(|| {
        let res = unsafe {
            let esp_rom_spiflash_erase_sector: unsafe extern "C" fn(u32) -> i32 =
                core::mem::transmute(ESP_ROM_SPIFLASH_ERASE_SECTOR);
            esp_rom_spiflash_erase_sector(sector_number)
        };
        spiflash_wait_for_ready();
        res
    })
}

#[inline(always)]
#[link_section = ".rwtext"]
fn spi_write_enable() {
    spiflash_wait_for_ready();
    unsafe {
        let spi = &*crate::peripherals::SPI1::PTR;
        spi.rd_status().modify(|_, w| w.bits(0));
        spi.cmd().modify(|_, w| w.flash_wren().set_bit());
        while spi.cmd().read().bits() != 0 {}
    }
}

#[inline(never)]
#[link_section = ".rwtext"]
pub(crate) fn esp_rom_spiflash_write(dest_addr: u32, data: *const u32, len: u32) -> i32 {
    maybe_with_critical_section(|| {
        begin();

        let flashchip = FLASH_CHIP_ADDR as *const EspRomSpiflashChipT;
        let mut status: u32 = 0;

        spiflash_wait_for_ready();
        if spi_read_status_high(flashchip, &mut status) != 0 {
            return -1;
        }

        spiflash_wait_for_ready();
        unsafe {
            let spi = &*crate::peripherals::SPI1::PTR;

            spi.user().modify(|_, w| w.usr_dummy().clear_bit());

            spi.user1().modify(|_, w| {
                w.usr_addr_bitlen()
                    .bits(ESP_ROM_SPIFLASH_W_SIO_ADDR_BITSLEN)
            });

            for block in (0..len).step_by(32) {
                spiflash_wait_for_ready();
                spi_write_enable();
                let block_len = if len - block < 32 { len - block } else { 32 };
                spi.addr()
                    .modify(|_, w| w.bits(((dest_addr + block) & 0xffffff) | block_len << 24));

                let data_ptr = data.offset((block / 4) as isize);
                for i in 0..block_len / 4 {
                    spi.w(i)
                        .modify(|_, w| w.bits(data_ptr.offset(i as isize).read_volatile()));
                }

                spi.rd_status().modify(|_, w| w.bits(0));
                spi.cmd().modify(|_, w| w.flash_pp().set_bit());
                while spi.cmd().read().bits() != 0 {}
            }

            wait_for_ready();
        }

        spiflash_wait_for_ready();
        if spi_write_status(flashchip, status) != 0 {
            end();
            return -1;
        }
        spiflash_wait_for_ready();

        end();
        0
    })
}

#[inline(always)]
#[link_section = ".rwtext"]
fn wait_for_ready() {
    unsafe {
        while (&*crate::peripherals::SPI1::PTR).ext2().read().st().bits() != 0 {}
        // ESP32_OR_LATER ... we don't support anything earlier
        while (&*crate::peripherals::SPI0::PTR).ext2().read().st().bits() != 0 {}
    }
}

#[inline(always)]
#[link_section = ".rwtext"]
fn spiflash_wait_for_ready() {
    let flashchip = FLASH_CHIP_ADDR as *const EspRomSpiflashChipT;

    loop {
        wait_for_ready();
        let mut status = 0;
        spi_read_status(flashchip, &mut status);
        if status & STATUS_WIP_BIT == 0 {
            break;
        }
    }
}

#[inline(never)]
#[link_section = ".rwtext"]
pub(crate) fn esp_rom_spiflash_unlock() -> i32 {
    let flashchip = FLASH_CHIP_ADDR as *const EspRomSpiflashChipT;
    if unsafe { (*flashchip).device_id } >> 16 & 0xff == 0x9D {
        panic!("ISSI flash is not supported");
    }

    let g_rom_spiflash_dummy_len_plus = FLASH_DUMMY_LEN_PLUS_ADDR as *const u8;
    if unsafe { g_rom_spiflash_dummy_len_plus.add(1).read_volatile() } == 0 {
        panic!("Unsupported flash chip");
    }

    maybe_with_critical_section(|| {
        begin();
        spiflash_wait_for_ready();

        let mut status: u32 = 0;
        if spi_read_status_high(flashchip, &mut status) != 0 {
            return -1;
        }

        // Clear all bits except QE, if it is set
        status &= STATUS_QIE_BIT;
        unsafe {
            (&*crate::peripherals::SPI1::PTR)
                .ctrl()
                .modify(|_, w| w.wrsr_2b().set_bit());
        }

        spiflash_wait_for_ready();
        if spi_write_status(flashchip, status) != 0 {
            end();
            return -1;
        }
        spiflash_wait_for_ready();
        end();
        0
    })
}
