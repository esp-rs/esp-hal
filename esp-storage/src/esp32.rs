use crate::maybe_with_critical_section;

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

crate::rom_fn! {
    fn esp_rom_cache_flush(cpu_num: u32) = 0x40009a14;
    fn esp_rom_cache_read_enable(cpu_num: u32) = 0x40009a84;
    fn esp_rom_spiflash_read(src_addr: u32, data: *const u32, len: u32) -> i32 = 0x40062ed8;
    fn esp_rom_spiflash_erase_sector(sector_number: u32) -> i32 = 0x40062ccc;
    fn esp_rom_spi_read_status_high(
        flash_chip: *const EspRomSpiflashChipT,
        status: *mut u32
    ) -> i32 = 0x40062448;
    fn esp_rom_spi_read_status(flash_chip: *const EspRomSpiflashChipT, status: *mut u32) -> i32 = 0x4006226c;
    fn esp_rom_spi_write_status(flash_chip: *const EspRomSpiflashChipT, status_value: u32) -> i32 = 0x400622f0;
}

#[inline(always)]
#[unsafe(link_section = ".rwtext")]
pub(crate) fn spi_read_status_high(
    flash_chip: *const EspRomSpiflashChipT,
    status: &mut u32,
) -> i32 {
    esp_rom_spi_read_status_high(flash_chip, status as *mut u32)
}

#[inline(always)]
#[unsafe(link_section = ".rwtext")]
pub(crate) fn spi_read_status(flash_chip: *const EspRomSpiflashChipT, status: &mut u32) -> i32 {
    esp_rom_spi_read_status(flash_chip, status as *mut u32)
}

#[inline(always)]
#[unsafe(link_section = ".rwtext")]
pub(crate) fn spi_write_status(flash_chip: *const EspRomSpiflashChipT, status_value: u32) -> i32 {
    esp_rom_spi_write_status(flash_chip, status_value)
}

#[inline(always)]
#[unsafe(link_section = ".rwtext")]
fn begin() {
    // on some chips disabling cache access caused issues - we don't really need
    // it
}

#[inline(always)]
#[unsafe(link_section = ".rwtext")]
fn end() {
    esp_rom_cache_flush(0);
    esp_rom_cache_flush(1);
    esp_rom_cache_read_enable(0);
    esp_rom_cache_read_enable(1);
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
#[unsafe(link_section = ".rwtext")]
pub(crate) fn spiflash_read(src_addr: u32, data: *const u32, len: u32) -> i32 {
    maybe_with_critical_section(|| {
        spiflash_wait_for_ready();
        esp_rom_spiflash_read(src_addr, data, len)
    })
}

#[inline(never)]
#[unsafe(link_section = ".rwtext")]
pub(crate) fn spiflash_erase_sector(sector_number: u32) -> i32 {
    maybe_with_critical_section(|| {
        let res = esp_rom_spiflash_erase_sector(sector_number);
        spiflash_wait_for_ready();
        res
    })
}

#[inline(always)]
#[unsafe(link_section = ".rwtext")]
fn spi_write_enable() {
    spiflash_wait_for_ready();

    write_register(SPI_RD_STATUS_REG, 0);
    write_register(SPI_CMD_REG, SPI_FLASH_WREN);
    while read_register(SPI_CMD_REG) != 0 {}
}

#[inline(never)]
#[unsafe(link_section = ".rwtext")]
pub(crate) fn spiflash_write(dest_addr: u32, data: *const u32, len: u32) -> i32 {
    maybe_with_critical_section(|| {
        begin();

        let flashchip = FLASH_CHIP_ADDR as *const EspRomSpiflashChipT;
        let mut status: u32 = 0;

        spiflash_wait_for_ready();
        if spi_read_status_high(flashchip, &mut status) != 0 {
            return -1;
        }

        spiflash_wait_for_ready();

        write_register(SPI_USER_REG, read_register(SPI_USER_REG) & !SPI_USR_DUMMY);
        let addrbits = ESP_ROM_SPIFLASH_W_SIO_ADDR_BITSLEN;
        let mut regval = read_register(SPI_USER1_REG);
        regval &= !SPI_USR_ADDR_BITLEN_M;
        regval |= addrbits << SPI_USR_ADDR_BITLEN_S;
        write_register(SPI_USER1_REG, regval);

        for block in (0..len).step_by(32) {
            spiflash_wait_for_ready();
            spi_write_enable();

            let block_len = if len - block < 32 { len - block } else { 32 };
            write_register(
                SPI_ADDR_REG,
                ((dest_addr + block) & 0xffffff) | (block_len << 24),
            );

            let data_ptr = unsafe { data.offset((block / 4) as isize) };
            for i in 0..block_len / 4 {
                write_register(SPI_W0_REG + (4 * i), unsafe {
                    data_ptr.offset(i as isize).read_volatile()
                });
            }

            write_register(SPI_RD_STATUS_REG, 0);
            write_register(SPI_CMD_REG, 1 << 25); // FLASH PP
            while read_register(SPI_CMD_REG) != 0 { /* wait */ }

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
#[unsafe(link_section = ".rwtext")]
pub fn read_register(address: u32) -> u32 {
    unsafe { (address as *const u32).read_volatile() }
}

#[inline(always)]
#[unsafe(link_section = ".rwtext")]
pub fn write_register(address: u32, value: u32) {
    unsafe {
        (address as *mut u32).write_volatile(value);
    }
}

#[inline(always)]
#[unsafe(link_section = ".rwtext")]
fn wait_for_ready() {
    while (read_register(SPI_EXT2_REG) & SPI_ST) != 0 {}
    // ESP32_OR_LATER ... we don't support anything earlier
    while (read_register(SPI0_EXT2_REG) & SPI_ST) != 0 {}
}

#[inline(always)]
#[unsafe(link_section = ".rwtext")]
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
#[unsafe(link_section = ".rwtext")]
pub(crate) fn spiflash_unlock() -> i32 {
    let flashchip = FLASH_CHIP_ADDR as *const EspRomSpiflashChipT;
    if (unsafe { (*flashchip).device_id } >> 16) & 0xff == 0x9D {
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

        write_register(SPI_CTRL_REG, read_register(SPI_CTRL_REG) | SPI_WRSR_2B);

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
