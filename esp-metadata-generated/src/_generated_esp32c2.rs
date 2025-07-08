/// The name of the chip as `&str`
///
/// # Example
///
/// ```rust, no_run
/// use esp_hal::chip;
/// let chip_name = chip!();
/// ```
#[macro_export]
macro_rules! chip {
    () => {
        "esp32c2"
    };
}
/// The properties of this chip and its drivers.
#[macro_export]
macro_rules! property {
    ("chip") => {
        "esp32c2"
    };
    ("arch") => {
        "riscv"
    };
    ("cores") => {
        1
    };
    ("cores", str) => {
        stringify!(1)
    };
    ("trm") => {
        "https://www.espressif.com/sites/default/files/documentation/esp8684_technical_reference_manual_en.pdf"
    };
    ("assist_debug.has_sp_monitor") => {
        true
    };
    ("assist_debug.has_region_monitor") => {
        false
    };
    ("gpio.has_bank_1") => {
        false
    };
    ("gpio.gpio_function") => {
        1
    };
    ("gpio.gpio_function", str) => {
        stringify!(1)
    };
    ("gpio.constant_0_input") => {
        31
    };
    ("gpio.constant_0_input", str) => {
        stringify!(31)
    };
    ("gpio.constant_1_input") => {
        30
    };
    ("gpio.constant_1_input", str) => {
        stringify!(30)
    };
    ("gpio.remap_iomux_pin_registers") => {
        false
    };
    ("gpio.func_in_sel_offset") => {
        0
    };
    ("gpio.func_in_sel_offset", str) => {
        stringify!(0)
    };
    ("gpio.input_signal_max") => {
        100
    };
    ("gpio.input_signal_max", str) => {
        stringify!(100)
    };
    ("gpio.output_signal_max") => {
        128
    };
    ("gpio.output_signal_max", str) => {
        stringify!(128)
    };
    ("i2c_master.has_fsm_timeouts") => {
        true
    };
    ("i2c_master.has_hw_bus_clear") => {
        true
    };
    ("i2c_master.has_bus_timeout_enable") => {
        true
    };
    ("i2c_master.separate_filter_config_registers") => {
        false
    };
    ("i2c_master.can_estimate_nack_reason") => {
        false
    };
    ("i2c_master.has_conf_update") => {
        true
    };
    ("i2c_master.has_reliable_fsm_reset") => {
        false
    };
    ("i2c_master.has_arbitration_en") => {
        true
    };
    ("i2c_master.has_tx_fifo_watermark") => {
        true
    };
    ("i2c_master.bus_timeout_is_exponential") => {
        true
    };
    ("i2c_master.max_bus_timeout") => {
        31
    };
    ("i2c_master.max_bus_timeout", str) => {
        stringify!(31)
    };
    ("i2c_master.ll_intr_mask") => {
        262143
    };
    ("i2c_master.ll_intr_mask", str) => {
        stringify!(262143)
    };
    ("i2c_master.fifo_size") => {
        16
    };
    ("i2c_master.fifo_size", str) => {
        stringify!(16)
    };
    ("interrupts.status_registers") => {
        2
    };
    ("interrupts.status_registers", str) => {
        stringify!(2)
    };
    ("spi_master.has_octal") => {
        false
    };
    ("timergroup.timg_has_timer1") => {
        false
    };
    ("wifi.has_wifi6") => {
        false
    };
}
/// Macro to get the address range of the given memory region.
#[macro_export]
macro_rules! memory_range {
    ("DRAM") => {
        1070202880..1070465024
    };
}
#[macro_export]
macro_rules! for_each_i2c_master {
    ($($pattern:tt => $code:tt;)*) => {
        macro_rules! _for_each_inner { $(($pattern) => $code;)* ($other : tt) => {} }
        _for_each_inner!((I2C0, I2cExt0, I2CEXT0_SCL, I2CEXT0_SDA));
        _for_each_inner!((all(I2C0, I2cExt0, I2CEXT0_SCL, I2CEXT0_SDA)));
    };
}
#[macro_export]
macro_rules! for_each_uart {
    ($($pattern:tt => $code:tt;)*) => {
        macro_rules! _for_each_inner { $(($pattern) => $code;)* ($other : tt) => {} }
        _for_each_inner!((UART0, Uart0, U0RXD, U0TXD, U0CTS, U0RTS));
        _for_each_inner!((UART1, Uart1, U1RXD, U1TXD, U1CTS, U1RTS));
        _for_each_inner!((all(UART0, Uart0, U0RXD, U0TXD, U0CTS, U0RTS), (UART1, Uart1,
        U1RXD, U1TXD, U1CTS, U1RTS)));
    };
}
#[macro_export]
macro_rules! for_each_spi_master {
    ($($pattern:tt => $code:tt;)*) => {
        macro_rules! _for_each_inner { $(($pattern) => $code;)* ($other : tt) => {} }
        _for_each_inner!((SPI2, Spi2, FSPICLK[FSPICS0, FSPICS1, FSPICS2, FSPICS3,
        FSPICS4, FSPICS5] [FSPID, FSPIQ, FSPIWP, FSPIHD], true));
        _for_each_inner!((all(SPI2, Spi2, FSPICLK[FSPICS0, FSPICS1, FSPICS2, FSPICS3,
        FSPICS4, FSPICS5] [FSPID, FSPIQ, FSPIWP, FSPIHD], true)));
    };
}
#[macro_export]
macro_rules! for_each_spi_slave {
    ($($pattern:tt => $code:tt;)*) => {
        macro_rules! _for_each_inner { $(($pattern) => $code;)* ($other : tt) => {} }
        _for_each_inner!((SPI2, Spi2, FSPICLK, FSPID, FSPIQ, FSPICS0));
        _for_each_inner!((all(SPI2, Spi2, FSPICLK, FSPID, FSPIQ, FSPICS0)));
    };
}
#[macro_export]
macro_rules! for_each_peripheral {
    ($($pattern:tt => $code:tt;)*) => {
        macro_rules! _for_each_inner { $(($pattern) => $code;)* ($other : tt) => {} }
        _for_each_inner!((GPIO0 <= virtual())); _for_each_inner!((GPIO1 <= virtual()));
        _for_each_inner!((GPIO2 <= virtual())); _for_each_inner!((GPIO3 <= virtual()));
        _for_each_inner!((GPIO4 <= virtual())); _for_each_inner!((GPIO5 <= virtual()));
        _for_each_inner!((GPIO6 <= virtual())); _for_each_inner!((GPIO7 <= virtual()));
        _for_each_inner!((GPIO8 <= virtual())); _for_each_inner!((GPIO9 <= virtual()));
        _for_each_inner!((GPIO10 <= virtual())); _for_each_inner!((GPIO11 <= virtual()));
        _for_each_inner!((GPIO12 <= virtual())); _for_each_inner!((GPIO13 <= virtual()));
        _for_each_inner!((GPIO14 <= virtual())); _for_each_inner!((GPIO15 <= virtual()));
        _for_each_inner!((GPIO16 <= virtual())); _for_each_inner!((GPIO17 <= virtual()));
        _for_each_inner!((GPIO18 <= virtual())); _for_each_inner!((GPIO19 <= virtual()));
        _for_each_inner!((GPIO20 <= virtual())); _for_each_inner!((APB_CTRL <= APB_CTRL()
        (unstable))); _for_each_inner!((APB_SARADC <= APB_SARADC() (unstable)));
        _for_each_inner!((BB <= BB() (unstable))); _for_each_inner!((ASSIST_DEBUG <=
        ASSIST_DEBUG() (unstable))); _for_each_inner!((DMA <= DMA() (unstable)));
        _for_each_inner!((ECC <= ECC() (unstable))); _for_each_inner!((EFUSE <= EFUSE()
        (unstable))); _for_each_inner!((EXTMEM <= EXTMEM() (unstable)));
        _for_each_inner!((GPIO <= GPIO() (unstable))); _for_each_inner!((I2C_ANA_MST <=
        I2C_ANA_MST() (unstable))); _for_each_inner!((I2C0 <= I2C0(I2C_EXT0 : {
        bind_peri_interrupt, enable_peri_interrupt, disable_peri_interrupt })));
        _for_each_inner!((INTERRUPT_CORE0 <= INTERRUPT_CORE0() (unstable)));
        _for_each_inner!((IO_MUX <= IO_MUX() (unstable))); _for_each_inner!((LEDC <=
        LEDC() (unstable))); _for_each_inner!((RNG <= RNG() (unstable)));
        _for_each_inner!((LPWR <= RTC_CNTL() (unstable))); _for_each_inner!((MODEM_CLKRST
        <= MODEM_CLKRST() (unstable))); _for_each_inner!((SENSITIVE <= SENSITIVE()
        (unstable))); _for_each_inner!((SHA <= SHA() (unstable))); _for_each_inner!((SPI0
        <= SPI0() (unstable))); _for_each_inner!((SPI1 <= SPI1() (unstable)));
        _for_each_inner!((SPI2 <= SPI2(SPI2 : { bind_peri_interrupt,
        enable_peri_interrupt, disable_peri_interrupt }))); _for_each_inner!((SYSTEM <=
        SYSTEM() (unstable))); _for_each_inner!((SYSTIMER <= SYSTIMER() (unstable)));
        _for_each_inner!((TIMG0 <= TIMG0() (unstable))); _for_each_inner!((UART0 <=
        UART0(UART0 : { bind_peri_interrupt, enable_peri_interrupt,
        disable_peri_interrupt }))); _for_each_inner!((UART1 <= UART1(UART1 : {
        bind_peri_interrupt, enable_peri_interrupt, disable_peri_interrupt })));
        _for_each_inner!((XTS_AES <= XTS_AES() (unstable))); _for_each_inner!((DMA_CH0 <=
        virtual() (unstable))); _for_each_inner!((ADC1 <= virtual() (unstable)));
        _for_each_inner!((BT <= virtual() (unstable))); _for_each_inner!((RADIO_CLK <=
        virtual() (unstable))); _for_each_inner!((SW_INTERRUPT <= virtual() (unstable)));
        _for_each_inner!((WIFI <= virtual() (unstable))); _for_each_inner!((MEM2MEM1 <=
        virtual() (unstable))); _for_each_inner!((MEM2MEM2 <= virtual() (unstable)));
        _for_each_inner!((MEM2MEM3 <= virtual() (unstable))); _for_each_inner!((MEM2MEM4
        <= virtual() (unstable))); _for_each_inner!((MEM2MEM5 <= virtual() (unstable)));
        _for_each_inner!((MEM2MEM6 <= virtual() (unstable))); _for_each_inner!((MEM2MEM7
        <= virtual() (unstable))); _for_each_inner!((MEM2MEM8 <= virtual() (unstable)));
        _for_each_inner!((all(GPIO0 <= virtual()), (GPIO1 <= virtual()), (GPIO2 <=
        virtual()), (GPIO3 <= virtual()), (GPIO4 <= virtual()), (GPIO5 <= virtual()),
        (GPIO6 <= virtual()), (GPIO7 <= virtual()), (GPIO8 <= virtual()), (GPIO9 <=
        virtual()), (GPIO10 <= virtual()), (GPIO11 <= virtual()), (GPIO12 <= virtual()),
        (GPIO13 <= virtual()), (GPIO14 <= virtual()), (GPIO15 <= virtual()), (GPIO16 <=
        virtual()), (GPIO17 <= virtual()), (GPIO18 <= virtual()), (GPIO19 <= virtual()),
        (GPIO20 <= virtual()), (APB_CTRL <= APB_CTRL() (unstable)), (APB_SARADC <=
        APB_SARADC() (unstable)), (BB <= BB() (unstable)), (ASSIST_DEBUG <=
        ASSIST_DEBUG() (unstable)), (DMA <= DMA() (unstable)), (ECC <= ECC() (unstable)),
        (EFUSE <= EFUSE() (unstable)), (EXTMEM <= EXTMEM() (unstable)), (GPIO <= GPIO()
        (unstable)), (I2C_ANA_MST <= I2C_ANA_MST() (unstable)), (I2C0 <= I2C0(I2C_EXT0 :
        { bind_peri_interrupt, enable_peri_interrupt, disable_peri_interrupt })),
        (INTERRUPT_CORE0 <= INTERRUPT_CORE0() (unstable)), (IO_MUX <= IO_MUX()
        (unstable)), (LEDC <= LEDC() (unstable)), (RNG <= RNG() (unstable)), (LPWR <=
        RTC_CNTL() (unstable)), (MODEM_CLKRST <= MODEM_CLKRST() (unstable)), (SENSITIVE
        <= SENSITIVE() (unstable)), (SHA <= SHA() (unstable)), (SPI0 <= SPI0()
        (unstable)), (SPI1 <= SPI1() (unstable)), (SPI2 <= SPI2(SPI2 : {
        bind_peri_interrupt, enable_peri_interrupt, disable_peri_interrupt })), (SYSTEM
        <= SYSTEM() (unstable)), (SYSTIMER <= SYSTIMER() (unstable)), (TIMG0 <= TIMG0()
        (unstable)), (UART0 <= UART0(UART0 : { bind_peri_interrupt,
        enable_peri_interrupt, disable_peri_interrupt })), (UART1 <= UART1(UART1 : {
        bind_peri_interrupt, enable_peri_interrupt, disable_peri_interrupt })), (XTS_AES
        <= XTS_AES() (unstable)), (DMA_CH0 <= virtual() (unstable)), (ADC1 <= virtual()
        (unstable)), (BT <= virtual() (unstable)), (RADIO_CLK <= virtual() (unstable)),
        (SW_INTERRUPT <= virtual() (unstable)), (WIFI <= virtual() (unstable)), (MEM2MEM1
        <= virtual() (unstable)), (MEM2MEM2 <= virtual() (unstable)), (MEM2MEM3 <=
        virtual() (unstable)), (MEM2MEM4 <= virtual() (unstable)), (MEM2MEM5 <= virtual()
        (unstable)), (MEM2MEM6 <= virtual() (unstable)), (MEM2MEM7 <= virtual()
        (unstable)), (MEM2MEM8 <= virtual() (unstable))));
    };
}
#[macro_export]
macro_rules! for_each_gpio {
    ($($pattern:tt => $code:tt;)*) => {
        macro_rules! _for_each_inner { $(($pattern) => $code;)* ($other : tt) => {} }
        _for_each_inner!((0, GPIO0() () (Input Output Analog RtcIo RtcIoOutput)));
        _for_each_inner!((1, GPIO1() () (Input Output Analog RtcIo RtcIoOutput)));
        _for_each_inner!((2, GPIO2(_2 => FSPIQ) (_2 => FSPIQ) (Input Output Analog RtcIo
        RtcIoOutput))); _for_each_inner!((3, GPIO3() () (Input Output Analog RtcIo
        RtcIoOutput))); _for_each_inner!((4, GPIO4(_0 => MTMS _2 => FSPIHD) (_2 =>
        FSPIHD) (Input Output Analog RtcIo RtcIoOutput))); _for_each_inner!((5, GPIO5(_0
        => MTDI _2 => FSPIWP) (_2 => FSPIWP) (Input Output Analog RtcIo RtcIoOutput)));
        _for_each_inner!((6, GPIO6(_0 => MTCK _2 => FSPICLK) (_2 => FSPICLK) (Input
        Output))); _for_each_inner!((7, GPIO7(_2 => FSPID) (_0 => MTDO _2 => FSPID)
        (Input Output))); _for_each_inner!((8, GPIO8() () (Input Output)));
        _for_each_inner!((9, GPIO9() () (Input Output))); _for_each_inner!((10, GPIO10()
        () (Input Output))); _for_each_inner!((11, GPIO11(_0 => SPIHD) (_0 => SPIHD)
        (Input Output))); _for_each_inner!((12, GPIO12(_0 => SPIHD) (_0 => SPIHD) (Input
        Output))); _for_each_inner!((13, GPIO13(_0 => SPIWP) (_0 => SPIWP) (Input
        Output))); _for_each_inner!((14, GPIO14() (_0 => SPICS0) (Input Output)));
        _for_each_inner!((15, GPIO15() (_0 => SPICLK) (Input Output)));
        _for_each_inner!((16, GPIO16(_0 => SPID) (_0 => SPID) (Input Output)));
        _for_each_inner!((17, GPIO17(_0 => SPIQ) (_0 => SPIQ) (Input Output)));
        _for_each_inner!((18, GPIO18() () (Input Output))); _for_each_inner!((19,
        GPIO19(_0 => U0RXD) () (Input Output))); _for_each_inner!((20, GPIO20() (_0 =>
        U0TXD) (Input Output))); _for_each_inner!((all(0, GPIO0() () (Input Output Analog
        RtcIo RtcIoOutput)), (1, GPIO1() () (Input Output Analog RtcIo RtcIoOutput)), (2,
        GPIO2(_2 => FSPIQ) (_2 => FSPIQ) (Input Output Analog RtcIo RtcIoOutput)), (3,
        GPIO3() () (Input Output Analog RtcIo RtcIoOutput)), (4, GPIO4(_0 => MTMS _2 =>
        FSPIHD) (_2 => FSPIHD) (Input Output Analog RtcIo RtcIoOutput)), (5, GPIO5(_0 =>
        MTDI _2 => FSPIWP) (_2 => FSPIWP) (Input Output Analog RtcIo RtcIoOutput)), (6,
        GPIO6(_0 => MTCK _2 => FSPICLK) (_2 => FSPICLK) (Input Output)), (7, GPIO7(_2 =>
        FSPID) (_0 => MTDO _2 => FSPID) (Input Output)), (8, GPIO8() () (Input Output)),
        (9, GPIO9() () (Input Output)), (10, GPIO10() () (Input Output)), (11, GPIO11(_0
        => SPIHD) (_0 => SPIHD) (Input Output)), (12, GPIO12(_0 => SPIHD) (_0 => SPIHD)
        (Input Output)), (13, GPIO13(_0 => SPIWP) (_0 => SPIWP) (Input Output)), (14,
        GPIO14() (_0 => SPICS0) (Input Output)), (15, GPIO15() (_0 => SPICLK) (Input
        Output)), (16, GPIO16(_0 => SPID) (_0 => SPID) (Input Output)), (17, GPIO17(_0 =>
        SPIQ) (_0 => SPIQ) (Input Output)), (18, GPIO18() () (Input Output)), (19,
        GPIO19(_0 => U0RXD) () (Input Output)), (20, GPIO20() (_0 => U0TXD) (Input
        Output))));
    };
}
#[macro_export]
macro_rules! if_pin_is_type {
    (GPIO0, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO0, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO0, Analog, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO0, RtcIo, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO0, RtcIoOutput, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO0, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO1, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO1, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO1, Analog, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO1, RtcIo, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO1, RtcIoOutput, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO1, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO2, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO2, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO2, Analog, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO2, RtcIo, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO2, RtcIoOutput, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO2, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO3, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO3, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO3, Analog, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO3, RtcIo, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO3, RtcIoOutput, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO3, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO4, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO4, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO4, Analog, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO4, RtcIo, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO4, RtcIoOutput, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO4, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO5, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO5, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO5, Analog, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO5, RtcIo, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO5, RtcIoOutput, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO5, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO6, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO6, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO6, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO7, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO7, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO7, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO8, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO8, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO8, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO9, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO9, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO9, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO10, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO10, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO10, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO11, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO11, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO11, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO12, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO12, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO12, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO13, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO13, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO13, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO14, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO14, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO14, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO15, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO15, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO15, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO16, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO16, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO16, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO17, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO17, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO17, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO18, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO18, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO18, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO19, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO19, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO19, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO20, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO20, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO20, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
}
#[macro_export]
#[expect(clippy::crate_in_macro_def)]
macro_rules! impl_for_pin_type {
    ($any_pin:ident, $inner_ident:ident, $on_type:tt, $code:tt else $otherwise:tt) => {
        match $any_pin .number() { 0 => if_pin_is_type!(GPIO0, $on_type, { {
        #[allow(unused_unsafe, unused_mut)] let mut $inner_ident = unsafe { crate
        ::peripherals::GPIO0::steal() }; #[allow(unused_braces)] $code } } else {
        $otherwise }), 1 => if_pin_is_type!(GPIO1, $on_type, { { #[allow(unused_unsafe,
        unused_mut)] let mut $inner_ident = unsafe { crate ::peripherals::GPIO1::steal()
        }; #[allow(unused_braces)] $code } } else { $otherwise }), 2 =>
        if_pin_is_type!(GPIO2, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO2::steal() };
        #[allow(unused_braces)] $code } } else { $otherwise }), 3 =>
        if_pin_is_type!(GPIO3, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO3::steal() };
        #[allow(unused_braces)] $code } } else { $otherwise }), 4 =>
        if_pin_is_type!(GPIO4, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO4::steal() };
        #[allow(unused_braces)] $code } } else { $otherwise }), 5 =>
        if_pin_is_type!(GPIO5, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO5::steal() };
        #[allow(unused_braces)] $code } } else { $otherwise }), 6 =>
        if_pin_is_type!(GPIO6, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO6::steal() };
        #[allow(unused_braces)] $code } } else { $otherwise }), 7 =>
        if_pin_is_type!(GPIO7, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO7::steal() };
        #[allow(unused_braces)] $code } } else { $otherwise }), 8 =>
        if_pin_is_type!(GPIO8, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO8::steal() };
        #[allow(unused_braces)] $code } } else { $otherwise }), 9 =>
        if_pin_is_type!(GPIO9, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO9::steal() };
        #[allow(unused_braces)] $code } } else { $otherwise }), 10 =>
        if_pin_is_type!(GPIO10, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO10::steal() };
        #[allow(unused_braces)] $code } } else { $otherwise }), 11 =>
        if_pin_is_type!(GPIO11, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO11::steal() };
        #[allow(unused_braces)] $code } } else { $otherwise }), 12 =>
        if_pin_is_type!(GPIO12, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO12::steal() };
        #[allow(unused_braces)] $code } } else { $otherwise }), 13 =>
        if_pin_is_type!(GPIO13, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO13::steal() };
        #[allow(unused_braces)] $code } } else { $otherwise }), 14 =>
        if_pin_is_type!(GPIO14, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO14::steal() };
        #[allow(unused_braces)] $code } } else { $otherwise }), 15 =>
        if_pin_is_type!(GPIO15, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO15::steal() };
        #[allow(unused_braces)] $code } } else { $otherwise }), 16 =>
        if_pin_is_type!(GPIO16, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO16::steal() };
        #[allow(unused_braces)] $code } } else { $otherwise }), 17 =>
        if_pin_is_type!(GPIO17, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO17::steal() };
        #[allow(unused_braces)] $code } } else { $otherwise }), 18 =>
        if_pin_is_type!(GPIO18, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO18::steal() };
        #[allow(unused_braces)] $code } } else { $otherwise }), 19 =>
        if_pin_is_type!(GPIO19, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO19::steal() };
        #[allow(unused_braces)] $code } } else { $otherwise }), 20 =>
        if_pin_is_type!(GPIO20, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO20::steal() };
        #[allow(unused_braces)] $code } } else { $otherwise }), _ => $otherwise, }
    };
    ($any_pin:ident, $inner_ident:ident, $on_type:tt, $code:tt) => {
        impl_for_pin_type!($any_pin, $inner_ident, $on_type, $code else {
        panic!("Unsupported") })
    };
}
#[macro_export]
macro_rules! define_io_mux_signals {
    () => {
        #[allow(non_camel_case_types, clippy::upper_case_acronyms)]
        #[derive(Debug, PartialEq, Copy, Clone)]
        #[cfg_attr(feature = "defmt", derive(defmt::Format))]
        #[doc(hidden)]
        pub enum InputSignal {
            SPIQ          = 0,
            SPID          = 1,
            SPIHD         = 2,
            SPIWP         = 3,
            U0RXD         = 6,
            U0CTS         = 7,
            U0DSR         = 8,
            U1RXD         = 9,
            U1CTS         = 10,
            U1DSR         = 11,
            CPU_GPIO_0    = 28,
            CPU_GPIO_1    = 29,
            CPU_GPIO_2    = 30,
            CPU_GPIO_3    = 31,
            CPU_GPIO_4    = 32,
            CPU_GPIO_5    = 33,
            CPU_GPIO_6    = 34,
            CPU_GPIO_7    = 35,
            EXT_ADC_START = 45,
            RMT_SIG_0     = 51,
            RMT_SIG_1     = 52,
            I2CEXT0_SCL   = 53,
            I2CEXT0_SDA   = 54,
            FSPICLK       = 63,
            FSPIQ         = 64,
            FSPID         = 65,
            FSPIHD        = 66,
            FSPIWP        = 67,
            FSPICS0       = 68,
            SIG_FUNC_97   = 97,
            SIG_FUNC_98   = 98,
            SIG_FUNC_99   = 99,
            SIG_FUNC_100  = 100,
            MTCK,
            MTMS,
            MTDI,
        }
        #[allow(non_camel_case_types, clippy::upper_case_acronyms)]
        #[derive(Debug, PartialEq, Copy, Clone)]
        #[cfg_attr(feature = "defmt", derive(defmt::Format))]
        #[doc(hidden)]
        pub enum OutputSignal {
            SPIQ          = 0,
            SPID          = 1,
            SPIHD         = 2,
            SPIWP         = 3,
            SPICLK        = 4,
            SPICS0        = 5,
            U0TXD         = 6,
            U0RTS         = 7,
            U0DTR         = 8,
            U1TXD         = 9,
            U1RTS         = 10,
            U1DTR         = 11,
            SPIQ_MONITOR  = 15,
            SPID_MONITOR  = 16,
            SPIHD_MONITOR = 17,
            SPIWP_MONITOR = 18,
            SPICS1        = 19,
            CPU_GPIO_0    = 28,
            CPU_GPIO_1    = 29,
            CPU_GPIO_2    = 30,
            CPU_GPIO_3    = 31,
            CPU_GPIO_4    = 32,
            CPU_GPIO_5    = 33,
            CPU_GPIO_6    = 34,
            CPU_GPIO_7    = 35,
            LEDC_LS_SIG0  = 45,
            LEDC_LS_SIG1  = 46,
            LEDC_LS_SIG2  = 47,
            LEDC_LS_SIG3  = 48,
            LEDC_LS_SIG4  = 49,
            LEDC_LS_SIG5  = 50,
            RMT_SIG_0     = 51,
            RMT_SIG_1     = 52,
            I2CEXT0_SCL   = 53,
            I2CEXT0_SDA   = 54,
            FSPICLK       = 63,
            FSPIQ         = 64,
            FSPID         = 65,
            FSPIHD        = 66,
            FSPIWP        = 67,
            FSPICS0       = 68,
            FSPICS1       = 69,
            FSPICS3       = 70,
            FSPICS2       = 71,
            FSPICS4       = 72,
            FSPICS5       = 73,
            ANT_SEL0      = 89,
            ANT_SEL1      = 90,
            ANT_SEL2      = 91,
            ANT_SEL3      = 92,
            ANT_SEL4      = 93,
            ANT_SEL5      = 94,
            ANT_SEL6      = 95,
            ANT_SEL7      = 96,
            SIG_FUNC_97   = 97,
            SIG_FUNC_98   = 98,
            SIG_FUNC_99   = 99,
            SIG_FUNC_100  = 100,
            CLK_OUT1      = 123,
            CLK_OUT2      = 124,
            CLK_OUT3      = 125,
            GPIO          = 128,
            MTDO,
        }
    };
}
#[macro_export]
#[expect(clippy::crate_in_macro_def)]
macro_rules! define_io_mux_reg {
    () => {
        pub(crate) fn io_mux_reg(gpio_num: u8) -> &'static crate::pac::io_mux::GPIO {
            crate::peripherals::IO_MUX::regs().gpio(gpio_num as usize)
        }
    };
}
