/// The name of the chip as `&str`
#[macro_export]
macro_rules! chip {
    () => {
        "esp32s2"
    };
}
/// A link to the Technical Reference Manual (TRM) for the chip.
#[doc(hidden)]
#[macro_export]
macro_rules! property {
    ("chip") => {
        "esp32s2"
    };
    ("arch") => {
        "xtensa"
    };
    ("cores") => {
        1
    };
    ("cores", str) => {
        stringify!(1)
    };
    ("trm") => {
        "https://www.espressif.com/sites/default/files/documentation/esp32-s2_technical_reference_manual_en.pdf"
    };
    ("gpio.has_bank_1") => {
        true
    };
    ("gpio.gpio_function") => {
        1
    };
    ("gpio.gpio_function", str) => {
        stringify!(1)
    };
    ("gpio.input_signal_max") => {
        204
    };
    ("gpio.input_signal_max", str) => {
        stringify!(204)
    };
    ("gpio.output_signal_max") => {
        256
    };
    ("gpio.output_signal_max", str) => {
        stringify!(256)
    };
    ("gpio.constant_0_input") => {
        60
    };
    ("gpio.constant_0_input", str) => {
        stringify!(60)
    };
    ("gpio.constant_1_input") => {
        56
    };
    ("gpio.constant_1_input", str) => {
        stringify!(56)
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
    ("i2c_master.has_fsm_timeouts") => {
        false
    };
    ("i2c_master.has_hw_bus_clear") => {
        false
    };
    ("i2c_master.has_bus_timeout_enable") => {
        true
    };
    ("i2c_master.separate_filter_config_registers") => {
        true
    };
    ("i2c_master.can_estimate_nack_reason") => {
        false
    };
    ("i2c_master.has_conf_update") => {
        false
    };
    ("i2c_master.has_reliable_fsm_reset") => {
        false
    };
    ("i2c_master.has_arbitration_en") => {
        true
    };
    ("i2c_master.has_tx_fifo_watermark") => {
        false
    };
    ("i2c_master.bus_timeout_is_exponential") => {
        false
    };
    ("i2c_master.i2c0_data_register_ahb_address") => {
        1610690588
    };
    ("i2c_master.i2c0_data_register_ahb_address", str) => {
        stringify!(1610690588)
    };
    ("i2c_master.max_bus_timeout") => {
        16777215
    };
    ("i2c_master.max_bus_timeout", str) => {
        stringify!(16777215)
    };
    ("i2c_master.ll_intr_mask") => {
        131071
    };
    ("i2c_master.ll_intr_mask", str) => {
        stringify!(131071)
    };
    ("i2c_master.fifo_size") => {
        32
    };
    ("i2c_master.fifo_size", str) => {
        stringify!(32)
    };
    ("interrupts.status_registers") => {
        3
    };
    ("interrupts.status_registers", str) => {
        stringify!(3)
    };
    ("rmt.ram_start") => {
        1061250048
    };
    ("rmt.ram_start", str) => {
        stringify!(1061250048)
    };
    ("rmt.channel_ram_size") => {
        64
    };
    ("rmt.channel_ram_size", str) => {
        stringify!(64)
    };
    ("spi_master.has_octal") => {
        true
    };
    ("timergroup.timg_has_timer1") => {
        true
    };
    ("wifi.has_wifi6") => {
        false
    };
}
/// Macro to get the address range of the given memory region.
#[macro_export]
#[doc(hidden)]
macro_rules! memory_range {
    ("DRAM") => {
        1073414144..1073741824
    };
}
#[macro_export]
macro_rules! for_each_i2c_master {
    ($($pattern:tt => $code:tt;)*) => {
        macro_rules! _for_each_inner { $(($pattern) => $code;)* ($other : tt) => {} }
        _for_each_inner!((I2C0, I2cExt0, I2CEXT0_SCL, I2CEXT0_SDA));
        _for_each_inner!((I2C1, I2cExt1, I2CEXT1_SCL, I2CEXT1_SDA));
        _for_each_inner!((all(I2C0, I2cExt0, I2CEXT0_SCL, I2CEXT0_SDA), (I2C1, I2cExt1,
        I2CEXT1_SCL, I2CEXT1_SDA)));
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
        FSPICS4, FSPICS5] [FSPID, FSPIQ, FSPIWP, FSPIHD, FSPIIO4, FSPIIO5, FSPIIO6,
        FSPIIO7], true)); _for_each_inner!((SPI3, Spi3, SPI3_CLK[SPI3_CS0, SPI3_CS1,
        SPI3_CS2] [SPI3_D, SPI3_Q])); _for_each_inner!((all(SPI2, Spi2, FSPICLK[FSPICS0,
        FSPICS1, FSPICS2, FSPICS3, FSPICS4, FSPICS5] [FSPID, FSPIQ, FSPIWP, FSPIHD,
        FSPIIO4, FSPIIO5, FSPIIO6, FSPIIO7], true), (SPI3, Spi3, SPI3_CLK[SPI3_CS0,
        SPI3_CS1, SPI3_CS2] [SPI3_D, SPI3_Q])));
    };
}
#[macro_export]
macro_rules! for_each_spi_slave {
    ($($pattern:tt => $code:tt;)*) => {
        macro_rules! _for_each_inner { $(($pattern) => $code;)* ($other : tt) => {} }
        _for_each_inner!((SPI2, Spi2, FSPICLK, FSPID, FSPIQ, FSPICS0));
        _for_each_inner!((SPI3, Spi3, SPI3_CLK, SPI3_D, SPI3_Q, SPI3_CS0));
        _for_each_inner!((all(SPI2, Spi2, FSPICLK, FSPID, FSPIQ, FSPICS0), (SPI3, Spi3,
        SPI3_CLK, SPI3_D, SPI3_Q, SPI3_CS0)));
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
        _for_each_inner!((GPIO20 <= virtual())); _for_each_inner!((GPIO21 <= virtual()));
        _for_each_inner!((GPIO26 <= virtual())); _for_each_inner!((GPIO27 <= virtual()));
        _for_each_inner!((GPIO28 <= virtual())); _for_each_inner!((GPIO29 <= virtual()));
        _for_each_inner!((GPIO30 <= virtual())); _for_each_inner!((GPIO31 <= virtual()));
        _for_each_inner!((GPIO32 <= virtual())); _for_each_inner!((GPIO33 <= virtual()));
        _for_each_inner!((GPIO34 <= virtual())); _for_each_inner!((GPIO35 <= virtual()));
        _for_each_inner!((GPIO36 <= virtual())); _for_each_inner!((GPIO37 <= virtual()));
        _for_each_inner!((GPIO38 <= virtual())); _for_each_inner!((GPIO39 <= virtual()));
        _for_each_inner!((GPIO40 <= virtual())); _for_each_inner!((GPIO41 <= virtual()));
        _for_each_inner!((GPIO42 <= virtual())); _for_each_inner!((GPIO43 <= virtual()));
        _for_each_inner!((GPIO44 <= virtual())); _for_each_inner!((GPIO45 <= virtual()));
        _for_each_inner!((GPIO46 <= virtual())); _for_each_inner!((AES <= AES()
        (unstable))); _for_each_inner!((APB_SARADC <= APB_SARADC() (unstable)));
        _for_each_inner!((DEDICATED_GPIO <= DEDICATED_GPIO() (unstable)));
        _for_each_inner!((DS <= DS() (unstable))); _for_each_inner!((EFUSE <= EFUSE()
        (unstable))); _for_each_inner!((EXTMEM <= EXTMEM() (unstable)));
        _for_each_inner!((FE <= FE() (unstable))); _for_each_inner!((FE2 <= FE2()
        (unstable))); _for_each_inner!((GPIO <= GPIO() (unstable)));
        _for_each_inner!((GPIO_SD <= GPIO_SD() (unstable))); _for_each_inner!((HMAC <=
        HMAC() (unstable))); _for_each_inner!((I2C_ANA_MST <= I2C_ANA_MST() (unstable)));
        _for_each_inner!((I2C0 <= I2C0(I2C_EXT0 : { bind_peri_interrupt,
        enable_peri_interrupt, disable_peri_interrupt }))); _for_each_inner!((I2C1 <=
        I2C1(I2C_EXT1 : { bind_peri_interrupt, enable_peri_interrupt,
        disable_peri_interrupt }))); _for_each_inner!((I2S0 <= I2S0(I2S0 : {
        bind_peri_interrupt, enable_peri_interrupt, disable_peri_interrupt })
        (unstable))); _for_each_inner!((INTERRUPT_CORE0 <= INTERRUPT_CORE0()
        (unstable))); _for_each_inner!((IO_MUX <= IO_MUX() (unstable)));
        _for_each_inner!((LEDC <= LEDC() (unstable))); _for_each_inner!((NRX <= NRX()
        (unstable))); _for_each_inner!((PCNT <= PCNT() (unstable)));
        _for_each_inner!((PMS <= PMS() (unstable))); _for_each_inner!((RMT <= RMT()
        (unstable))); _for_each_inner!((RNG <= RNG() (unstable))); _for_each_inner!((RSA
        <= RSA() (unstable))); _for_each_inner!((LPWR <= RTC_CNTL() (unstable)));
        _for_each_inner!((RTC_I2C <= RTC_I2C() (unstable))); _for_each_inner!((RTC_IO <=
        RTC_IO() (unstable))); _for_each_inner!((SENS <= SENS() (unstable)));
        _for_each_inner!((SHA <= SHA() (unstable))); _for_each_inner!((SPI0 <= SPI0()
        (unstable))); _for_each_inner!((SPI1 <= SPI1() (unstable)));
        _for_each_inner!((SPI2 <= SPI2(SPI2_DMA : { bind_dma_interrupt,
        enable_dma_interrupt, disable_dma_interrupt }, SPI2 : { bind_peri_interrupt,
        enable_peri_interrupt, disable_peri_interrupt }))); _for_each_inner!((SPI3 <=
        SPI3(SPI3_DMA : { bind_dma_interrupt, enable_dma_interrupt, disable_dma_interrupt
        }, SPI3 : { bind_peri_interrupt, enable_peri_interrupt, disable_peri_interrupt
        }))); _for_each_inner!((SYSCON <= SYSCON() (unstable))); _for_each_inner!((SYSTEM
        <= SYSTEM() (unstable))); _for_each_inner!((SYSTIMER <= SYSTIMER() (unstable)));
        _for_each_inner!((TIMG0 <= TIMG0() (unstable))); _for_each_inner!((TIMG1 <=
        TIMG1() (unstable))); _for_each_inner!((TWAI0 <= TWAI0() (unstable)));
        _for_each_inner!((UART0 <= UART0(UART0 : { bind_peri_interrupt,
        enable_peri_interrupt, disable_peri_interrupt }))); _for_each_inner!((UART1 <=
        UART1(UART1 : { bind_peri_interrupt, enable_peri_interrupt,
        disable_peri_interrupt }))); _for_each_inner!((UHCI0 <= UHCI0() (unstable)));
        _for_each_inner!((USB0 <= USB0() (unstable))); _for_each_inner!((USB_WRAP <=
        USB_WRAP() (unstable))); _for_each_inner!((XTS_AES <= XTS_AES() (unstable)));
        _for_each_inner!((WIFI <= WIFI() (unstable))); _for_each_inner!((DMA_SPI2 <=
        SPI2() (unstable))); _for_each_inner!((DMA_SPI3 <= SPI3() (unstable)));
        _for_each_inner!((DMA_I2S0 <= I2S0() (unstable))); _for_each_inner!((DMA_CRYPTO
        <= CRYPTO_DMA() (unstable))); _for_each_inner!((DMA_COPY <= COPY_DMA()
        (unstable))); _for_each_inner!((ADC1 <= virtual() (unstable)));
        _for_each_inner!((ADC2 <= virtual() (unstable))); _for_each_inner!((DAC1 <=
        virtual() (unstable))); _for_each_inner!((DAC2 <= virtual() (unstable)));
        _for_each_inner!((PSRAM <= virtual() (unstable))); _for_each_inner!((RADIO_CLK <=
        virtual() (unstable))); _for_each_inner!((SW_INTERRUPT <= virtual() (unstable)));
        _for_each_inner!((ULP_RISCV_CORE <= virtual() (unstable)));
        _for_each_inner!((all(GPIO0 <= virtual()), (GPIO1 <= virtual()), (GPIO2 <=
        virtual()), (GPIO3 <= virtual()), (GPIO4 <= virtual()), (GPIO5 <= virtual()),
        (GPIO6 <= virtual()), (GPIO7 <= virtual()), (GPIO8 <= virtual()), (GPIO9 <=
        virtual()), (GPIO10 <= virtual()), (GPIO11 <= virtual()), (GPIO12 <= virtual()),
        (GPIO13 <= virtual()), (GPIO14 <= virtual()), (GPIO15 <= virtual()), (GPIO16 <=
        virtual()), (GPIO17 <= virtual()), (GPIO18 <= virtual()), (GPIO19 <= virtual()),
        (GPIO20 <= virtual()), (GPIO21 <= virtual()), (GPIO26 <= virtual()), (GPIO27 <=
        virtual()), (GPIO28 <= virtual()), (GPIO29 <= virtual()), (GPIO30 <= virtual()),
        (GPIO31 <= virtual()), (GPIO32 <= virtual()), (GPIO33 <= virtual()), (GPIO34 <=
        virtual()), (GPIO35 <= virtual()), (GPIO36 <= virtual()), (GPIO37 <= virtual()),
        (GPIO38 <= virtual()), (GPIO39 <= virtual()), (GPIO40 <= virtual()), (GPIO41 <=
        virtual()), (GPIO42 <= virtual()), (GPIO43 <= virtual()), (GPIO44 <= virtual()),
        (GPIO45 <= virtual()), (GPIO46 <= virtual()), (AES <= AES() (unstable)),
        (APB_SARADC <= APB_SARADC() (unstable)), (DEDICATED_GPIO <= DEDICATED_GPIO()
        (unstable)), (DS <= DS() (unstable)), (EFUSE <= EFUSE() (unstable)), (EXTMEM <=
        EXTMEM() (unstable)), (FE <= FE() (unstable)), (FE2 <= FE2() (unstable)), (GPIO
        <= GPIO() (unstable)), (GPIO_SD <= GPIO_SD() (unstable)), (HMAC <= HMAC()
        (unstable)), (I2C_ANA_MST <= I2C_ANA_MST() (unstable)), (I2C0 <= I2C0(I2C_EXT0 :
        { bind_peri_interrupt, enable_peri_interrupt, disable_peri_interrupt })), (I2C1
        <= I2C1(I2C_EXT1 : { bind_peri_interrupt, enable_peri_interrupt,
        disable_peri_interrupt })), (I2S0 <= I2S0(I2S0 : { bind_peri_interrupt,
        enable_peri_interrupt, disable_peri_interrupt }) (unstable)), (INTERRUPT_CORE0 <=
        INTERRUPT_CORE0() (unstable)), (IO_MUX <= IO_MUX() (unstable)), (LEDC <= LEDC()
        (unstable)), (NRX <= NRX() (unstable)), (PCNT <= PCNT() (unstable)), (PMS <=
        PMS() (unstable)), (RMT <= RMT() (unstable)), (RNG <= RNG() (unstable)), (RSA <=
        RSA() (unstable)), (LPWR <= RTC_CNTL() (unstable)), (RTC_I2C <= RTC_I2C()
        (unstable)), (RTC_IO <= RTC_IO() (unstable)), (SENS <= SENS() (unstable)), (SHA
        <= SHA() (unstable)), (SPI0 <= SPI0() (unstable)), (SPI1 <= SPI1() (unstable)),
        (SPI2 <= SPI2(SPI2_DMA : { bind_dma_interrupt, enable_dma_interrupt,
        disable_dma_interrupt }, SPI2 : { bind_peri_interrupt, enable_peri_interrupt,
        disable_peri_interrupt })), (SPI3 <= SPI3(SPI3_DMA : { bind_dma_interrupt,
        enable_dma_interrupt, disable_dma_interrupt }, SPI3 : { bind_peri_interrupt,
        enable_peri_interrupt, disable_peri_interrupt })), (SYSCON <= SYSCON()
        (unstable)), (SYSTEM <= SYSTEM() (unstable)), (SYSTIMER <= SYSTIMER()
        (unstable)), (TIMG0 <= TIMG0() (unstable)), (TIMG1 <= TIMG1() (unstable)), (TWAI0
        <= TWAI0() (unstable)), (UART0 <= UART0(UART0 : { bind_peri_interrupt,
        enable_peri_interrupt, disable_peri_interrupt })), (UART1 <= UART1(UART1 : {
        bind_peri_interrupt, enable_peri_interrupt, disable_peri_interrupt })), (UHCI0 <=
        UHCI0() (unstable)), (USB0 <= USB0() (unstable)), (USB_WRAP <= USB_WRAP()
        (unstable)), (XTS_AES <= XTS_AES() (unstable)), (WIFI <= WIFI() (unstable)),
        (DMA_SPI2 <= SPI2() (unstable)), (DMA_SPI3 <= SPI3() (unstable)), (DMA_I2S0 <=
        I2S0() (unstable)), (DMA_CRYPTO <= CRYPTO_DMA() (unstable)), (DMA_COPY <=
        COPY_DMA() (unstable)), (ADC1 <= virtual() (unstable)), (ADC2 <= virtual()
        (unstable)), (DAC1 <= virtual() (unstable)), (DAC2 <= virtual() (unstable)),
        (PSRAM <= virtual() (unstable)), (RADIO_CLK <= virtual() (unstable)),
        (SW_INTERRUPT <= virtual() (unstable)), (ULP_RISCV_CORE <= virtual()
        (unstable))));
    };
}
#[macro_export]
macro_rules! for_each_gpio {
    ($($pattern:tt => $code:tt;)*) => {
        macro_rules! _for_each_inner { $(($pattern) => $code;)* ($other : tt) => {} }
        _for_each_inner!((0, GPIO0() () (Input Output Analog RtcIo RtcIoOutput)));
        _for_each_inner!((1, GPIO1() () (Input Output Analog RtcIo RtcIoOutput)));
        _for_each_inner!((2, GPIO2() () (Input Output Analog RtcIo RtcIoOutput)));
        _for_each_inner!((3, GPIO3() () (Input Output Analog RtcIo RtcIoOutput)));
        _for_each_inner!((4, GPIO4() () (Input Output Analog RtcIo RtcIoOutput)));
        _for_each_inner!((5, GPIO5() () (Input Output Analog RtcIo RtcIoOutput)));
        _for_each_inner!((6, GPIO6() () (Input Output Analog RtcIo RtcIoOutput)));
        _for_each_inner!((7, GPIO7() () (Input Output Analog RtcIo RtcIoOutput)));
        _for_each_inner!((8, GPIO8() (_3 => SUBSPICS1) (Input Output Analog RtcIo
        RtcIoOutput))); _for_each_inner!((9, GPIO9(_3 => SUBSPIHD _4 => FSPIHD) (_3 =>
        SUBSPIHD _4 => FSPIHD) (Input Output Analog RtcIo RtcIoOutput)));
        _for_each_inner!((10, GPIO10(_2 => FSPIIO4 _4 => FSPICS0) (_2 => FSPIIO4 _3 =>
        SUBSPICS0 _4 => FSPICS0) (Input Output Analog RtcIo RtcIoOutput)));
        _for_each_inner!((11, GPIO11(_2 => FSPIIO5 _3 => SUBSPID _4 => FSPID) (_2 =>
        FSPIIO5 _3 => SUBSPID _4 => FSPID) (Input Output Analog RtcIo RtcIoOutput)));
        _for_each_inner!((12, GPIO12(_2 => FSPIIO6 _4 => FSPICLK) (_2 => FSPIIO6 _3 =>
        SUBSPICLK _4 => FSPICLK) (Input Output Analog RtcIo RtcIoOutput)));
        _for_each_inner!((13, GPIO13(_2 => FSPIIO7 _3 => SUBSPIQ _4 => FSPIQ) (_2 =>
        FSPIIO7 _3 => SUBSPIQ _4 => FSPIQ) (Input Output Analog RtcIo RtcIoOutput)));
        _for_each_inner!((14, GPIO14(_3 => SUBSPIWP _4 => FSPIWP) (_2 => FSPIDQS _3 =>
        SUBSPIWP _4 => FSPIWP) (Input Output Analog RtcIo RtcIoOutput)));
        _for_each_inner!((15, GPIO15() (_2 => U0RTS) (Input Output Analog RtcIo
        RtcIoOutput))); _for_each_inner!((16, GPIO16(_2 => U0CTS) () (Input Output Analog
        RtcIo RtcIoOutput))); _for_each_inner!((17, GPIO17() (_2 => U1TXD) (Input Output
        Analog RtcIo RtcIoOutput))); _for_each_inner!((18, GPIO18(_2 => U1RXD) (_3 =>
        CLK_OUT3) (Input Output Analog RtcIo RtcIoOutput))); _for_each_inner!((19,
        GPIO19() (_2 => U1RTS _3 => CLK_OUT2) (Input Output Analog RtcIo RtcIoOutput
        UsbDm))); _for_each_inner!((20, GPIO20(_2 => U1CTS) (_3 => CLK_OUT1) (Input
        Output Analog RtcIo RtcIoOutput UsbDp))); _for_each_inner!((21, GPIO21() ()
        (Input Output Analog RtcIo RtcIoOutput))); _for_each_inner!((26, GPIO26() (_0 =>
        SPICS1) (Input Output))); _for_each_inner!((27, GPIO27(_0 => SPIHD) (_0 => SPIHD)
        (Input Output))); _for_each_inner!((28, GPIO28(_0 => SPIWP) (_0 => SPIWP) (Input
        Output))); _for_each_inner!((29, GPIO29() (_0 => SPICS0) (Input Output)));
        _for_each_inner!((30, GPIO30() (_0 => SPICLK) (Input Output)));
        _for_each_inner!((31, GPIO31(_0 => SPIQ) (_0 => SPIQ) (Input Output)));
        _for_each_inner!((32, GPIO32(_0 => SPID) (_0 => SPID) (Input Output)));
        _for_each_inner!((33, GPIO33(_2 => FSPIHD _3 => SUBSPIHD) (_2 => FSPIHD _3 =>
        SUBSPIHD) (Input Output))); _for_each_inner!((34, GPIO34(_2 => FSPICS0) (_2 =>
        FSPICS0 _3 => SUBSPICS0) (Input Output))); _for_each_inner!((35, GPIO35(_2 =>
        FSPID _3 => SUBSPID) (_2 => FSPID _3 => SUBSPID) (Input Output)));
        _for_each_inner!((36, GPIO36(_2 => FSPICLK) (_2 => FSPICLK _3 => SUBSPICLK)
        (Input Output))); _for_each_inner!((37, GPIO37(_2 => FSPIQ _3 => SUBSPIQ _4 =>
        SPIDQS) (_2 => FSPIQ _3 => SUBSPIQ _4 => SPIDQS) (Input Output)));
        _for_each_inner!((38, GPIO38(_2 => FSPIWP _3 => SUBSPIWP) (_2 => FSPIWP _3 =>
        SUBSPIWP) (Input Output))); _for_each_inner!((39, GPIO39(_0 => MTCK) (_2 =>
        CLK_OUT3 _3 => SUBSPICS1) (Input Output))); _for_each_inner!((40, GPIO40() (_0 =>
        MTDO _2 => CLK_OUT2) (Input Output))); _for_each_inner!((41, GPIO41(_0 => MTDI)
        (_2 => CLK_OUT1) (Input Output))); _for_each_inner!((42, GPIO42(_0 => MTMS) ()
        (Input Output))); _for_each_inner!((43, GPIO43() (_0 => U0TXD _2 => CLK_OUT1)
        (Input Output))); _for_each_inner!((44, GPIO44(_0 => U0RXD) (_2 => CLK_OUT2)
        (Input Output))); _for_each_inner!((45, GPIO45() () (Input Output)));
        _for_each_inner!((46, GPIO46() () (Input Output))); _for_each_inner!((all(0,
        GPIO0() () (Input Output Analog RtcIo RtcIoOutput)), (1, GPIO1() () (Input Output
        Analog RtcIo RtcIoOutput)), (2, GPIO2() () (Input Output Analog RtcIo
        RtcIoOutput)), (3, GPIO3() () (Input Output Analog RtcIo RtcIoOutput)), (4,
        GPIO4() () (Input Output Analog RtcIo RtcIoOutput)), (5, GPIO5() () (Input Output
        Analog RtcIo RtcIoOutput)), (6, GPIO6() () (Input Output Analog RtcIo
        RtcIoOutput)), (7, GPIO7() () (Input Output Analog RtcIo RtcIoOutput)), (8,
        GPIO8() (_3 => SUBSPICS1) (Input Output Analog RtcIo RtcIoOutput)), (9, GPIO9(_3
        => SUBSPIHD _4 => FSPIHD) (_3 => SUBSPIHD _4 => FSPIHD) (Input Output Analog
        RtcIo RtcIoOutput)), (10, GPIO10(_2 => FSPIIO4 _4 => FSPICS0) (_2 => FSPIIO4 _3
        => SUBSPICS0 _4 => FSPICS0) (Input Output Analog RtcIo RtcIoOutput)), (11,
        GPIO11(_2 => FSPIIO5 _3 => SUBSPID _4 => FSPID) (_2 => FSPIIO5 _3 => SUBSPID _4
        => FSPID) (Input Output Analog RtcIo RtcIoOutput)), (12, GPIO12(_2 => FSPIIO6 _4
        => FSPICLK) (_2 => FSPIIO6 _3 => SUBSPICLK _4 => FSPICLK) (Input Output Analog
        RtcIo RtcIoOutput)), (13, GPIO13(_2 => FSPIIO7 _3 => SUBSPIQ _4 => FSPIQ) (_2 =>
        FSPIIO7 _3 => SUBSPIQ _4 => FSPIQ) (Input Output Analog RtcIo RtcIoOutput)), (14,
        GPIO14(_3 => SUBSPIWP _4 => FSPIWP) (_2 => FSPIDQS _3 => SUBSPIWP _4 => FSPIWP)
        (Input Output Analog RtcIo RtcIoOutput)), (15, GPIO15() (_2 => U0RTS) (Input
        Output Analog RtcIo RtcIoOutput)), (16, GPIO16(_2 => U0CTS) () (Input Output
        Analog RtcIo RtcIoOutput)), (17, GPIO17() (_2 => U1TXD) (Input Output Analog
        RtcIo RtcIoOutput)), (18, GPIO18(_2 => U1RXD) (_3 => CLK_OUT3) (Input Output
        Analog RtcIo RtcIoOutput)), (19, GPIO19() (_2 => U1RTS _3 => CLK_OUT2) (Input
        Output Analog RtcIo RtcIoOutput UsbDm)), (20, GPIO20(_2 => U1CTS) (_3 =>
        CLK_OUT1) (Input Output Analog RtcIo RtcIoOutput UsbDp)), (21, GPIO21() () (Input
        Output Analog RtcIo RtcIoOutput)), (26, GPIO26() (_0 => SPICS1) (Input Output)),
        (27, GPIO27(_0 => SPIHD) (_0 => SPIHD) (Input Output)), (28, GPIO28(_0 => SPIWP)
        (_0 => SPIWP) (Input Output)), (29, GPIO29() (_0 => SPICS0) (Input Output)), (30,
        GPIO30() (_0 => SPICLK) (Input Output)), (31, GPIO31(_0 => SPIQ) (_0 => SPIQ)
        (Input Output)), (32, GPIO32(_0 => SPID) (_0 => SPID) (Input Output)), (33,
        GPIO33(_2 => FSPIHD _3 => SUBSPIHD) (_2 => FSPIHD _3 => SUBSPIHD) (Input
        Output)), (34, GPIO34(_2 => FSPICS0) (_2 => FSPICS0 _3 => SUBSPICS0) (Input
        Output)), (35, GPIO35(_2 => FSPID _3 => SUBSPID) (_2 => FSPID _3 => SUBSPID)
        (Input Output)), (36, GPIO36(_2 => FSPICLK) (_2 => FSPICLK _3 => SUBSPICLK)
        (Input Output)), (37, GPIO37(_2 => FSPIQ _3 => SUBSPIQ _4 => SPIDQS) (_2 => FSPIQ
        _3 => SUBSPIQ _4 => SPIDQS) (Input Output)), (38, GPIO38(_2 => FSPIWP _3 =>
        SUBSPIWP) (_2 => FSPIWP _3 => SUBSPIWP) (Input Output)), (39, GPIO39(_0 => MTCK)
        (_2 => CLK_OUT3 _3 => SUBSPICS1) (Input Output)), (40, GPIO40() (_0 => MTDO _2 =>
        CLK_OUT2) (Input Output)), (41, GPIO41(_0 => MTDI) (_2 => CLK_OUT1) (Input
        Output)), (42, GPIO42(_0 => MTMS) () (Input Output)), (43, GPIO43() (_0 => U0TXD
        _2 => CLK_OUT1) (Input Output)), (44, GPIO44(_0 => U0RXD) (_2 => CLK_OUT2) (Input
        Output)), (45, GPIO45() () (Input Output)), (46, GPIO46() () (Input Output))));
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
    (GPIO6, Analog, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO6, RtcIo, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO6, RtcIoOutput, $then_tt:tt else $else_tt:tt) => {
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
    (GPIO7, Analog, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO7, RtcIo, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO7, RtcIoOutput, $then_tt:tt else $else_tt:tt) => {
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
    (GPIO8, Analog, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO8, RtcIo, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO8, RtcIoOutput, $then_tt:tt else $else_tt:tt) => {
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
    (GPIO9, Analog, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO9, RtcIo, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO9, RtcIoOutput, $then_tt:tt else $else_tt:tt) => {
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
    (GPIO10, Analog, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO10, RtcIo, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO10, RtcIoOutput, $then_tt:tt else $else_tt:tt) => {
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
    (GPIO11, Analog, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO11, RtcIo, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO11, RtcIoOutput, $then_tt:tt else $else_tt:tt) => {
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
    (GPIO12, Analog, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO12, RtcIo, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO12, RtcIoOutput, $then_tt:tt else $else_tt:tt) => {
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
    (GPIO13, Analog, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO13, RtcIo, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO13, RtcIoOutput, $then_tt:tt else $else_tt:tt) => {
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
    (GPIO14, Analog, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO14, RtcIo, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO14, RtcIoOutput, $then_tt:tt else $else_tt:tt) => {
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
    (GPIO15, Analog, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO15, RtcIo, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO15, RtcIoOutput, $then_tt:tt else $else_tt:tt) => {
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
    (GPIO16, Analog, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO16, RtcIo, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO16, RtcIoOutput, $then_tt:tt else $else_tt:tt) => {
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
    (GPIO17, Analog, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO17, RtcIo, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO17, RtcIoOutput, $then_tt:tt else $else_tt:tt) => {
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
    (GPIO18, Analog, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO18, RtcIo, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO18, RtcIoOutput, $then_tt:tt else $else_tt:tt) => {
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
    (GPIO19, Analog, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO19, RtcIo, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO19, RtcIoOutput, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO19, UsbDm, $then_tt:tt else $else_tt:tt) => {
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
    (GPIO20, Analog, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO20, RtcIo, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO20, RtcIoOutput, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO20, UsbDp, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO20, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO21, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO21, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO21, Analog, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO21, RtcIo, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO21, RtcIoOutput, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO21, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO26, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO26, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO26, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO27, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO27, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO27, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO28, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO28, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO28, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO29, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO29, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO29, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO30, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO30, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO30, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO31, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO31, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO31, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO32, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO32, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO32, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO33, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO33, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO33, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO34, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO34, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO34, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO35, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO35, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO35, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO36, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO36, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO36, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO37, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO37, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO37, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO38, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO38, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO38, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO39, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO39, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO39, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO40, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO40, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO40, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO41, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO41, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO41, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO42, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO42, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO42, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO43, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO43, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO43, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO44, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO44, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO44, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO45, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO45, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO45, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO46, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO46, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO46, $t:tt, $then_tt:tt else $else_tt:tt) => {
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
        #[allow(unused_braces)] $code } } else { $otherwise }), 21 =>
        if_pin_is_type!(GPIO21, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO21::steal() };
        #[allow(unused_braces)] $code } } else { $otherwise }), 26 =>
        if_pin_is_type!(GPIO26, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO26::steal() };
        #[allow(unused_braces)] $code } } else { $otherwise }), 27 =>
        if_pin_is_type!(GPIO27, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO27::steal() };
        #[allow(unused_braces)] $code } } else { $otherwise }), 28 =>
        if_pin_is_type!(GPIO28, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO28::steal() };
        #[allow(unused_braces)] $code } } else { $otherwise }), 29 =>
        if_pin_is_type!(GPIO29, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO29::steal() };
        #[allow(unused_braces)] $code } } else { $otherwise }), 30 =>
        if_pin_is_type!(GPIO30, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO30::steal() };
        #[allow(unused_braces)] $code } } else { $otherwise }), 31 =>
        if_pin_is_type!(GPIO31, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO31::steal() };
        #[allow(unused_braces)] $code } } else { $otherwise }), 32 =>
        if_pin_is_type!(GPIO32, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO32::steal() };
        #[allow(unused_braces)] $code } } else { $otherwise }), 33 =>
        if_pin_is_type!(GPIO33, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO33::steal() };
        #[allow(unused_braces)] $code } } else { $otherwise }), 34 =>
        if_pin_is_type!(GPIO34, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO34::steal() };
        #[allow(unused_braces)] $code } } else { $otherwise }), 35 =>
        if_pin_is_type!(GPIO35, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO35::steal() };
        #[allow(unused_braces)] $code } } else { $otherwise }), 36 =>
        if_pin_is_type!(GPIO36, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO36::steal() };
        #[allow(unused_braces)] $code } } else { $otherwise }), 37 =>
        if_pin_is_type!(GPIO37, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO37::steal() };
        #[allow(unused_braces)] $code } } else { $otherwise }), 38 =>
        if_pin_is_type!(GPIO38, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO38::steal() };
        #[allow(unused_braces)] $code } } else { $otherwise }), 39 =>
        if_pin_is_type!(GPIO39, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO39::steal() };
        #[allow(unused_braces)] $code } } else { $otherwise }), 40 =>
        if_pin_is_type!(GPIO40, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO40::steal() };
        #[allow(unused_braces)] $code } } else { $otherwise }), 41 =>
        if_pin_is_type!(GPIO41, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO41::steal() };
        #[allow(unused_braces)] $code } } else { $otherwise }), 42 =>
        if_pin_is_type!(GPIO42, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO42::steal() };
        #[allow(unused_braces)] $code } } else { $otherwise }), 43 =>
        if_pin_is_type!(GPIO43, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO43::steal() };
        #[allow(unused_braces)] $code } } else { $otherwise }), 44 =>
        if_pin_is_type!(GPIO44, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO44::steal() };
        #[allow(unused_braces)] $code } } else { $otherwise }), 45 =>
        if_pin_is_type!(GPIO45, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO45::steal() };
        #[allow(unused_braces)] $code } } else { $otherwise }), 46 =>
        if_pin_is_type!(GPIO46, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO46::steal() };
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
        #[allow(non_camel_case_types, clippy::upper_case_acronyms)] #[derive(Debug,
        PartialEq, Copy, Clone)] #[cfg_attr(feature = "defmt", derive(defmt::Format))]
        #[doc(hidden)] pub enum InputSignal { SPIQ = 0, SPID = 1, SPIHD = 2, SPIWP = 3,
        SPID4 = 7, SPID5 = 8, SPID6 = 9, SPID7 = 10, SPIDQS = 11, U0RXD = 14, U0CTS = 15,
        U0DSR = 16, U1RXD = 17, U1CTS = 18, U1DSR = 21, I2S0O_BCK = 23, I2S0O_WS = 25,
        I2S0I_BCK = 27, I2S0I_WS = 28, I2CEXT0_SCL = 29, I2CEXT0_SDA = 30, PCNT0_SIG_CH0
        = 39, PCNT0_SIG_CH1 = 40, PCNT0_CTRL_CH0 = 41, PCNT0_CTRL_CH1 = 42, PCNT1_SIG_CH0
        = 43, PCNT1_SIG_CH1 = 44, PCNT1_CTRL_CH0 = 45, PCNT1_CTRL_CH1 = 46, PCNT2_SIG_CH0
        = 47, PCNT2_SIG_CH1 = 48, PCNT2_CTRL_CH0 = 49, PCNT2_CTRL_CH1 = 50, PCNT3_SIG_CH0
        = 51, PCNT3_SIG_CH1 = 52, PCNT3_CTRL_CH0 = 53, PCNT3_CTRL_CH1 = 54, USB_EXTPHY_VP
        = 61, USB_EXTPHY_VM = 62, USB_EXTPHY_RCV = 63, USB_OTG_IDDIG = 64, USB_OTG_AVALID
        = 65, USB_SRP_BVALID = 66, USB_OTG_VBUSVALID = 67, USB_SRP_SESSEND = 68, SPI3_CLK
        = 72, SPI3_Q = 73, SPI3_D = 74, SPI3_HD = 75, SPI3_CS0 = 76, RMT_SIG_0 = 83,
        RMT_SIG_1 = 84, RMT_SIG_2 = 85, RMT_SIG_3 = 86, I2CEXT1_SCL = 95, I2CEXT1_SDA =
        96, FSPICLK = 108, FSPIQ = 109, FSPID = 110, FSPIHD = 111, FSPIWP = 112, FSPIIO4
        = 113, FSPIIO5 = 114, FSPIIO6 = 115, FSPIIO7 = 116, FSPICS0 = 117, TWAI_RX = 123,
        SUBSPIQ = 127, SUBSPID = 128, SUBSPIHD = 129, SUBSPIWP = 130, I2S0I_DATA_IN15 =
        158, SUBSPID4 = 167, SUBSPID5 = 168, SUBSPID6 = 169, SUBSPID7 = 170, SUBSPIDQS =
        171, PCMFSYNC = 203, PCMCLK = 204, MTDI, MTCK, MTMS, }
        #[allow(non_camel_case_types, clippy::upper_case_acronyms)] #[derive(Debug,
        PartialEq, Copy, Clone)] #[cfg_attr(feature = "defmt", derive(defmt::Format))]
        #[doc(hidden)] pub enum OutputSignal { SPIQ = 0, SPID = 1, SPIHD = 2, SPIWP = 3,
        SPICLK = 4, SPICS0 = 5, SPICS1 = 6, SPID4 = 7, SPID5 = 8, SPID6 = 9, SPID7 = 10,
        SPIDQS = 11, U0TXD = 14, U0RTS = 15, U0DTR = 16, U1TXD = 17, U1RTS = 18, U1DTR =
        21, I2S0O_BCK = 23, I2S0O_WS = 25, I2S0I_BCK = 27, I2S0I_WS = 28, I2CEXT0_SCL =
        29, I2CEXT0_SDA = 30, SDIO_TOHOST_INT = 31, USB_EXTPHY_OEN = 61, USB_EXTPHY_VPO =
        63, USB_EXTPHY_VMO = 64, SPI3_CLK = 72, SPI3_Q = 73, SPI3_D = 74, SPI3_HD = 75,
        SPI3_CS0 = 76, SPI3_CS1 = 77, SPI3_CS2 = 78, LEDC_LS_SIG0 = 79, LEDC_LS_SIG1 =
        80, LEDC_LS_SIG2 = 81, LEDC_LS_SIG3 = 82, LEDC_LS_SIG4 = 83, LEDC_LS_SIG5 = 84,
        LEDC_LS_SIG6 = 85, LEDC_LS_SIG7 = 86, RMT_SIG_0 = 87, RMT_SIG_1 = 88, RMT_SIG_2 =
        89, RMT_SIG_3 = 90, I2CEXT1_SCL = 95, I2CEXT1_SDA = 96, GPIO_SD0 = 100, GPIO_SD1
        = 101, GPIO_SD2 = 102, GPIO_SD3 = 103, GPIO_SD4 = 104, GPIO_SD5 = 105, GPIO_SD6 =
        106, GPIO_SD7 = 107, FSPICLK = 108, FSPIQ = 109, FSPID = 110, FSPIHD = 111,
        FSPIWP = 112, FSPIIO4 = 113, FSPIIO5 = 114, FSPIIO6 = 115, FSPIIO7 = 116, FSPICS0
        = 117, FSPICS1 = 118, FSPICS2 = 119, FSPICS3 = 120, FSPICS4 = 121, FSPICS5 = 122,
        TWAI_TX = 123, SUBSPICLK = 126, SUBSPIQ = 127, SUBSPID = 128, SUBSPIHD = 129,
        SUBSPIWP = 130, SUBSPICS0 = 131, SUBSPICS1 = 132, FSPIDQS = 133, FSPI_HSYNC =
        134, FSPI_VSYNC = 135, FSPI_DE = 136, FSPICD = 137, SPI3_CD = 139, SPI3_DQS =
        140, I2S0O_DATA_OUT23 = 166, SUBSPID4 = 167, SUBSPID5 = 168, SUBSPID6 = 169,
        SUBSPID7 = 170, SUBSPIDQS = 171, PCMFSYNC = 209, PCMCLK = 210, CLK_I2S = 251,
        GPIO = 256, CLK_OUT1, CLK_OUT2, CLK_OUT3, MTDO, }
    };
}
#[macro_export]
#[expect(clippy::crate_in_macro_def)]
macro_rules! define_io_mux_reg {
    () => {
        pub (crate) fn io_mux_reg(gpio_num : u8) -> & 'static crate ::pac::io_mux::GPIO {
        crate ::peripherals::IO_MUX::regs().gpio(gpio_num as usize) }
    };
}
