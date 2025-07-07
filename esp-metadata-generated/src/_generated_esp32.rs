/// The name of the chip as `&str`
#[macro_export]
macro_rules! chip {
    () => {
        "esp32"
    };
}
/// The properties of this chip and its drivers.
#[macro_export]
macro_rules! property {
    ("chip") => {
        "esp32"
    };
    ("arch") => {
        "xtensa"
    };
    ("cores") => {
        2
    };
    ("cores", str) => {
        stringify!(2)
    };
    ("trm") => {
        "https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf"
    };
    ("gpio.has_bank_1") => {
        true
    };
    ("gpio.gpio_function") => {
        2
    };
    ("gpio.gpio_function", str) => {
        stringify!(2)
    };
    ("gpio.input_signal_max") => {
        206
    };
    ("gpio.input_signal_max", str) => {
        stringify!(206)
    };
    ("gpio.output_signal_max") => {
        256
    };
    ("gpio.output_signal_max", str) => {
        stringify!(256)
    };
    ("gpio.constant_0_input") => {
        48
    };
    ("gpio.constant_0_input", str) => {
        stringify!(48)
    };
    ("gpio.constant_1_input") => {
        56
    };
    ("gpio.constant_1_input", str) => {
        stringify!(56)
    };
    ("gpio.remap_iomux_pin_registers") => {
        true
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
        false
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
        false
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
        1048575
    };
    ("i2c_master.max_bus_timeout", str) => {
        stringify!(1048575)
    };
    ("i2c_master.ll_intr_mask") => {
        262143
    };
    ("i2c_master.ll_intr_mask", str) => {
        stringify!(262143)
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
        1073047552
    };
    ("rmt.ram_start", str) => {
        stringify!(1073047552)
    };
    ("rmt.channel_ram_size") => {
        64
    };
    ("rmt.channel_ram_size", str) => {
        stringify!(64)
    };
    ("spi_master.has_octal") => {
        false
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
macro_rules! memory_range {
    ("DRAM") => {
        1073405952..1073741824
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
        _for_each_inner!((UART2, Uart2, U2RXD, U2TXD, U2CTS, U2RTS));
        _for_each_inner!((all(UART0, Uart0, U0RXD, U0TXD, U0CTS, U0RTS), (UART1, Uart1,
        U1RXD, U1TXD, U1CTS, U1RTS), (UART2, Uart2, U2RXD, U2TXD, U2CTS, U2RTS)));
    };
}
#[macro_export]
macro_rules! for_each_spi_master {
    ($($pattern:tt => $code:tt;)*) => {
        macro_rules! _for_each_inner { $(($pattern) => $code;)* ($other : tt) => {} }
        _for_each_inner!((SPI2, Spi2, HSPICLK[HSPICS0, HSPICS1, HSPICS2] [HSPID, HSPIQ,
        HSPIWP, HSPIHD], true)); _for_each_inner!((SPI3, Spi3, VSPICLK[VSPICS0, VSPICS1,
        VSPICS2] [VSPID, VSPIQ, VSPIWP, VSPIHD], true)); _for_each_inner!((all(SPI2,
        Spi2, HSPICLK[HSPICS0, HSPICS1, HSPICS2] [HSPID, HSPIQ, HSPIWP, HSPIHD], true),
        (SPI3, Spi3, VSPICLK[VSPICS0, VSPICS1, VSPICS2] [VSPID, VSPIQ, VSPIWP, VSPIHD],
        true)));
    };
}
#[macro_export]
macro_rules! for_each_spi_slave {
    ($($pattern:tt => $code:tt;)*) => {
        macro_rules! _for_each_inner { $(($pattern) => $code;)* ($other : tt) => {} }
        _for_each_inner!((SPI2, Spi2, HSPICLK, HSPID, HSPIQ, HSPICS0));
        _for_each_inner!((SPI3, Spi3, VSPICLK, VSPID, VSPIQ, VSPICS0));
        _for_each_inner!((all(SPI2, Spi2, HSPICLK, HSPID, HSPIQ, HSPICS0), (SPI3, Spi3,
        VSPICLK, VSPID, VSPIQ, VSPICS0)));
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
        _for_each_inner!((GPIO22 <= virtual())); _for_each_inner!((GPIO23 <= virtual()));
        _for_each_inner!((GPIO25 <= virtual())); _for_each_inner!((GPIO26 <= virtual()));
        _for_each_inner!((GPIO27 <= virtual())); _for_each_inner!((GPIO32 <= virtual()));
        _for_each_inner!((GPIO33 <= virtual())); _for_each_inner!((GPIO34 <= virtual()));
        _for_each_inner!((GPIO35 <= virtual())); _for_each_inner!((GPIO36 <= virtual()));
        _for_each_inner!((GPIO37 <= virtual())); _for_each_inner!((GPIO38 <= virtual()));
        _for_each_inner!((GPIO39 <= virtual())); _for_each_inner!((AES <= AES()
        (unstable))); _for_each_inner!((APB_CTRL <= APB_CTRL() (unstable)));
        _for_each_inner!((BB <= BB() (unstable))); _for_each_inner!((DPORT <= DPORT()
        (unstable))); _for_each_inner!((SYSTEM <= DPORT() (unstable)));
        _for_each_inner!((EFUSE <= EFUSE() (unstable))); _for_each_inner!((EMAC_DMA <=
        EMAC_DMA() (unstable))); _for_each_inner!((EMAC_EXT <= EMAC_EXT() (unstable)));
        _for_each_inner!((EMAC_MAC <= EMAC_MAC() (unstable)));
        _for_each_inner!((FLASH_ENCRYPTION <= FLASH_ENCRYPTION() (unstable)));
        _for_each_inner!((FRC_TIMER <= FRC_TIMER() (unstable))); _for_each_inner!((GPIO
        <= GPIO() (unstable))); _for_each_inner!((GPIO_SD <= GPIO_SD() (unstable)));
        _for_each_inner!((HINF <= HINF() (unstable))); _for_each_inner!((I2C0 <=
        I2C0(I2C_EXT0 : { bind_peri_interrupt, enable_peri_interrupt,
        disable_peri_interrupt }))); _for_each_inner!((I2C1 <= I2C1(I2C_EXT1 : {
        bind_peri_interrupt, enable_peri_interrupt, disable_peri_interrupt })));
        _for_each_inner!((I2S0 <= I2S0(I2S0 : { bind_peri_interrupt,
        enable_peri_interrupt, disable_peri_interrupt }) (unstable)));
        _for_each_inner!((I2S1 <= I2S1(I2S1 : { bind_peri_interrupt,
        enable_peri_interrupt, disable_peri_interrupt }) (unstable)));
        _for_each_inner!((IO_MUX <= IO_MUX() (unstable))); _for_each_inner!((LEDC <=
        LEDC() (unstable))); _for_each_inner!((MCPWM0 <= MCPWM0() (unstable)));
        _for_each_inner!((MCPWM1 <= MCPWM1() (unstable))); _for_each_inner!((NRX <= NRX()
        (unstable))); _for_each_inner!((PCNT <= PCNT() (unstable)));
        _for_each_inner!((RMT <= RMT() (unstable))); _for_each_inner!((RNG <= RNG()
        (unstable))); _for_each_inner!((RSA <= RSA() (unstable))); _for_each_inner!((LPWR
        <= RTC_CNTL() (unstable))); _for_each_inner!((RTC_I2C <= RTC_I2C() (unstable)));
        _for_each_inner!((RTC_IO <= RTC_IO() (unstable))); _for_each_inner!((SDHOST <=
        SDHOST() (unstable))); _for_each_inner!((SENS <= SENS() (unstable)));
        _for_each_inner!((SHA <= SHA() (unstable))); _for_each_inner!((SLC <= SLC()
        (unstable))); _for_each_inner!((SLCHOST <= SLCHOST() (unstable)));
        _for_each_inner!((SPI0 <= SPI0() (unstable))); _for_each_inner!((SPI1 <= SPI1()
        (unstable))); _for_each_inner!((SPI2 <= SPI2(SPI2_DMA : { bind_dma_interrupt,
        enable_dma_interrupt, disable_dma_interrupt }, SPI2 : { bind_peri_interrupt,
        enable_peri_interrupt, disable_peri_interrupt }))); _for_each_inner!((SPI3 <=
        SPI3(SPI3_DMA : { bind_dma_interrupt, enable_dma_interrupt, disable_dma_interrupt
        }, SPI3 : { bind_peri_interrupt, enable_peri_interrupt, disable_peri_interrupt
        }))); _for_each_inner!((TIMG0 <= TIMG0() (unstable))); _for_each_inner!((TIMG1 <=
        TIMG1() (unstable))); _for_each_inner!((TWAI0 <= TWAI0() (unstable)));
        _for_each_inner!((UART0 <= UART0(UART0 : { bind_peri_interrupt,
        enable_peri_interrupt, disable_peri_interrupt }))); _for_each_inner!((UART1 <=
        UART1(UART1 : { bind_peri_interrupt, enable_peri_interrupt,
        disable_peri_interrupt }))); _for_each_inner!((UART2 <= UART2(UART2 : {
        bind_peri_interrupt, enable_peri_interrupt, disable_peri_interrupt })));
        _for_each_inner!((UHCI0 <= UHCI0() (unstable))); _for_each_inner!((UHCI1 <=
        UHCI1() (unstable))); _for_each_inner!((WIFI <= WIFI() (unstable)));
        _for_each_inner!((DMA_SPI2 <= SPI2() (unstable))); _for_each_inner!((DMA_SPI3 <=
        SPI3() (unstable))); _for_each_inner!((DMA_I2S0 <= I2S0() (unstable)));
        _for_each_inner!((DMA_I2S1 <= I2S1() (unstable))); _for_each_inner!((ADC1 <=
        virtual() (unstable))); _for_each_inner!((ADC2 <= virtual() (unstable)));
        _for_each_inner!((BT <= virtual() (unstable))); _for_each_inner!((CPU_CTRL <=
        virtual() (unstable))); _for_each_inner!((DAC1 <= virtual() (unstable)));
        _for_each_inner!((DAC2 <= virtual() (unstable))); _for_each_inner!((PSRAM <=
        virtual() (unstable))); _for_each_inner!((RADIO_CLK <= virtual() (unstable)));
        _for_each_inner!((SW_INTERRUPT <= virtual() (unstable))); _for_each_inner!((TOUCH
        <= virtual() (unstable))); _for_each_inner!((all(GPIO0 <= virtual()), (GPIO1 <=
        virtual()), (GPIO2 <= virtual()), (GPIO3 <= virtual()), (GPIO4 <= virtual()),
        (GPIO5 <= virtual()), (GPIO6 <= virtual()), (GPIO7 <= virtual()), (GPIO8 <=
        virtual()), (GPIO9 <= virtual()), (GPIO10 <= virtual()), (GPIO11 <= virtual()),
        (GPIO12 <= virtual()), (GPIO13 <= virtual()), (GPIO14 <= virtual()), (GPIO15 <=
        virtual()), (GPIO16 <= virtual()), (GPIO17 <= virtual()), (GPIO18 <= virtual()),
        (GPIO19 <= virtual()), (GPIO20 <= virtual()), (GPIO21 <= virtual()), (GPIO22 <=
        virtual()), (GPIO23 <= virtual()), (GPIO25 <= virtual()), (GPIO26 <= virtual()),
        (GPIO27 <= virtual()), (GPIO32 <= virtual()), (GPIO33 <= virtual()), (GPIO34 <=
        virtual()), (GPIO35 <= virtual()), (GPIO36 <= virtual()), (GPIO37 <= virtual()),
        (GPIO38 <= virtual()), (GPIO39 <= virtual()), (AES <= AES() (unstable)),
        (APB_CTRL <= APB_CTRL() (unstable)), (BB <= BB() (unstable)), (DPORT <= DPORT()
        (unstable)), (SYSTEM <= DPORT() (unstable)), (EFUSE <= EFUSE() (unstable)),
        (EMAC_DMA <= EMAC_DMA() (unstable)), (EMAC_EXT <= EMAC_EXT() (unstable)),
        (EMAC_MAC <= EMAC_MAC() (unstable)), (FLASH_ENCRYPTION <= FLASH_ENCRYPTION()
        (unstable)), (FRC_TIMER <= FRC_TIMER() (unstable)), (GPIO <= GPIO() (unstable)),
        (GPIO_SD <= GPIO_SD() (unstable)), (HINF <= HINF() (unstable)), (I2C0 <=
        I2C0(I2C_EXT0 : { bind_peri_interrupt, enable_peri_interrupt,
        disable_peri_interrupt })), (I2C1 <= I2C1(I2C_EXT1 : { bind_peri_interrupt,
        enable_peri_interrupt, disable_peri_interrupt })), (I2S0 <= I2S0(I2S0 : {
        bind_peri_interrupt, enable_peri_interrupt, disable_peri_interrupt })
        (unstable)), (I2S1 <= I2S1(I2S1 : { bind_peri_interrupt, enable_peri_interrupt,
        disable_peri_interrupt }) (unstable)), (IO_MUX <= IO_MUX() (unstable)), (LEDC <=
        LEDC() (unstable)), (MCPWM0 <= MCPWM0() (unstable)), (MCPWM1 <= MCPWM1()
        (unstable)), (NRX <= NRX() (unstable)), (PCNT <= PCNT() (unstable)), (RMT <=
        RMT() (unstable)), (RNG <= RNG() (unstable)), (RSA <= RSA() (unstable)), (LPWR <=
        RTC_CNTL() (unstable)), (RTC_I2C <= RTC_I2C() (unstable)), (RTC_IO <= RTC_IO()
        (unstable)), (SDHOST <= SDHOST() (unstable)), (SENS <= SENS() (unstable)), (SHA
        <= SHA() (unstable)), (SLC <= SLC() (unstable)), (SLCHOST <= SLCHOST()
        (unstable)), (SPI0 <= SPI0() (unstable)), (SPI1 <= SPI1() (unstable)), (SPI2 <=
        SPI2(SPI2_DMA : { bind_dma_interrupt, enable_dma_interrupt, disable_dma_interrupt
        }, SPI2 : { bind_peri_interrupt, enable_peri_interrupt, disable_peri_interrupt
        })), (SPI3 <= SPI3(SPI3_DMA : { bind_dma_interrupt, enable_dma_interrupt,
        disable_dma_interrupt }, SPI3 : { bind_peri_interrupt, enable_peri_interrupt,
        disable_peri_interrupt })), (TIMG0 <= TIMG0() (unstable)), (TIMG1 <= TIMG1()
        (unstable)), (TWAI0 <= TWAI0() (unstable)), (UART0 <= UART0(UART0 : {
        bind_peri_interrupt, enable_peri_interrupt, disable_peri_interrupt })), (UART1 <=
        UART1(UART1 : { bind_peri_interrupt, enable_peri_interrupt,
        disable_peri_interrupt })), (UART2 <= UART2(UART2 : { bind_peri_interrupt,
        enable_peri_interrupt, disable_peri_interrupt })), (UHCI0 <= UHCI0() (unstable)),
        (UHCI1 <= UHCI1() (unstable)), (WIFI <= WIFI() (unstable)), (DMA_SPI2 <= SPI2()
        (unstable)), (DMA_SPI3 <= SPI3() (unstable)), (DMA_I2S0 <= I2S0() (unstable)),
        (DMA_I2S1 <= I2S1() (unstable)), (ADC1 <= virtual() (unstable)), (ADC2 <=
        virtual() (unstable)), (BT <= virtual() (unstable)), (CPU_CTRL <= virtual()
        (unstable)), (DAC1 <= virtual() (unstable)), (DAC2 <= virtual() (unstable)),
        (PSRAM <= virtual() (unstable)), (RADIO_CLK <= virtual() (unstable)),
        (SW_INTERRUPT <= virtual() (unstable)), (TOUCH <= virtual() (unstable))));
    };
}
#[macro_export]
macro_rules! for_each_gpio {
    ($($pattern:tt => $code:tt;)*) => {
        macro_rules! _for_each_inner { $(($pattern) => $code;)* ($other : tt) => {} }
        _for_each_inner!((0, GPIO0(_5 => EMAC_TX_CLK) (_1 => CLK_OUT1 _5 => EMAC_TX_CLK)
        (Input Output Analog RtcIo RtcIoOutput Touch))); _for_each_inner!((1, GPIO1(_5 =>
        EMAC_RXD2) (_0 => U0TXD _1 => CLK_OUT3) (Input Output))); _for_each_inner!((2,
        GPIO2(_1 => HSPIWP _3 => HS2_DATA0 _4 => SD_DATA0) (_1 => HSPIWP _3 => HS2_DATA0
        _4 => SD_DATA0) (Input Output Analog RtcIo RtcIoOutput Touch)));
        _for_each_inner!((3, GPIO3(_0 => U0RXD) (_1 => CLK_OUT2) (Input Output)));
        _for_each_inner!((4, GPIO4(_1 => HSPIHD _3 => HS2_DATA1 _4 => SD_DATA1 _5 =>
        EMAC_TX_ER) (_1 => HSPIHD _3 => HS2_DATA1 _4 => SD_DATA1 _5 => EMAC_TX_ER) (Input
        Output Analog RtcIo RtcIoOutput Touch))); _for_each_inner!((5, GPIO5(_1 =>
        VSPICS0 _3 => HS1_DATA6 _5 => EMAC_RX_CLK) (_1 => VSPICS0 _3 => HS1_DATA6) (Input
        Output))); _for_each_inner!((6, GPIO6(_1 => SPICLK _4 => U1CTS) (_0 => SD_CLK _1
        => SPICLK _3 => HS1_CLK) (Input Output))); _for_each_inner!((7, GPIO7(_0 =>
        SD_DATA0 _1 => SPIQ _3 => HS1_DATA0) (_0 => SD_DATA0 _1 => SPIQ _3 => HS1_DATA0
        _4 => U2RTS) (Input Output))); _for_each_inner!((8, GPIO8(_0 => SD_DATA1 _1 =>
        SPID _3 => HS1_DATA1 _4 => U2CTS) (_0 => SD_DATA1 _1 => SPID _3 => HS1_DATA1)
        (Input Output))); _for_each_inner!((9, GPIO9(_0 => SD_DATA2 _1 => SPIHD _3 =>
        HS1_DATA2 _4 => U1RXD) (_0 => SD_DATA2 _1 => SPIHD _3 => HS1_DATA2) (Input
        Output))); _for_each_inner!((10, GPIO10(_0 => SD_DATA3 _1 => SPIWP _3 =>
        HS1_DATA3) (_0 => SD_DATA3 _1 => SPIWP _3 => HS1_DATA3 _4 => U1TXD) (Input
        Output))); _for_each_inner!((11, GPIO11(_0 => SD_CMD _1 => SPICS0) (_0 => SD_CMD
        _1 => SPICS0 _3 => HS1_CMD _4 => U1RTS) (Input Output))); _for_each_inner!((12,
        GPIO12(_0 => MTDI _1 => HSPIQ _3 => HS2_DATA2 _4 => SD_DATA2) (_1 => HSPIQ _3 =>
        HS2_DATA2 _4 => SD_DATA2 _5 => EMAC_TXD3) (Input Output Analog RtcIo RtcIoOutput
        Touch))); _for_each_inner!((13, GPIO13(_0 => MTCK _1 => HSPID _3 => HS2_DATA3 _4
        => SD_DATA3 _5 => EMAC_RX_ER) (_1 => HSPID _3 => HS2_DATA3 _4 => SD_DATA3 _5 =>
        EMAC_RX_ER) (Input Output Analog RtcIo RtcIoOutput Touch)));
        _for_each_inner!((14, GPIO14(_0 => MTMS _1 => HSPICLK) (_1 => HSPICLK _3 =>
        HS2_CLK _4 => SD_CLK _5 => EMAC_TXD2) (Input Output Analog RtcIo RtcIoOutput
        Touch))); _for_each_inner!((15, GPIO15(_1 => HSPICS0 _4 => SD_CMD _5 =>
        EMAC_RXD3) (_0 => MTDO _1 => HSPICS0 _3 => HS2_CMD _4 => SD_CMD) (Input Output
        Analog RtcIo RtcIoOutput Touch))); _for_each_inner!((16, GPIO16(_3 => HS1_DATA4
        _4 => U2RXD) (_3 => HS1_DATA4 _5 => EMAC_CLK_OUT) (Input Output)));
        _for_each_inner!((17, GPIO17(_3 => HS1_DATA5) (_3 => HS1_DATA5 _4 => U2TXD _5 =>
        EMAC_CLK_180) (Input Output))); _for_each_inner!((18, GPIO18(_1 => VSPICLK _3 =>
        HS1_DATA7) (_1 => VSPICLK _3 => HS1_DATA7) (Input Output)));
        _for_each_inner!((19, GPIO19(_1 => VSPIQ _3 => U0CTS) (_1 => VSPIQ _5 =>
        EMAC_TXD0) (Input Output))); _for_each_inner!((20, GPIO20() () (Input Output)));
        _for_each_inner!((21, GPIO21(_1 => VSPIHD) (_1 => VSPIHD _5 => EMAC_TX_EN) (Input
        Output))); _for_each_inner!((22, GPIO22(_1 => VSPIWP) (_1 => VSPIWP _3 => U0RTS
        _5 => EMAC_TXD1) (Input Output))); _for_each_inner!((23, GPIO23(_1 => VSPID) (_1
        => VSPID _3 => HS1_STROBE) (Input Output))); _for_each_inner!((25, GPIO25(_5 =>
        EMAC_RXD0) () (Input Output Analog RtcIo RtcIoOutput))); _for_each_inner!((26,
        GPIO26(_5 => EMAC_RXD1) () (Input Output Analog RtcIo RtcIoOutput)));
        _for_each_inner!((27, GPIO27(_5 => EMAC_RX_DV) () (Input Output Analog RtcIo
        RtcIoOutput Touch))); _for_each_inner!((32, GPIO32() () (Input Output Analog
        RtcIo RtcIoOutput Touch))); _for_each_inner!((33, GPIO33() () (Input Output
        Analog RtcIo RtcIoOutput Touch))); _for_each_inner!((34, GPIO34() () (Input
        Analog RtcIo))); _for_each_inner!((35, GPIO35() () (Input Analog RtcIo)));
        _for_each_inner!((36, GPIO36() () (Input Analog RtcIo))); _for_each_inner!((37,
        GPIO37() () (Input Analog RtcIo))); _for_each_inner!((38, GPIO38() () (Input
        Analog RtcIo))); _for_each_inner!((39, GPIO39() () (Input Analog RtcIo)));
        _for_each_inner!((all(0, GPIO0(_5 => EMAC_TX_CLK) (_1 => CLK_OUT1 _5 =>
        EMAC_TX_CLK) (Input Output Analog RtcIo RtcIoOutput Touch)), (1, GPIO1(_5 =>
        EMAC_RXD2) (_0 => U0TXD _1 => CLK_OUT3) (Input Output)), (2, GPIO2(_1 => HSPIWP
        _3 => HS2_DATA0 _4 => SD_DATA0) (_1 => HSPIWP _3 => HS2_DATA0 _4 => SD_DATA0)
        (Input Output Analog RtcIo RtcIoOutput Touch)), (3, GPIO3(_0 => U0RXD) (_1 =>
        CLK_OUT2) (Input Output)), (4, GPIO4(_1 => HSPIHD _3 => HS2_DATA1 _4 => SD_DATA1
        _5 => EMAC_TX_ER) (_1 => HSPIHD _3 => HS2_DATA1 _4 => SD_DATA1 _5 => EMAC_TX_ER)
        (Input Output Analog RtcIo RtcIoOutput Touch)), (5, GPIO5(_1 => VSPICS0 _3 =>
        HS1_DATA6 _5 => EMAC_RX_CLK) (_1 => VSPICS0 _3 => HS1_DATA6) (Input Output)), (6,
        GPIO6(_1 => SPICLK _4 => U1CTS) (_0 => SD_CLK _1 => SPICLK _3 => HS1_CLK) (Input
        Output)), (7, GPIO7(_0 => SD_DATA0 _1 => SPIQ _3 => HS1_DATA0) (_0 => SD_DATA0 _1
        => SPIQ _3 => HS1_DATA0 _4 => U2RTS) (Input Output)), (8, GPIO8(_0 => SD_DATA1 _1
        => SPID _3 => HS1_DATA1 _4 => U2CTS) (_0 => SD_DATA1 _1 => SPID _3 => HS1_DATA1)
        (Input Output)), (9, GPIO9(_0 => SD_DATA2 _1 => SPIHD _3 => HS1_DATA2 _4 =>
        U1RXD) (_0 => SD_DATA2 _1 => SPIHD _3 => HS1_DATA2) (Input Output)), (10,
        GPIO10(_0 => SD_DATA3 _1 => SPIWP _3 => HS1_DATA3) (_0 => SD_DATA3 _1 => SPIWP _3
        => HS1_DATA3 _4 => U1TXD) (Input Output)), (11, GPIO11(_0 => SD_CMD _1 => SPICS0)
        (_0 => SD_CMD _1 => SPICS0 _3 => HS1_CMD _4 => U1RTS) (Input Output)), (12,
        GPIO12(_0 => MTDI _1 => HSPIQ _3 => HS2_DATA2 _4 => SD_DATA2) (_1 => HSPIQ _3 =>
        HS2_DATA2 _4 => SD_DATA2 _5 => EMAC_TXD3) (Input Output Analog RtcIo RtcIoOutput
        Touch)), (13, GPIO13(_0 => MTCK _1 => HSPID _3 => HS2_DATA3 _4 => SD_DATA3 _5 =>
        EMAC_RX_ER) (_1 => HSPID _3 => HS2_DATA3 _4 => SD_DATA3 _5 => EMAC_RX_ER) (Input
        Output Analog RtcIo RtcIoOutput Touch)), (14, GPIO14(_0 => MTMS _1 => HSPICLK)
        (_1 => HSPICLK _3 => HS2_CLK _4 => SD_CLK _5 => EMAC_TXD2) (Input Output Analog
        RtcIo RtcIoOutput Touch)), (15, GPIO15(_1 => HSPICS0 _4 => SD_CMD _5 =>
        EMAC_RXD3) (_0 => MTDO _1 => HSPICS0 _3 => HS2_CMD _4 => SD_CMD) (Input Output
        Analog RtcIo RtcIoOutput Touch)), (16, GPIO16(_3 => HS1_DATA4 _4 => U2RXD) (_3 =>
        HS1_DATA4 _5 => EMAC_CLK_OUT) (Input Output)), (17, GPIO17(_3 => HS1_DATA5) (_3
        => HS1_DATA5 _4 => U2TXD _5 => EMAC_CLK_180) (Input Output)), (18, GPIO18(_1 =>
        VSPICLK _3 => HS1_DATA7) (_1 => VSPICLK _3 => HS1_DATA7) (Input Output)), (19,
        GPIO19(_1 => VSPIQ _3 => U0CTS) (_1 => VSPIQ _5 => EMAC_TXD0) (Input Output)),
        (20, GPIO20() () (Input Output)), (21, GPIO21(_1 => VSPIHD) (_1 => VSPIHD _5 =>
        EMAC_TX_EN) (Input Output)), (22, GPIO22(_1 => VSPIWP) (_1 => VSPIWP _3 => U0RTS
        _5 => EMAC_TXD1) (Input Output)), (23, GPIO23(_1 => VSPID) (_1 => VSPID _3 =>
        HS1_STROBE) (Input Output)), (25, GPIO25(_5 => EMAC_RXD0) () (Input Output Analog
        RtcIo RtcIoOutput)), (26, GPIO26(_5 => EMAC_RXD1) () (Input Output Analog RtcIo
        RtcIoOutput)), (27, GPIO27(_5 => EMAC_RX_DV) () (Input Output Analog RtcIo
        RtcIoOutput Touch)), (32, GPIO32() () (Input Output Analog RtcIo RtcIoOutput
        Touch)), (33, GPIO33() () (Input Output Analog RtcIo RtcIoOutput Touch)), (34,
        GPIO34() () (Input Analog RtcIo)), (35, GPIO35() () (Input Analog RtcIo)), (36,
        GPIO36() () (Input Analog RtcIo)), (37, GPIO37() () (Input Analog RtcIo)), (38,
        GPIO38() () (Input Analog RtcIo)), (39, GPIO39() () (Input Analog RtcIo))));
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
    (GPIO0, Touch, $then_tt:tt else $else_tt:tt) => {
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
    (GPIO2, Touch, $then_tt:tt else $else_tt:tt) => {
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
    (GPIO4, Touch, $then_tt:tt else $else_tt:tt) => {
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
    (GPIO12, Analog, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO12, RtcIo, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO12, RtcIoOutput, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO12, Touch, $then_tt:tt else $else_tt:tt) => {
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
    (GPIO13, Touch, $then_tt:tt else $else_tt:tt) => {
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
    (GPIO14, Touch, $then_tt:tt else $else_tt:tt) => {
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
    (GPIO15, Touch, $then_tt:tt else $else_tt:tt) => {
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
    (GPIO21, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO21, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO21, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO22, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO22, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO22, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO23, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO23, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO23, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO25, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO25, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO25, Analog, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO25, RtcIo, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO25, RtcIoOutput, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO25, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO26, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO26, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO26, Analog, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO26, RtcIo, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO26, RtcIoOutput, $then_tt:tt else $else_tt:tt) => {
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
    (GPIO27, Analog, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO27, RtcIo, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO27, RtcIoOutput, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO27, Touch, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO27, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO32, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO32, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO32, Analog, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO32, RtcIo, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO32, RtcIoOutput, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO32, Touch, $then_tt:tt else $else_tt:tt) => {
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
    (GPIO33, Analog, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO33, RtcIo, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO33, RtcIoOutput, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO33, Touch, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO33, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO34, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO34, Analog, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO34, RtcIo, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO34, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO35, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO35, Analog, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO35, RtcIo, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO35, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO36, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO36, Analog, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO36, RtcIo, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO36, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO37, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO37, Analog, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO37, RtcIo, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO37, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO38, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO38, Analog, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO38, RtcIo, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO38, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO39, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO39, Analog, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO39, RtcIo, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO39, $t:tt, $then_tt:tt else $else_tt:tt) => {
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
        #[allow(unused_braces)] $code } } else { $otherwise }), 22 =>
        if_pin_is_type!(GPIO22, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO22::steal() };
        #[allow(unused_braces)] $code } } else { $otherwise }), 23 =>
        if_pin_is_type!(GPIO23, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO23::steal() };
        #[allow(unused_braces)] $code } } else { $otherwise }), 25 =>
        if_pin_is_type!(GPIO25, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO25::steal() };
        #[allow(unused_braces)] $code } } else { $otherwise }), 26 =>
        if_pin_is_type!(GPIO26, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO26::steal() };
        #[allow(unused_braces)] $code } } else { $otherwise }), 27 =>
        if_pin_is_type!(GPIO27, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO27::steal() };
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
            SPICLK                = 0,
            SPIQ                  = 1,
            SPID                  = 2,
            SPIHD                 = 3,
            SPIWP                 = 4,
            SPICS0                = 5,
            SPICS1                = 6,
            SPICS2                = 7,
            HSPICLK               = 8,
            HSPIQ                 = 9,
            HSPID                 = 10,
            HSPICS0               = 11,
            HSPIHD                = 12,
            HSPIWP                = 13,
            U0RXD                 = 14,
            U0CTS                 = 15,
            U0DSR                 = 16,
            U1RXD                 = 17,
            U1CTS                 = 18,
            I2S0O_BCK             = 23,
            I2S1O_BCK             = 24,
            I2S0O_WS              = 25,
            I2S1O_WS              = 26,
            I2S0I_BCK             = 27,
            I2S0I_WS              = 28,
            I2CEXT0_SCL           = 29,
            I2CEXT0_SDA           = 30,
            PWM0_SYNC0            = 31,
            PWM0_SYNC1            = 32,
            PWM0_SYNC2            = 33,
            PWM0_F0               = 34,
            PWM0_F1               = 35,
            PWM0_F2               = 36,
            PCNT0_SIG_CH0         = 39,
            PCNT0_SIG_CH1         = 40,
            PCNT0_CTRL_CH0        = 41,
            PCNT0_CTRL_CH1        = 42,
            PCNT1_SIG_CH0         = 43,
            PCNT1_SIG_CH1         = 44,
            PCNT1_CTRL_CH0        = 45,
            PCNT1_CTRL_CH1        = 46,
            PCNT2_SIG_CH0         = 47,
            PCNT2_SIG_CH1         = 48,
            PCNT2_CTRL_CH0        = 49,
            PCNT2_CTRL_CH1        = 50,
            PCNT3_SIG_CH0         = 51,
            PCNT3_SIG_CH1         = 52,
            PCNT3_CTRL_CH0        = 53,
            PCNT3_CTRL_CH1        = 54,
            PCNT4_SIG_CH0         = 55,
            PCNT4_SIG_CH1         = 56,
            PCNT4_CTRL_CH0        = 57,
            PCNT4_CTRL_CH1        = 58,
            HSPICS1               = 61,
            HSPICS2               = 62,
            VSPICLK               = 63,
            VSPIQ                 = 64,
            VSPID                 = 65,
            VSPIHD                = 66,
            VSPIWP                = 67,
            VSPICS0               = 68,
            VSPICS1               = 69,
            VSPICS2               = 70,
            PCNT5_SIG_CH0         = 71,
            PCNT5_SIG_CH1         = 72,
            PCNT5_CTRL_CH0        = 73,
            PCNT5_CTRL_CH1        = 74,
            PCNT6_SIG_CH0         = 75,
            PCNT6_SIG_CH1         = 76,
            PCNT6_CTRL_CH0        = 77,
            PCNT6_CTRL_CH1        = 78,
            PCNT7_SIG_CH0         = 79,
            PCNT7_SIG_CH1         = 80,
            PCNT7_CTRL_CH0        = 81,
            PCNT7_CTRL_CH1        = 82,
            RMT_SIG_0             = 83,
            RMT_SIG_1             = 84,
            RMT_SIG_2             = 85,
            RMT_SIG_3             = 86,
            RMT_SIG_4             = 87,
            RMT_SIG_5             = 88,
            RMT_SIG_6             = 89,
            RMT_SIG_7             = 90,
            TWAI_RX               = 94,
            I2CEXT1_SCL           = 95,
            I2CEXT1_SDA           = 96,
            HOST_CARD_DETECT_N_1  = 97,
            HOST_CARD_DETECT_N_2  = 98,
            HOST_CARD_WRITE_PRT_1 = 99,
            HOST_CARD_WRITE_PRT_2 = 100,
            HOST_CARD_INT_N_1     = 101,
            HOST_CARD_INT_N_2     = 102,
            PWM1_SYNC0            = 103,
            PWM1_SYNC1            = 104,
            PWM1_SYNC2            = 105,
            PWM1_F0               = 106,
            PWM1_F1               = 107,
            PWM1_F2               = 108,
            PWM0_CAP0             = 109,
            PWM0_CAP1             = 110,
            PWM0_CAP2             = 111,
            PWM1_CAP0             = 112,
            PWM1_CAP1             = 113,
            PWM1_CAP2             = 114,
            I2S0I_DATA_0          = 140,
            I2S0I_DATA_1          = 141,
            I2S0I_DATA_2          = 142,
            I2S0I_DATA_3          = 143,
            I2S0I_DATA_4          = 144,
            I2S0I_DATA_5          = 145,
            I2S0I_DATA_6          = 146,
            I2S0I_DATA_7          = 147,
            I2S0I_DATA_8          = 148,
            I2S0I_DATA_9          = 149,
            I2S0I_DATA_10         = 150,
            I2S0I_DATA_11         = 151,
            I2S0I_DATA_12         = 152,
            I2S0I_DATA_13         = 153,
            I2S0I_DATA_14         = 154,
            I2S0I_DATA_15         = 155,
            I2S1I_BCK             = 164,
            I2S1I_WS              = 165,
            I2S1I_DATA_0          = 166,
            I2S1I_DATA_1          = 167,
            I2S1I_DATA_2          = 168,
            I2S1I_DATA_3          = 169,
            I2S1I_DATA_4          = 170,
            I2S1I_DATA_5          = 171,
            I2S1I_DATA_6          = 172,
            I2S1I_DATA_7          = 173,
            I2S1I_DATA_8          = 174,
            I2S1I_DATA_9          = 175,
            I2S1I_DATA_10         = 176,
            I2S1I_DATA_11         = 177,
            I2S1I_DATA_12         = 178,
            I2S1I_DATA_13         = 179,
            I2S1I_DATA_14         = 180,
            I2S1I_DATA_15         = 181,
            I2S0I_H_SYNC          = 190,
            I2S0I_V_SYNC          = 191,
            I2S0I_H_ENABLE        = 192,
            I2S1I_H_SYNC          = 193,
            I2S1I_V_SYNC          = 194,
            I2S1I_H_ENABLE        = 195,
            U2RXD                 = 198,
            U2CTS                 = 199,
            EMAC_MDC              = 200,
            EMAC_MDI              = 201,
            EMAC_CRS              = 202,
            EMAC_COL              = 203,
            PCMFSYNC              = 204,
            PCMCLK                = 205,
            PCMDIN                = 206,
            SD_CMD,
            SD_DATA0,
            SD_DATA1,
            SD_DATA2,
            SD_DATA3,
            HS1_DATA0,
            HS1_DATA1,
            HS1_DATA2,
            HS1_DATA3,
            HS1_DATA4,
            HS1_DATA5,
            HS1_DATA6,
            HS1_DATA7,
            HS2_DATA0,
            HS2_DATA1,
            HS2_DATA2,
            HS2_DATA3,
            EMAC_TX_CLK,
            EMAC_RXD2,
            EMAC_TX_ER,
            EMAC_RX_CLK,
            EMAC_RX_ER,
            EMAC_RXD3,
            EMAC_RXD0,
            EMAC_RXD1,
            EMAC_RX_DV,
            MTDI,
            MTCK,
            MTMS,
        }
        #[allow(non_camel_case_types, clippy::upper_case_acronyms)]
        #[derive(Debug, PartialEq, Copy, Clone)]
        #[cfg_attr(feature = "defmt", derive(defmt::Format))]
        #[doc(hidden)]
        pub enum OutputSignal {
            SPICLK                   = 0,
            SPIQ                     = 1,
            SPID                     = 2,
            SPIHD                    = 3,
            SPIWP                    = 4,
            SPICS0                   = 5,
            SPICS1                   = 6,
            SPICS2                   = 7,
            HSPICLK                  = 8,
            HSPIQ                    = 9,
            HSPID                    = 10,
            HSPICS0                  = 11,
            HSPIHD                   = 12,
            HSPIWP                   = 13,
            U0TXD                    = 14,
            U0RTS                    = 15,
            U0DTR                    = 16,
            U1TXD                    = 17,
            U1RTS                    = 18,
            I2S0O_BCK                = 23,
            I2S1O_BCK                = 24,
            I2S0O_WS                 = 25,
            I2S1O_WS                 = 26,
            I2S0I_BCK                = 27,
            I2S0I_WS                 = 28,
            I2CEXT0_SCL              = 29,
            I2CEXT0_SDA              = 30,
            SDIO_TOHOSTT             = 31,
            PWM0_0A                  = 32,
            PWM0_0B                  = 33,
            PWM0_1A                  = 34,
            PWM0_1B                  = 35,
            PWM0_2A                  = 36,
            PWM0_2B                  = 37,
            HSPICS1                  = 61,
            HSPICS2                  = 62,
            VSPICLK                  = 63,
            VSPIQ                    = 64,
            VSPID                    = 65,
            VSPIHD                   = 66,
            VSPIWP                   = 67,
            VSPICS0                  = 68,
            VSPICS1                  = 69,
            VSPICS2                  = 70,
            LEDC_HS_SIG0             = 71,
            LEDC_HS_SIG1             = 72,
            LEDC_HS_SIG2             = 73,
            LEDC_HS_SIG3             = 74,
            LEDC_HS_SIG4             = 75,
            LEDC_HS_SIG5             = 76,
            LEDC_HS_SIG6             = 77,
            LEDC_HS_SIG7             = 78,
            LEDC_LS_SIG0             = 79,
            LEDC_LS_SIG1             = 80,
            LEDC_LS_SIG2             = 81,
            LEDC_LS_SIG3             = 82,
            LEDC_LS_SIG4             = 83,
            LEDC_LS_SIG5             = 84,
            LEDC_LS_SIG6             = 85,
            LEDC_LS_SIG7             = 86,
            RMT_SIG_0                = 87,
            RMT_SIG_1                = 88,
            RMT_SIG_2                = 89,
            RMT_SIG_3                = 90,
            RMT_SIG_4                = 91,
            RMT_SIG_5                = 92,
            RMT_SIG_6                = 93,
            RMT_SIG_7                = 94,
            I2CEXT1_SCL              = 95,
            I2CEXT1_SDA              = 96,
            HOST_CCMD_OD_PULLUP_EN_N = 97,
            HOST_RST_N_1             = 98,
            HOST_RST_N_2             = 99,
            GPIO_SD0                 = 100,
            GPIO_SD1                 = 101,
            GPIO_SD2                 = 102,
            GPIO_SD3                 = 103,
            GPIO_SD4                 = 104,
            GPIO_SD5                 = 105,
            GPIO_SD6                 = 106,
            GPIO_SD7                 = 107,
            PWM1_0A                  = 108,
            PWM1_0B                  = 109,
            PWM1_1A                  = 110,
            PWM1_1B                  = 111,
            PWM1_2A                  = 112,
            PWM1_2B                  = 113,
            TWAI_TX                  = 123,
            TWAI_BUS_OFF_ON          = 124,
            TWAI_CLKOUT              = 125,
            I2S0O_DATA_0             = 140,
            I2S0O_DATA_1             = 141,
            I2S0O_DATA_2             = 142,
            I2S0O_DATA_3             = 143,
            I2S0O_DATA_4             = 144,
            I2S0O_DATA_5             = 145,
            I2S0O_DATA_6             = 146,
            I2S0O_DATA_7             = 147,
            I2S0O_DATA_8             = 148,
            I2S0O_DATA_9             = 149,
            I2S0O_DATA_10            = 150,
            I2S0O_DATA_11            = 151,
            I2S0O_DATA_12            = 152,
            I2S0O_DATA_13            = 153,
            I2S0O_DATA_14            = 154,
            I2S0O_DATA_15            = 155,
            I2S0O_DATA_16            = 156,
            I2S0O_DATA_17            = 157,
            I2S0O_DATA_18            = 158,
            I2S0O_DATA_19            = 159,
            I2S0O_DATA_20            = 160,
            I2S0O_DATA_21            = 161,
            I2S0O_DATA_22            = 162,
            I2S0O_DATA_23            = 163,
            I2S1I_BCK                = 164,
            I2S1I_WS                 = 165,
            I2S1O_DATA_0             = 166,
            I2S1O_DATA_1             = 167,
            I2S1O_DATA_2             = 168,
            I2S1O_DATA_3             = 169,
            I2S1O_DATA_4             = 170,
            I2S1O_DATA_5             = 171,
            I2S1O_DATA_6             = 172,
            I2S1O_DATA_7             = 173,
            I2S1O_DATA_8             = 174,
            I2S1O_DATA_9             = 175,
            I2S1O_DATA_10            = 176,
            I2S1O_DATA_11            = 177,
            I2S1O_DATA_12            = 178,
            I2S1O_DATA_13            = 179,
            I2S1O_DATA_14            = 180,
            I2S1O_DATA_15            = 181,
            I2S1O_DATA_16            = 182,
            I2S1O_DATA_17            = 183,
            I2S1O_DATA_18            = 184,
            I2S1O_DATA_19            = 185,
            I2S1O_DATA_20            = 186,
            I2S1O_DATA_21            = 187,
            I2S1O_DATA_22            = 188,
            I2S1O_DATA_23            = 189,
            U2TXD                    = 198,
            U2RTS                    = 199,
            EMAC_MDC                 = 200,
            EMAC_MDO                 = 201,
            EMAC_CRS                 = 202,
            EMAC_COL                 = 203,
            BT_AUDIO0RQ              = 204,
            BT_AUDIO1RQ              = 205,
            BT_AUDIO2RQ              = 206,
            BLE_AUDIO0RQ             = 207,
            BLE_AUDIO1RQ             = 208,
            BLE_AUDIO2RQ             = 209,
            PCMFSYNC                 = 210,
            PCMCLK                   = 211,
            PCMDOUT                  = 212,
            BLE_AUDIO_SYNC0_P        = 213,
            BLE_AUDIO_SYNC1_P        = 214,
            BLE_AUDIO_SYNC2_P        = 215,
            ANT_SEL0                 = 216,
            ANT_SEL1                 = 217,
            ANT_SEL2                 = 218,
            ANT_SEL3                 = 219,
            ANT_SEL4                 = 220,
            ANT_SEL5                 = 221,
            ANT_SEL6                 = 222,
            ANT_SEL7                 = 223,
            SIGNAL_224               = 224,
            SIGNAL_225               = 225,
            SIGNAL_226               = 226,
            SIGNAL_227               = 227,
            SIGNAL_228               = 228,
            GPIO                     = 256,
            CLK_OUT1,
            CLK_OUT2,
            CLK_OUT3,
            SD_CLK,
            SD_CMD,
            SD_DATA0,
            SD_DATA1,
            SD_DATA2,
            SD_DATA3,
            HS1_CLK,
            HS1_CMD,
            HS1_DATA0,
            HS1_DATA1,
            HS1_DATA2,
            HS1_DATA3,
            HS1_DATA4,
            HS1_DATA5,
            HS1_DATA6,
            HS1_DATA7,
            HS1_STROBE,
            HS2_CLK,
            HS2_CMD,
            HS2_DATA0,
            HS2_DATA1,
            HS2_DATA2,
            HS2_DATA3,
            EMAC_TX_CLK,
            EMAC_TX_ER,
            EMAC_TXD3,
            EMAC_RX_ER,
            EMAC_TXD2,
            EMAC_CLK_OUT,
            EMAC_CLK_180,
            EMAC_TXD0,
            EMAC_TX_EN,
            EMAC_TXD1,
            MTDO,
        }
    };
}
#[macro_export]
#[expect(clippy::crate_in_macro_def)]
macro_rules! define_io_mux_reg {
    () => {
        pub(crate) fn io_mux_reg(gpio_num: u8) -> &'static crate::pac::io_mux::GPIO0 {
            use core::mem::transmute;

            use crate::{pac::io_mux, peripherals::IO_MUX};
            let iomux = IO_MUX::regs();
            unsafe {
                match gpio_num {
                    0 => transmute::<&'static io_mux::GPIO0, &'static io_mux::GPIO0>(iomux.gpio0()),
                    1 => transmute::<&'static io_mux::GPIO1, &'static io_mux::GPIO0>(iomux.gpio1()),
                    2 => transmute::<&'static io_mux::GPIO2, &'static io_mux::GPIO0>(iomux.gpio2()),
                    3 => transmute::<&'static io_mux::GPIO3, &'static io_mux::GPIO0>(iomux.gpio3()),
                    4 => transmute::<&'static io_mux::GPIO4, &'static io_mux::GPIO0>(iomux.gpio4()),
                    5 => transmute::<&'static io_mux::GPIO5, &'static io_mux::GPIO0>(iomux.gpio5()),
                    6 => transmute::<&'static io_mux::GPIO6, &'static io_mux::GPIO0>(iomux.gpio6()),
                    7 => transmute::<&'static io_mux::GPIO7, &'static io_mux::GPIO0>(iomux.gpio7()),
                    8 => transmute::<&'static io_mux::GPIO8, &'static io_mux::GPIO0>(iomux.gpio8()),
                    9 => transmute::<&'static io_mux::GPIO9, &'static io_mux::GPIO0>(iomux.gpio9()),
                    10 => {
                        transmute::<&'static io_mux::GPIO10, &'static io_mux::GPIO0>(iomux.gpio10())
                    }
                    11 => {
                        transmute::<&'static io_mux::GPIO11, &'static io_mux::GPIO0>(iomux.gpio11())
                    }
                    12 => {
                        transmute::<&'static io_mux::GPIO12, &'static io_mux::GPIO0>(iomux.gpio12())
                    }
                    13 => {
                        transmute::<&'static io_mux::GPIO13, &'static io_mux::GPIO0>(iomux.gpio13())
                    }
                    14 => {
                        transmute::<&'static io_mux::GPIO14, &'static io_mux::GPIO0>(iomux.gpio14())
                    }
                    15 => {
                        transmute::<&'static io_mux::GPIO15, &'static io_mux::GPIO0>(iomux.gpio15())
                    }
                    16 => {
                        transmute::<&'static io_mux::GPIO16, &'static io_mux::GPIO0>(iomux.gpio16())
                    }
                    17 => {
                        transmute::<&'static io_mux::GPIO17, &'static io_mux::GPIO0>(iomux.gpio17())
                    }
                    18 => {
                        transmute::<&'static io_mux::GPIO18, &'static io_mux::GPIO0>(iomux.gpio18())
                    }
                    19 => {
                        transmute::<&'static io_mux::GPIO19, &'static io_mux::GPIO0>(iomux.gpio19())
                    }
                    20 => {
                        transmute::<&'static io_mux::GPIO20, &'static io_mux::GPIO0>(iomux.gpio20())
                    }
                    21 => {
                        transmute::<&'static io_mux::GPIO21, &'static io_mux::GPIO0>(iomux.gpio21())
                    }
                    22 => {
                        transmute::<&'static io_mux::GPIO22, &'static io_mux::GPIO0>(iomux.gpio22())
                    }
                    23 => {
                        transmute::<&'static io_mux::GPIO23, &'static io_mux::GPIO0>(iomux.gpio23())
                    }
                    25 => {
                        transmute::<&'static io_mux::GPIO25, &'static io_mux::GPIO0>(iomux.gpio25())
                    }
                    26 => {
                        transmute::<&'static io_mux::GPIO26, &'static io_mux::GPIO0>(iomux.gpio26())
                    }
                    27 => {
                        transmute::<&'static io_mux::GPIO27, &'static io_mux::GPIO0>(iomux.gpio27())
                    }
                    32 => {
                        transmute::<&'static io_mux::GPIO32, &'static io_mux::GPIO0>(iomux.gpio32())
                    }
                    33 => {
                        transmute::<&'static io_mux::GPIO33, &'static io_mux::GPIO0>(iomux.gpio33())
                    }
                    34 => {
                        transmute::<&'static io_mux::GPIO34, &'static io_mux::GPIO0>(iomux.gpio34())
                    }
                    35 => {
                        transmute::<&'static io_mux::GPIO35, &'static io_mux::GPIO0>(iomux.gpio35())
                    }
                    36 => {
                        transmute::<&'static io_mux::GPIO36, &'static io_mux::GPIO0>(iomux.gpio36())
                    }
                    37 => {
                        transmute::<&'static io_mux::GPIO37, &'static io_mux::GPIO0>(iomux.gpio37())
                    }
                    38 => {
                        transmute::<&'static io_mux::GPIO38, &'static io_mux::GPIO0>(iomux.gpio38())
                    }
                    39 => {
                        transmute::<&'static io_mux::GPIO39, &'static io_mux::GPIO0>(iomux.gpio39())
                    }
                    other => panic!("GPIO {} does not exist", other),
                }
            }
        }
    };
}
