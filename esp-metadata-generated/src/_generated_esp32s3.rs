/// The name of the chip as `&str`
#[macro_export]
macro_rules! chip {
    () => {
        "esp32s3"
    };
}
/// The properties of this chip and its drivers.
#[macro_export]
macro_rules! property {
    ("chip") => {
        "esp32s3"
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
        "https://www.espressif.com/sites/default/files/documentation/esp32-s3_technical_reference_manual_en.pdf"
    };
    ("assist_debug.has_sp_monitor") => {
        false
    };
    ("assist_debug.has_region_monitor") => {
        true
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
    ("gpio.input_signal_max") => {
        255
    };
    ("gpio.input_signal_max", str) => {
        stringify!(255)
    };
    ("gpio.output_signal_max") => {
        256
    };
    ("gpio.output_signal_max", str) => {
        stringify!(256)
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
        true
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
        32
    };
    ("i2c_master.fifo_size", str) => {
        stringify!(32)
    };
    ("interrupts.status_registers") => {
        4
    };
    ("interrupts.status_registers", str) => {
        stringify!(4)
    };
    ("rmt.ram_start") => {
        1610704896
    };
    ("rmt.ram_start", str) => {
        stringify!(1610704896)
    };
    ("rmt.channel_ram_size") => {
        48
    };
    ("rmt.channel_ram_size", str) => {
        stringify!(48)
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
macro_rules! memory_range {
    ("DRAM") => {
        1070104576..1070596096
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
        _for_each_inner!((SPI2, Spi2, FSPICLK[FSPICS0, FSPICS1, FSPICS2, FSPICS3,
        FSPICS4, FSPICS5] [FSPID, FSPIQ, FSPIWP, FSPIHD, FSPIIO4, FSPIIO5, FSPIIO6,
        FSPIIO7], true)); _for_each_inner!((SPI3, Spi3, SPI3_CLK[SPI3_CS0, SPI3_CS1,
        SPI3_CS2] [SPI3_D, SPI3_Q, SPI3_WP, SPI3_HD], true)); _for_each_inner!((all(SPI2,
        Spi2, FSPICLK[FSPICS0, FSPICS1, FSPICS2, FSPICS3, FSPICS4, FSPICS5] [FSPID,
        FSPIQ, FSPIWP, FSPIHD, FSPIIO4, FSPIIO5, FSPIIO6, FSPIIO7], true), (SPI3, Spi3,
        SPI3_CLK[SPI3_CS0, SPI3_CS1, SPI3_CS2] [SPI3_D, SPI3_Q, SPI3_WP, SPI3_HD],
        true)));
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
        _for_each_inner!((GPIO46 <= virtual())); _for_each_inner!((GPIO47 <= virtual()));
        _for_each_inner!((GPIO48 <= virtual())); _for_each_inner!((AES <= AES()
        (unstable))); _for_each_inner!((APB_CTRL <= APB_CTRL() (unstable)));
        _for_each_inner!((APB_SARADC <= APB_SARADC() (unstable)));
        _for_each_inner!((ASSIST_DEBUG <= ASSIST_DEBUG() (unstable)));
        _for_each_inner!((DMA <= DMA() (unstable))); _for_each_inner!((DS <= DS()
        (unstable))); _for_each_inner!((EFUSE <= EFUSE() (unstable)));
        _for_each_inner!((EXTMEM <= EXTMEM() (unstable))); _for_each_inner!((GPIO <=
        GPIO() (unstable))); _for_each_inner!((GPIO_SD <= GPIO_SD() (unstable)));
        _for_each_inner!((HMAC <= HMAC() (unstable))); _for_each_inner!((I2C0 <=
        I2C0(I2C_EXT0 : { bind_peri_interrupt, enable_peri_interrupt,
        disable_peri_interrupt }))); _for_each_inner!((I2C1 <= I2C1(I2C_EXT1 : {
        bind_peri_interrupt, enable_peri_interrupt, disable_peri_interrupt })));
        _for_each_inner!((I2S0 <= I2S0(I2S0 : { bind_peri_interrupt,
        enable_peri_interrupt, disable_peri_interrupt }) (unstable)));
        _for_each_inner!((I2S1 <= I2S1(I2S1 : { bind_peri_interrupt,
        enable_peri_interrupt, disable_peri_interrupt }) (unstable)));
        _for_each_inner!((INTERRUPT_CORE0 <= INTERRUPT_CORE0() (unstable)));
        _for_each_inner!((INTERRUPT_CORE1 <= INTERRUPT_CORE1() (unstable)));
        _for_each_inner!((IO_MUX <= IO_MUX() (unstable))); _for_each_inner!((LCD_CAM <=
        LCD_CAM() (unstable))); _for_each_inner!((LEDC <= LEDC() (unstable)));
        _for_each_inner!((LPWR <= RTC_CNTL() (unstable))); _for_each_inner!((MCPWM0 <=
        MCPWM0() (unstable))); _for_each_inner!((MCPWM1 <= MCPWM1() (unstable)));
        _for_each_inner!((PCNT <= PCNT() (unstable))); _for_each_inner!((PERI_BACKUP <=
        PERI_BACKUP() (unstable))); _for_each_inner!((RMT <= RMT() (unstable)));
        _for_each_inner!((RNG <= RNG() (unstable))); _for_each_inner!((RSA <= RSA()
        (unstable))); _for_each_inner!((RTC_CNTL <= RTC_CNTL() (unstable)));
        _for_each_inner!((RTC_I2C <= RTC_I2C() (unstable))); _for_each_inner!((RTC_IO <=
        RTC_IO() (unstable))); _for_each_inner!((SDHOST <= SDHOST() (unstable)));
        _for_each_inner!((SENS <= SENS() (unstable))); _for_each_inner!((SENSITIVE <=
        SENSITIVE() (unstable))); _for_each_inner!((SHA <= SHA() (unstable)));
        _for_each_inner!((SPI0 <= SPI0() (unstable))); _for_each_inner!((SPI1 <= SPI1()
        (unstable))); _for_each_inner!((SPI2 <= SPI2(SPI2 : { bind_peri_interrupt,
        enable_peri_interrupt, disable_peri_interrupt }))); _for_each_inner!((SPI3 <=
        SPI3(SPI3 : { bind_peri_interrupt, enable_peri_interrupt, disable_peri_interrupt
        }))); _for_each_inner!((SYSTEM <= SYSTEM() (unstable)));
        _for_each_inner!((SYSTIMER <= SYSTIMER() (unstable))); _for_each_inner!((TIMG0 <=
        TIMG0() (unstable))); _for_each_inner!((TIMG1 <= TIMG1() (unstable)));
        _for_each_inner!((TWAI0 <= TWAI0() (unstable))); _for_each_inner!((UART0 <=
        UART0(UART0 : { bind_peri_interrupt, enable_peri_interrupt,
        disable_peri_interrupt }))); _for_each_inner!((UART1 <= UART1(UART1 : {
        bind_peri_interrupt, enable_peri_interrupt, disable_peri_interrupt })));
        _for_each_inner!((UART2 <= UART2(UART2 : { bind_peri_interrupt,
        enable_peri_interrupt, disable_peri_interrupt }))); _for_each_inner!((UHCI0 <=
        UHCI0() (unstable))); _for_each_inner!((UHCI1 <= UHCI1() (unstable)));
        _for_each_inner!((USB0 <= USB0() (unstable))); _for_each_inner!((USB_DEVICE <=
        USB_DEVICE(USB_DEVICE : { bind_peri_interrupt, enable_peri_interrupt,
        disable_peri_interrupt }) (unstable))); _for_each_inner!((USB_WRAP <= USB_WRAP()
        (unstable))); _for_each_inner!((WCL <= WCL() (unstable)));
        _for_each_inner!((XTS_AES <= XTS_AES() (unstable))); _for_each_inner!((DMA_CH0 <=
        virtual() (unstable))); _for_each_inner!((DMA_CH1 <= virtual() (unstable)));
        _for_each_inner!((DMA_CH2 <= virtual() (unstable))); _for_each_inner!((DMA_CH3 <=
        virtual() (unstable))); _for_each_inner!((DMA_CH4 <= virtual() (unstable)));
        _for_each_inner!((ADC1 <= virtual() (unstable))); _for_each_inner!((ADC2 <=
        virtual() (unstable))); _for_each_inner!((BT <= virtual() (unstable)));
        _for_each_inner!((CPU_CTRL <= virtual() (unstable))); _for_each_inner!((PSRAM <=
        virtual() (unstable))); _for_each_inner!((SW_INTERRUPT <= virtual() (unstable)));
        _for_each_inner!((ULP_RISCV_CORE <= virtual() (unstable)));
        _for_each_inner!((WIFI <= virtual() (unstable))); _for_each_inner!((all(GPIO0 <=
        virtual()), (GPIO1 <= virtual()), (GPIO2 <= virtual()), (GPIO3 <= virtual()),
        (GPIO4 <= virtual()), (GPIO5 <= virtual()), (GPIO6 <= virtual()), (GPIO7 <=
        virtual()), (GPIO8 <= virtual()), (GPIO9 <= virtual()), (GPIO10 <= virtual()),
        (GPIO11 <= virtual()), (GPIO12 <= virtual()), (GPIO13 <= virtual()), (GPIO14 <=
        virtual()), (GPIO15 <= virtual()), (GPIO16 <= virtual()), (GPIO17 <= virtual()),
        (GPIO18 <= virtual()), (GPIO19 <= virtual()), (GPIO20 <= virtual()), (GPIO21 <=
        virtual()), (GPIO26 <= virtual()), (GPIO27 <= virtual()), (GPIO28 <= virtual()),
        (GPIO29 <= virtual()), (GPIO30 <= virtual()), (GPIO31 <= virtual()), (GPIO32 <=
        virtual()), (GPIO33 <= virtual()), (GPIO34 <= virtual()), (GPIO35 <= virtual()),
        (GPIO36 <= virtual()), (GPIO37 <= virtual()), (GPIO38 <= virtual()), (GPIO39 <=
        virtual()), (GPIO40 <= virtual()), (GPIO41 <= virtual()), (GPIO42 <= virtual()),
        (GPIO43 <= virtual()), (GPIO44 <= virtual()), (GPIO45 <= virtual()), (GPIO46 <=
        virtual()), (GPIO47 <= virtual()), (GPIO48 <= virtual()), (AES <= AES()
        (unstable)), (APB_CTRL <= APB_CTRL() (unstable)), (APB_SARADC <= APB_SARADC()
        (unstable)), (ASSIST_DEBUG <= ASSIST_DEBUG() (unstable)), (DMA <= DMA()
        (unstable)), (DS <= DS() (unstable)), (EFUSE <= EFUSE() (unstable)), (EXTMEM <=
        EXTMEM() (unstable)), (GPIO <= GPIO() (unstable)), (GPIO_SD <= GPIO_SD()
        (unstable)), (HMAC <= HMAC() (unstable)), (I2C0 <= I2C0(I2C_EXT0 : {
        bind_peri_interrupt, enable_peri_interrupt, disable_peri_interrupt })), (I2C1 <=
        I2C1(I2C_EXT1 : { bind_peri_interrupt, enable_peri_interrupt,
        disable_peri_interrupt })), (I2S0 <= I2S0(I2S0 : { bind_peri_interrupt,
        enable_peri_interrupt, disable_peri_interrupt }) (unstable)), (I2S1 <= I2S1(I2S1
        : { bind_peri_interrupt, enable_peri_interrupt, disable_peri_interrupt })
        (unstable)), (INTERRUPT_CORE0 <= INTERRUPT_CORE0() (unstable)), (INTERRUPT_CORE1
        <= INTERRUPT_CORE1() (unstable)), (IO_MUX <= IO_MUX() (unstable)), (LCD_CAM <=
        LCD_CAM() (unstable)), (LEDC <= LEDC() (unstable)), (LPWR <= RTC_CNTL()
        (unstable)), (MCPWM0 <= MCPWM0() (unstable)), (MCPWM1 <= MCPWM1() (unstable)),
        (PCNT <= PCNT() (unstable)), (PERI_BACKUP <= PERI_BACKUP() (unstable)), (RMT <=
        RMT() (unstable)), (RNG <= RNG() (unstable)), (RSA <= RSA() (unstable)),
        (RTC_CNTL <= RTC_CNTL() (unstable)), (RTC_I2C <= RTC_I2C() (unstable)), (RTC_IO
        <= RTC_IO() (unstable)), (SDHOST <= SDHOST() (unstable)), (SENS <= SENS()
        (unstable)), (SENSITIVE <= SENSITIVE() (unstable)), (SHA <= SHA() (unstable)),
        (SPI0 <= SPI0() (unstable)), (SPI1 <= SPI1() (unstable)), (SPI2 <= SPI2(SPI2 : {
        bind_peri_interrupt, enable_peri_interrupt, disable_peri_interrupt })), (SPI3 <=
        SPI3(SPI3 : { bind_peri_interrupt, enable_peri_interrupt, disable_peri_interrupt
        })), (SYSTEM <= SYSTEM() (unstable)), (SYSTIMER <= SYSTIMER() (unstable)), (TIMG0
        <= TIMG0() (unstable)), (TIMG1 <= TIMG1() (unstable)), (TWAI0 <= TWAI0()
        (unstable)), (UART0 <= UART0(UART0 : { bind_peri_interrupt,
        enable_peri_interrupt, disable_peri_interrupt })), (UART1 <= UART1(UART1 : {
        bind_peri_interrupt, enable_peri_interrupt, disable_peri_interrupt })), (UART2 <=
        UART2(UART2 : { bind_peri_interrupt, enable_peri_interrupt,
        disable_peri_interrupt })), (UHCI0 <= UHCI0() (unstable)), (UHCI1 <= UHCI1()
        (unstable)), (USB0 <= USB0() (unstable)), (USB_DEVICE <= USB_DEVICE(USB_DEVICE :
        { bind_peri_interrupt, enable_peri_interrupt, disable_peri_interrupt })
        (unstable)), (USB_WRAP <= USB_WRAP() (unstable)), (WCL <= WCL() (unstable)),
        (XTS_AES <= XTS_AES() (unstable)), (DMA_CH0 <= virtual() (unstable)), (DMA_CH1 <=
        virtual() (unstable)), (DMA_CH2 <= virtual() (unstable)), (DMA_CH3 <= virtual()
        (unstable)), (DMA_CH4 <= virtual() (unstable)), (ADC1 <= virtual() (unstable)),
        (ADC2 <= virtual() (unstable)), (BT <= virtual() (unstable)), (CPU_CTRL <=
        virtual() (unstable)), (PSRAM <= virtual() (unstable)), (SW_INTERRUPT <=
        virtual() (unstable)), (ULP_RISCV_CORE <= virtual() (unstable)), (WIFI <=
        virtual() (unstable))));
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
        UsbDm UsbDevice))); _for_each_inner!((20, GPIO20(_2 => U1CTS) (_3 => CLK_OUT1)
        (Input Output Analog RtcIo RtcIoOutput UsbDp UsbDevice))); _for_each_inner!((21,
        GPIO21() () (Input Output Analog RtcIo RtcIoOutput))); _for_each_inner!((26,
        GPIO26() (_0 => SPICS1) (Input Output))); _for_each_inner!((27, GPIO27(_0 =>
        SPIHD) (_0 => SPIHD) (Input Output))); _for_each_inner!((28, GPIO28(_0 => SPIWP)
        (_0 => SPIWP) (Input Output))); _for_each_inner!((29, GPIO29() (_0 => SPICS0)
        (Input Output))); _for_each_inner!((30, GPIO30() (_0 => SPICLK) (Input Output)));
        _for_each_inner!((31, GPIO31(_0 => SPIQ) (_0 => SPIQ) (Input Output)));
        _for_each_inner!((32, GPIO32(_0 => SPID) (_0 => SPID) (Input Output)));
        _for_each_inner!((33, GPIO33(_2 => FSPIHD _3 => SUBSPIHD _4 => SPIIO4) (_2 =>
        FSPIHD _3 => SUBSPIHD _4 => SPIIO4) (Input Output))); _for_each_inner!((34,
        GPIO34(_2 => FSPICS0 _4 => SPIIO5) (_2 => FSPICS0 _3 => SUBSPICS0 _4 => SPIIO5)
        (Input Output))); _for_each_inner!((35, GPIO35(_2 => FSPID _3 => SUBSPID _4 =>
        SPIIO6) (_2 => FSPID _3 => SUBSPID _4 => SPIIO6) (Input Output)));
        _for_each_inner!((36, GPIO36(_2 => FSPICLK _4 => SPIIO7) (_2 => FSPICLK _3 =>
        SUBSPICLK _4 => SPIIO7) (Input Output))); _for_each_inner!((37, GPIO37(_2 =>
        FSPIQ _3 => SUBSPIQ _4 => SPIDQS) (_2 => FSPIQ _3 => SUBSPIQ _4 => SPIDQS) (Input
        Output))); _for_each_inner!((38, GPIO38(_2 => FSPIWP _3 => SUBSPIWP) (_2 =>
        FSPIWP _3 => SUBSPIWP) (Input Output))); _for_each_inner!((39, GPIO39() (_2 =>
        CLK_OUT3 _3 => SUBSPICS1) (Input Output))); _for_each_inner!((40, GPIO40() (_2 =>
        CLK_OUT2) (Input Output))); _for_each_inner!((41, GPIO41() (_2 => CLK_OUT1)
        (Input Output))); _for_each_inner!((42, GPIO42() () (Input Output)));
        _for_each_inner!((43, GPIO43() (_0 => U0TXD _2 => CLK_OUT1) (Input Output)));
        _for_each_inner!((44, GPIO44(_0 => U0RXD) (_2 => CLK_OUT2) (Input Output)));
        _for_each_inner!((45, GPIO45() () (Input Output))); _for_each_inner!((46,
        GPIO46() () (Input Output))); _for_each_inner!((47, GPIO47() (_0 => SPICLK_P_DIFF
        _2 => SUBSPICLK_P_DIFF) (Input Output))); _for_each_inner!((48, GPIO48() (_0 =>
        SPICLK_N_DIFF _2 => SUBSPICLK_N_DIFF) (Input Output))); _for_each_inner!((all(0,
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
        Output Analog RtcIo RtcIoOutput UsbDm UsbDevice)), (20, GPIO20(_2 => U1CTS) (_3
        => CLK_OUT1) (Input Output Analog RtcIo RtcIoOutput UsbDp UsbDevice)), (21,
        GPIO21() () (Input Output Analog RtcIo RtcIoOutput)), (26, GPIO26() (_0 =>
        SPICS1) (Input Output)), (27, GPIO27(_0 => SPIHD) (_0 => SPIHD) (Input Output)),
        (28, GPIO28(_0 => SPIWP) (_0 => SPIWP) (Input Output)), (29, GPIO29() (_0 =>
        SPICS0) (Input Output)), (30, GPIO30() (_0 => SPICLK) (Input Output)), (31,
        GPIO31(_0 => SPIQ) (_0 => SPIQ) (Input Output)), (32, GPIO32(_0 => SPID) (_0 =>
        SPID) (Input Output)), (33, GPIO33(_2 => FSPIHD _3 => SUBSPIHD _4 => SPIIO4) (_2
        => FSPIHD _3 => SUBSPIHD _4 => SPIIO4) (Input Output)), (34, GPIO34(_2 => FSPICS0
        _4 => SPIIO5) (_2 => FSPICS0 _3 => SUBSPICS0 _4 => SPIIO5) (Input Output)), (35,
        GPIO35(_2 => FSPID _3 => SUBSPID _4 => SPIIO6) (_2 => FSPID _3 => SUBSPID _4 =>
        SPIIO6) (Input Output)), (36, GPIO36(_2 => FSPICLK _4 => SPIIO7) (_2 => FSPICLK
        _3 => SUBSPICLK _4 => SPIIO7) (Input Output)), (37, GPIO37(_2 => FSPIQ _3 =>
        SUBSPIQ _4 => SPIDQS) (_2 => FSPIQ _3 => SUBSPIQ _4 => SPIDQS) (Input Output)),
        (38, GPIO38(_2 => FSPIWP _3 => SUBSPIWP) (_2 => FSPIWP _3 => SUBSPIWP) (Input
        Output)), (39, GPIO39() (_2 => CLK_OUT3 _3 => SUBSPICS1) (Input Output)), (40,
        GPIO40() (_2 => CLK_OUT2) (Input Output)), (41, GPIO41() (_2 => CLK_OUT1) (Input
        Output)), (42, GPIO42() () (Input Output)), (43, GPIO43() (_0 => U0TXD _2 =>
        CLK_OUT1) (Input Output)), (44, GPIO44(_0 => U0RXD) (_2 => CLK_OUT2) (Input
        Output)), (45, GPIO45() () (Input Output)), (46, GPIO46() () (Input Output)),
        (47, GPIO47() (_0 => SPICLK_P_DIFF _2 => SUBSPICLK_P_DIFF) (Input Output)), (48,
        GPIO48() (_0 => SPICLK_N_DIFF _2 => SUBSPICLK_N_DIFF) (Input Output))));
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
    (GPIO19, UsbDevice, $then_tt:tt else $else_tt:tt) => {
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
    (GPIO20, UsbDevice, $then_tt:tt else $else_tt:tt) => {
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
    (GPIO47, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO47, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO47, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO48, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO48, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO48, $t:tt, $then_tt:tt else $else_tt:tt) => {
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
        #[allow(unused_braces)] $code } } else { $otherwise }), 47 =>
        if_pin_is_type!(GPIO47, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO47::steal() };
        #[allow(unused_braces)] $code } } else { $otherwise }), 48 =>
        if_pin_is_type!(GPIO48, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO48::steal() };
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
            SPIQ                    = 0,
            SPID                    = 1,
            SPIHD                   = 2,
            SPIWP                   = 3,
            SPID4                   = 7,
            SPID5                   = 8,
            SPID6                   = 9,
            SPID7                   = 10,
            SPIDQS                  = 11,
            U0RXD                   = 12,
            U0CTS                   = 13,
            U0DSR                   = 14,
            U1RXD                   = 15,
            U1CTS                   = 16,
            U1DSR                   = 17,
            U2RXD                   = 18,
            U2CTS                   = 19,
            U2DSR                   = 20,
            I2S1_MCLK               = 21,
            I2S0O_BCK               = 22,
            I2S0_MCLK               = 23,
            I2S0O_WS                = 24,
            I2S0I_SD                = 25,
            I2S0I_BCK               = 26,
            I2S0I_WS                = 27,
            I2S1O_BCK               = 28,
            I2S1O_WS                = 29,
            I2S1I_SD                = 30,
            I2S1I_BCK               = 31,
            I2S1I_WS                = 32,
            PCNT0_SIG_CH0           = 33,
            PCNT0_SIG_CH1           = 34,
            PCNT0_CTRL_CH0          = 35,
            PCNT0_CTRL_CH1          = 36,
            PCNT1_SIG_CH0           = 37,
            PCNT1_SIG_CH1           = 38,
            PCNT1_CTRL_CH0          = 39,
            PCNT1_CTRL_CH1          = 40,
            PCNT2_SIG_CH0           = 41,
            PCNT2_SIG_CH1           = 42,
            PCNT2_CTRL_CH0          = 43,
            PCNT2_CTRL_CH1          = 44,
            PCNT3_SIG_CH0           = 45,
            PCNT3_SIG_CH1           = 46,
            PCNT3_CTRL_CH0          = 47,
            PCNT3_CTRL_CH1          = 48,
            I2S0I_SD1               = 51,
            I2S0I_SD2               = 52,
            I2S0I_SD3               = 53,
            USB_EXTPHY_VP           = 55,
            USB_EXTPHY_VM           = 56,
            USB_EXTPHY_RCV          = 57,
            USB_OTG_IDDIG           = 58,
            USB_OTG_AVALID          = 59,
            USB_SRP_BVALID          = 60,
            USB_OTG_VBUSVALID       = 61,
            USB_SRP_SESSEND         = 62,
            SPI3_CLK                = 66,
            SPI3_Q                  = 67,
            SPI3_D                  = 68,
            SPI3_HD                 = 69,
            SPI3_WP                 = 70,
            SPI3_CS0                = 71,
            RMT_SIG_0               = 81,
            RMT_SIG_1               = 82,
            RMT_SIG_2               = 83,
            RMT_SIG_3               = 84,
            I2CEXT0_SCL             = 89,
            I2CEXT0_SDA             = 90,
            I2CEXT1_SCL             = 91,
            I2CEXT1_SDA             = 92,
            FSPICLK                 = 101,
            FSPIQ                   = 102,
            FSPID                   = 103,
            FSPIHD                  = 104,
            FSPIWP                  = 105,
            FSPIIO4                 = 106,
            FSPIIO5                 = 107,
            FSPIIO6                 = 108,
            FSPIIO7                 = 109,
            FSPICS0                 = 110,
            TWAI_RX                 = 116,
            SUBSPIQ                 = 120,
            SUBSPID                 = 121,
            SUBSPIHD                = 122,
            SUBSPIWP                = 123,
            CAM_DATA_0              = 133,
            CAM_DATA_1              = 134,
            CAM_DATA_2              = 135,
            CAM_DATA_3              = 136,
            CAM_DATA_4              = 137,
            CAM_DATA_5              = 138,
            CAM_DATA_6              = 139,
            CAM_DATA_7              = 140,
            CAM_DATA_8              = 141,
            CAM_DATA_9              = 142,
            CAM_DATA_10             = 143,
            CAM_DATA_11             = 144,
            CAM_DATA_12             = 145,
            CAM_DATA_13             = 146,
            CAM_DATA_14             = 147,
            CAM_DATA_15             = 148,
            CAM_PCLK                = 149,
            CAM_H_ENABLE            = 150,
            CAM_H_SYNC              = 151,
            CAM_V_SYNC              = 152,
            SUBSPID4                = 155,
            SUBSPID5                = 156,
            SUBSPID6                = 157,
            SUBSPID7                = 158,
            SUBSPIDQS               = 159,
            PWM0_SYNC0              = 160,
            PWM0_SYNC1              = 161,
            PWM0_SYNC2              = 162,
            PWM0_F0                 = 163,
            PWM0_F1                 = 164,
            PWM0_F2                 = 165,
            PWM0_CAP0               = 166,
            PWM0_CAP1               = 167,
            PWM0_CAP2               = 168,
            PWM1_SYNC0              = 169,
            PWM1_SYNC1              = 170,
            PWM1_SYNC2              = 171,
            PWM1_F0                 = 172,
            PWM1_F1                 = 173,
            PWM1_F2                 = 174,
            PWM1_CAP0               = 175,
            PWM1_CAP1               = 176,
            PWM1_CAP2               = 177,
            SDHOST_CCMD_IN_1        = 178,
            SDHOST_CCMD_IN_2        = 179,
            SDHOST_CDATA_IN_10      = 180,
            SDHOST_CDATA_IN_11      = 181,
            SDHOST_CDATA_IN_12      = 182,
            SDHOST_CDATA_IN_13      = 183,
            SDHOST_CDATA_IN_14      = 184,
            SDHOST_CDATA_IN_15      = 185,
            SDHOST_CDATA_IN_16      = 186,
            SDHOST_CDATA_IN_17      = 187,
            SDHOST_DATA_STROBE_1    = 192,
            SDHOST_DATA_STROBE_2    = 193,
            SDHOST_CARD_DETECT_N_1  = 194,
            SDHOST_CARD_DETECT_N_2  = 195,
            SDHOST_CARD_WRITE_PRT_1 = 196,
            SDHOST_CARD_WRITE_PRT_2 = 197,
            SDHOST_CARD_INT_N_1     = 198,
            SDHOST_CARD_INT_N_2     = 199,
            SDHOST_CDATA_IN_20      = 213,
            SDHOST_CDATA_IN_21      = 214,
            SDHOST_CDATA_IN_22      = 215,
            SDHOST_CDATA_IN_23      = 216,
            SDHOST_CDATA_IN_24      = 217,
            SDHOST_CDATA_IN_25      = 218,
            SDHOST_CDATA_IN_26      = 219,
            SDHOST_CDATA_IN_27      = 220,
            PRO_ALONEGPIO_IN0       = 221,
            PRO_ALONEGPIO_IN1       = 222,
            PRO_ALONEGPIO_IN2       = 223,
            PRO_ALONEGPIO_IN3       = 224,
            PRO_ALONEGPIO_IN4       = 225,
            PRO_ALONEGPIO_IN5       = 226,
            PRO_ALONEGPIO_IN6       = 227,
            PRO_ALONEGPIO_IN7       = 228,
            USB_JTAG_TDO_BRIDGE     = 251,
            CORE1_GPIO_IN3          = 252,
            CORE1_GPIO_IN4          = 253,
            CORE1_GPIO_IN5          = 254,
            CORE1_GPIO_IN6          = 255,
            SPIIO4,
            SPIIO5,
            SPIIO6,
            SPIIO7,
            MTDI,
            MTCK,
            MTMS,
        }
        #[allow(non_camel_case_types, clippy::upper_case_acronyms)]
        #[derive(Debug, PartialEq, Copy, Clone)]
        #[cfg_attr(feature = "defmt", derive(defmt::Format))]
        #[doc(hidden)]
        pub enum OutputSignal {
            SPIQ                       = 0,
            SPID                       = 1,
            SPIHD                      = 2,
            SPIWP                      = 3,
            SPICLK                     = 4,
            SPICS0                     = 5,
            SPICS1                     = 6,
            SPID4                      = 7,
            SPID5                      = 8,
            SPID6                      = 9,
            SPID7                      = 10,
            SPIDQS                     = 11,
            U0TXD                      = 12,
            U0RTS                      = 13,
            U0DTR                      = 14,
            U1TXD                      = 15,
            U1RTS                      = 16,
            U1DTR                      = 17,
            U2TXD                      = 18,
            U2RTS                      = 19,
            U2DTR                      = 20,
            I2S1_MCLK                  = 21,
            I2S0O_BCK                  = 22,
            I2S0_MCLK                  = 23,
            I2S0O_WS                   = 24,
            I2S0O_SD                   = 25,
            I2S0I_BCK                  = 26,
            I2S0I_WS                   = 27,
            I2S1O_BCK                  = 28,
            I2S1O_WS                   = 29,
            I2S1O_SD                   = 30,
            I2S1I_BCK                  = 31,
            I2S1I_WS                   = 32,
            USB_EXTPHY_OEN             = 55,
            USB_EXTPHY_VPO             = 57,
            USB_EXTPHY_VMO             = 58,
            SPI3_CLK                   = 66,
            SPI3_Q                     = 67,
            SPI3_D                     = 68,
            SPI3_HD                    = 69,
            SPI3_WP                    = 70,
            SPI3_CS0                   = 71,
            SPI3_CS1                   = 72,
            LEDC_LS_SIG0               = 73,
            LEDC_LS_SIG1               = 74,
            LEDC_LS_SIG2               = 75,
            LEDC_LS_SIG3               = 76,
            LEDC_LS_SIG4               = 77,
            LEDC_LS_SIG5               = 78,
            LEDC_LS_SIG6               = 79,
            LEDC_LS_SIG7               = 80,
            RMT_SIG_0                  = 81,
            RMT_SIG_1                  = 82,
            RMT_SIG_2                  = 83,
            RMT_SIG_3                  = 84,
            I2CEXT0_SCL                = 89,
            I2CEXT0_SDA                = 90,
            I2CEXT1_SCL                = 91,
            I2CEXT1_SDA                = 92,
            GPIO_SD0                   = 93,
            GPIO_SD1                   = 94,
            GPIO_SD2                   = 95,
            GPIO_SD3                   = 96,
            GPIO_SD4                   = 97,
            GPIO_SD5                   = 98,
            GPIO_SD6                   = 99,
            GPIO_SD7                   = 100,
            FSPICLK                    = 101,
            FSPIQ                      = 102,
            FSPID                      = 103,
            FSPIHD                     = 104,
            FSPIWP                     = 105,
            FSPIIO4                    = 106,
            FSPIIO5                    = 107,
            FSPIIO6                    = 108,
            FSPIIO7                    = 109,
            FSPICS0                    = 110,
            FSPICS1                    = 111,
            FSPICS2                    = 112,
            FSPICS3                    = 113,
            FSPICS4                    = 114,
            FSPICS5                    = 115,
            TWAI_TX                    = 116,
            SUBSPICLK                  = 119,
            SUBSPIQ                    = 120,
            SUBSPID                    = 121,
            SUBSPIHD                   = 122,
            SUBSPIWP                   = 123,
            SUBSPICS0                  = 124,
            SUBSPICS1                  = 125,
            FSPIDQS                    = 126,
            SPI3_CS2                   = 127,
            I2S0O_SD1                  = 128,
            LCD_CS                     = 132,
            LCD_DATA_0                 = 133,
            LCD_DATA_1                 = 134,
            LCD_DATA_2                 = 135,
            LCD_DATA_3                 = 136,
            LCD_DATA_4                 = 137,
            LCD_DATA_5                 = 138,
            LCD_DATA_6                 = 139,
            LCD_DATA_7                 = 140,
            LCD_DATA_8                 = 141,
            LCD_DATA_9                 = 142,
            LCD_DATA_10                = 143,
            LCD_DATA_11                = 144,
            LCD_DATA_12                = 145,
            LCD_DATA_13                = 146,
            LCD_DATA_14                = 147,
            LCD_DATA_15                = 148,
            CAM_CLK                    = 149,
            LCD_H_ENABLE               = 150,
            LCD_H_SYNC                 = 151,
            LCD_V_SYNC                 = 152,
            LCD_DC                     = 153,
            LCD_PCLK                   = 154,
            SUBSPID4                   = 155,
            SUBSPID5                   = 156,
            SUBSPID6                   = 157,
            SUBSPID7                   = 158,
            SUBSPIDQS                  = 159,
            PWM0_0A                    = 160,
            PWM0_0B                    = 161,
            PWM0_1A                    = 162,
            PWM0_1B                    = 163,
            PWM0_2A                    = 164,
            PWM0_2B                    = 165,
            PWM1_0A                    = 166,
            PWM1_0B                    = 167,
            PWM1_1A                    = 168,
            PWM1_1B                    = 169,
            PWM1_2A                    = 170,
            PWM1_2B                    = 171,
            SDHOST_CCLK_OUT_1          = 172,
            SDHOST_CCLK_OUT_2          = 173,
            SDHOST_RST_N_1             = 174,
            SDHOST_RST_N_2             = 175,
            SDHOST_CCMD_OD_PULLUP_EN_N = 176,
            SDIO_TOHOST_INT            = 177,
            SDHOST_CCMD_OUT_1          = 178,
            SDHOST_CCMD_OUT_2          = 179,
            SDHOST_CDATA_OUT_10        = 180,
            SDHOST_CDATA_OUT_11        = 181,
            SDHOST_CDATA_OUT_12        = 182,
            SDHOST_CDATA_OUT_13        = 183,
            SDHOST_CDATA_OUT_14        = 184,
            SDHOST_CDATA_OUT_15        = 185,
            SDHOST_CDATA_OUT_16        = 186,
            SDHOST_CDATA_OUT_17        = 187,
            SDHOST_CDATA_OUT_20        = 213,
            SDHOST_CDATA_OUT_21        = 214,
            SDHOST_CDATA_OUT_22        = 215,
            SDHOST_CDATA_OUT_23        = 216,
            SDHOST_CDATA_OUT_24        = 217,
            SDHOST_CDATA_OUT_25        = 218,
            SDHOST_CDATA_OUT_26        = 219,
            SDHOST_CDATA_OUT_27        = 220,
            PRO_ALONEGPIO_OUT0         = 221,
            PRO_ALONEGPIO_OUT1         = 222,
            PRO_ALONEGPIO_OUT2         = 223,
            PRO_ALONEGPIO_OUT3         = 224,
            PRO_ALONEGPIO_OUT4         = 225,
            PRO_ALONEGPIO_OUT5         = 226,
            PRO_ALONEGPIO_OUT6         = 227,
            PRO_ALONEGPIO_OUT7         = 228,
            USB_JTAG_TRST              = 251,
            CORE1_GPIO_OUT3            = 252,
            CORE1_GPIO_OUT4            = 253,
            CORE1_GPIO_OUT5            = 254,
            CORE1_GPIO_OUT6            = 255,
            GPIO                       = 256,
            SPIIO4,
            SPIIO5,
            SPIIO6,
            SPIIO7,
            CLK_OUT1,
            CLK_OUT2,
            CLK_OUT3,
            SPICLK_P_DIFF,
            SPICLK_N_DIFF,
            SUBSPICLK_P_DIFF,
            SUBSPICLK_N_DIFF,
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
