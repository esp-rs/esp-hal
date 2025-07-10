/// The name of the chip as `&str`
///
/// # Example
///
/// ```rust, no_run
/// use esp_hal::chip;
/// let chip_name = chip!();
#[doc = concat!("assert_eq!(chip_name, ", chip!(), ")")]
/// ```
#[macro_export]
macro_rules! chip {
    () => {
        "esp32h2"
    };
}
/// The properties of this chip and its drivers.
#[macro_export]
macro_rules! property {
    ("chip") => {
        "esp32h2"
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
        "https://www.espressif.com/sites/default/files/documentation/esp32-h2_technical_reference_manual_en.pdf"
    };
    ("assist_debug.has_sp_monitor") => {
        true
    };
    ("assist_debug.has_region_monitor") => {
        true
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
        124
    };
    ("gpio.input_signal_max", str) => {
        stringify!(124)
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
        true
    };
    ("i2c_master.has_conf_update") => {
        true
    };
    ("i2c_master.has_reliable_fsm_reset") => {
        true
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
        2
    };
    ("interrupts.status_registers", str) => {
        stringify!(2)
    };
    ("rmt.ram_start") => {
        1610642432
    };
    ("rmt.ram_start", str) => {
        stringify!(1610642432)
    };
    ("rmt.channel_ram_size") => {
        48
    };
    ("rmt.channel_ram_size", str) => {
        stringify!(48)
    };
    ("spi_master.has_octal") => {
        false
    };
    ("timergroup.timg_has_timer1") => {
        false
    };
}
/// Macro to get the address range of the given memory region.
#[macro_export]
macro_rules! memory_range {
    ("DRAM") => {
        1082130432..1082458112
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
        _for_each_inner!((GPIO14 <= virtual())); _for_each_inner!((GPIO22 <= virtual()));
        _for_each_inner!((GPIO23 <= virtual())); _for_each_inner!((GPIO24 <= virtual()));
        _for_each_inner!((GPIO25 <= virtual())); _for_each_inner!((GPIO26 <= virtual()));
        _for_each_inner!((GPIO27 <= virtual())); _for_each_inner!((AES <= AES()
        (unstable))); _for_each_inner!((APB_SARADC <= APB_SARADC() (unstable)));
        _for_each_inner!((ASSIST_DEBUG <= ASSIST_DEBUG() (unstable)));
        _for_each_inner!((DMA <= DMA() (unstable))); _for_each_inner!((DS <= DS()
        (unstable))); _for_each_inner!((ECC <= ECC() (unstable)));
        _for_each_inner!((EFUSE <= EFUSE() (unstable))); _for_each_inner!((GPIO <= GPIO()
        (unstable))); _for_each_inner!((GPIO_SD <= GPIO_SD() (unstable)));
        _for_each_inner!((HMAC <= HMAC() (unstable))); _for_each_inner!((HP_APM <=
        HP_APM() (unstable))); _for_each_inner!((HP_SYS <= HP_SYS() (unstable)));
        _for_each_inner!((I2C_ANA_MST <= I2C_ANA_MST() (unstable)));
        _for_each_inner!((I2C0 <= I2C0(I2C_EXT0 : { bind_peri_interrupt,
        enable_peri_interrupt, disable_peri_interrupt }))); _for_each_inner!((I2C1 <=
        I2C1(I2C_EXT1 : { bind_peri_interrupt, enable_peri_interrupt,
        disable_peri_interrupt }))); _for_each_inner!((I2S0 <= I2S0(I2S0 : {
        bind_peri_interrupt, enable_peri_interrupt, disable_peri_interrupt })
        (unstable))); _for_each_inner!((IEEE802154 <= IEEE802154() (unstable)));
        _for_each_inner!((INTERRUPT_CORE0 <= INTERRUPT_CORE0() (unstable)));
        _for_each_inner!((INTPRI <= INTPRI() (unstable))); _for_each_inner!((IO_MUX <=
        IO_MUX() (unstable))); _for_each_inner!((LEDC <= LEDC() (unstable)));
        _for_each_inner!((LPWR <= LP_CLKRST() (unstable))); _for_each_inner!((LP_ANA <=
        LP_ANA() (unstable))); _for_each_inner!((LP_AON <= LP_AON() (unstable)));
        _for_each_inner!((LP_APM <= LP_APM() (unstable))); _for_each_inner!((LP_APM0 <=
        LP_APM0() (unstable))); _for_each_inner!((LP_CLKRST <= LP_CLKRST() (unstable)));
        _for_each_inner!((LP_PERI <= LP_PERI() (unstable))); _for_each_inner!((LP_TIMER
        <= LP_TIMER() (unstable))); _for_each_inner!((LP_WDT <= LP_WDT() (unstable)));
        _for_each_inner!((MCPWM0 <= MCPWM0() (unstable))); _for_each_inner!((MEM_MONITOR
        <= MEM_MONITOR() (unstable))); _for_each_inner!((MODEM_LPCON <= MODEM_LPCON()
        (unstable))); _for_each_inner!((MODEM_SYSCON <= MODEM_SYSCON() (unstable)));
        _for_each_inner!((OTP_DEBUG <= OTP_DEBUG() (unstable)));
        _for_each_inner!((PARL_IO <= PARL_IO(PARL_IO_RX : { bind_rx_interrupt,
        enable_rx_interrupt, disable_rx_interrupt }, PARL_IO_TX : { bind_tx_interrupt,
        enable_tx_interrupt, disable_tx_interrupt }) (unstable))); _for_each_inner!((PAU
        <= PAU() (unstable))); _for_each_inner!((PCNT <= PCNT() (unstable)));
        _for_each_inner!((PCR <= PCR() (unstable))); _for_each_inner!((PLIC_MX <=
        PLIC_MX() (unstable))); _for_each_inner!((PMU <= PMU() (unstable)));
        _for_each_inner!((RMT <= RMT() (unstable))); _for_each_inner!((RNG <= RNG()
        (unstable))); _for_each_inner!((RSA <= RSA() (unstable))); _for_each_inner!((SHA
        <= SHA() (unstable))); _for_each_inner!((ETM <= SOC_ETM() (unstable)));
        _for_each_inner!((SPI0 <= SPI0() (unstable))); _for_each_inner!((SPI1 <= SPI1()
        (unstable))); _for_each_inner!((SPI2 <= SPI2(SPI2 : { bind_peri_interrupt,
        enable_peri_interrupt, disable_peri_interrupt }))); _for_each_inner!((SYSTEM <=
        PCR() (unstable))); _for_each_inner!((SYSTIMER <= SYSTIMER() (unstable)));
        _for_each_inner!((TEE <= TEE() (unstable))); _for_each_inner!((TIMG0 <= TIMG0()
        (unstable))); _for_each_inner!((TIMG1 <= TIMG1() (unstable)));
        _for_each_inner!((TRACE0 <= TRACE() (unstable))); _for_each_inner!((TWAI0 <=
        TWAI0() (unstable))); _for_each_inner!((UART0 <= UART0(UART0 : {
        bind_peri_interrupt, enable_peri_interrupt, disable_peri_interrupt })));
        _for_each_inner!((UART1 <= UART1(UART1 : { bind_peri_interrupt,
        enable_peri_interrupt, disable_peri_interrupt }))); _for_each_inner!((UHCI0 <=
        UHCI0() (unstable))); _for_each_inner!((USB_DEVICE <= USB_DEVICE(USB_DEVICE : {
        bind_peri_interrupt, enable_peri_interrupt, disable_peri_interrupt })
        (unstable))); _for_each_inner!((DMA_CH0 <= virtual() (unstable)));
        _for_each_inner!((DMA_CH1 <= virtual() (unstable))); _for_each_inner!((DMA_CH2 <=
        virtual() (unstable))); _for_each_inner!((ADC1 <= virtual() (unstable)));
        _for_each_inner!((BT <= virtual() (unstable))); _for_each_inner!((SW_INTERRUPT <=
        virtual() (unstable))); _for_each_inner!((MEM2MEM1 <= virtual() (unstable)));
        _for_each_inner!((MEM2MEM4 <= virtual() (unstable))); _for_each_inner!((MEM2MEM5
        <= virtual() (unstable))); _for_each_inner!((MEM2MEM10 <= virtual() (unstable)));
        _for_each_inner!((MEM2MEM11 <= virtual() (unstable)));
        _for_each_inner!((MEM2MEM12 <= virtual() (unstable)));
        _for_each_inner!((MEM2MEM13 <= virtual() (unstable)));
        _for_each_inner!((MEM2MEM14 <= virtual() (unstable)));
        _for_each_inner!((MEM2MEM15 <= virtual() (unstable)));
        _for_each_inner!((all(GPIO0 <= virtual()), (GPIO1 <= virtual()), (GPIO2 <=
        virtual()), (GPIO3 <= virtual()), (GPIO4 <= virtual()), (GPIO5 <= virtual()),
        (GPIO6 <= virtual()), (GPIO7 <= virtual()), (GPIO8 <= virtual()), (GPIO9 <=
        virtual()), (GPIO10 <= virtual()), (GPIO11 <= virtual()), (GPIO12 <= virtual()),
        (GPIO13 <= virtual()), (GPIO14 <= virtual()), (GPIO22 <= virtual()), (GPIO23 <=
        virtual()), (GPIO24 <= virtual()), (GPIO25 <= virtual()), (GPIO26 <= virtual()),
        (GPIO27 <= virtual()), (AES <= AES() (unstable)), (APB_SARADC <= APB_SARADC()
        (unstable)), (ASSIST_DEBUG <= ASSIST_DEBUG() (unstable)), (DMA <= DMA()
        (unstable)), (DS <= DS() (unstable)), (ECC <= ECC() (unstable)), (EFUSE <=
        EFUSE() (unstable)), (GPIO <= GPIO() (unstable)), (GPIO_SD <= GPIO_SD()
        (unstable)), (HMAC <= HMAC() (unstable)), (HP_APM <= HP_APM() (unstable)),
        (HP_SYS <= HP_SYS() (unstable)), (I2C_ANA_MST <= I2C_ANA_MST() (unstable)), (I2C0
        <= I2C0(I2C_EXT0 : { bind_peri_interrupt, enable_peri_interrupt,
        disable_peri_interrupt })), (I2C1 <= I2C1(I2C_EXT1 : { bind_peri_interrupt,
        enable_peri_interrupt, disable_peri_interrupt })), (I2S0 <= I2S0(I2S0 : {
        bind_peri_interrupt, enable_peri_interrupt, disable_peri_interrupt })
        (unstable)), (IEEE802154 <= IEEE802154() (unstable)), (INTERRUPT_CORE0 <=
        INTERRUPT_CORE0() (unstable)), (INTPRI <= INTPRI() (unstable)), (IO_MUX <=
        IO_MUX() (unstable)), (LEDC <= LEDC() (unstable)), (LPWR <= LP_CLKRST()
        (unstable)), (LP_ANA <= LP_ANA() (unstable)), (LP_AON <= LP_AON() (unstable)),
        (LP_APM <= LP_APM() (unstable)), (LP_APM0 <= LP_APM0() (unstable)), (LP_CLKRST <=
        LP_CLKRST() (unstable)), (LP_PERI <= LP_PERI() (unstable)), (LP_TIMER <=
        LP_TIMER() (unstable)), (LP_WDT <= LP_WDT() (unstable)), (MCPWM0 <= MCPWM0()
        (unstable)), (MEM_MONITOR <= MEM_MONITOR() (unstable)), (MODEM_LPCON <=
        MODEM_LPCON() (unstable)), (MODEM_SYSCON <= MODEM_SYSCON() (unstable)),
        (OTP_DEBUG <= OTP_DEBUG() (unstable)), (PARL_IO <= PARL_IO(PARL_IO_RX : {
        bind_rx_interrupt, enable_rx_interrupt, disable_rx_interrupt }, PARL_IO_TX : {
        bind_tx_interrupt, enable_tx_interrupt, disable_tx_interrupt }) (unstable)), (PAU
        <= PAU() (unstable)), (PCNT <= PCNT() (unstable)), (PCR <= PCR() (unstable)),
        (PLIC_MX <= PLIC_MX() (unstable)), (PMU <= PMU() (unstable)), (RMT <= RMT()
        (unstable)), (RNG <= RNG() (unstable)), (RSA <= RSA() (unstable)), (SHA <= SHA()
        (unstable)), (ETM <= SOC_ETM() (unstable)), (SPI0 <= SPI0() (unstable)), (SPI1 <=
        SPI1() (unstable)), (SPI2 <= SPI2(SPI2 : { bind_peri_interrupt,
        enable_peri_interrupt, disable_peri_interrupt })), (SYSTEM <= PCR() (unstable)),
        (SYSTIMER <= SYSTIMER() (unstable)), (TEE <= TEE() (unstable)), (TIMG0 <= TIMG0()
        (unstable)), (TIMG1 <= TIMG1() (unstable)), (TRACE0 <= TRACE() (unstable)),
        (TWAI0 <= TWAI0() (unstable)), (UART0 <= UART0(UART0 : { bind_peri_interrupt,
        enable_peri_interrupt, disable_peri_interrupt })), (UART1 <= UART1(UART1 : {
        bind_peri_interrupt, enable_peri_interrupt, disable_peri_interrupt })), (UHCI0 <=
        UHCI0() (unstable)), (USB_DEVICE <= USB_DEVICE(USB_DEVICE : {
        bind_peri_interrupt, enable_peri_interrupt, disable_peri_interrupt })
        (unstable)), (DMA_CH0 <= virtual() (unstable)), (DMA_CH1 <= virtual()
        (unstable)), (DMA_CH2 <= virtual() (unstable)), (ADC1 <= virtual() (unstable)),
        (BT <= virtual() (unstable)), (SW_INTERRUPT <= virtual() (unstable)), (MEM2MEM1
        <= virtual() (unstable)), (MEM2MEM4 <= virtual() (unstable)), (MEM2MEM5 <=
        virtual() (unstable)), (MEM2MEM10 <= virtual() (unstable)), (MEM2MEM11 <=
        virtual() (unstable)), (MEM2MEM12 <= virtual() (unstable)), (MEM2MEM13 <=
        virtual() (unstable)), (MEM2MEM14 <= virtual() (unstable)), (MEM2MEM15 <=
        virtual() (unstable))));
    };
}
/// This macro can be used to generate code for each GPIOn instance.
///
/// The basic syntax of this macro looks like a macro definition with two distinct syntax options:
///
/// ```rust, no_run
/// for_each_gpio! {
///     // Individual matcher, invoked separately for each GPIO
///     ( <match arm> ) => { /* some code */ };
///
///     // Repeated matcher, invoked once with all GPIOs
///     ( all $( (<individual match syntax>) ),* ) => { /* some code */ };
/// }
/// ```
///
/// You can specify any number of matchers.
///
/// ## Using the individual matcher
///
/// In this use case, each GPIO's data is individually passed through the macro. This can be used to
/// generate code for each GPIO separately, allowing specializing the implementation where needed.
///
/// ```rust,no_run
/// for_each_gpio! {
///   // Example data: `(0, GPIO0 (_5 => EMAC_TX_CLK) (_1 => CLK_OUT1 _5 => EMAC_TX_CLK) (Input Output))`
///   ($n:literal, $gpio:ident ($($digital_input_function:ident => $digital_input_signal:ident)*) ($($digital_output_function:ident => $digital_output_signal:ident)*) ($($pin_attribute:ident)*)) => { /* some code */ };
///
///   // You can create matchers with data filled in. This example will specifically match GPIO2
///   ($n:literal, GPIO2 $input_af:tt $output_af:tt $attributes:tt) => { /* Additional case only for GPIO2 */ };
/// }
/// ```
///
/// ## Repeated matcher
///
/// With this option, all GPIO data is passed through the macro all at once. This form can be used
/// to, for example, generate struct fields.
///
/// ```rust,no_run
/// // Example usage to create a struct containing all GPIOs:
/// for_each_gpio! {
///     (all $( ($n:literal, $gpio:ident $_af_ins:tt $_af_outs:tt $_attrs:tt) ),*) => {
///         struct Gpios {
///             $(
///                 #[doc = concat!(" The ", stringify!($n), "th GPIO pin")]
///                 pub $gpio: Gpio<$n>,
///             )*
///         }
///     };
/// }
/// ```
#[macro_export]
macro_rules! for_each_gpio {
    ($($pattern:tt => $code:tt;)*) => {
        macro_rules! _for_each_inner { $(($pattern) => $code;)* ($other : tt) => {} }
        _for_each_inner!((0, GPIO0(_2 => FSPIQ) (_2 => FSPIQ) (Input Output)));
        _for_each_inner!((1, GPIO1(_2 => FSPICS0) (_2 => FSPICS0) (Input Output)));
        _for_each_inner!((2, GPIO2(_0 => MTMS _2 => FSPIWP) (_2 => FSPIWP) (Input
        Output))); _for_each_inner!((3, GPIO3(_0 => MTDI _2 => FSPIHD) (_2 => FSPIHD)
        (Input Output))); _for_each_inner!((4, GPIO4(_0 => MTCK _2 => FSPICLK) (_2 =>
        FSPICLK) (Input Output))); _for_each_inner!((5, GPIO5(_2 => FSPID) (_0 => MTDO _2
        => FSPID) (Input Output))); _for_each_inner!((6, GPIO6() () (Input Output)));
        _for_each_inner!((7, GPIO7() () (Input Output))); _for_each_inner!((8, GPIO8() ()
        (Input Output))); _for_each_inner!((9, GPIO9() () (Input Output)));
        _for_each_inner!((10, GPIO10() () (Input Output))); _for_each_inner!((11,
        GPIO11() () (Input Output))); _for_each_inner!((12, GPIO12() () (Input Output)));
        _for_each_inner!((13, GPIO13() () (Input Output))); _for_each_inner!((14,
        GPIO14() () (Input Output))); _for_each_inner!((22, GPIO22() () (Input Output)));
        _for_each_inner!((23, GPIO23(_0 => U0RXD) (_2 => FSPICS1) (Input Output)));
        _for_each_inner!((24, GPIO24() (_0 => U0TXD _2 => FSPICS2) (Input Output)));
        _for_each_inner!((25, GPIO25() (_2 => FSPICS3) (Input Output)));
        _for_each_inner!((26, GPIO26() (_2 => FSPICS4) (Input Output)));
        _for_each_inner!((27, GPIO27() (_2 => FSPICS5) (Input Output)));
        _for_each_inner!((all(0, GPIO0(_2 => FSPIQ) (_2 => FSPIQ) (Input Output)), (1,
        GPIO1(_2 => FSPICS0) (_2 => FSPICS0) (Input Output)), (2, GPIO2(_0 => MTMS _2 =>
        FSPIWP) (_2 => FSPIWP) (Input Output)), (3, GPIO3(_0 => MTDI _2 => FSPIHD) (_2 =>
        FSPIHD) (Input Output)), (4, GPIO4(_0 => MTCK _2 => FSPICLK) (_2 => FSPICLK)
        (Input Output)), (5, GPIO5(_2 => FSPID) (_0 => MTDO _2 => FSPID) (Input Output)),
        (6, GPIO6() () (Input Output)), (7, GPIO7() () (Input Output)), (8, GPIO8() ()
        (Input Output)), (9, GPIO9() () (Input Output)), (10, GPIO10() () (Input
        Output)), (11, GPIO11() () (Input Output)), (12, GPIO12() () (Input Output)),
        (13, GPIO13() () (Input Output)), (14, GPIO14() () (Input Output)), (22, GPIO22()
        () (Input Output)), (23, GPIO23(_0 => U0RXD) (_2 => FSPICS1) (Input Output)),
        (24, GPIO24() (_0 => U0TXD _2 => FSPICS2) (Input Output)), (25, GPIO25() (_2 =>
        FSPICS3) (Input Output)), (26, GPIO26() (_2 => FSPICS4) (Input Output)), (27,
        GPIO27() (_2 => FSPICS5) (Input Output))));
    };
}
/// This macro can be used to generate code for each analog function of each GPIO.
///
/// For an explanation on the general syntax, as well as usage of individual/repeated
/// matchers, refer to [for_each_gpio].
///
/// This macro has four options for its "Individual matcher" case:
///
/// - `($signal:ident, $gpio:ident)` - simple case where you only need identifiers
/// - `($signal:ident, ($gpio:ident, $gpio_num:literal))` - expanded GPIO case, where you need the
///   GPIO's number
/// - `(($signal:ident, $group:ident $(, $number:literal)*), $gpio:ident)` - expanded signal case,
///   where you need the number(s) of a signal, or the general group to which the signal belongs.
///   For example, in case of `ADC2_CH3` the expanded form looks like `(ADC2_CH3, ADCn_CHm, 2, 3)`.
/// - `($signal:ident, $gpio:ident)` - fully expanded case
#[macro_export]
macro_rules! for_each_analog_function {
    ($($pattern:tt => $code:tt;)*) => {
        macro_rules! _for_each_inner { $(($pattern) => $code;)* ($other : tt) => {} }
        _for_each_inner!((ADC1_CH0, GPIO1)); _for_each_inner!((ADC1_CH0, (GPIO1, 1)));
        _for_each_inner!(((ADC1_CH0, ADCn_CHm, 1, 0), GPIO1));
        _for_each_inner!(((ADC1_CH0, ADCn_CHm, 1, 0), (GPIO1, 1)));
        _for_each_inner!((ADC1_CH1, GPIO2)); _for_each_inner!((ADC1_CH1, (GPIO2, 2)));
        _for_each_inner!(((ADC1_CH1, ADCn_CHm, 1, 1), GPIO2));
        _for_each_inner!(((ADC1_CH1, ADCn_CHm, 1, 1), (GPIO2, 2)));
        _for_each_inner!((ADC1_CH2, GPIO3)); _for_each_inner!((ADC1_CH2, (GPIO3, 3)));
        _for_each_inner!(((ADC1_CH2, ADCn_CHm, 1, 2), GPIO3));
        _for_each_inner!(((ADC1_CH2, ADCn_CHm, 1, 2), (GPIO3, 3)));
        _for_each_inner!((ADC1_CH3, GPIO4)); _for_each_inner!((ADC1_CH3, (GPIO4, 4)));
        _for_each_inner!(((ADC1_CH3, ADCn_CHm, 1, 3), GPIO4));
        _for_each_inner!(((ADC1_CH3, ADCn_CHm, 1, 3), (GPIO4, 4)));
        _for_each_inner!((ADC1_CH4, GPIO5)); _for_each_inner!((ADC1_CH4, (GPIO5, 5)));
        _for_each_inner!(((ADC1_CH4, ADCn_CHm, 1, 4), GPIO5));
        _for_each_inner!(((ADC1_CH4, ADCn_CHm, 1, 4), (GPIO5, 5)));
        _for_each_inner!((ZCD0, GPIO10)); _for_each_inner!((ZCD0, (GPIO10, 10)));
        _for_each_inner!(((ZCD0, ZCDn, 0), GPIO10)); _for_each_inner!(((ZCD0, ZCDn, 0),
        (GPIO10, 10))); _for_each_inner!((ZCD1, GPIO11)); _for_each_inner!((ZCD1,
        (GPIO11, 11))); _for_each_inner!(((ZCD1, ZCDn, 1), GPIO11));
        _for_each_inner!(((ZCD1, ZCDn, 1), (GPIO11, 11))); _for_each_inner!((XTAL_32K_P,
        GPIO13)); _for_each_inner!((XTAL_32K_P, (GPIO13, 13)));
        _for_each_inner!((XTAL_32K_N, GPIO14)); _for_each_inner!((XTAL_32K_N, (GPIO14,
        14))); _for_each_inner!((USB_DM, GPIO26)); _for_each_inner!((USB_DM, (GPIO26,
        26))); _for_each_inner!((USB_DP, GPIO27)); _for_each_inner!((USB_DP, (GPIO27,
        27))); _for_each_inner!((all(ADC1_CH0, GPIO1), (ADC1_CH0, (GPIO1, 1)),
        ((ADC1_CH0, ADCn_CHm, 1, 0), GPIO1), ((ADC1_CH0, ADCn_CHm, 1, 0), (GPIO1, 1)),
        (ADC1_CH1, GPIO2), (ADC1_CH1, (GPIO2, 2)), ((ADC1_CH1, ADCn_CHm, 1, 1), GPIO2),
        ((ADC1_CH1, ADCn_CHm, 1, 1), (GPIO2, 2)), (ADC1_CH2, GPIO3), (ADC1_CH2, (GPIO3,
        3)), ((ADC1_CH2, ADCn_CHm, 1, 2), GPIO3), ((ADC1_CH2, ADCn_CHm, 1, 2), (GPIO3,
        3)), (ADC1_CH3, GPIO4), (ADC1_CH3, (GPIO4, 4)), ((ADC1_CH3, ADCn_CHm, 1, 3),
        GPIO4), ((ADC1_CH3, ADCn_CHm, 1, 3), (GPIO4, 4)), (ADC1_CH4, GPIO5), (ADC1_CH4,
        (GPIO5, 5)), ((ADC1_CH4, ADCn_CHm, 1, 4), GPIO5), ((ADC1_CH4, ADCn_CHm, 1, 4),
        (GPIO5, 5)), (ZCD0, GPIO10), (ZCD0, (GPIO10, 10)), ((ZCD0, ZCDn, 0), GPIO10),
        ((ZCD0, ZCDn, 0), (GPIO10, 10)), (ZCD1, GPIO11), (ZCD1, (GPIO11, 11)), ((ZCD1,
        ZCDn, 1), GPIO11), ((ZCD1, ZCDn, 1), (GPIO11, 11)), (XTAL_32K_P, GPIO13),
        (XTAL_32K_P, (GPIO13, 13)), (XTAL_32K_N, GPIO14), (XTAL_32K_N, (GPIO14, 14)),
        (USB_DM, GPIO26), (USB_DM, (GPIO26, 26)), (USB_DP, GPIO27), (USB_DP, (GPIO27,
        27))));
    };
}
/// This macro can be used to generate code for each LP/RTC function of each GPIO.
///
/// For an explanation on the general syntax, as well as usage of individual/repeated
/// matchers, refer to [for_each_gpio].
///
/// This macro has four options for its "Individual matcher" case:
///
/// - `($signal:ident, $gpio:ident)` - simple case where you only need identifiers
/// - `($signal:ident, ($gpio:ident, $gpio_num:literal))` - expanded GPIO case, where you need the
///   GPIO's number
/// - `(($signal:ident, $group:ident $(, $number:literal)*), $gpio:ident)` - expanded signal case,
///   where you need the number(s) of a signal, or the general group to which the signal belongs.
///   For example, in case of `SAR_I2C_SCL_1` the expanded form looks like `(SAR_I2C_SCL_1,
///   SAR_I2C_SCL_n, 1)`.
/// - `($signal:ident, $gpio:ident)` - fully expanded case
#[macro_export]
macro_rules! for_each_lp_function {
    ($($pattern:tt => $code:tt;)*) => {
        macro_rules! _for_each_inner { $(($pattern) => $code;)* ($other : tt) => {} }
        _for_each_inner!((all));
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
    (GPIO24, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO24, Output, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO24, $t:tt, $then_tt:tt else $else_tt:tt) => {
        $else_tt
    };
    (GPIO25, Input, $then_tt:tt else $else_tt:tt) => {
        $then_tt
    };
    (GPIO25, Output, $then_tt:tt else $else_tt:tt) => {
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
        #[allow(unused_braces)] $code } } else { $otherwise }), 22 =>
        if_pin_is_type!(GPIO22, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO22::steal() };
        #[allow(unused_braces)] $code } } else { $otherwise }), 23 =>
        if_pin_is_type!(GPIO23, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO23::steal() };
        #[allow(unused_braces)] $code } } else { $otherwise }), 24 =>
        if_pin_is_type!(GPIO24, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO24::steal() };
        #[allow(unused_braces)] $code } } else { $otherwise }), 25 =>
        if_pin_is_type!(GPIO25, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO25::steal() };
        #[allow(unused_braces)] $code } } else { $otherwise }), 26 =>
        if_pin_is_type!(GPIO26, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO26::steal() };
        #[allow(unused_braces)] $code } } else { $otherwise }), 27 =>
        if_pin_is_type!(GPIO27, $on_type, { { #[allow(unused_unsafe, unused_mut)] let mut
        $inner_ident = unsafe { crate ::peripherals::GPIO27::steal() };
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
            EXT_ADC_START       = 0,
            U0RXD               = 6,
            U0CTS               = 7,
            U0DSR               = 8,
            U1RXD               = 9,
            U1CTS               = 10,
            U1DSR               = 11,
            I2S_MCLK            = 12,
            I2SO_BCK            = 13,
            I2SO_WS             = 14,
            I2SI_SD             = 15,
            I2SI_BCK            = 16,
            I2SI_WS             = 17,
            USB_JTAG_TDO_BRIDGE = 19,
            CPU_GPIO0           = 28,
            CPU_GPIO1           = 29,
            CPU_GPIO2           = 30,
            CPU_GPIO3           = 31,
            CPU_GPIO4           = 32,
            CPU_GPIO5           = 33,
            CPU_GPIO6           = 34,
            CPU_GPIO7           = 35,
            I2CEXT0_SCL         = 45,
            I2CEXT0_SDA         = 46,
            PARL_RX_DATA0       = 47,
            PARL_RX_DATA1       = 48,
            PARL_RX_DATA2       = 49,
            PARL_RX_DATA3       = 50,
            PARL_RX_DATA4       = 51,
            PARL_RX_DATA5       = 52,
            PARL_RX_DATA6       = 53,
            PARL_RX_DATA7       = 54,
            I2CEXT1_SCL         = 55,
            I2CEXT1_SDA         = 56,
            FSPICLK             = 63,
            FSPIQ               = 64,
            FSPID               = 65,
            FSPIHD              = 66,
            FSPIWP              = 67,
            FSPICS0             = 68,
            PARL_RX_CLK         = 69,
            PARL_TX_CLK         = 70,
            RMT_SIG_0           = 71,
            RMT_SIG_1           = 72,
            TWAI0_RX            = 73,
            PWM0_SYNC0          = 87,
            PWM0_SYNC1          = 88,
            PWM0_SYNC2          = 89,
            PWM0_F0             = 90,
            PWM0_F1             = 91,
            PWM0_F2             = 92,
            PWM0_CAP0           = 93,
            PWM0_CAP1           = 94,
            PWM0_CAP2           = 95,
            SIG_FUNC_97         = 97,
            SIG_FUNC_98         = 98,
            SIG_FUNC_99         = 99,
            SIG_FUNC_100        = 100,
            PCNT0_SIG_CH0       = 101,
            PCNT0_SIG_CH1       = 102,
            PCNT0_CTRL_CH0      = 103,
            PCNT0_CTRL_CH1      = 104,
            PCNT1_SIG_CH0       = 105,
            PCNT1_SIG_CH1       = 106,
            PCNT1_CTRL_CH0      = 107,
            PCNT1_CTRL_CH1      = 108,
            PCNT2_SIG_CH0       = 109,
            PCNT2_SIG_CH1       = 110,
            PCNT2_CTRL_CH0      = 111,
            PCNT2_CTRL_CH1      = 112,
            PCNT3_SIG_CH0       = 113,
            PCNT3_SIG_CH1       = 114,
            PCNT3_CTRL_CH0      = 115,
            PCNT3_CTRL_CH1      = 116,
            SPIQ                = 121,
            SPID                = 122,
            SPIHD               = 123,
            SPIWP               = 124,
            MTDI,
            MTCK,
            MTMS,
        }
        #[allow(non_camel_case_types, clippy::upper_case_acronyms)]
        #[derive(Debug, PartialEq, Copy, Clone)]
        #[cfg_attr(feature = "defmt", derive(defmt::Format))]
        #[doc(hidden)]
        pub enum OutputSignal {
            LEDC_LS_SIG0     = 0,
            LEDC_LS_SIG1     = 1,
            LEDC_LS_SIG2     = 2,
            LEDC_LS_SIG3     = 3,
            LEDC_LS_SIG4     = 4,
            LEDC_LS_SIG5     = 5,
            U0TXD            = 6,
            U0RTS            = 7,
            U0DTR            = 8,
            U1TXD            = 9,
            U1RTS            = 10,
            U1DTR            = 11,
            I2S_MCLK         = 12,
            I2SO_BCK         = 13,
            I2SO_WS          = 14,
            I2SO_SD          = 15,
            I2SI_BCK         = 16,
            I2SI_WS          = 17,
            I2SO_SD1         = 18,
            USB_JTAG_TRST    = 19,
            CPU_GPIO_OUT0    = 28,
            CPU_GPIO_OUT1    = 29,
            CPU_GPIO_OUT2    = 30,
            CPU_GPIO_OUT3    = 31,
            CPU_GPIO_OUT4    = 32,
            CPU_GPIO_OUT5    = 33,
            CPU_GPIO_OUT6    = 34,
            CPU_GPIO_OUT7    = 35,
            I2CEXT0_SCL      = 45,
            I2CEXT0_SDA      = 46,
            PARL_TX_DATA0    = 47,
            PARL_TX_DATA1    = 48,
            PARL_TX_DATA2    = 49,
            PARL_TX_DATA3    = 50,
            PARL_TX_DATA4    = 51,
            PARL_TX_DATA5    = 52,
            PARL_TX_DATA6    = 53,
            PARL_TX_DATA7    = 54,
            I2CEXT1_SCL      = 55,
            I2CEXT1_SDA      = 56,
            FSPICLK          = 63,
            FSPIQ            = 64,
            FSPID            = 65,
            FSPIHD           = 66,
            FSPIWP           = 67,
            FSPICS0          = 68,
            PARL_RX_CLK      = 69,
            PARL_TX_CLK      = 70,
            RMT_SIG_0        = 71,
            RMT_SIG_1        = 72,
            TWAI0_TX         = 73,
            TWAI0_BUS_OFF_ON = 74,
            TWAI0_CLKOUT     = 75,
            TWAI0_STANDBY    = 76,
            CTE_ANT7         = 78,
            CTE_ANT8         = 79,
            CTE_ANT9         = 80,
            GPIO_SD0         = 83,
            GPIO_SD1         = 84,
            GPIO_SD2         = 85,
            GPIO_SD3         = 86,
            PWM0_0A          = 87,
            PWM0_0B          = 88,
            PWM0_1A          = 89,
            PWM0_1B          = 90,
            PWM0_2A          = 91,
            PWM0_2B          = 92,
            SIG_IN_FUNC97    = 97,
            SIG_IN_FUNC98    = 98,
            SIG_IN_FUNC99    = 99,
            SIG_IN_FUNC100   = 100,
            FSPICS1          = 101,
            FSPICS2          = 102,
            FSPICS3          = 103,
            FSPICS4          = 104,
            FSPICS5          = 105,
            CTE_ANT10        = 106,
            CTE_ANT11        = 107,
            CTE_ANT12        = 108,
            CTE_ANT13        = 109,
            CTE_ANT14        = 110,
            CTE_ANT15        = 111,
            SPICLK           = 114,
            SPICS0           = 115,
            SPICS1           = 116,
            SPIQ             = 121,
            SPID             = 122,
            SPIHD            = 123,
            SPIWP            = 124,
            CLK_OUT_OUT1     = 125,
            CLK_OUT_OUT2     = 126,
            CLK_OUT_OUT3     = 127,
            GPIO             = 128,
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
