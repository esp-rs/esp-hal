use esp_hal_common::gpio::gpio;

use crate::hal_gpio::*;

type OutputSignalType = u16;
const OUTPUT_SIGNAL_MAX: u16 = 548;
const INPUT_SIGNAL_MAX: u16 = 539;

/// Peripheral input signals for the GPIO mux
#[allow(non_camel_case_types)]
#[derive(PartialEq, Copy, Clone)]
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
    I2CM_SDA              = 20,
    EXT_I2C_SDA           = 22,
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
    GPIO_BT_ACTIVE        = 37,
    GPIO_BT_PRIORITY      = 38,
    PCNT_SIG_CH0_0        = 39,
    PCNT_SIG_CH1_0        = 40,
    PCNT_CTRL_CH0_0       = 41,
    PCNT_CTRL_CH1_0       = 42,
    PCNT_SIG_CH0_1        = 43,
    PCNT_SIG_CH1_1        = 44,
    PCNT_CTRL_CH0_1       = 45,
    PCNT_CTRL_CH1_1       = 46,
    PCNT_SIG_CH0_2        = 47,
    PCNT_SIG_CH1_2        = 48,
    PCNT_CTRL_CH0_2       = 49,
    PCNT_CTRL_CH1_2       = 50,
    PCNT_SIG_CH0_3        = 51,
    PCNT_SIG_CH1_3        = 52,
    PCNT_CTRL_CH0_3       = 53,
    PCNT_CTRL_CH1_3       = 54,
    PCNT_SIG_CH0_4        = 55,
    PCNT_SIG_CH1_4        = 56,
    PCNT_CTRL_CH0_4       = 57,
    PCNT_CTRL_CH1_4       = 58,
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
    PCNT_SIG_CH0_5        = 71,
    PCNT_SIG_CH1_5        = 72,
    PCNT_CTRL_CH0_5       = 73,
    PCNT_CTRL_CH1_5       = 74,
    PCNT_SIG_CH0_6        = 75,
    PCNT_SIG_CH1_6        = 76,
    PCNT_CTRL_CH0_6       = 77,
    PCNT_CTRL_CH1_6       = 78,
    PCNT_SIG_CH0_7        = 79,
    PCNT_SIG_CH1_7        = 80,
    PCNT_CTRL_CH0_7       = 81,
    PCNT_CTRL_CH1_7       = 82,
    RMT_SIG_0             = 83,
    RMT_SIG_1             = 84,
    RMT_SIG_2             = 85,
    RMT_SIG_3             = 86,
    RMT_SIG_4             = 87,
    RMT_SIG_5             = 88,
    RMT_SIG_6             = 89,
    RMT_SIG_7             = 90,
    EXT_ADC_START         = 93,
    CAN_RX                = 94,
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
    PWM2_FLTA             = 115,
    PWM2_FLTB             = 116,
    PWM2_CAP1             = 117,
    PWM2_CAP2             = 118,
    PWM2_CAP3             = 119,
    PWM3_FLTA             = 120,
    PWM3_FLTB             = 121,
    PWM3_CAP1             = 122,
    PWM3_CAP2             = 123,
    PWM3_CAP3             = 124,
    CAN_CLKOUT            = 125,
    SPID4                 = 128,
    SPID5                 = 129,
    SPID6                 = 130,
    SPID7                 = 131,
    HSPID4                = 132,
    HSPID5                = 133,
    HSPID6                = 134,
    HSPID7                = 135,
    VSPID4                = 136,
    VSPID5                = 137,
    VSPID6                = 138,
    VSPID7                = 139,
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
    SIG_IN_FUNC224        = 224,
    SIG_IN_FUNC225        = 225,
    SIG_IN_FUNC226        = 226,
    SIG_IN_FUNC227        = 227,
    SIG_IN_FUNC228        = 228,

    SD_DATA0              = 512,
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

/// Peripheral output signals for the GPIO mux
#[allow(non_camel_case_types)]
#[derive(PartialEq, Copy, Clone)]
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
    I2CM_SCL                 = 19,
    I2CM_SDA                 = 20,
    EXT2C_SCL                = 21,
    EXT2C_SDA                = 22,
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
    GPIO_WLAN_ACTIVE         = 40,
    BB_DIAG0                 = 41,
    BB_DIAG1                 = 42,
    BB_DIAG2                 = 43,
    BB_DIAG3                 = 44,
    BB_DIAG4                 = 45,
    BB_DIAG5                 = 46,
    BB_DIAG6                 = 47,
    BB_DIAG7                 = 48,
    BB_DIAG8                 = 49,
    BB_DIAG9                 = 50,
    BB_DIAG10                = 51,
    BB_DIAG11                = 52,
    BB_DIAG12                = 53,
    BB_DIAG13                = 54,
    BB_DIAG14                = 55,
    BB_DIAG15                = 56,
    BB_DIAG16                = 57,
    BB_DIAG17                = 58,
    BB_DIAG18                = 59,
    BB_DIAG19                = 60,
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
    LEDC_HS_SIG_0            = 71,
    LEDC_HS_SIG_1            = 72,
    LEDC_HS_SIG_2            = 73,
    LEDC_HS_SIG_3            = 74,
    LEDC_HS_SIG_4            = 75,
    LEDC_HS_SIG_5            = 76,
    LEDC_HS_SIG_6            = 77,
    LEDC_HS_SIG_7            = 78,
    LEDC_LS_SIG_0            = 79,
    LEDC_LS_SIG_1            = 80,
    LEDC_LS_SIG_2            = 81,
    LEDC_LS_SIG_3            = 82,
    LEDC_LS_SIG_4            = 83,
    LEDC_LS_SIG_5            = 84,
    LEDC_LS_SIG_6            = 85,
    LEDC_LS_SIG_7            = 86,
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
    PWM2_1H                  = 114,
    PWM2_1L                  = 115,
    PWM2_2H                  = 116,
    PWM2_2L                  = 117,
    PWM2_3H                  = 118,
    PWM2_3L                  = 119,
    PWM2_4H                  = 120,
    PWM2_4L                  = 121,
    CAN_TX                   = 123,
    CAN_BUS_OFF_ON           = 124,
    SPID4                    = 128,
    SPID5                    = 129,
    SPID6                    = 130,
    SPID7                    = 131,
    HSPID4                   = 132,
    HSPID5                   = 133,
    HSPID6                   = 134,
    HSPID7                   = 135,
    VSPID4                   = 136,
    VSPID5                   = 137,
    VSPID6                   = 138,
    VSPID7                   = 139,
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
    PWM3_1H                  = 190,
    PWM3_1L                  = 191,
    PWM3_2H                  = 192,
    PWM3_2L                  = 193,
    PWM3_3H                  = 194,
    PWM3_3L                  = 195,
    PWM3_4H                  = 196,
    PWM3_4L                  = 197,
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

    CLK_OUT1                 = 512,
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

gpio! {
    Function2,
    DualCore,
    Gpio0: (gpio0, 0, gpio0, IO, RTC, Bank0), (EMAC_TX_CLK: Function5), (CLK_OUT1: Function1),
    Gpio1: (gpio1, 1, gpio1, IO, 0, Bank0), (EMAC_RXD2: Function5), (U0TXD: Function1, CLK_OUT3: Function1),
    Gpio2: (gpio2, 2, gpio2, IO, RTC, Bank0), (HSPIWP: Function1, HS2_DATA0: Function3, SD_DATA0: Function4), (HS2_DATA0: Function3, SD_DATA0: Function4),
    Gpio3: (gpio3, 3, gpio3, IO, 0, Bank0), (U0RXD: Function0), (CLK_OUT2: Function1),
    Gpio4: (gpio4, 4, gpio4, IO, RTC, Bank0), (HSPIHD: Function1, HS2_DATA1: Function3, SD_DATA1: Function4, EMAC_TX_ER: Function5), (HS2_DATA1: Function3, SD_DATA1: Function4),
    Gpio5: (gpio5, 5, gpio5, IO, 0, Bank0), (VSPICS0: Function1, HS1_DATA6: Function3, EMAC_RX_CLK: Function5), (HS1_DATA6: Function3),
    Gpio6: (gpio6, 6, gpio6, IO, 0, Bank0), (U1CTS: Function4), (SD_CLK: Function0, SPICLK: Function1, HS1_CLK: Function3),
    Gpio7: (gpio7, 7, gpio7, IO, 0, Bank0), (SD_DATA0: Function0, SPIQ: Function1, HS1_DATA0: Function3), (SD_DATA0: Function0, SPIQ: Function1, HS1_DATA0: Function3, U2RTS: Function4),
    Gpio8: (gpio8, 8, gpio8, IO, 0, Bank0), (SD_DATA1: Function0, SPID: Function1, HS1_DATA1: Function3, U2CTS: Function4), (SD_DATA1: Function0, SPID: Function1, HS1_DATA1: Function3),
    Gpio9: (gpio9, 9, gpio9, IO, 0, Bank0), (SD_DATA2: Function0, SPIHD: Function1, HS1_DATA2: Function3, U1RXD: Function4), (SD_DATA2: Function0, SPIHD: Function1, HS1_DATA2: Function3),
    Gpio10: (gpio10, 10, gpio10, IO, 0, Bank0), (SD_DATA3: Function0, SPIWP: Function1, HS1_DATA3: Function3), (SD_DATA3: Function0, SPIWP: Function1, HS1_DATA3: Function3, U1TXD: Function4),
    Gpio11: (gpio11, 11, gpio11, IO, 0, Bank0), (SPICS0: Function1), (SD_CMD: Function0, SPICS0: Function1, HS1_CMD: Function3, U1RTS: Function4),
    Gpio12: (gpio12, 12, gpio12, IO, RTC, Bank0), (MTDI: Function0, HSPIQ: Function1, HS2_DATA2: Function3, SD_DATA2: Function4), (HSPIQ: Function1, HS2_DATA2: Function3, SD_DATA2: Function4, EMAC_TXD3: Function5),
    Gpio13: (gpio13, 13, gpio13, IO, RTC, Bank0), (MTCK: Function0, HSPID: Function1, HS2_DATA3: Function3, SD_DATA3: Function4), (HSPID: Function1, HS2_DATA3: Function3, SD_DATA3: Function4, EMAC_RX_ER: Function5),
    Gpio14: (gpio14, 14, gpio14, IO, RTC, Bank0), (MTMS: Function0, HSPICLK: Function1), (HSPICLK: Function1, HS2_CLK: Function3, SD_CLK: Function4, EMAC_TXD2: Function5),
    Gpio15: (gpio15, 15, gpio15, IO, RTC, Bank0), (HSPICS0: Function1, EMAC_RXD3: Function5), (MTDO: Function0, HSPICS0: Function1, HS2_CMD: Function3, SD_CMD: Function4),
    Gpio16: (gpio16, 16, gpio16, IO, 0, Bank0), (HS1_DATA4: Function3, U2RXD: Function4), (HS1_DATA4: Function3, EMAC_CLK_OUT: Function5),
    Gpio17: (gpio17, 17, gpio17, IO, 0, Bank0), (HS1_DATA5: Function3), (HS1_DATA5: Function3, U2TXD: Function4, EMAC_CLK_180: Function5),
    Gpio18: (gpio18, 18, gpio18, IO, 0, Bank0), (VSPICLK: Function1, HS1_DATA7: Function3), (VSPICLK: Function1, HS1_DATA7: Function3),
    Gpio19: (gpio19, 19, gpio19, IO, 0, Bank0), (VSPIQ: Function1, U0CTS: Function3), (VSPIQ: Function1, EMAC_TXD0: Function5),
    Gpio20: (gpio20, 20, gpio20, IO, 0, Bank0),
    Gpio21: (gpio21, 21, gpio21, IO, 0, Bank0), (VSPIHD: Function1), (VSPIHD: Function1, EMAC_TX_EN: Function5),

    Gpio22: (gpio22, 22, gpio22, IO, 0, Bank0), (VSPIWP: Function1), (VSPIWP: Function1, U0RTS: Function3, EMAC_TXD1: Function5),
    Gpio23: (gpio23, 23, gpio23, IO, 0, Bank0), (VSPID: Function1), (VSPID: Function1, HS1_STROBE: Function3),
    Gpio24: (gpio24, 24, gpio24, IO, 0, Bank0),
    Gpio25: (gpio25, 25, gpio25, IO, 0, Bank0), (EMAC_RXD0: Function5), (),
    Gpio26: (gpio26, 26, gpio26, IO, 0, Bank0), (EMAC_RXD1: Function5), (),
    Gpio27: (gpio27, 27, gpio27, IO, 0, Bank0), (EMAC_RX_DV: Function5), (),

    Gpio32: (gpio32, 32, gpio32, IO, 0, Bank1),
    Gpio33: (gpio33, 33, gpio33, IO, 0, Bank1),
    Gpio34: (gpio34, 34, gpio34, Input, 0, Bank1),
    Gpio35: (gpio35, 35, gpio35, Input, 0, Bank1),
    Gpio36: (gpio36, 36, gpio36, Input, 0, Bank1),
    Gpio37: (gpio37, 37, gpio37, Input, 0, Bank1),
    Gpio38: (gpio38, 38, gpio38, Input, 0, Bank1),
    Gpio39: (gpio39, 39, gpio39, Input, 0, Bank1),
}
