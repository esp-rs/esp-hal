//! FTM PHY compensation constants used by the Wi-Fi blob.
//!
//! Values come from ESP-IDF `esp_private/ftm_calibration_data.h` / `ftm_load_calibration.c`.

#![allow(non_upper_case_globals)]

macro_rules! ftm_u16 {
    ($($name:ident = $val:expr;)*) => {
        $(
            #[unsafe(no_mangle)]
            #[used]
            pub static mut $name: u16 = $val;
        )*
    };
}

#[cfg(esp32s2)]
mod values {
    pub const I_20_20U: u16 = 367;
    pub const R_20_20U: u16 = 366;
    pub const I_20_20D: u16 = 374;
    pub const R_20_20D: u16 = 359;
    pub const I_20_20U_DIS: u16 = 364;
    pub const I_20_20D_DIS: u16 = 373;
    pub const R_20_20U_DIS: u16 = 366;
    pub const R_20_20D_DIS: u16 = 359;
    pub const I_20_40U: u16 = 229;
    pub const R_20_40U: u16 = 363;
    pub const I_20_40D: u16 = 234;
    pub const R_20_40D: u16 = 356;
    pub const I_20_40U_DIS: u16 = 221;
    pub const R_20_40U_DIS: u16 = 222;
    pub const I_20_40D_DIS: u16 = 228;
    pub const R_20_40D_DIS: u16 = 216;
    pub const I_40_40U: u16 = 403;
    pub const R_40_40U: u16 = 42;
    pub const I_40_40D: u16 = 403;
    pub const R_40_40D: u16 = 41;
    pub const I_40_40U_DIS: u16 = 403;
    pub const R_40_40U_DIS: u16 = 42;
    pub const I_40_40D_DIS: u16 = 402;
    pub const R_40_40D_DIS: u16 = 41;
}

#[cfg(esp32c3)]
mod values {
    pub const I_20_20U: u16 = 451;
    pub const R_20_20U: u16 = 439;
    pub const I_20_20D: u16 = 454;
    pub const R_20_20D: u16 = 431;
    pub const I_20_20U_DIS: u16 = 448;
    pub const I_20_20D_DIS: u16 = 455;
    pub const R_20_20U_DIS: u16 = 441;
    pub const R_20_20D_DIS: u16 = 433;
    pub const I_20_40U: u16 = 260;
    pub const R_20_40U: u16 = 252;
    pub const I_20_40D: u16 = 268;
    pub const R_20_40D: u16 = 247;
    pub const I_20_40U_DIS: u16 = 267;
    pub const R_20_40U_DIS: u16 = 259;
    pub const I_20_40D_DIS: u16 = 270;
    pub const R_20_40D_DIS: u16 = 250;
    pub const I_40_40U: u16 = 443;
    pub const R_40_40U: u16 = 82;
    pub const I_40_40D: u16 = 440;
    pub const R_40_40D: u16 = 80;
    pub const I_40_40U_DIS: u16 = 439;
    pub const R_40_40U_DIS: u16 = 86;
    pub const I_40_40D_DIS: u16 = 440;
    pub const R_40_40D_DIS: u16 = 81;
}

#[cfg(esp32c2)]
mod values {
    pub const I_20_20U: u16 = 443;
    pub const R_20_20U: u16 = 441;
    pub const I_20_20D: u16 = 451;
    pub const R_20_20D: u16 = 432;
    pub const I_20_20U_DIS: u16 = 443;
    pub const I_20_20D_DIS: u16 = 452;
    pub const R_20_20U_DIS: u16 = 441;
    pub const R_20_20D_DIS: u16 = 432;
}

#[cfg(esp32s3)]
mod values {
    pub const I_20_20U: u16 = 453;
    pub const R_20_20U: u16 = 444;
    pub const I_20_20D: u16 = 453;
    pub const R_20_20D: u16 = 440;
    pub const I_20_20U_DIS: u16 = 453;
    pub const I_20_20D_DIS: u16 = 454;
    pub const R_20_20U_DIS: u16 = 444;
    pub const R_20_20D_DIS: u16 = 440;
    pub const I_20_40U: u16 = 271;
    pub const R_20_40U: u16 = 262;
    pub const I_20_40D: u16 = 270;
    pub const R_20_40D: u16 = 261;
    pub const I_20_40U_DIS: u16 = 269;
    pub const R_20_40U_DIS: u16 = 262;
    pub const I_20_40D_DIS: u16 = 270;
    pub const R_20_40D_DIS: u16 = 258;
    pub const I_40_40U: u16 = 443;
    pub const R_40_40U: u16 = 88;
    pub const I_40_40D: u16 = 443;
    pub const R_40_40D: u16 = 84;
    pub const I_40_40U_DIS: u16 = 442;
    pub const R_40_40U_DIS: u16 = 89;
    pub const I_40_40D_DIS: u16 = 445;
    pub const R_40_40D_DIS: u16 = 82;
}

#[cfg(esp32c6)]
mod values {
    pub const I_20_20U: u16 = 704;
    pub const R_20_20U: u16 = 714;
    pub const I_20_20D: u16 = 707;
    pub const R_20_20D: u16 = 712;
    pub const I_20_20U_DIS: u16 = 711;
    pub const I_20_20D_DIS: u16 = 717;
    pub const R_20_20U_DIS: u16 = 706;
    pub const R_20_20D_DIS: u16 = 702;
    pub const I_20_40U: u16 = 601;
    pub const R_20_40U: u16 = 609;
    pub const I_20_40D: u16 = 605;
    pub const R_20_40D: u16 = 608;
    pub const I_20_40U_DIS: u16 = 610;
    pub const R_20_40U_DIS: u16 = 614;
    pub const I_20_40D_DIS: u16 = 616;
    pub const R_20_40D_DIS: u16 = 611;
    pub const I_40_40U: u16 = 786;
    pub const R_40_40U: u16 = 436;
    pub const I_40_40D: u16 = 791;
    pub const R_40_40D: u16 = 436;
    pub const I_40_40U_DIS: u16 = 786;
    pub const R_40_40U_DIS: u16 = 436;
    pub const I_40_40D_DIS: u16 = 794;
    pub const R_40_40D_DIS: u16 = 433;
}

#[cfg(esp32c61)]
mod values {
    pub const I_20_20U: u16 = 854;
    pub const R_20_20U: u16 = 869;
    pub const I_20_20D: u16 = 857;
    pub const R_20_20D: u16 = 864;
    pub const I_20_20U_DIS: u16 = 854;
    pub const I_20_20D_DIS: u16 = 857;
    pub const R_20_20U_DIS: u16 = 869;
    pub const R_20_20D_DIS: u16 = 864;
    pub const I_20_40U: u16 = 739;
    pub const R_20_40U: u16 = 753;
    pub const I_20_40D: u16 = 740;
    pub const R_20_40D: u16 = 747;
    pub const I_20_40U_DIS: u16 = 739;
    pub const R_20_40U_DIS: u16 = 753;
    pub const I_20_40D_DIS: u16 = 740;
    pub const R_20_40D_DIS: u16 = 747;
    pub const I_40_40U: u16 = 922;
    pub const R_40_40U: u16 = 582;
    pub const I_40_40D: u16 = 932;
    pub const R_40_40D: u16 = 572;
    pub const I_40_40U_DIS: u16 = 922;
    pub const R_40_40U_DIS: u16 = 592;
    pub const I_40_40D_DIS: u16 = 932;
    pub const R_40_40D_DIS: u16 = 572;
}

#[cfg(esp32c5)]
mod values {
    pub const I_20_20U: u16 = 931;
    pub const R_20_20U: u16 = 972;
    pub const I_20_20D: u16 = 937;
    pub const R_20_20D: u16 = 918;
    pub const I_20_20U_DIS: u16 = 972;
    pub const I_20_20D_DIS: u16 = 937;
    pub const R_20_20U_DIS: u16 = 974;
    pub const R_20_20D_DIS: u16 = 918;
    pub const I_20_40U: u16 = 817;
    pub const R_20_40U: u16 = 815;
    pub const I_20_40D: u16 = 822;
    pub const R_20_40D: u16 = 833;
    pub const I_20_40U_DIS: u16 = 812;
    pub const R_20_40U_DIS: u16 = 808;
    pub const I_20_40D_DIS: u16 = 822;
    pub const R_20_40D_DIS: u16 = 807;
    pub const I_40_40U: u16 = 988;
    pub const R_40_40U: u16 = 640;
    pub const I_40_40D: u16 = 993;
    pub const R_40_40D: u16 = 629;
    pub const I_40_40U_DIS: u16 = 988;
    pub const R_40_40U_DIS: u16 = 640;
    pub const I_40_40D_DIS: u16 = 993;
    pub const R_40_40D_DIS: u16 = 629;
}

ftm_u16! {
    est_PHY_INIT_FTM_COMP_20_20U_MHZ = values::I_20_20U;
    est_PHY_RESP_FTM_COMP_20_20U_MHZ = values::R_20_20U;
    est_PHY_INIT_FTM_COMP_20_20D_MHZ = values::I_20_20D;
    est_PHY_RESP_FTM_COMP_20_20D_MHZ = values::R_20_20D;
    est_PHY_INIT_FTM_COMP_20_20U_MHZ_DIS = values::I_20_20U_DIS;
    est_PHY_INIT_FTM_COMP_20_20D_MHZ_DIS = values::I_20_20D_DIS;
    est_PHY_RESP_FTM_COMP_20_20U_MHZ_DIS = values::R_20_20U_DIS;
    est_PHY_RESP_FTM_COMP_20_20D_MHZ_DIS = values::R_20_20D_DIS;
}

#[cfg(not(esp32c2))]
ftm_u16! {
    est_PHY_INIT_FTM_COMP_40_40U_MHZ = values::I_40_40U;
    est_PHY_RESP_FTM_COMP_40_40U_MHZ = values::R_40_40U;
    est_PHY_INIT_FTM_COMP_40_40D_MHZ = values::I_40_40D;
    est_PHY_RESP_FTM_COMP_40_40D_MHZ = values::R_40_40D;
    est_PHY_INIT_FTM_COMP_20_40U_MHZ = values::I_20_40U;
    est_PHY_RESP_FTM_COMP_20_40U_MHZ = values::R_20_40U;
    est_PHY_INIT_FTM_COMP_20_40D_MHZ = values::I_20_40D;
    est_PHY_RESP_FTM_COMP_20_40D_MHZ = values::R_20_40D;
    est_PHY_INIT_FTM_COMP_20_40U_MHZ_DIS = values::I_20_40U_DIS;
    est_PHY_RESP_FTM_COMP_20_40U_MHZ_DIS = values::R_20_40U_DIS;
    est_PHY_INIT_FTM_COMP_20_40D_MHZ_DIS = values::I_20_40D_DIS;
    est_PHY_RESP_FTM_COMP_20_40D_MHZ_DIS = values::R_20_40D_DIS;
    est_PHY_INIT_FTM_COMP_40_40U_MHZ_DIS = values::I_40_40U_DIS;
    est_PHY_RESP_FTM_COMP_40_40U_MHZ_DIS = values::R_40_40U_DIS;
    est_PHY_INIT_FTM_COMP_40_40D_MHZ_DIS = values::I_40_40D_DIS;
    est_PHY_RESP_FTM_COMP_40_40D_MHZ_DIS = values::R_40_40D_DIS;
}

#[cfg(esp32c5)]
ftm_u16! {
    est_PHY_INIT_FTM_COMP_20_20_MHZ_5G = 975;
    est_PHY_INIT_FTM_COMP_20_20_MHZ_5G_DIS = 976;
    est_PHY_RESP_FTM_COMP_20_20_MHZ_5G = 966;
    est_PHY_RESP_FTM_COMP_20_20_MHZ_5G_DIS = 966;
    est_PHY_INIT_FTM_COMP_20_40_MHZ_5G = 798;
    est_PHY_INIT_FTM_COMP_20_40_MHZ_5G_DIS = 840;
    est_PHY_RESP_FTM_COMP_20_40_MHZ_5G = 966;
    est_PHY_RESP_FTM_COMP_20_40_MHZ_5G_DIS = 828;
    est_PHY_INIT_FTM_COMP_40_40_MHZ_5G = 825;
    est_PHY_INIT_FTM_COMP_40_40_MHZ_5G_DIS = 823;
    est_PHY_RESP_FTM_COMP_40_40_MHZ_5G = 840;
    est_PHY_RESP_FTM_COMP_40_40_MHZ_5G_DIS = 840;
}
