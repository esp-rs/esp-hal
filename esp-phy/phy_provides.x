EXTERN( __esp_phy_enter_critical );
EXTERN( __esp_phy_exit_critical );
EXTERN( __esp_phy_esp_dport_access_reg_read );
EXTERN( __esp_phy_rtc_get_xtal );

// If this isn't included, linking fails on the S3.
EXTERN( bt_bb_v2_init_cmplx );

PROVIDE( phy_enter_critical = __esp_phy_enter_critical );
PROVIDE( phy_exit_critical = __esp_phy_exit_critical );
PROVIDE( esp_dport_access_reg_read = __esp_phy_esp_dport_access_reg_read );
PROVIDE( rtc_get_xtal = __esp_phy_rtc_get_xtal );
