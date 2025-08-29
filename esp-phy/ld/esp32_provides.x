EXTERN( __esp_phy_enter_critical );
EXTERN( __esp_phy_exit_critical );

PROVIDE( phy_enter_critical = __esp_phy_enter_critical );
PROVIDE( phy_exit_critical = __esp_phy_exit_critical );
