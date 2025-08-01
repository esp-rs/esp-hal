PROVIDE( strdup = __esp_radio_strdup );
PROVIDE( g_wifi_osi_funcs = __ESP_RADIO_G_WIFI_OSI_FUNCS );
PROVIDE( g_wifi_feature_caps = __ESP_RADIO_G_WIFI_FEATURE_CAPS );
PROVIDE( WIFI_EVENT = __ESP_RADIO_WIFI_EVENT );
PROVIDE( gettimeofday = __esp_radio_gettimeofday );
PROVIDE( esp_fill_random = __esp_radio_esp_fill_random );
PROVIDE( strrchr = __esp_radio_strrchr );
PROVIDE( esp_dport_access_reg_read = __esp_radio_esp_dport_access_reg_read );
PROVIDE( rtc_get_xtal = __esp_radio_rtc_get_xtal );
PROVIDE( putchar = __esp_radio_putchar );
PROVIDE( _putchar = __esp_radio_putchar );
PROVIDE( fwrite = __esp_radio_fwrite );
PROVIDE( fopen = __esp_radio_fopen );
PROVIDE( fgets = __esp_radio_fgets );
PROVIDE( fclose = __esp_radio_fclose );
PROVIDE( strdup = __esp_radio_strdup );

#IF wifi
PROVIDE( esp_timer_get_time = __esp_radio_esp_timer_get_time );
PROVIDE( misc_nvs_deinit = __esp_radio_misc_nvs_deinit );
PROVIDE( misc_nvs_init = __esp_radio_misc_nvs_init );
PROVIDE( g_log_level = __ESP_RADIO_G_LOG_LEVEL );
PROVIDE( sleep = __esp_radio_sleep );
PROVIDE( usleep = __esp_radio_usleep );
#ENDIF
