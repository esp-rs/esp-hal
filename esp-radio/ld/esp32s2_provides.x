/* force inclusion from libs */
EXTERN( g_espnow_user_oui );

EXTERN( __esp_radio_strdup );
EXTERN( __esp_radio_esp_timer_get_time );
EXTERN( __ESP_RADIO_G_WIFI_OSI_FUNCS );
EXTERN( __ESP_RADIO_G_WIFI_FEATURE_CAPS );
EXTERN( __ESP_RADIO_WIFI_EVENT );
EXTERN( __esp_radio_gettimeofday );
EXTERN( __esp_radio_esp_fill_random );
EXTERN( __esp_radio_strrchr );
EXTERN( __esp_radio_misc_nvs_deinit );
EXTERN( __esp_radio_misc_nvs_init );
EXTERN( __esp_radio_misc_nvs_restore );
EXTERN( __ESP_RADIO_G_LOG_LEVEL );
EXTERN( __ESP_RADIO_G_MISC_NVS );


EXTERN( __esp_radio_putchar );
EXTERN( __esp_radio_putchar );
EXTERN( __esp_radio_fwrite );
EXTERN( __esp_radio_fopen );
EXTERN( __esp_radio_fgets );
EXTERN( __esp_radio_fclose );
EXTERN( __esp_radio_sleep );
EXTERN( __esp_radio_usleep );

EXTERN( __esp_radio_esp_event_post );
EXTERN( __esp_radio_vTaskDelay );
EXTERN( __esp_radio_puts );

PROVIDE( strdup = __esp_radio_strdup );
PROVIDE( esp_timer_get_time = __esp_radio_esp_timer_get_time );
PROVIDE( g_wifi_osi_funcs = __ESP_RADIO_G_WIFI_OSI_FUNCS );
PROVIDE( g_wifi_feature_caps = __ESP_RADIO_G_WIFI_FEATURE_CAPS );
PROVIDE( WIFI_EVENT = __ESP_RADIO_WIFI_EVENT );
PROVIDE( gettimeofday = __esp_radio_gettimeofday );
PROVIDE( esp_fill_random = __esp_radio_esp_fill_random );
PROVIDE( strrchr = __esp_radio_strrchr );
PROVIDE( misc_nvs_deinit = __esp_radio_misc_nvs_deinit );
PROVIDE( misc_nvs_init = __esp_radio_misc_nvs_init );
PROVIDE( misc_nvs_restore = __esp_radio_misc_nvs_restore );
PROVIDE( g_log_level = __ESP_RADIO_G_LOG_LEVEL );
PROVIDE( g_misc_nvs = __ESP_RADIO_G_MISC_NVS );


PROVIDE( putchar = __esp_radio_putchar );
PROVIDE( _putchar = __esp_radio_putchar );
PROVIDE( fwrite = __esp_radio_fwrite );
PROVIDE( fopen = __esp_radio_fopen );
PROVIDE( fgets = __esp_radio_fgets );
PROVIDE( fclose = __esp_radio_fclose );
PROVIDE( sleep = __esp_radio_sleep );
PROVIDE( usleep = __esp_radio_usleep );

PROVIDE( esp_event_post = __esp_radio_esp_event_post );
PROVIDE( vTaskDelay = __esp_radio_vTaskDelay );
PROVIDE( puts = __esp_radio_puts );
