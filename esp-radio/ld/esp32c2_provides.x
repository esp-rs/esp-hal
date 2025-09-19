EXTERN( __esp_radio_strdup );
EXTERN( __ESP_RADIO_G_WIFI_OSI_FUNCS );
EXTERN( __ESP_RADIO_G_WIFI_FEATURE_CAPS );
EXTERN( __ESP_RADIO_WIFI_EVENT );
EXTERN( __esp_radio_gettimeofday );
EXTERN( __esp_radio_esp_fill_random );
EXTERN( __esp_radio_strrchr );
EXTERN( __esp_radio_putchar );
EXTERN( __esp_radio_putchar );
EXTERN( __esp_radio_fwrite );
EXTERN( __esp_radio_fopen );
EXTERN( __esp_radio_fgets );
EXTERN( __esp_radio_fclose );
EXTERN( __esp_radio_esp_timer_get_time );
EXTERN( __esp_radio_esp_event_post );
EXTERN( __esp_radio_vTaskDelay );
EXTERN( __esp_radio_puts );
EXTERN( __esp_radio_sleep );
EXTERN( __esp_radio_usleep );

PROVIDE( strdup = __esp_radio_strdup );
PROVIDE( g_wifi_osi_funcs = __ESP_RADIO_G_WIFI_OSI_FUNCS );
PROVIDE( g_wifi_feature_caps = __ESP_RADIO_G_WIFI_FEATURE_CAPS );
PROVIDE( WIFI_EVENT = __ESP_RADIO_WIFI_EVENT );
PROVIDE( gettimeofday = __esp_radio_gettimeofday );
PROVIDE( esp_fill_random = __esp_radio_esp_fill_random );
PROVIDE( strrchr = __esp_radio_strrchr );
PROVIDE( putchar = __esp_radio_putchar );
PROVIDE( _putchar = __esp_radio_putchar );
PROVIDE( fwrite = __esp_radio_fwrite );
PROVIDE( fopen = __esp_radio_fopen );
PROVIDE( fgets = __esp_radio_fgets );
PROVIDE( fclose = __esp_radio_fclose );
PROVIDE( esp_timer_get_time = __esp_radio_esp_timer_get_time );
PROVIDE( esp_event_post = __esp_radio_esp_event_post );
PROVIDE( vTaskDelay = __esp_radio_vTaskDelay );
PROVIDE( puts = __esp_radio_puts );
PROVIDE( sleep = __esp_radio_sleep );
PROVIDE( usleep = __esp_radio_usleep );

#IF wifi
EXTERN( __esp_radio_misc_nvs_deinit );
EXTERN( __esp_radio_misc_nvs_init );
EXTERN( __ESP_RADIO_G_LOG_LEVEL );
EXTERN( __esp_radio_sleep );
EXTERN( __esp_radio_usleep );

PROVIDE( misc_nvs_deinit = __esp_radio_misc_nvs_deinit );
PROVIDE( misc_nvs_init = __esp_radio_misc_nvs_init );
PROVIDE( g_log_level = __ESP_RADIO_G_LOG_LEVEL );
PROVIDE( sleep = __esp_radio_sleep );
PROVIDE( usleep = __esp_radio_usleep );
#ENDIF
