#[embedded_test::tests(default_timeout = 60, executor = hil_test::Executor::new())]
mod tests {
    use embassy_net::{
        Runner,
        StackResources,
        dns::DnsSocket,
        tcp::client::{TcpClient, TcpClientState},
    };
    use embassy_time::{Duration, Timer};
    use esp_hal::{
        clock::CpuClock,
        interrupt::software::SoftwareInterruptControl,
        peripherals::Peripherals,
        rng::Rng,
        timer::timg::TimerGroup,
    };
    use esp_radio::wifi::{
        Config,
        ControllerConfig,
        Interface,
        WifiController,
        scan::ScanConfig,
        sta::StationConfig,
    };
    use hil_test::mk_static;
    use reqwless::{
        client::HttpClient,
        request::{Method, RequestBuilder},
        response::Status,
    };

    #[init]
    fn init() -> Peripherals {
        crate::init_heap();

        let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
        esp_hal::init(config)
    }

    #[embassy_executor::task]
    async fn net_task(mut runner: Runner<'static, Interface<'static>>) {
        defmt::debug!("[STA] Starting network task");
        runner.run().await
    }

    #[embassy_executor::task]
    async fn connection(mut controller: WifiController<'static>) {
        defmt::debug!("[STA] Start connection task");

        loop {
            match controller.connect_async().await {
                Ok(_) => {
                    defmt::info!("[STA] Wifi connected to");

                    // wait until we're no longer connected
                    controller.wait_for_disconnect_async().await.ok();
                    defmt::info!("[STA] Disconnected");
                }
                Err(e) => {
                    defmt::info!("[STA] Failed to connect to wifi: {:?}", e);
                }
            }

            Timer::after(Duration::from_millis(1500)).await
        }
    }

    #[test]
    async fn wifi_dhcp_http_test(p: Peripherals) {
        defmt::info!("[STA] Starting Wi-Fi DHCP HTTP test");
        let spawner = unsafe { embassy_executor::Spawner::for_current_executor().await };

        let timg0 = TimerGroup::new(p.TIMG0);
        let sw_ints = SoftwareInterruptControl::new(p.SW_INTERRUPT);
        esp_rtos::start(timg0.timer0, sw_ints.software_interrupt0);

        let station_config = Config::Station(
            StationConfig::default()
                .with_ssid("AP")
                .with_auth_method(esp_radio::wifi::AuthenticationMethod::None),
        );

        let (mut controller, interfaces) = esp_radio::wifi::new(
            p.WIFI,
            ControllerConfig::default().with_initial_config(station_config),
        )
        .unwrap();

        defmt::debug!("[STA] Wifi configured and started!");

        let wifi_interface = interfaces.station;

        let net_config = embassy_net::Config::dhcpv4(Default::default());

        let rng = Rng::new();
        let seed = (rng.random() as u64) << 32 | rng.random() as u64;

        // THEN create network stack
        let (stack, runner) = embassy_net::new(
            wifi_interface,
            net_config,
            mk_static!(StackResources<3>, StackResources::<3>::new()),
            seed,
        );

        let scan_config = ScanConfig::default().with_max(10);
        let result = controller.scan_async(&scan_config).await.unwrap();

        defmt::debug!("[STA] Scan complete, found {} networks", result.len());

        spawner.spawn(connection(controller)).unwrap();
        spawner.spawn(net_task(runner)).unwrap();
        stack.wait_config_up().await;

        Timer::after(Duration::from_millis(100)).await;

        defmt::debug!("[STA] Waiting for network to come up...");

        // HTTP client
        let tcp_client = TcpClient::new(
            stack,
            mk_static!(
                TcpClientState<1, 1500, 1500>,
                TcpClientState::<1, 1500, 1500>::new()
            ),
        );

        let dns_client = DnsSocket::new(stack);
        let mut http = HttpClient::new(&tcp_client, &dns_client);

        defmt::debug!("[STA] Starting HTTP requests loop");

        let mut rx_buf = [0u8; 4096];

        let request = http
            // .request(Method::GET, "http://1.1.1.1:8080/")
            .request(Method::GET, "http://192.168.2.1:8080")
            .await
            .unwrap();

        // Update the Host header to match the IP or remove it if reqwless handles it
        let mut request = request.headers(&[("Host", "192.168.2.1"), ("Connection", "close")]);

        defmt::debug!("[STA] Sending HTTP request");

        let response = request.send(&mut rx_buf).await.unwrap();

        defmt::debug!("[STA] eceived HTTP response");

        assert_eq!(response.status, Status::from(200));

        let body = response.body().read_to_end().await.unwrap();
        let body_str = core::str::from_utf8(body).unwrap();

        defmt::debug!("[STA] Response body: {}", body_str);

        assert!(
            body_str.contains("Hello Rust! Hello esp-radio!"),
            "Unexpected response body"
        );
    }

    // #[test]
    // async fn wifi_dhcp_http_test(p: Peripherals) {
    //     defmt::info!("Starting Wi-Fi DHCP HTTP test");
    //     let spawner = unsafe { embassy_executor::Spawner::for_current_executor().await };

    //     let timg0 = TimerGroup::new(p.TIMG0);
    //     let sw_ints = SoftwareInterruptControl::new(p.SW_INTERRUPT);
    //     esp_rtos::start(timg0.timer0, sw_ints.software_interrupt0);

    //     let station_config = Config::Station(
    //         StationConfig::default()
    //             .with_ssid("AP")
    //             .with_auth_method(esp_radio::wifi::AuthenticationMethod::None),
    //     );

    //     let (mut controller, interfaces) = esp_radio::wifi::new(
    //         p.WIFI,
    //         ControllerConfig::default().with_initial_config(station_config),
    //     )
    //     .unwrap();

    //     defmt::info!("Wifi configured and started!");

    //     let wifi_interface = interfaces.station;
    //     let net_config = embassy_net::Config::dhcpv4(Default::default());

    //     info!("net config");

    //     let rng = Rng::new();
    //     let seed = (rng.random() as u64) << 32 | rng.random() as u64;

    //     Timer::after(Duration::from_millis(500)).await;

    //     let (stack, runner) = embassy_net::new(
    //         wifi_interface,
    //         net_config,
    //         mk_static!(StackResources<3>, StackResources::<3>::new()),
    //         seed,
    //     );

    //     // Scan for AP with retry
    //     let mut scan_attempt = 0;
    //     let max_scan_attempts = 5;
    //     loop {
    //         let scan_config = ScanConfig::default().with_max(10);
    //         match controller.scan_async(&scan_config).await {
    //             Ok(result) => {
    //                 defmt::info!("Scan complete, found {} networks", result.len());
    //                 if result.iter().any(|ap| ap.ssid == "AP".into()) {
    //                     defmt::info!("Found AP in scan results");
    //                     break;
    //                 } else {
    //                     defmt::warn!("AP not found in scan results, retrying...");
    //                     scan_attempt += 1;
    //                     if scan_attempt >= max_scan_attempts {
    //                         panic!("AP not found after {} scan attempts", max_scan_attempts);
    //                     }
    //                     Timer::after(Duration::from_millis(500)).await;
    //                 }
    //             }
    //             Err(e) => {
    //                 defmt::warn!("Scan error: {:?}, retrying...", e);
    //                 scan_attempt += 1;
    //                 if scan_attempt >= max_scan_attempts {
    //                     panic!("Scan failed after {} attempts: {:?}", max_scan_attempts, e);
    //                 }
    //                 Timer::after(Duration::from_millis(500)).await;
    //             }
    //         }
    //     }

    //     spawner.spawn(net_task(runner)).unwrap();

    //     // Connect to AP with retry and exponential backoff
    //     let mut connect_attempt = 0;
    //     let max_connect_attempts = 10;
    //     let mut backoff_ms = 500;

    //     loop {
    //         defmt::info!(
    //             "WiFi connection attempt {} of {}",
    //             connect_attempt + 1,
    //             max_connect_attempts
    //         );

    //         match controller.connect_async().await {
    //             Ok(_) => {
    //                 defmt::info!("Successfully connected to WiFi!");
    //                 break;
    //             }
    //             Err(e) => {
    //                 defmt::warn!("Connection failed: {:?}", e);
    //                 connect_attempt += 1;

    //                 if connect_attempt >= max_connect_attempts {
    //                     panic!("Failed to connect after {} attempts", max_connect_attempts);
    //                 }

    //                 defmt::info!("Waiting {}ms before retry...", backoff_ms);
    //                 Timer::after(Duration::from_millis(backoff_ms)).await;

    //                 backoff_ms = (backoff_ms * 2).min(10000);
    //             }
    //         }
    //     }

    //     // Wait for network stack to be ready
    //     stack.wait_config_up().await;
    //     defmt::info!("Network stack ready!");

    //     // HTTP client
    //     let tcp_client = TcpClient::new(
    //         stack,
    //         mk_static!(
    //             TcpClientState<1, 1500, 1500>,
    //             TcpClientState::<1, 1500, 1500>::new()
    //         ),
    //     );

    //     let dns_client = DnsSocket::new(stack);
    //     let mut http = HttpClient::new(&tcp_client, &dns_client);

    //     defmt::info!("Making HTTP request to AP...");

    //     let mut rx_buf = [0u8; 4096];

    //     // HTTP request with retry - FIXED LIFETIME
    //     let mut http_attempt = 0;
    //     let max_http_attempts = 3;

    //     loop {
    //         defmt::info!(
    //             "Sending HTTP request, attempt {} of {}",
    //             http_attempt + 1,
    //             max_http_attempts
    //         );

    //         match http.request(Method::GET, "http://192.168.2.1:8080").await {
    //             Ok(request) => {
    //                 // Add headers and send in one expression (no intermediate binding)
    //                 match request
    //                     .headers(&[("Host", "192.168.2.1"), ("Connection", "close")])
    //                     .send(&mut rx_buf)
    //                     .await
    //                 {
    //                     Ok(response) => {
    //                         // Verify response status
    //                         if response.status != Status::from(200) {
    //                             defmt::warn!("Unexpected status code");
    //                             http_attempt += 1;
    //                             if http_attempt >= max_http_attempts {
    //                                 panic!("HTTP request failed");
    //                             }
    //                             Timer::after(Duration::from_millis(500)).await;
    //                             continue;
    //                         }

    //                         // Read response body
    //                         match response.body().read_to_end().await {
    //                             Ok(body) => match core::str::from_utf8(body) {
    //                                 Ok(body_str) => {
    //                                     info!("Response body: {}", body_str);

    //                                     if body_str.contains("Hello Rust! Hello esp-radio!") {
    //                                         defmt::info!("✅ Response contains expected text!");
    //                                         defmt::info!(
    //                                             "✅ WiFi DHCP HTTP test completed successfully!"
    //                                         );
    //                                         return;
    //                                     } else {
    //                                         defmt::warn!("Response does not contain expected
    // text");                                         http_attempt += 1;
    //                                         if http_attempt >= max_http_attempts {
    //                                             panic!("Unexpected response content");
    //                                         }
    //                                         Timer::after(Duration::from_millis(500)).await;
    //                                     }
    //                                 }
    //                                 Err(e) => {
    //                                     defmt::warn!("Failed to parse response body");
    //                                     http_attempt += 1;
    //                                     if http_attempt >= max_http_attempts {
    //                                         panic!("Failed to parse response");
    //                                     }
    //                                     Timer::after(Duration::from_millis(500)).await;
    //                                 }
    //                             },
    //                             Err(e) => {
    //                                 defmt::warn!("Failed to read response body");
    //                                 http_attempt += 1;
    //                                 if http_attempt >= max_http_attempts {
    //                                     panic!("Failed to read response body");
    //                                 }
    //                                 Timer::after(Duration::from_millis(500)).await;
    //                             }
    //                         }
    //                     }
    //                     Err(e) => {
    //                         defmt::warn!("HTTP send error");
    //                         http_attempt += 1;

    //                         if http_attempt >= max_http_attempts {
    //                             panic!("HTTP request failed after {} attempts",
    // max_http_attempts);                         }

    //                         Timer::after(Duration::from_millis(500)).await;
    //                     }
    //                 }
    //             }
    //             Err(e) => {
    //                 defmt::warn!("HTTP request error");
    //                 http_attempt += 1;

    //                 if http_attempt >= max_http_attempts {
    //                     panic!("HTTP request failed after {} attempts", max_http_attempts);
    //                 }

    //                 Timer::after(Duration::from_millis(500)).await;
    //             }
    //         }
    //     }
    // }

    // #[test]
    // async fn wifi_csi(p: Peripherals) {
    //     use core::sync::atomic::{AtomicU8, Ordering};

    //     use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
    //     use esp_radio::wifi::{csi::CsiConfig, event::WifiEvent};

    //     defmt::info!("Starting Wi-Fi CSI test");
    //     let spawner = unsafe { embassy_executor::Spawner::for_current_executor().await };

    //     let timg0 = TimerGroup::new(p.TIMG0);
    //     let sw_ints = SoftwareInterruptControl::new(p.SW_INTERRUPT);
    //     esp_rtos::start(timg0.timer0, sw_ints.software_interrupt0);

    //     let station_config = Config::Station(
    //         StationConfig::default()
    //             .with_ssid("AP")
    //             .with_auth_method(esp_radio::wifi::AuthenticationMethod::None),
    //     );

    //     let (mut controller, interfaces) = esp_radio::wifi::new(
    //         p.WIFI,
    //         ControllerConfig::default().with_initial_config(station_config),
    //     )
    //     .unwrap();

    //     defmt::info!("Wifi configured and started!");

    //     // enable some "interesting" events to be received in the connection task
    //     esp_radio::wifi::event::enable_wifi_events(
    //         WifiEvent::HomeChannelChange | WifiEvent::StationBeaconTimeout,
    //     );

    //     let wifi_interface = interfaces.station;

    //     let config = embassy_net::Config::dhcpv4(Default::default());

    //     let rng = Rng::new();
    //     let seed = (rng.random() as u64) << 32 | rng.random() as u64;

    //     // Small delay helps stability on real hardware
    //     Timer::after(Duration::from_millis(500)).await;

    //     // Init network stack
    //     let (stack, runner) = embassy_net::new(
    //         wifi_interface,
    //         config,
    //         mk_static!(StackResources<3>, StackResources::<3>::new()),
    //         seed,
    //     );

    //     let signal = mk_static!(Signal<CriticalSectionRawMutex, ()>, Signal::new());
    //     static COUNT: AtomicU8 = AtomicU8::new(0);

    //     let csi = CsiConfig::default();
    //     controller
    //         .set_csi(csi, |data: esp_radio::wifi::csi::WifiCsiInfo| {
    //             let current_count = COUNT.fetch_add(1, Ordering::Relaxed) + 1;
    //             defmt::info!("rssi: {:?} rate: {}", data.rssi(), data.rate());

    //             if current_count >= 10 {
    //                 signal.signal(());
    //             }
    //         })
    //         .unwrap();

    //     let scan_config = ScanConfig::default().with_max(10);
    //     let result = controller.scan_async(&scan_config).await.unwrap();

    //     defmt::info!("Scan complete, found {} networks", result.len());
    //     assert!(
    //         result.iter().any(|ap| ap.ssid == "AP".into()),
    //         "AP not found in scan results"
    //     );

    //     spawner.spawn(connection(controller)).unwrap();
    //     spawner.spawn(net_task(runner)).unwrap();
    //     stack.wait_config_up().await;
    // }

    #[test]
    async fn wifi_drop(p: Peripherals) {
        defmt::info!("[STA] Starting Wi-Fi Drop test");

        let timg0 = TimerGroup::new(p.TIMG0);
        let sw_ints = SoftwareInterruptControl::new(p.SW_INTERRUPT);
        esp_rtos::start(timg0.timer0, sw_ints.software_interrupt0);

        let mut wifi = p.WIFI;

        {
            let station_config = Config::Station(
                StationConfig::default()
                    .with_ssid("AP")
                    .with_auth_method(esp_radio::wifi::AuthenticationMethod::None),
            );

            let (mut controller, interfaces) = esp_radio::wifi::new(
                wifi.reborrow(),
                ControllerConfig::default().with_initial_config(station_config),
            )
            .unwrap();

            let mut wifi_interface = interfaces.station;

            let token = wifi_interface.transmit();
            assert!(matches!(token, None));

            match controller.connect_async().await {
                Ok(_) => {}
                Err(_) => {
                    Timer::after(Duration::from_millis(1000)).await;
                    controller.connect_async().await.unwrap();
                }
            }

            let token = wifi_interface.transmit();

            defmt::debug!("[STA] got token {}", token.is_some());

            core::mem::drop(controller);

            if let Some(token) = token {
                token.consume_token(10, |tx| tx.fill(0));
                defmt::info!("[STA] survived consumer_token");
            } else {
                defmt::info!("[STA] no token");
            }

            let token = wifi_interface.transmit();
            assert!(matches!(token, None));
        }
        {
            Timer::after(Duration::from_millis(2000)).await;
            let station_config = Config::Station(
                StationConfig::default()
                    .with_ssid("AP")
                    .with_auth_method(esp_radio::wifi::AuthenticationMethod::None),
            );

            let (mut controller, interfaces) = esp_radio::wifi::new(
                wifi.reborrow(),
                ControllerConfig::default().with_initial_config(station_config),
            )
            .unwrap();

            match controller.connect_async().await {
                Ok(_) => {}
                Err(_) => {
                    Timer::after(Duration::from_millis(1000)).await;
                    controller.connect_async().await.unwrap();
                }
            }

            let mut wifi_interface = interfaces.station;

            let token = wifi_interface.transmit();

            defmt::info!("[STA] got token {}", token.is_some());

            if let Some(token) = token {
                token.consume_token(10, |tx| tx.fill(0));
                defmt::info!("[STA] tx done");
            } else {
                defmt::info!("[STA] no token");
            }
        }
    }
}
