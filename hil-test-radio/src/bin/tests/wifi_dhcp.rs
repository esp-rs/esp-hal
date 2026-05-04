#[embedded_test::tests(default_timeout = 45, executor = esp_rtos::embassy::Executor::new())]
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
        // let spawner = executor_core0.start(Priority::Priority1);

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

        spawner.spawn(connection(controller).unwrap());
        spawner.spawn(net_task(runner).unwrap());
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
            .request(Method::GET, "http://192.168.2.1:8080")
            .await
            .unwrap();

        // Update the Host header to match the IP or remove it if reqwless handles it
        let mut request = request.headers(&[("Host", "192.168.2.1"), ("Connection", "close")]);

        defmt::debug!("[STA] Sending HTTP request");

        let response = request.send(&mut rx_buf).await.unwrap();

        defmt::debug!("[STA] received HTTP response");

        assert_eq!(response.status, Status::from(200));

        let body = response.body().read_to_end().await.unwrap();
        let body_str = core::str::from_utf8(body).unwrap();

        defmt::debug!("[STA] Response body: {}", body_str);

        assert!(
            body_str.contains("Hello Rust! Hello esp-radio!"),
            "Unexpected response body"
        );

        defmt::info!("[STA] TEST_DONE: PASS");
    }

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
