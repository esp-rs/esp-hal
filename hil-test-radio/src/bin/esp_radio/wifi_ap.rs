#[embedded_test::tests(default_timeout = 45, executor = esp_rtos::embassy::Executor::new())]
mod tests {
    use core::{net::Ipv4Addr, str::FromStr};

    use embassy_net::{
        IpListenEndpoint,
        Ipv4Cidr,
        Runner,
        Stack,
        StackResources,
        StaticConfigV4,
        tcp::TcpSocket,
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
        ap::AccessPointConfig,
    };
    use hil_test::mk_static;

    #[init]
    fn init() -> Peripherals {
        crate::init_heap();
        let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
        esp_hal::init(config)
    }

    #[embassy_executor::task]
    async fn net_task(mut runner: Runner<'static, Interface<'static>>) {
        defmt::debug!("[AP] Starting network task");
        runner.run().await
    }

    #[embassy_executor::task]
    async fn run_dhcp(stack: Stack<'static>, gw_ip_addr: &'static str) {
        use core::net::{Ipv4Addr, SocketAddrV4};

        use edge_dhcp::{
            io::{self, DEFAULT_SERVER_PORT},
            server::{Server, ServerOptions},
        };
        use edge_nal::UdpBind;
        use edge_nal_embassy::{Udp, UdpBuffers};

        defmt::debug!("[AP] Starting DHCP task");

        let ip = Ipv4Addr::from_str(gw_ip_addr).expect("dhcp task failed to parse gw ip");

        let mut buf = [0u8; 1500];
        let mut gw_buf = [Ipv4Addr::UNSPECIFIED];

        let buffers = UdpBuffers::<3, 1024, 1024, 10>::new();
        let unbound_socket = Udp::new(stack, &buffers);
        let mut bound_socket = unbound_socket
            .bind(core::net::SocketAddr::V4(SocketAddrV4::new(
                Ipv4Addr::UNSPECIFIED,
                DEFAULT_SERVER_PORT,
            )))
            .await
            .unwrap();

        loop {
            _ = io::server::run(
                &mut Server::<_, 64>::new_with_et(ip),
                &ServerOptions::new(ip, Some(&mut gw_buf)),
                &mut bound_socket,
                &mut buf,
            )
            .await;
            Timer::after(Duration::from_millis(500)).await;
        }
    }

    #[embassy_executor::task]
    async fn connection(controller: WifiController<'static>) {
        defmt::debug!("[AP] Start connection task");
        loop {
            let ev = controller
                .wait_for_access_point_connected_event_async()
                .await;
            match ev {
                Ok(esp_radio::wifi::AccessPointStationEventInfo::Connected(info)) => {
                    defmt::info!("[AP] Station connected: {:?}", info);
                }
                Ok(esp_radio::wifi::AccessPointStationEventInfo::Disconnected(info)) => {
                    defmt::info!("[AP] Station disconnected: {:?}", info);
                }
                _ => (),
            }
            Timer::after(Duration::from_millis(2000)).await
        }
    }

    #[test]
    async fn wifi_ap_test(p: Peripherals) {
        defmt::info!("Starting Wi-Fi AP test");
        let spawner = unsafe { embassy_executor::Spawner::for_current_executor().await };

        let timg0 = TimerGroup::new(p.TIMG0);
        let sw_ints = SoftwareInterruptControl::new(p.SW_INTERRUPT);
        esp_rtos::start(timg0.timer0, sw_ints.software_interrupt0);

        let access_point_config = Config::AccessPoint(
            AccessPointConfig::default()
                .with_ssid("AP")
                .with_auth_method(esp_radio::wifi::AuthenticationMethod::None),
        );

        let (controller, interfaces) = esp_radio::wifi::new(
            p.WIFI,
            ControllerConfig::default().with_initial_config(access_point_config),
        )
        .unwrap();

        defmt::debug!("[AP] Wifi configured and started!");

        let gw_ip_addr_str = "192.168.2.1";
        let gw_ip_addr = Ipv4Addr::from_str(gw_ip_addr_str).expect("failed to parse gateway ip");

        let net_config = embassy_net::Config::ipv4_static(StaticConfigV4 {
            address: Ipv4Cidr::new(gw_ip_addr, 24),
            gateway: Some(gw_ip_addr),
            dns_servers: Default::default(),
        });

        let rng = Rng::new();
        let seed = (rng.random() as u64) << 32 | rng.random() as u64;

        let (stack, runner) = embassy_net::new(
            interfaces.access_point,
            net_config,
            mk_static!(StackResources<3>, StackResources::<3>::new()),
            seed,
        );

        spawner.spawn(connection(controller).unwrap());
        spawner.spawn(net_task(runner).unwrap());
        spawner.spawn(run_dhcp(stack, gw_ip_addr_str).unwrap());

        stack.wait_config_up().await;

        defmt::info!("[AP] AP ready at {}:8080", gw_ip_addr_str);

        let mut rx_buffer = [0u8; 1536];
        let mut tx_buffer = [0u8; 1536];
        let mut socket = TcpSocket::new(stack, &mut rx_buffer, &mut tx_buffer);
        socket.set_timeout(Some(embassy_time::Duration::from_secs(10)));

        loop {
            defmt::debug!("[AP] Wait for connection...");
            let _r = socket
                .accept(IpListenEndpoint {
                    addr: None,
                    port: 8080,
                })
                .await;

            defmt::info!("[AP] Connected...");

            use embedded_io_async::Write;

            // NOTE: pos must be reset inside the outer loop, not outside it,
            // otherwise it accumulates across requests and buffer reads bleed over.
            let mut buffer = [0u8; 1024];
            let mut pos = 0;
            loop {
                match socket.read(&mut buffer).await {
                    Ok(0) => {
                        defmt::debug!("[AP] read EOF");
                        break;
                    }
                    Ok(len) => {
                        let to_print =
                            unsafe { core::str::from_utf8_unchecked(&buffer[..(pos + len)]) };

                        if to_print.contains("\r\n\r\n") {
                            defmt::info!("[AP] {}", to_print);
                            break;
                        }

                        pos += len;
                    }
                    Err(_) => {
                        break;
                    }
                }
            }

            let _r = socket
                .write_all(
                    b"HTTP/1.0 200 OK\r\n\r\n\
                    <html>\
                        <body>\
                            <h1>Hello Rust! Hello esp-radio!</h1>\
                        </body>\
                    </html>\r\n",
                )
                .await;

            let _r = socket.flush().await;

            Timer::after(Duration::from_millis(1000)).await;
            socket.close();
            Timer::after(Duration::from_millis(1000)).await;
            socket.abort();

            Timer::after(Duration::from_millis(3000)).await;

            defmt::info!("[AP] TEST_DONE: PASS");
        }
    }
}
