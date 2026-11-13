//! SoftAP HTTP protocomm provisioning (Espressif `protocomm` over HTTP POST).

use core::net::{Ipv4Addr, SocketAddrV4};

use embassy_futures::{
    join::join3,
    select::{Either, select},
};
use embassy_net::{
    IpEndpoint,
    IpListenEndpoint,
    Ipv4Cidr,
    Stack,
    StackResources,
    StaticConfigV4,
    tcp::TcpSocket,
    udp::{PacketMetadata, UdpSocket},
};
use embassy_time::{Duration, Instant, Timer};
use embedded_io_async::Write as _;
use esp_radio::wifi::{
    Config,
    Interface,
    WifiController,
    ap::AccessPointConfig,
    scan::ScanConfig,
    sta::StationConfig,
};
use rand_core::{CryptoRng, RngCore};

use crate::{
    PROTO_VER_JSON,
    handlers::ProvCommState,
    scan::cache_scan_results,
    session::creds_from_pending,
};

/// Parameters for a SoftAP provisioning session.
#[instability::unstable]
pub struct SoftApSession<'a> {
    /// Proof-of-possession for sec1.
    pub pop: &'a str,
    /// SoftAP SSID.
    pub ssid: &'a str,
    /// SoftAP gateway address (`192.168.4.1/24` by default).
    pub gateway: Ipv4Addr,
    /// Maximum session duration before giving up.
    pub timeout: Duration,
}

/// Minimal DNS: every A-record query resolves to the SoftAP gateway (captive-portal friendly).
async fn ap_dns(stack: Stack<'_>, gateway: Ipv4Addr) {
    let mut rx_meta = [PacketMetadata::EMPTY; 4];
    let mut rx_buf = [0u8; 512];
    let mut tx_meta = [PacketMetadata::EMPTY; 4];
    let mut tx_buf = [0u8; 512];

    let mut udp = UdpSocket::new(stack, &mut rx_meta, &mut rx_buf, &mut tx_meta, &mut tx_buf);
    if udp.bind(53).is_err() {
        warn!("[softap] DNS bind :53 failed");
        return;
    }
    info!("[softap] DNS on :53 (all A → {gateway})");

    let gw_octets = gateway.octets();
    let mut rx = [0u8; 512];
    let mut tx = [0u8; 512];

    loop {
        let (len, meta) = match udp.recv_from(&mut rx).await {
            Ok(v) => v,
            Err(e) => {
                warn!("[softap] DNS recv: {:?}", e);
                continue;
            }
        };
        if len < 12 {
            continue;
        }

        let mut pos = 12usize;
        while pos < len {
            let llen = rx[pos] as usize;
            if llen == 0 {
                pos += 1;
                break;
            }
            if llen & 0xC0 == 0xC0 {
                pos += 2;
                break;
            }
            pos += llen + 1;
        }
        if pos + 4 > len {
            continue;
        }
        let qtype = u16::from_be_bytes([rx[pos], rx[pos + 1]]);
        pos += 4;
        let q_end = pos;

        let mut w = 0usize;
        tx[w..w + 2].copy_from_slice(&rx[0..2]);
        w += 2;
        tx[w..w + 2].copy_from_slice(&[0x81, 0x80]);
        w += 2;
        tx[w..w + 2].copy_from_slice(&[0x00, 0x01]);
        w += 2;
        let add_answer = qtype == 1;
        tx[w..w + 2].copy_from_slice(if add_answer {
            &[0x00, 0x01]
        } else {
            &[0x00, 0x00]
        });
        w += 2;
        tx[w..w + 4].copy_from_slice(&[0x00, 0x00, 0x00, 0x00]);
        w += 4;
        let qlen = q_end - 12;
        if w + qlen + 16 > tx.len() {
            continue;
        }
        tx[w..w + qlen].copy_from_slice(&rx[12..q_end]);
        w += qlen;
        if add_answer {
            tx[w..w + 2].copy_from_slice(&[0xC0, 0x0C]);
            w += 2;
            tx[w..w + 2].copy_from_slice(&[0x00, 0x01]);
            w += 2;
            tx[w..w + 2].copy_from_slice(&[0x00, 0x01]);
            w += 2;
            tx[w..w + 4].copy_from_slice(&[0x00, 0x00, 0x00, 0x00]);
            w += 4;
            tx[w..w + 2].copy_from_slice(&[0x00, 0x04]);
            w += 2;
            tx[w..w + 4].copy_from_slice(&gw_octets);
            w += 4;
        }
        let dest = IpEndpoint::new(meta.endpoint.addr, 53);
        let _ = udp.send_to(&tx[..w], dest).await;
    }
}

async fn ap_dhcp(stack: Stack<'_>, gateway: Ipv4Addr) {
    use edge_dhcp::{
        io::{self, DEFAULT_SERVER_PORT},
        server::{Server, ServerOptions},
    };
    use edge_nal::UdpBind;
    use edge_nal_embassy::{Udp, UdpBuffers};

    let mut buf = [0u8; 1500];
    let mut gw_buf = [gateway];
    let buffers = UdpBuffers::<3, 1024, 1024, 10>::new();
    let unbound_socket = Udp::new(stack, &buffers);
    let mut bound_socket = match unbound_socket
        .bind(core::net::SocketAddr::V4(SocketAddrV4::new(
            Ipv4Addr::UNSPECIFIED,
            DEFAULT_SERVER_PORT,
        )))
        .await
    {
        Ok(s) => s,
        Err(e) => {
            warn!("[softap] DHCP bind failed: {e:?}");
            return;
        }
    };
    info!("[softap] DHCP on :67");

    loop {
        _ = io::server::run(
            &mut Server::<_, 64>::new_with_et(gateway),
            &ServerOptions::new(gateway, Some(&mut gw_buf)),
            &mut bound_socket,
            &mut buf,
        )
        .await
        .inspect_err(|e| warn!("[softap] DHCP server error: {e:?}"));
        Timer::after(Duration::from_millis(500)).await;
    }
}

/// Scan in STA mode, bring up the provisioning AP, and serve HTTP protocomm.
///
/// Returns credentials after ApplyConfig + GetStatus, or `None` on timeout /
/// early failure. Ending the future drops the SoftAP net stack and releases the AP
/// `Interface` singleton for the application to recreate; this crate does not stop Wi-Fi
/// or join STA.
#[instability::unstable]
pub async fn run(
    controller: &mut WifiController<'_>,
    ap_device: Interface,
    rng: &mut (impl RngCore + CryptoRng),
    session: &SoftApSession<'_>,
) -> Option<crate::WifiCredentials> {
    scan_wifi_for_prov(controller).await;

    let ap_cfg = Config::AccessPoint(
        AccessPointConfig::default().with_ssid(session.ssid.try_into().unwrap()),
    );
    if let Err(e) = controller.set_config(&ap_cfg) {
        error!("[softap] AP set_config: {:?}", e);
        return None;
    }
    let _ = controller.set_max_tx_power(44);

    let gateway = session.gateway;
    let net_cfg = embassy_net::Config::ipv4_static(StaticConfigV4 {
        address: Ipv4Cidr::new(gateway, 24),
        gateway: Some(gateway),
        dns_servers: Default::default(),
    });
    let seed = Instant::now().as_micros() as u64 ^ 0xdead_beef_cafe_babe;
    let mut resources = StackResources::<5>::new();
    let (stack, mut runner) = embassy_net::new(ap_device, net_cfg, &mut resources, seed);
    let timeout = session.timeout;

    let session_fut = async {
        stack.wait_config_up().await;
        info!(
            "[softap] AP '{}' at {} — QR: {{\"ver\":\"v1\",\"name\":\"{}\",\"pop\":\"{}\",\"transport\":\"softap\"}}",
            session.ssid, gateway, session.ssid, session.pop,
        );

        let mut http_rx = [0u8; 2048];
        let mut http_tx = [0u8; 2048];
        let mut http_req = [0u8; 2048];
        let http_fut = http_prov_server(
            stack,
            rng,
            session.pop,
            &mut http_rx,
            &mut http_tx,
            &mut http_req,
        );
        match select(http_fut, Timer::after(timeout)).await {
            Either::First(creds) => creds,
            Either::Second(_) => {
                info!("[softap] provisioning timeout ({} s)", timeout.as_secs());
                None
            }
        }
    };

    match select(
        session_fut,
        join3(
            runner.run(),
            ap_dhcp(stack, gateway),
            ap_dns(stack, gateway),
        ),
    )
    .await
    {
        Either::First(creds) => creds,
        Either::Second(_) => {
            warn!("[softap] network stopped before provisioning finished");
            None
        }
    }
}

async fn scan_wifi_for_prov(controller: &mut WifiController<'_>) {
    if let Err(e) = controller.set_config(&Config::Station(StationConfig::default())) {
        warn!("[softap] scan STA set_config: {:?}", e);
        return;
    }
    let scan_config = ScanConfig::default().with_max(16);
    match controller.scan_async(&scan_config).await {
        Ok(results) => {
            info!("[softap] cached {} APs for /prov-scan", results.len());
            cache_scan_results(&results);
        }
        Err(e) => warn!("[softap] scan failed: {:?}", e),
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum ReqKind {
    ProtoVer,
    Session,
    Config,
    Scan,
    CaptiveCheck,
    Unknown,
}

async fn http_prov_server(
    stack: Stack<'_>,
    rng: &mut (impl RngCore + CryptoRng),
    pop: &str,
    sock_rx: &mut [u8],
    sock_tx: &mut [u8],
    req_buf: &mut [u8],
) -> Option<crate::WifiCredentials> {
    let mut state = ProvCommState::new();
    let mut mode_announced = false;

    loop {
        let mut socket = TcpSocket::new(stack, sock_rx, sock_tx);
        socket.set_timeout(Some(Duration::from_secs(30)));
        if socket
            .accept(IpListenEndpoint {
                addr: None,
                port: 80,
            })
            .await
            .is_err()
        {
            warn!("[softap] HTTP accept failed");
            Timer::after(Duration::from_millis(100)).await;
            continue;
        }
        info!("[softap] HTTP client connected");

        // Keep the TCP open for multiple POSTs so sec1 survives into /prov-scan.
        loop {
            let Some((kind, body_start, body_len)) = read_request(&mut socket, req_buf).await
            else {
                warn!("[softap] HTTP read failed / peer closed");
                socket.abort();
                let _ = socket.flush().await;
                break;
            };
            info!("[softap] HTTP {}", req_kind_name(kind));
            if kind == ReqKind::Session && !mode_announced {
                info!("Provisioning SoftAP mode selected.");
                mode_announced = true;
            }

            let done = handle_request(
                &mut socket,
                kind,
                &req_buf[body_start..body_start + body_len],
                &mut state,
                pop.as_bytes(),
                rng,
            )
            .await;
            if done {
                return creds_from_pending(
                    state.pending_ssid.as_str(),
                    state.pending_pass.as_str(),
                );
            }
        }
    }
}

fn req_kind_name(kind: ReqKind) -> &'static str {
    match kind {
        ReqKind::ProtoVer => "proto-ver",
        ReqKind::Session => "prov-session",
        ReqKind::Config => "prov-config",
        ReqKind::Scan => "prov-scan",
        ReqKind::CaptiveCheck => "captive check",
        ReqKind::Unknown => "unknown",
    }
}

async fn read_request(
    socket: &mut TcpSocket<'_>,
    buf: &mut [u8],
) -> Option<(ReqKind, usize, usize)> {
    let mut n = 0usize;
    let header_end = loop {
        if n >= buf.len() {
            return None;
        }
        match socket.read(&mut buf[n..n + 1]).await {
            Ok(0) | Err(_) => return None,
            Ok(_) => {}
        }
        n += 1;
        if n >= 4 && &buf[n - 4..n] == b"\r\n\r\n" {
            break n;
        }
    };

    let kind = classify(&buf[..header_end]);
    let content_length: usize = buf[..header_end]
        .windows(16)
        .position(|w| {
            w.eq_ignore_ascii_case(b"content-length: ")
                || w.eq_ignore_ascii_case(b"content-length:\t")
        })
        .and_then(|pos| {
            let rest = &buf[pos + 16..header_end];
            let end = rest
                .iter()
                .position(|&b| b == b'\r' || b == b'\n')
                .unwrap_or(rest.len());
            core::str::from_utf8(&rest[..end])
                .ok()
                .and_then(|s| s.trim().parse().ok())
        })
        .unwrap_or(0);

    let body_start = header_end;
    let body_end = body_start + content_length;
    if body_end > buf.len() {
        return None;
    }
    while n < body_end {
        match socket.read(&mut buf[n..body_end]).await {
            Ok(0) | Err(_) => return None,
            Ok(got) => n += got,
        }
    }
    Some((kind, body_start, content_length))
}

fn classify(header_bytes: &[u8]) -> ReqKind {
    let h = header_bytes;
    if contains(h, b"/proto-ver") {
        ReqKind::ProtoVer
    } else if contains(h, b"/prov-session") {
        ReqKind::Session
    } else if contains(h, b"/prov-config") {
        ReqKind::Config
    } else if contains(h, b"/prov-scan") {
        ReqKind::Scan
    } else if contains(h, b"/generate_204")
        || contains(h, b"/hotspot-detect")
        || contains(h, b"/ncsi.txt")
        || contains(h, b"/canonical")
        || contains(h, b"/redirect")
    {
        ReqKind::CaptiveCheck
    } else {
        ReqKind::Unknown
    }
}

fn contains(haystack: &[u8], needle: &[u8]) -> bool {
    haystack.windows(needle.len()).any(|w| w == needle)
}

async fn handle_request(
    socket: &mut TcpSocket<'_>,
    kind: ReqKind,
    body: &[u8],
    state: &mut ProvCommState,
    pop: &[u8],
    rng: &mut (impl RngCore + CryptoRng),
) -> bool {
    match kind {
        ReqKind::ProtoVer => send_ok(socket, PROTO_VER_JSON).await,

        ReqKind::Session => {
            let mut session_resp_buf = [0u8; 256];
            let wire = state.handle_session(body, pop, rng, &mut session_resp_buf, "[softap]");
            send_ok(socket, wire.as_bytes()).await;
        }

        ReqKind::Config => {
            let mut plain = [0u8; 256];
            let n = body.len().min(plain.len());
            plain[..n].copy_from_slice(&body[..n]);
            state.decrypt_body(&mut plain, n);

            let mut config_resp = [0u8; 64];
            let (done, cr_len) = state.handle_config(&plain[..n], &mut config_resp, "[softap]");
            state.encrypt_response(&mut config_resp, cr_len);
            send_ok(socket, &config_resp[..cr_len]).await;
            if done {
                return true;
            }
        }

        ReqKind::Scan => {
            let mut plain = [0u8; 64];
            let n = body.len().min(plain.len());
            plain[..n].copy_from_slice(&body[..n]);
            state.decrypt_body(&mut plain, n);

            let mut scan_resp = [0u8; 512];
            let sr_len = state.handle_scan(&plain[..n], &mut scan_resp);
            state.encrypt_response(&mut scan_resp, sr_len);
            send_ok(socket, &scan_resp[..sr_len]).await;
        }

        ReqKind::CaptiveCheck => {
            let _ = socket
                .write_all(b"HTTP/1.0 204 No Content\r\nContent-Length: 0\r\n\r\n")
                .await;
            let _ = socket.flush().await;
        }

        ReqKind::Unknown => {
            let _ = socket
                .write_all(b"HTTP/1.0 404 Not Found\r\nContent-Length: 0\r\n\r\n")
                .await;
            let _ = socket.flush().await;
        }
    }
    false
}

async fn send_ok(socket: &mut TcpSocket<'_>, body: &[u8]) {
    let mut hdr = [0u8; 96];
    let hdr_len = build_ok_header(&mut hdr, body.len());
    let _ = socket.write_all(&hdr[..hdr_len]).await;
    let _ = socket.write_all(body).await;
    let _ = socket.flush().await;
}

fn build_ok_header(buf: &mut [u8; 96], body_len: usize) -> usize {
    let prefix = b"HTTP/1.0 200 OK\r\nContent-Type: application/octet-stream\r\nContent-Length: ";
    let suffix = b"\r\n\r\n";
    let mut pos = 0;
    buf[pos..pos + prefix.len()].copy_from_slice(prefix);
    pos += prefix.len();
    let mut num_buf = [0u8; 10];
    let num = fmt_usize(body_len, &mut num_buf);
    buf[pos..pos + num.len()].copy_from_slice(num);
    pos += num.len();
    buf[pos..pos + suffix.len()].copy_from_slice(suffix);
    pos + suffix.len()
}

fn fmt_usize(n: usize, buf: &mut [u8; 10]) -> &[u8] {
    if n == 0 {
        buf[0] = b'0';
        return &buf[..1];
    }
    let mut pos = 10;
    let mut v = n;
    while v > 0 {
        pos -= 1;
        buf[pos] = b'0' + (v % 10) as u8;
        v /= 10;
    }
    &buf[pos..]
}
