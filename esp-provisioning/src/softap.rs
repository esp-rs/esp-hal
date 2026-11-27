//! SoftAP HTTP protocomm on a stack the application already started.

use core::{
    cell::RefCell,
    fmt::Write as _,
    net::{IpAddr, Ipv4Addr, SocketAddr, SocketAddrV4},
};

use edge_nal::TcpBind;
use embassy_futures::{
    join::join,
    select::{Either3, select3},
};
use embassy_net::Stack;
use embassy_time::{Duration, Timer};
use embedded_io_async::{Read, Write};
use esp_radio::wifi::scan::AccessPointInfo;
use rand_core::{CryptoRng, RngCore};

use crate::{
    Config,
    ProvError,
    Session,
    handlers::{ConnState, Endpoint},
    sec1::{self, Sec1},
};

struct Shared<R> {
    conn: ConnState,
    rng: R,
}

/// Serves protocomm HTTP on `stack` until a reported `GetStatus` is on the wire, or until this
/// future is dropped.
///
/// `stack` is an access-point interface the application configured and seeded.
/// `aps` is the scan list the application took before switching to AP mode.
/// DHCP and a captive-portal DNS resolver run beside the HTTP server. This
/// function does not configure the radio or construct a network stack.
///
/// The handshake runs outside any critical section. One accepted socket still
/// owns one sec1 state: the state is moved out for the request and put back
/// before the next await.
///
/// # Errors
///
/// [`ProvError::Transport`] when the TCP listener fails.
///
/// # Cancellation Safety
///
/// Dropping this future stops HTTP, DHCP, and DNS. The other transport and the session keep
/// running.
#[instability::unstable]
pub async fn serve<R>(
    stack: Stack<'_>,
    aps: &[AccessPointInfo],
    config: &Config,
    session: &Session,
    rng: R,
) -> Result<(), ProvError>
where
    R: RngCore + CryptoRng,
{
    let gateway = config.gateway();
    let mut gateway_text = heapless::String::<15>::new();
    let _ = core::fmt::Write::write_fmt(&mut gateway_text, format_args!("{gateway}"));
    info!(
        "[softap] serving '{}' at {}",
        config.softap_ssid().as_str(),
        gateway_text.as_str()
    );
    let mut conn = ConnState::<Sec1>::new();
    conn.set_transport("softap");
    let state = RefCell::new(Some(Shared { conn, rng }));

    let http = http_task(stack, aps, config, session, &state);
    // `edge_captive` returns when its timer elapses. Restart it. Do not let
    // that return cancel HTTP or DHCP.
    let dns = async {
        loop {
            captive_dns(stack, gateway).await;
            Timer::after(Duration::from_millis(500)).await;
        }
    };
    let dhcp = ap_dhcp(stack, gateway);

    match select3(http, session.wait_finished(), join(dns, dhcp)).await {
        Either3::First(result) => result,
        Either3::Second(_) | Either3::Third(_) => Ok(()),
    }
}

async fn http_task<R>(
    stack: Stack<'_>,
    aps: &[AccessPointInfo],
    config: &Config,
    session: &Session,
    state: &RefCell<Option<Shared<R>>>,
) -> Result<(), ProvError>
where
    R: RngCore + CryptoRng,
{
    let buffers = edge_nal_embassy::TcpBuffers::<1, 1536, 1536>::new();
    let tcp = edge_nal_embassy::Tcp::new(stack, &buffers);
    let acceptor = tcp
        .bind(SocketAddr::from((Ipv4Addr::UNSPECIFIED, 80)))
        .await
        .map_err(|_| ProvError::Transport)?;

    let acceptor = ResetAccept {
        inner: acceptor,
        state,
    };
    let handler = ProvHttp {
        state,
        session,
        aps,
        pop: config.pop().as_str(),
    };
    let mut server = edge_http::io::server::Server::<1, 2048, 32>::new();
    server
        .run(Some(60_000), acceptor, &handler)
        .await
        .map_err(|_| ProvError::Transport)
}

struct ResetAccept<'a, A, R> {
    inner: A,
    state: &'a RefCell<Option<Shared<R>>>,
}

impl<A, R> edge_nal::TcpAccept for ResetAccept<'_, A, R>
where
    A: edge_nal::TcpAccept,
{
    type Error = A::Error;
    type Socket<'a>
        = A::Socket<'a>
    where
        Self: 'a;

    async fn accept(&self) -> Result<(SocketAddr, Self::Socket<'_>), Self::Error> {
        let accepted = self.inner.accept().await?;
        if let Some(shared) = self.state.borrow_mut().as_mut() {
            shared.conn = ConnState::<Sec1>::new();
            shared.conn.set_transport("softap");
        }
        Ok(accepted)
    }
}

struct ProvHttp<'a, R> {
    state: &'a RefCell<Option<Shared<R>>>,
    session: &'a Session,
    aps: &'a [AccessPointInfo],
    pop: &'a str,
}

impl<R> edge_http::io::server::Handler for ProvHttp<'_, R>
where
    R: RngCore + CryptoRng,
{
    type Error<E>
        = edge_http::io::Error<E>
    where
        E: core::fmt::Debug;

    async fn handle<T, const N: usize>(
        &self,
        _task_id: impl core::fmt::Display + Copy,
        connection: &mut edge_http::io::server::Connection<'_, T, N>,
    ) -> Result<(), Self::Error<T::Error>>
    where
        T: Read + Write + edge_nal::TcpSplit,
    {
        let mut path_raw = [0u8; 48];
        let path_len = {
            let headers = connection.headers()?;
            let bytes = headers.path.as_bytes();
            let n = bytes.len().min(path_raw.len());
            path_raw[..n].copy_from_slice(&bytes[..n]);
            n
        };
        let path = core::str::from_utf8(&path_raw[..path_len]).unwrap_or("");

        let mut body = [0u8; 1024];
        let mut body_len = 0usize;
        {
            let (_headers, reader) = connection.split();
            while body_len < body.len() {
                let n = reader.read(&mut body[body_len..]).await?;
                if n == 0 {
                    break;
                }
                body_len += n;
            }
        }

        let Some(endpoint) = route(path) else {
            connection
                .initiate_response(404, Some("Not Found"), &[("Content-Length", "0")])
                .await?;
            return Ok(());
        };

        let mut response = [0u8; 1024];
        // Move the session out so X25519, SHA-256, and AES do not run while a
        // lock is held. The HTTP task does not await until the state is back.
        let Some(mut shared) = self.state.borrow_mut().take() else {
            connection
                .initiate_response(400, Some("Bad Request"), &[("Content-Length", "0")])
                .await?;
            return Ok(());
        };
        let reply = shared.conn.handle(
            endpoint,
            &body[..body_len],
            &mut response,
            self.session,
            self.aps,
            &mut shared.rng,
            self.pop.as_bytes(),
        );
        *self.state.borrow_mut() = Some(shared);

        match reply {
            Ok(reply) => {
                send_reply(connection, &response[..reply.len], reply.json).await?;
                if reply.complete {
                    connection.complete().await?;
                    sec1::log_report("softap", "GetStatus sent");
                    // finish() drops this task. Stay up until the response can leave.
                    Timer::after(Duration::from_secs(1)).await;
                    self.session.finish();
                }
            }
            Err(_) => {
                connection
                    .initiate_response(400, Some("Bad Request"), &[("Content-Length", "0")])
                    .await?;
            }
        }
        Ok(())
    }
}

async fn send_reply<T, const N: usize>(
    connection: &mut edge_http::io::server::Connection<'_, T, N>,
    body: &[u8],
    json: bool,
) -> Result<(), edge_http::io::Error<T::Error>>
where
    T: Read + Write + edge_nal::TcpSplit,
{
    let mut len_buf = heapless::String::<8>::new();
    let _ = write!(len_buf, "{}", body.len());
    let content_type = if json {
        "application/json"
    } else {
        "application/x-protobuf"
    };
    connection
        .initiate_response(
            200,
            Some("OK"),
            &[
                ("Content-Type", content_type),
                ("Content-Length", len_buf.as_str()),
            ],
        )
        .await?;
    connection.write_all(body).await?;
    Ok(())
}

fn route(path: &str) -> Option<Endpoint> {
    let path = path.split('?').next().unwrap_or(path);
    match path {
        "/proto-ver" => Some(Endpoint::ProtoVer),
        "/prov-session" => Some(Endpoint::Session),
        "/prov-config" => Some(Endpoint::Config),
        "/prov-scan" => Some(Endpoint::Scan),
        "/prov-ctrl" => Some(Endpoint::Ctrl),
        _ => None,
    }
}

async fn captive_dns(stack: Stack<'_>, gateway: Ipv4Addr) {
    let buffers = edge_nal_embassy::UdpBuffers::<2, 512, 512, 4>::new();
    let udp = edge_nal_embassy::Udp::new(stack, &buffers);
    let mut tx = [0u8; 512];
    let mut rx = [0u8; 512];
    if let Err(err) = edge_captive::io::run(
        &udp,
        SocketAddr::new(IpAddr::V4(Ipv4Addr::UNSPECIFIED), 53),
        &mut tx,
        &mut rx,
        gateway,
        core::time::Duration::from_secs(60),
    )
    .await
    {
        warn!(
            "[softap] captive DNS error: {}",
            crate::fmt::Debug2Format(&err)
        );
    }
}

async fn ap_dhcp(stack: Stack<'_>, gateway: Ipv4Addr) {
    use edge_dhcp::{
        io::{self, DEFAULT_SERVER_PORT},
        server::{Server, ServerOptions},
    };
    use edge_nal::UdpBind;

    let mut buf = [0u8; 1500];
    let mut gw_buf = [gateway];
    let buffers = edge_nal_embassy::UdpBuffers::<3, 1024, 1024, 10>::new();
    let unbound = edge_nal_embassy::Udp::new(stack, &buffers);
    let mut bound = match unbound
        .bind(SocketAddr::V4(SocketAddrV4::new(
            Ipv4Addr::UNSPECIFIED,
            DEFAULT_SERVER_PORT,
        )))
        .await
    {
        Ok(socket) => socket,
        Err(err) => {
            warn!(
                "[softap] DHCP bind failed: {}",
                crate::fmt::Debug2Format(&err)
            );
            return;
        }
    };
    debug!("[softap] DHCP on :67");

    loop {
        _ = io::server::run(
            &mut Server::<_, 64>::new_with_et(gateway),
            &ServerOptions::new(gateway, Some(&mut gw_buf)),
            &mut bound,
            &mut buf,
        )
        .await
        .inspect_err(|err| {
            warn!(
                "[softap] DHCP server error: {}",
                crate::fmt::Debug2Format(&err)
            );
        });
        Timer::after(Duration::from_millis(500)).await;
    }
}
