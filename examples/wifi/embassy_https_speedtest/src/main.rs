//! Embassy HTTPS speed test example
//!
//!
//! Set SSID and PASSWORD env variable before running this example.
//!
//! This connects to a wifi network, then measures download and upload speed
//! against a local go-httpbin test server running on the same lan:
//!
//! - download: GET https://<server>/bytes/4194304 (4 MiB)
//! - upload:   PUT https://<server>/put (4 MiB)
//!
//! The Makefile next to this example runs the server and generates the
//! self-signed certificate that is embedded as the trust anchor (run
//! `source ~/export-esp.sh` first):
//!
//! ```text
//! make serve          # go-httpbin on 0.0.0.0:8443 (HTTPS)
//! make serve-http     # go-httpbin on 0.0.0.0:8080 (plain HTTP)
//! make run SSID=mywifi PASSWORD=secret  # build + flash, SERVER_IP auto-detected
//! ```
//!
//! The server address is baked into the firmware at compile time via the
//! SERVER_IP environment variable (the Makefile detects the first
//! non-loopback address and puts it into the certificate's CN/SAN). The
//! firmware defaults to 192.168.1.5 if SERVER_IP is not set. Certificate
//! validity dates are not checked (no wall clock is available).
//!
//! With the `plain-http` feature, TLS is disabled entirely and both
//! directions run over plain HTTP (port 8080), so the result measures the
//! wifi link without the encryption overhead.
//!
//! The payload buffer and the TLS record buffers are allocated from PSRAM,
//! so this example needs an ESP32-S3 with PSRAM.
//!
//! The test loops download/upload/download/upload until reset.

//% CHIP_FILTER: esp32s3

#![no_std]
#![no_main]

extern crate alloc;

use alloc::format;
use alloc::vec::Vec;

use embassy_executor::Spawner;
use embassy_net::{
    IpAddress,
    Ipv4Address,
    IpEndpoint,
    Runner,
    Stack,
    StackResources,
    dns::DnsQueryType,
    tcp::TcpSocket,
};
use embassy_time::{Duration, Instant, Timer};
use embedded_io_async::{Read as _, Write as _};
#[cfg(not(feature = "plain-http"))]
use embedded_tls::{
    Aes128GcmSha256,
    Certificate,
    CryptoProvider,
    FlushPolicy,
    NoClock,
    TlsConfig,
    TlsConnection,
    TlsContext,
    TlsError,
    TlsVerifier,
    pki::CertVerifier,
};
use esp_alloc as _;
use esp_backtrace as _;
use esp_hal::{
    clock::CpuClock,
    interrupt::software::SoftwareInterruptControl,
    rng::Rng,
    timer::timg::TimerGroup,
};
use esp_println::println;
use esp_radio::wifi::{
    Config,
    ControllerConfig,
    Interface,
    WifiController,
    sta::StationConfig,
};
#[cfg(not(feature = "plain-http"))]
use rand_core::{CryptoRng, RngCore};

esp_bootloader_esp_idf::esp_app_desc!();

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write(($val));
        x
    }};
}

const SSID: &str = env!("SSID");
const PASSWORD: &str = env!("PASSWORD");

// The test server: a local go-httpbin instance run on the same network.
const SERVER_IP: &str = match option_env!("SERVER_IP") {
    Some(ip) => ip,
    None => "192.168.1.5",
};
const DOWN_HOST: &str = SERVER_IP;
const DOWN_URL: &str = "/bytes";
const UP_HOST: &str = SERVER_IP;
const UP_URL: &str = "/put";
const UP_METHOD: &str = "PUT";
const TEST_BYTES: usize = 4 * 1024 * 1024;

#[cfg(not(feature = "plain-http"))]
const PORT: u16 = 8443;
#[cfg(feature = "plain-http")]
const PORT: u16 = 8080;

// TCP socket buffers: 32 KiB windows
const TCP_BUF_SIZE: usize = 32768;
#[cfg(not(feature = "plain-http"))]
// The read buffer must fit the largest encrypted TLS record: 16640 bytes.
const TLS_READ_BUF_SIZE: usize = 16640;
#[cfg(not(feature = "plain-http"))]
// The write buffer is sized so a record's plaintext content never exceeds the TLS 1.3 limit of
// 16384 bytes
const TLS_WRITE_BUF_SIZE: usize = 16384 + embedded_tls::TLS_RECORD_OVERHEAD + 5;
const HEADER_BUF_SIZE: usize = 4096;

// The self-signed certificate of the local test server
#[cfg(not(feature = "plain-http"))]
const CA_CERT: &[u8] = include_bytes!("../server-cert.der");

#[cfg(not(feature = "plain-http"))]
type TlsConn<'a> = TlsConnection<'a, TcpSocket<'a>, Aes128GcmSha256>;

// The connection to the server: a raw TCP socket, or a TLS session over one.
enum Transport<'a> {
    #[cfg_attr(not(feature = "plain-http"), allow(dead_code))]
    Tcp(TcpSocket<'a>),
    #[cfg(not(feature = "plain-http"))]
    Tls(TlsConn<'a>),
}

#[derive(Debug)]
enum TransportError {
    Tcp(embassy_net::tcp::Error),
    #[cfg(not(feature = "plain-http"))]
    Tls(TlsError),
}

impl core::fmt::Display for TransportError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Tcp(e) => write!(f, "{e:?}"),
            #[cfg(not(feature = "plain-http"))]
            Self::Tls(e) => write!(f, "{e:?}"),
        }
    }
}

impl core::error::Error for TransportError {}

impl embedded_io::Error for TransportError {
    fn kind(&self) -> embedded_io::ErrorKind {
        match self {
            Self::Tcp(e) => e.kind(),
            #[cfg(not(feature = "plain-http"))]
            Self::Tls(e) => match e {
                TlsError::Io(kind) => *kind,
                _ => embedded_io::ErrorKind::Other,
            },
        }
    }
}

impl embedded_io_async::ErrorType for Transport<'_> {
    type Error = TransportError;
}

impl embedded_io_async::Read for Transport<'_> {
    async fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        match self {
            Self::Tcp(s) => s.read(buf).await.map_err(TransportError::Tcp),
            #[cfg(not(feature = "plain-http"))]
            Self::Tls(t) => t.read(buf).await.map_err(TransportError::Tls),
        }
    }
}

impl embedded_io_async::Write for Transport<'_> {
    async fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        match self {
            Self::Tcp(s) => s.write(buf).await.map_err(TransportError::Tcp),
            #[cfg(not(feature = "plain-http"))]
            Self::Tls(t) => t.write(buf).await.map_err(TransportError::Tls),
        }
    }

    async fn flush(&mut self) -> Result<(), Self::Error> {
        match self {
            Self::Tcp(s) => s.flush().await.map_err(TransportError::Tcp),
            #[cfg(not(feature = "plain-http"))]
            Self::Tls(t) => t.flush().await.map_err(TransportError::Tls),
        }
    }
}

// Newtype so we can implement the foreign `CryptoRng` marker trait.
#[cfg(not(feature = "plain-http"))]
struct WifiRng(Rng);

#[cfg(not(feature = "plain-http"))]
impl RngCore for WifiRng {
    fn next_u32(&mut self) -> u32 {
        self.0.random()
    }

    fn next_u64(&mut self) -> u64 {
        (self.0.random() as u64) << 32 | self.0.random() as u64
    }

    fn fill_bytes(&mut self, dest: &mut [u8]) {
        for chunk in dest.chunks_mut(4) {
            let val = self.0.random().to_le_bytes();
            let n = chunk.len().min(4);
            chunk.copy_from_slice(&val[..n]);
        }
    }

    fn try_fill_bytes(&mut self, dest: &mut [u8]) -> Result<(), rand_core::Error> {
        self.fill_bytes(dest);
        Ok(())
    }
}

#[cfg(not(feature = "plain-http"))]
impl CryptoRng for WifiRng {}

#[cfg(not(feature = "plain-http"))]
struct VerifyingProvider {
    rng: WifiRng,
    // NB NoClock: certificate expiry dates are not checked, only the chain signatures and the hostname.
    verifier: CertVerifier<'static, Aes128GcmSha256, NoClock, 4096>,
}

#[cfg(not(feature = "plain-http"))]
impl CryptoProvider for VerifyingProvider {
    type CipherSuite = Aes128GcmSha256;
    type Signature = heapless::Vec<u8, 128>;

    fn rng(&mut self) -> impl rand_core::CryptoRngCore {
        &mut self.rng
    }

    fn verifier(&mut self) -> Result<&mut impl TlsVerifier<Self::CipherSuite>, TlsError> {
        Ok(&mut self.verifier)
    }
}

#[derive(Debug)]
enum Error {
    Connect(embassy_net::tcp::ConnectError),
    Transport(TransportError),
    Dns,
    Http,
    Timeout,
}

impl From<embassy_net::tcp::ConnectError> for Error {
    fn from(e: embassy_net::tcp::ConnectError) -> Self {
        Self::Connect(e)
    }
}


#[esp_hal::main]
async fn main(spawner: Spawner) -> ! {
    esp_println::logger::init_logger_from_env();
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(size: 64 * 1024);
    esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    let sw_int = SoftwareInterruptControl::new(peripherals.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    let station_config = Config::Station(
        StationConfig::default()
            .with_ssid(SSID)
            .with_password(PASSWORD.into()),
    );

    println!("Starting wifi");
    let wifi_interface = Interface::station();
    let controller = WifiController::new(
        peripherals.WIFI,
        ControllerConfig::default().with_initial_config(station_config),
    )
    .unwrap();
    println!("Wifi configured and started!");

    let rng = Rng::new();
    let seed = (rng.random() as u64) << 32 | rng.random() as u64;

    let (stack, runner) = embassy_net::new(
        wifi_interface,
        embassy_net::Config::dhcpv4(Default::default()),
        mk_static!(StackResources<3>, StackResources::<3>::new()),
        seed,
    );

    spawner.spawn(connection_task(controller).unwrap());
    spawner.spawn(net_task(runner).unwrap());

    stack.wait_config_up().await;
    if let Some(config) = stack.config_v4() {
        println!("Got IP: {}", config.address);
    }

    let mut payload = Vec::with_capacity(TEST_BYTES);
    payload.resize(TEST_BYTES, 0);

    let tcp_rx_buf = mk_static!([u8; TCP_BUF_SIZE], [0u8; TCP_BUF_SIZE]);
    let tcp_tx_buf = mk_static!([u8; TCP_BUF_SIZE], [0u8; TCP_BUF_SIZE]);

    // The TLS record and header buffers live in internal RAM: the software
    // AES-GCM decrypt is much faster there than in PSRAM, and a slow reader
    // makes the server's streaming write time out (truncating the body).
    #[cfg(not(feature = "plain-http"))]
    let tls_read_buf = mk_static!([u8; TLS_READ_BUF_SIZE], [0u8; TLS_READ_BUF_SIZE]);
    #[cfg(not(feature = "plain-http"))]
    let tls_write_buf = mk_static!([u8; TLS_WRITE_BUF_SIZE], [0u8; TLS_WRITE_BUF_SIZE]);
    let header_buf = mk_static!([u8; HEADER_BUF_SIZE], [0u8; HEADER_BUF_SIZE]);

    let mut pass = 0u32;
    loop {
        pass += 1;

        println!("--- pass {pass}: download ---");
        match run_download(
            &stack,
            &mut payload,
            tcp_rx_buf,
            tcp_tx_buf,
            #[cfg(not(feature = "plain-http"))]
            tls_read_buf,
            #[cfg(not(feature = "plain-http"))]
            tls_write_buf,
            header_buf,
        )
        .await
        {
            Ok(mbps) => println!("download: {:.2} Mbit/s", mbps),
            Err(e) => report_failure("download", e),
        }

        Timer::after(Duration::from_millis(500)).await;

        println!("--- pass {pass}: upload ---");
        match run_upload(
            &stack,
            &payload,
            tcp_rx_buf,
            tcp_tx_buf,
            #[cfg(not(feature = "plain-http"))]
            tls_read_buf,
            #[cfg(not(feature = "plain-http"))]
            tls_write_buf,
            header_buf,
        )
        .await
        {
            Ok(mbps) => println!("upload: {:.2} Mbit/s", mbps),
            Err(e) => report_failure("upload", e),
        }

        Timer::after(Duration::from_millis(500)).await;
    }
}


#[embassy_executor::task]
async fn connection_task(mut controller: WifiController<'static>) {
    println!("start connection task");

    loop {
        println!("About to connect...");

        match controller.connect_async().await {
            Ok(info) => {
                println!("Wifi connected to {:?}", info);

                // wait until we're no longer connected
                let info = controller.wait_for_disconnect_async().await.ok();
                println!("Disconnected: {:?}", info);
            }
            Err(e) => {
                println!("Failed to connect to wifi: {e:?}");
            }
        }

        Timer::after(Duration::from_millis(5000)).await
    }
}

#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, Interface>) {
    runner.run().await
}


async fn connect<'a>(
    stack: &Stack<'static>,
    host: &'static str,
    tcp_rx_buf: &'a mut [u8],
    tcp_tx_buf: &'a mut [u8],
    #[cfg(not(feature = "plain-http"))] ca_cert: &'static [u8],
    #[cfg(not(feature = "plain-http"))] read_buf: &'a mut [u8],
    #[cfg(not(feature = "plain-http"))] write_buf: &'a mut [u8],
) -> Result<Transport<'a>, Error> {
    let addrs = stack
        .dns_query(host, DnsQueryType::A)
        .await
        .map_err(|_| Error::Dns)?;
    let addr: Ipv4Address = addrs
        .first()
        .and_then(|a| match a {
            IpAddress::Ipv4(v4) => Some(*v4),
        })
        .ok_or(Error::Dns)?;

    let mut socket = TcpSocket::new(*stack, tcp_rx_buf, tcp_tx_buf);
    socket
        .connect(IpEndpoint::from((addr, PORT)))
        .await
        .map_err(Error::Connect)?;

    #[cfg(not(feature = "plain-http"))]
    {
        // Box::pin keeps the large handshake future off the task stack.
        let mut tls =
            alloc::boxed::Box::pin(tls_open_helper(socket, host, ca_cert, read_buf, write_buf))
                .await
                .map_err(|e| Error::Transport(TransportError::Tls(e)))?;

        // Relaxed: TLS records pipeline to the TCP stack without an ACK-wait per
        // record, which matters for upload throughput.
        tls.set_flush_policy(FlushPolicy::Relaxed);

        Ok(Transport::Tls(tls))
    }
    #[cfg(feature = "plain-http")]
    {
        Ok(Transport::Tcp(socket))
    }
}

#[cfg(not(feature = "plain-http"))]
async fn tls_open_helper<'a>(
    tcp: TcpSocket<'a>,
    host: &'static str,
    ca_cert: &'static [u8],
    read_buf: &'a mut [u8],
    write_buf: &'a mut [u8],
) -> Result<TlsConn<'a>, TlsError> {
    let mut tls = TlsConnection::new(tcp, read_buf, write_buf);
    let config = TlsConfig::new()
        .with_server_name(host)
        .with_alpn(&[b"http/1.1"]);
    let provider = VerifyingProvider {
        rng: WifiRng(Rng::new()),
        verifier: CertVerifier::new(Certificate::X509(ca_cert)),
    };
    tls.open(TlsContext::new(&config, provider)).await?;
    Ok(tls)
}

async fn run_download<'a>(
    stack: &Stack<'static>,
    payload: &mut [u8],
    tcp_rx_buf: &'a mut [u8],
    tcp_tx_buf: &'a mut [u8],
    #[cfg(not(feature = "plain-http"))] read_buf: &'a mut [u8],
    #[cfg(not(feature = "plain-http"))] write_buf: &'a mut [u8],
    header_buf: &'a mut [u8],
) -> Result<f32, Error> {
    let mut transport = connect(
        stack,
        DOWN_HOST,
        tcp_rx_buf,
        tcp_tx_buf,
        #[cfg(not(feature = "plain-http"))]
        CA_CERT,
        #[cfg(not(feature = "plain-http"))]
        read_buf,
        #[cfg(not(feature = "plain-http"))]
        write_buf,
    )
    .await?;

    let request = format!(
        "GET {DOWN_URL}/{TEST_BYTES} HTTP/1.1\r\nHost: {DOWN_HOST}\r\nConnection: close\r\nAccept: */*\r\n\r\n"
    );
    transport
        .write_all(request.as_bytes())
        .await
        .map_err(Error::Transport)?;
    transport.flush().await.map_err(Error::Transport)?;

    let (status, content_length, head_end, head_len) =
        read_head(&mut transport, header_buf).await?;
    if content_length == 0 {
        println!("download response has no Content-Length header");
    }
    if (status != 200 && status != 206) || content_length == 0 || content_length > payload.len() {
        print_response_body(&mut transport, header_buf).await;
        return Err(Error::Http);
    }

    // Body bytes that arrived in the same chunk as the headers are stored in
    // the header buffer; keep them.
    let mut received = 0;
    if head_len > head_end {
        let seed = (head_len - head_end).min(payload.len());
        payload[..seed].copy_from_slice(&header_buf[head_end..head_end + seed]);
        received = seed;
    }

    let start = Instant::now();
    let mut last_log = Instant::now();
    loop {
        if received == content_length {
            break;
        }
        // The server occasionally stops delivering data partway through the
        // body; time out so the outer loop can retry on the next pass.
        let result = embassy_time::with_timeout(
            Duration::from_secs(5),
            transport.read(&mut payload[received..]),
        )
        .await;
        match result {
            Ok(Ok(0)) => {
                print_socket_state(transport).await;
                return Err(Error::Http);
            }
            Ok(Ok(n)) => {
                received += n;
                if last_log.elapsed() > Duration::from_secs(1) {
                    println!(
                        "downloading {received} / {content_length} bytes, {:.2} Mbit/s",
                        mbps(received, start.elapsed())
                    );
                    last_log = Instant::now();
                }
            }
            Ok(Err(e)) => {
                print_socket_state(transport).await;
                return Err(Error::Transport(e));
            }
            Err(_) => return Err(Error::Timeout),
        }
    }
    let elapsed = start.elapsed();

    println!("downloaded {received} bytes in {} ms", elapsed.as_millis());
    Ok(mbps(received, elapsed))
}

async fn run_upload<'a>(
    stack: &Stack<'static>,
    payload: &[u8],
    tcp_rx_buf: &'a mut [u8],
    tcp_tx_buf: &'a mut [u8],
    #[cfg(not(feature = "plain-http"))] read_buf: &'a mut [u8],
    #[cfg(not(feature = "plain-http"))] write_buf: &'a mut [u8],
    header_buf: &'a mut [u8],
) -> Result<f32, Error> {
    let mut transport = connect(
        stack,
        UP_HOST,
        tcp_rx_buf,
        tcp_tx_buf,
        #[cfg(not(feature = "plain-http"))]
        CA_CERT,
        #[cfg(not(feature = "plain-http"))]
        read_buf,
        #[cfg(not(feature = "plain-http"))]
        write_buf,
    )
    .await?;

    // The server parses the body as a form unless the content type marks it
    // as opaque binary data.
    let request = format!(
        "{UP_METHOD} {UP_URL} HTTP/1.1\r\nHost: {UP_HOST}\r\nContent-Length: {}\r\nContent-Type: application/octet-stream\r\nConnection: close\r\n\r\n",
        payload.len()
    );
    transport
        .write_all(request.as_bytes())
        .await
        .map_err(Error::Transport)?;
    transport.flush().await.map_err(Error::Transport)?;

    let start = Instant::now();
    let mut last_log = Instant::now();
    let mut sent = 0;
    while sent < payload.len() {
        let n = transport.write(&payload[sent..]).await;
        match n {
            Ok(0) => {
                // Nothing was buffered: the record buffer was full and just got flushed, so a
                // retry will make progress.
                continue;
            }
            Ok(n) => {
                sent += n;
                if last_log.elapsed() > Duration::from_secs(1) {
                    println!(
                        "uploading {sent} / {} bytes, {:.2} Mbit/s",
                        payload.len(),
                        mbps(sent, start.elapsed())
                    );
                    last_log = Instant::now();
                }
            }
            Err(e) => {
                print_socket_state(transport).await;
                return Err(Error::Transport(e));
            }
        }
    }
    transport.flush().await.map_err(Error::Transport)?;
    let elapsed = start.elapsed();

    let (status, _, _, _) = read_head(&mut transport, header_buf).await?;
    println!("upload response status: {status}");
    if status != 200 {
        print_response_body(&mut transport, header_buf).await;
        return Err(Error::Http);
    }

    println!("uploaded {sent} bytes in {} ms", elapsed.as_millis());
    Ok(mbps(sent, elapsed))
}

// Reads the status line and headers. Returns the status code, the Content-Length value, the
// position right after the header terminator and the total number of bytes read: a plain TCP read
// can deliver the start of the body in the same chunk as the headers, and the caller must treat
// those bytes as body data.
async fn read_head(
    transport: &mut Transport<'_>,
    buf: &mut [u8],
) -> Result<(u16, usize, usize, usize), Error> {
    let mut len = 0;
    let head_end = loop {
        let n = transport.read(&mut buf[len..]).await.map_err(Error::Transport)?;
        if n == 0 {
            println!("read_head: connection closed before the response headers arrived");
            return Err(Error::Http);
        }
        len += n;
        if let Some(pos) = buf[..len].windows(4).position(|w| w == b"\r\n\r\n") {
            break pos + 4;
        }
        if len == buf.len() {
            println!("read_head: response headers exceed {len} bytes");
            return Err(Error::Http);
        }
    };

    let head = &buf[..head_end];
    let status_line_end = head.iter().position(|&b| b == b'\r').ok_or(Error::Http)?;
    let status_line = core::str::from_utf8(&head[..status_line_end])
        .map_err(|_| {
            println!("read_head: non-utf8 status line");
            Error::Http
        })?;
    println!("response head: {status_line}");
    let status: u16 = status_line
        .split(' ')
        .nth(1)
        .ok_or(Error::Http)?
        .parse()
        .map_err(|_| {
            println!("read_head: could not parse status code");
            Error::Http
        })?;

    let mut content_length = None;
    for line in head.split(|&b| b == b'\n') {
        let line = line.strip_suffix(b"\r").unwrap_or(line);
        if let Some(rest) = line.strip_prefix(b"Content-Length:") {
            content_length = Some(
                core::str::from_utf8(rest.trim_ascii())
                    .map_err(|_| Error::Http)?
                    .parse()
                    .map_err(|_| Error::Http)?,
            );
        }
    }

    Ok((status, content_length.unwrap_or(0), head_end, len))
}
fn mbps(bytes: usize, elapsed: Duration) -> f32 {
    (bytes as f32) * 8.0 / 1_000_000.0 / (elapsed.as_millis() as f32 / 1000.0)
}

// Consumes the transport, recovers the raw socket and reports its TCP state. A non-empty rx queue
// at this point means the stack still held undelivered bytes when the transfer failed.
async fn print_socket_state(transport: Transport<'_>) {
    match transport {
        Transport::Tcp(s) => {
            println!(
                "tcp state: {:?} rxq={} txq={}",
                s.state(),
                s.recv_queue(),
                s.send_queue()
            );
        }
        #[cfg(not(feature = "plain-http"))]
        Transport::Tls(t) => match t.close().await {
            Ok(s) => println!(
                "tcp state: {:?} rxq={} txq={}",
                s.state(),
                s.recv_queue(),
                s.send_queue()
            ),
            Err((s, _)) => println!(
                "tcp state: {:?} rxq={} txq={}",
                s.state(),
                s.recv_queue(),
                s.send_queue()
            ),
        },
    }
}

// Reads and prints the body of an unexpected response (e.g. a 400 with an error detail), up to the
// size of `buf`.
async fn print_response_body(transport: &mut Transport<'_>, buf: &mut [u8]) {
    let mut got = 0;
    loop {
        match transport.read(&mut buf[got..]).await {
            Ok(0) | Err(_) => break,
            Ok(n) => {
                got += n;
                if got >= buf.len() {
                    break;
                }
            }
        }
    }
    if got > 0 {
        println!(
            "response body: {}",
            core::str::from_utf8(&buf[..got]).unwrap_or("(non-utf8)")
        );
    }
}

fn report_failure(what: &str, e: Error) {
    match e {
        Error::Connect(e) => println!("{what} failed: connect error: {e:?}"),
        Error::Transport(e) => println!("{what} failed: transport error: {e:?}"),
        Error::Dns => println!("{what} failed: dns resolution error"),
        Error::Http => println!("{what} failed: http protocol error"),
        Error::Timeout => println!("{what} failed: server stopped sending data (timeout)"),
    }
}
