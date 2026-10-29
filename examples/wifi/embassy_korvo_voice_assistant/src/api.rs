//! The answer to a question, from Claude or from OpenAI, whichever has an API key.

use alloc::{format, string::String};
use core::fmt::{self, Write as _};

use allocator_api2::vec::Vec;
use embassy_net::{
    Stack,
    dns::{self, DnsQueryType, DnsSocket},
    tcp::{ConnectError, TcpSocket},
};
use embedded_io_async::Write;
use embedded_tls::{
    Aes128GcmSha256,
    Certificate,
    CryptoProvider,
    CryptoRngCore,
    NoClock,
    TlsConfig,
    TlsConnection,
    TlsContext,
    TlsError,
    TlsVerifier,
    pki::CertVerifier,
};
use esp_alloc::ExternalMemory;
use esp_hal::rng::Rng;
use rand_chacha::ChaCha20Rng;
use rand_core::SeedableRng;
use reqwless::{headers::ContentType, request::Method, response::Response};
use serde::Deserialize;
use serde_json::json;

const ANTHROPIC_API_KEY: &str = key(option_env!("ANTHROPIC_API_KEY"));
const OPENAI_API_KEY: &str = key(option_env!("OPENAI_API_KEY"));

/// An unset key is empty.
const fn key(key: Option<&'static str>) -> &'static str {
    match key {
        Some(key) => key,
        None => "",
    }
}

/// The service that answers questions: the one with an API key. Without one, the assistant only
/// handles the phrases that need no answer.
pub const SERVICE: Option<Service> = match (ANTHROPIC_API_KEY.is_empty(), OPENAI_API_KEY.is_empty())
{
    (false, true) => Some(Service::Anthropic),
    (true, false) => Some(Service::OpenAi),
    (true, true) => None,
    (false, false) => panic!("Set only one of ANTHROPIC_API_KEY and OPENAI_API_KEY"),
};

#[derive(Clone, Copy, Debug)]
pub enum Service {
    Anthropic,
    OpenAi,
}

impl Service {
    const fn host(self) -> &'static str {
        match self {
            Service::Anthropic => "api.anthropic.com",
            Service::OpenAi => "api.openai.com",
        }
    }

    const fn path(self) -> &'static str {
        match self {
            Service::Anthropic => "/v1/messages",
            Service::OpenAi => "/v1/chat/completions",
        }
    }
}

impl fmt::Display for Service {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Service::Anthropic => "Claude",
            Service::OpenAi => "OpenAI",
        })
    }
}

/// The root certificate both servers chain up to, in DER. `embedded-tls` checks the chain the
/// server sends against one certificate: the issuer of the last certificate in the chain. Both
/// servers end their chain with GTS Root R4 cross-signed by GlobalSign Root CA, so this is the
/// cross-signing root. It comes from the macOS system trust store.
///
/// GlobalSign Root CA, SHA-256 fingerprint
/// `EBD41040E4BB3EC742C9E381D31EF2A41A48B6685C96E7CEF3C1DF6CD4331C99`, valid until 2028-01-28.
const ROOT_CA: &[u8] = include_bytes!("../certs/globalsign-root-ca.der");

const CLAUDE_MODEL: &str = "claude-haiku-5-5";
const OPENAI_MODEL: &str = "gpt-5-mini";

const INSTRUCTIONS: &str = "You are a voice assistant running on an ESP32-S31 microcontroller. \
    Answer in one to three short sentences of plain text without markdown or emoji, because your \
    answer is shown on a small screen.";

/// The largest response body. An answer of 1024 tokens takes about 6 KiB.
const RESPONSE_BUFFER_SIZE: usize = 16 * 1024;
/// The response status line and headers.
const RESPONSE_HEADER_BUFFER_SIZE: usize = 4 * 1024;
/// The certificate chain of the server.
const CERTIFICATE_CHAIN_SIZE: usize = 8 * 1024;
/// One received TLS record: up to 16 KiB of data plus its overhead.
const TLS_READ_BUFFER_SIZE: usize = 16 * 1024 + 256;
/// `embedded-tls` puts the whole write buffer, minus the record overhead, into one record, which
/// must not carry more than 16 KiB of data.
const TLS_WRITE_BUFFER_SIZE: usize = 16 * 1024;
const TCP_BUFFER_SIZE: usize = 16 * 1024;

#[derive(Debug)]
pub enum Error {
    Dns(dns::Error),
    Connect(ConnectError),
    Tls(TlsError),
    Http(reqwless::Error),
    Api {
        service: Service,
        status: u16,
        message: String,
    },
    Json(serde_json::Error),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Dns(error) => write!(f, "DNS lookup failed: {error:?}"),
            Error::Connect(error) => write!(f, "TCP connection failed: {error:?}"),
            Error::Tls(error) => write!(f, "TLS failed: {error:?}"),
            Error::Http(error) => write!(f, "HTTP request failed: {error:?}"),
            Error::Api {
                service,
                status,
                message,
            } => write!(f, "{service} returned {status}: {message}"),
            Error::Json(error) => write!(f, "Unexpected response: {error}"),
        }
    }
}

impl From<dns::Error> for Error {
    fn from(error: dns::Error) -> Self {
        Error::Dns(error)
    }
}

impl From<ConnectError> for Error {
    fn from(error: ConnectError) -> Self {
        Error::Connect(error)
    }
}

impl From<TlsError> for Error {
    fn from(error: TlsError) -> Self {
        Error::Tls(error)
    }
}

impl From<reqwless::Error> for Error {
    fn from(error: reqwless::Error) -> Self {
        Error::Http(error)
    }
}

impl From<serde_json::Error> for Error {
    fn from(error: serde_json::Error) -> Self {
        Error::Json(error)
    }
}

pub struct Api {
    stack: Stack<'static>,
    tcp_rx_buffer: Vec<u8, ExternalMemory>,
    tcp_tx_buffer: Vec<u8, ExternalMemory>,
    tls_read_buffer: Vec<u8, ExternalMemory>,
    tls_write_buffer: Vec<u8, ExternalMemory>,
    response_header: Vec<u8, ExternalMemory>,
    response: Vec<u8, ExternalMemory>,
}

impl Api {
    pub fn new(stack: Stack<'static>) -> Self {
        Self {
            stack,
            tcp_rx_buffer: zeroed(TCP_BUFFER_SIZE),
            tcp_tx_buffer: zeroed(TCP_BUFFER_SIZE),
            tls_read_buffer: zeroed(TLS_READ_BUFFER_SIZE),
            tls_write_buffer: zeroed(TLS_WRITE_BUFFER_SIZE),
            response_header: zeroed(RESPONSE_HEADER_BUFFER_SIZE),
            response: zeroed(RESPONSE_BUFFER_SIZE),
        }
    }

    /// Answers the question.
    pub async fn chat(&mut self, service: Service, question: &str) -> Result<String, Error> {
        match service {
            Service::Anthropic => self.chat_anthropic(question).await,
            Service::OpenAi => self.chat_openai(question).await,
        }
    }

    async fn chat_anthropic(&mut self, question: &str) -> Result<String, Error> {
        #[derive(Deserialize)]
        struct Message {
            content: alloc::vec::Vec<Block>,
        }
        #[derive(Deserialize)]
        struct Block {
            /// Only `text` blocks have text.
            #[serde(default)]
            text: String,
        }

        let request = serde_json::to_vec(&json!({
            "model": CLAUDE_MODEL,
            "max_tokens": 1024,
            // Thinking delays the answer and can use up `max_tokens` before any text.
            "thinking": { "type": "disabled" },
            "system": INSTRUCTIONS,
            "messages": [{ "role": "user", "content": question }],
        }))?;
        let headers = [
            ("x-api-key", ANTHROPIC_API_KEY),
            ("anthropic-version", "2023-06-01"),
            ("Content-Type", ContentType::ApplicationJson.as_str()),
        ];
        let body = self.post(Service::Anthropic, &headers, &request).await?;
        let message = serde_json::from_slice::<Message>(body)?;
        Ok(message
            .content
            .into_iter()
            .map(|block| block.text)
            .collect())
    }

    async fn chat_openai(&mut self, question: &str) -> Result<String, Error> {
        #[derive(Deserialize)]
        struct Completion {
            choices: alloc::vec::Vec<Choice>,
        }
        #[derive(Deserialize)]
        struct Choice {
            message: Message,
        }
        #[derive(Deserialize)]
        struct Message {
            /// Missing when the model refuses.
            #[serde(default)]
            content: Option<String>,
        }

        let request = serde_json::to_vec(&json!({
            "model": OPENAI_MODEL,
            // Includes the reasoning tokens.
            "max_completion_tokens": 1024,
            // More reasoning delays the answer and can use up the tokens before any text.
            "reasoning_effort": "minimal",
            "messages": [
                { "role": "developer", "content": INSTRUCTIONS },
                { "role": "user", "content": question },
            ],
        }))?;
        let authorization = format!("Bearer {OPENAI_API_KEY}");
        let headers = [
            ("Authorization", authorization.as_str()),
            ("Content-Type", ContentType::ApplicationJson.as_str()),
        ];
        let body = self.post(Service::OpenAi, &headers, &request).await?;
        let completion = serde_json::from_slice::<Completion>(body)?;
        Ok(completion
            .choices
            .into_iter()
            .filter_map(|choice| choice.message.content)
            .collect())
    }

    /// Sends a POST request over a new TLS connection and returns the response body.
    ///
    /// The server certificate must chain up to [`ROOT_CA`] and name the host of `service`. Its
    /// validity period is not checked, because the board does not know the time.
    async fn post(
        &mut self,
        service: Service,
        headers: &[(&str, &str)],
        body: &[u8],
    ) -> Result<&[u8], Error> {
        let host = service.host();
        let addresses = DnsSocket::new(self.stack)
            .query(host, DnsQueryType::A)
            .await?;
        let Some(&address) = addresses.first() else {
            return Err(Error::Dns(dns::Error::Failed));
        };
        let mut socket =
            TcpSocket::new(self.stack, &mut self.tcp_rx_buffer, &mut self.tcp_tx_buffer);
        socket.connect((address, 443)).await?;

        let config = TlsConfig::new()
            .with_server_name(host)
            .with_ca(Certificate::X509(ROOT_CA));
        let mut tls = TlsConnection::<_, Aes128GcmSha256>::new(
            socket,
            &mut self.tls_read_buffer,
            &mut self.tls_write_buffer,
        );
        tls.open(TlsContext::new(&config, Provider::new())).await?;

        let mut head = format!(
            "POST {} HTTP/1.1\r\nHost: {host}\r\nContent-Length: {}\r\nConnection: close\r\n",
            service.path(),
            body.len()
        );
        for (name, value) in headers {
            write!(head, "{name}: {value}\r\n").unwrap();
        }
        head.push_str("\r\n");
        tls.write_all(head.as_bytes()).await?;
        tls.write_all(body).await?;
        tls.flush().await?;

        let response = Response::read(&mut tls, Method::POST, &mut self.response_header).await?;
        let status = response.status.0;
        let len = response
            .body()
            .reader()
            .read_to_end(&mut self.response)
            .await?;
        let body = &self.response[..len];

        if !(200..300).contains(&status) {
            return Err(Error::Api {
                service,
                status,
                message: error_message(body),
            });
        }
        Ok(body)
    }
}

/// Verifies the server certificate and provides the randomness of a TLS connection.
struct Provider {
    rng: ChaCha20Rng,
    verifier: CertVerifier<Aes128GcmSha256, NoClock, CERTIFICATE_CHAIN_SIZE>,
}

impl Provider {
    fn new() -> Self {
        let mut seed = [0; 32];
        Rng::new().read(&mut seed);
        Self {
            rng: ChaCha20Rng::from_seed(seed),
            verifier: CertVerifier::new(),
        }
    }
}

impl CryptoProvider for Provider {
    type CipherSuite = Aes128GcmSha256;
    /// Unused: the client does not authenticate itself.
    type Signature = &'static [u8];

    fn rng(&mut self) -> impl CryptoRngCore {
        &mut self.rng
    }

    fn verifier(&mut self) -> Result<&mut impl TlsVerifier<Aes128GcmSha256>, TlsError> {
        Ok(&mut self.verifier)
    }
}

fn zeroed(len: usize) -> Vec<u8, ExternalMemory> {
    let mut buffer = Vec::with_capacity_in(len, ExternalMemory);
    buffer.resize(len, 0);
    buffer
}

/// Extracts `error.message` from an error response.
fn error_message(body: &[u8]) -> String {
    #[derive(Deserialize)]
    struct Response {
        error: Error,
    }
    #[derive(Deserialize)]
    struct Error {
        message: String,
    }

    match serde_json::from_slice::<Response>(body) {
        Ok(response) => response.error.message,
        Err(_) => String::from_utf8_lossy(body).into_owned(),
    }
}
