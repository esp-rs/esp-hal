//! The three API calls of a conversation turn: speech to text (Deepgram), the answer (Claude), and
//! text to speech (Deepgram).

use alloc::string::String;
use core::fmt;

use allocator_api2::vec::Vec;
use embassy_net::{
    Stack,
    dns::DnsSocket,
    tcp::client::{TcpClient, TcpClientState},
};
use embedded_io_async::Write;
use esp_alloc::ExternalMemory;
use esp_hal::rng::Rng;
use reqwless::{
    client::{HttpClient, TlsConfig, TlsVerify},
    headers::ContentType,
    request::{Method, RequestBody, RequestBuilder},
};
use serde::Deserialize;
use serde_json::json;

use crate::audio::{PLAYBACK_SAMPLE_RATE, RECORDING_SAMPLE_RATE};

const DEEPGRAM_AUTHORIZATION: &str = concat!("Token ", env!("DEEPGRAM_API_KEY"));
const CLAUDE_API_KEY: &str = env!("CLAUDE_API_KEY");

/// Raw 16-bit 16 kHz mono speech in, text out.
const LISTEN_URL: &str = "https://api.deepgram.com/v1/listen?model=nova-3&smart_format=true\
    &encoding=linear16&sample_rate=16000&channels=1";
const MESSAGES_URL: &str = "https://api.anthropic.com/v1/messages";
/// Text in, raw 16-bit 24 kHz mono speech out.
const SPEAK_URL: &str = "https://api.deepgram.com/v1/speak?model=aura-2-thalia-en\
    &encoding=linear16&sample_rate=24000&container=none";

const CLAUDE_MODEL: &str = "claude-haiku-5-5";

const INSTRUCTIONS: &str = "You are a voice assistant running on an ESP32-S31 microcontroller. \
    Answer in one to three short sentences of plain text without markdown or emoji, because your \
    answer is spoken aloud and shown on a small screen.";

/// The largest response body. It limits the spoken answer to about 40 seconds.
const RESPONSE_BUFFER_SIZE: usize = 2 * 1024 * 1024;
/// One received TLS record: up to 16 KiB of data plus its overhead.
const TLS_READ_BUFFER_SIZE: usize = 16 * 1024 + 256;
/// `embedded-tls` puts the whole write buffer, minus the record overhead, into one record, which
/// must not carry more than 16 KiB of data.
const TLS_WRITE_BUFFER_SIZE: usize = 16 * 1024;
const TCP_BUFFER_SIZE: usize = 16 * 1024;

#[derive(Debug)]
pub enum Error {
    Http(reqwless::Error),
    Api {
        host: String,
        status: u16,
        message: String,
    },
    Json(serde_json::Error),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Error::Http(error) => write!(f, "HTTP request failed: {error:?}"),
            Error::Api {
                host,
                status,
                message,
            } => write!(f, "{host} returned {status}: {message}"),
            Error::Json(error) => write!(f, "Unexpected response: {error}"),
        }
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
    tcp: TcpClient<'static, 1, TCP_BUFFER_SIZE, TCP_BUFFER_SIZE>,
    dns: DnsSocket<'static>,
    tls_read_buffer: Vec<u8, ExternalMemory>,
    tls_write_buffer: Vec<u8, ExternalMemory>,
    response: Vec<u8, ExternalMemory>,
}

impl Api {
    pub fn new(stack: Stack<'static>, tcp_state: &'static TcpState) -> Self {
        Self {
            tcp: TcpClient::new(stack, tcp_state),
            dns: DnsSocket::new(stack),
            tls_read_buffer: zeroed(TLS_READ_BUFFER_SIZE),
            tls_write_buffer: zeroed(TLS_WRITE_BUFFER_SIZE),
            response: zeroed(RESPONSE_BUFFER_SIZE),
        }
    }

    /// Converts 16 kHz mono speech to text.
    pub async fn transcribe(&mut self, speech: &[i16]) -> Result<String, Error> {
        const _: () = assert!(RECORDING_SAMPLE_RATE == 16_000, "See `LISTEN_URL`");

        #[derive(Deserialize)]
        struct Response {
            results: Results,
        }
        #[derive(Deserialize)]
        struct Results {
            channels: [Channel; 1],
        }
        #[derive(Deserialize)]
        struct Channel {
            alternatives: [Alternative; 1],
        }
        #[derive(Deserialize)]
        struct Alternative {
            transcript: String,
        }

        let headers = [
            ("Authorization", DEEPGRAM_AUTHORIZATION),
            ("Content-Type", ContentType::ApplicationOctetStream.as_str()),
        ];
        let body = self.post(LISTEN_URL, &headers, Pcm(speech)).await?;
        let [channel] = serde_json::from_slice::<Response>(body)?.results.channels;
        let [alternative] = channel.alternatives;
        Ok(alternative.transcript)
    }

    /// Answers the question.
    pub async fn chat(&mut self, question: &str) -> Result<String, Error> {
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
            ("x-api-key", CLAUDE_API_KEY),
            ("anthropic-version", "2023-06-01"),
            ("Content-Type", ContentType::ApplicationJson.as_str()),
        ];
        let body = self
            .post(MESSAGES_URL, &headers, request.as_slice())
            .await?;
        let message = serde_json::from_slice::<Message>(body)?;
        Ok(message
            .content
            .into_iter()
            .map(|block| block.text)
            .collect())
    }

    /// Converts text to 24 kHz mono speech.
    pub async fn speak(&mut self, text: &str) -> Result<Vec<i16, ExternalMemory>, Error> {
        const _: () = assert!(PLAYBACK_SAMPLE_RATE == 24_000, "See `SPEAK_URL`");

        let request = serde_json::to_vec(&json!({ "text": text }))?;
        let headers = [
            ("Authorization", DEEPGRAM_AUTHORIZATION),
            ("Content-Type", ContentType::ApplicationJson.as_str()),
        ];
        let body = self.post(SPEAK_URL, &headers, request.as_slice()).await?;

        let mut speech = Vec::with_capacity_in(body.len() / 2, ExternalMemory);
        speech.extend(
            body.as_chunks::<2>()
                .0
                .iter()
                .map(|&bytes| i16::from_le_bytes(bytes)),
        );
        Ok(speech)
    }

    /// Sends a POST request over a new TLS connection and returns the response body.
    async fn post(
        &mut self,
        url: &str,
        headers: &[(&str, &str)],
        body: impl RequestBody,
    ) -> Result<&[u8], Error> {
        let rng = Rng::new();
        let seed = u64::from(rng.random()) << 32 | u64::from(rng.random());
        // TODO: verify the server certificate.
        let tls = TlsConfig::new(
            seed,
            &mut self.tls_read_buffer,
            &mut self.tls_write_buffer,
            TlsVerify::None,
        );
        let mut client = HttpClient::new_with_tls(&self.tcp, &self.dns, tls);

        let mut request = client
            .request(Method::POST, url)
            .await?
            .headers(headers)
            .body(body);
        let response = request.send(&mut self.response).await?;
        let status = response.status.0;
        let body = response.body().read_to_end().await?;

        if !(200..300).contains(&status) {
            return Err(Error::Api {
                host: url.split('/').nth(2).unwrap_or(url).into(),
                status,
                message: error_message(body),
            });
        }
        Ok(body)
    }
}

pub type TcpState = TcpClientState<1, TCP_BUFFER_SIZE, TCP_BUFFER_SIZE>;

fn zeroed(len: usize) -> Vec<u8, ExternalMemory> {
    let mut buffer = Vec::with_capacity_in(len, ExternalMemory);
    buffer.resize(len, 0);
    buffer
}

/// Extracts the message from a Claude (`error.message`) or Deepgram (`err_msg`) error response.
fn error_message(body: &[u8]) -> String {
    #[derive(Deserialize)]
    struct Response {
        error: Option<Error>,
        err_msg: Option<String>,
    }
    #[derive(Deserialize)]
    struct Error {
        message: String,
    }

    match serde_json::from_slice::<Response>(body) {
        Ok(Response {
            error: Some(error), ..
        }) => error.message,
        Ok(Response {
            err_msg: Some(message),
            ..
        }) => message,
        _ => String::from_utf8_lossy(body).into_owned(),
    }
}

/// Speech as little-endian 16-bit samples.
struct Pcm<'a>(&'a [i16]);

impl RequestBody for Pcm<'_> {
    fn len(&self) -> Option<usize> {
        Some(self.0.len() * 2)
    }

    async fn write<W: Write>(&self, writer: &mut W) -> Result<(), W::Error> {
        let mut bytes = [0; 1024];
        for samples in self.0.chunks(bytes.len() / 2) {
            for (bytes, sample) in bytes.as_chunks_mut::<2>().0.iter_mut().zip(samples) {
                *bytes = sample.to_le_bytes();
            }
            writer.write_all(&bytes[..samples.len() * 2]).await?;
        }
        Ok(())
    }
}
