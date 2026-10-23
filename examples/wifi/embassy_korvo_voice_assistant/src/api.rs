//! The three API calls of a conversation turn: speech to text (OpenAI), the answer (Claude), and
//! text to speech (OpenAI).

use alloc::{format, string::String};
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

const OPENAI_AUTHORIZATION: &str = concat!("Bearer ", env!("OPENAI_API_KEY"));
const CLAUDE_API_KEY: &str = env!("CLAUDE_API_KEY");

const TRANSCRIPTION_URL: &str = "https://api.openai.com/v1/audio/transcriptions";
const MESSAGES_URL: &str = "https://api.anthropic.com/v1/messages";
const SPEECH_URL: &str = "https://api.openai.com/v1/audio/speech";

const TRANSCRIPTION_MODEL: &str = "gpt-4o-mini-transcribe";
const CLAUDE_MODEL: &str = "claude-haiku-5-5";
const SPEECH_MODEL: &str = "gpt-4o-mini-tts";
const VOICE: &str = "alloy";

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
        #[derive(Deserialize)]
        struct Transcription {
            text: String,
        }

        let content_type = format!("multipart/form-data; boundary={BOUNDARY}");
        let headers = [
            ("Authorization", OPENAI_AUTHORIZATION),
            ("Content-Type", content_type.as_str()),
        ];
        let body = self
            .post(TRANSCRIPTION_URL, &headers, WavUpload::new(speech))
            .await?;
        Ok(serde_json::from_slice::<Transcription>(body)?.text)
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
            "max_tokens": 300,
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
        const _: () = assert!(
            PLAYBACK_SAMPLE_RATE == 24_000,
            "OpenAI sends `pcm` at 24 kHz"
        );

        let request = serde_json::to_vec(&json!({
            "model": SPEECH_MODEL,
            "voice": VOICE,
            "input": text,
            "response_format": "pcm",
        }))?;
        let headers = [
            ("Authorization", OPENAI_AUTHORIZATION),
            ("Content-Type", ContentType::ApplicationJson.as_str()),
        ];
        let body = self.post(SPEECH_URL, &headers, request.as_slice()).await?;

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

/// Extracts `error.message` from an OpenAI or Claude error response.
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

const BOUNDARY: &str = "esp-hal-voice-assistant";
const WAV_HEADER_LEN: usize = 44;

/// A multipart form with the transcription model and the speech as a WAV file.
struct WavUpload<'a> {
    head: String,
    speech: &'a [i16],
    tail: String,
}

impl<'a> WavUpload<'a> {
    fn new(speech: &'a [i16]) -> Self {
        let head = format!(
            "--{BOUNDARY}\r\n\
             Content-Disposition: form-data; name=\"model\"\r\n\r\n\
             {TRANSCRIPTION_MODEL}\r\n\
             --{BOUNDARY}\r\n\
             Content-Disposition: form-data; name=\"file\"; filename=\"speech.wav\"\r\n\
             Content-Type: audio/wav\r\n\r\n"
        );
        let tail = format!("\r\n--{BOUNDARY}--\r\n");
        Self { head, speech, tail }
    }

    fn data_len(&self) -> usize {
        self.speech.len() * 2
    }

    /// A RIFF header for 16-bit mono PCM.
    fn wav_header(&self) -> [u8; WAV_HEADER_LEN] {
        let data_len = self.data_len() as u32;
        let byte_rate = RECORDING_SAMPLE_RATE * 2;

        let mut header = [0; WAV_HEADER_LEN];
        header[0..4].copy_from_slice(b"RIFF");
        header[4..8].copy_from_slice(&(36 + data_len).to_le_bytes());
        header[8..16].copy_from_slice(b"WAVEfmt ");
        header[16..20].copy_from_slice(&16u32.to_le_bytes());
        header[20..22].copy_from_slice(&1u16.to_le_bytes()); // PCM
        header[22..24].copy_from_slice(&1u16.to_le_bytes()); // mono
        header[24..28].copy_from_slice(&RECORDING_SAMPLE_RATE.to_le_bytes());
        header[28..32].copy_from_slice(&byte_rate.to_le_bytes());
        header[32..34].copy_from_slice(&2u16.to_le_bytes()); // bytes per frame
        header[34..36].copy_from_slice(&16u16.to_le_bytes()); // bits per sample
        header[36..40].copy_from_slice(b"data");
        header[40..44].copy_from_slice(&data_len.to_le_bytes());
        header
    }
}

impl RequestBody for WavUpload<'_> {
    fn len(&self) -> Option<usize> {
        Some(self.head.len() + WAV_HEADER_LEN + self.data_len() + self.tail.len())
    }

    async fn write<W: Write>(&self, writer: &mut W) -> Result<(), W::Error> {
        writer.write_all(self.head.as_bytes()).await?;
        writer.write_all(&self.wav_header()).await?;

        let mut bytes = [0; 1024];
        for samples in self.speech.chunks(bytes.len() / 2) {
            for (bytes, sample) in bytes.as_chunks_mut::<2>().0.iter_mut().zip(samples) {
                *bytes = sample.to_le_bytes();
            }
            writer.write_all(&bytes[..samples.len() * 2]).await?;
        }

        writer.write_all(self.tail.as_bytes()).await
    }
}
