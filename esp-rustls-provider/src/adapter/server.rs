use super::ConnectionError;

/// Wrapper for [embedded_io] to be used as a server connection
pub struct ServerConnection<'s, S>
where
    S: embedded_io::Read + embedded_io::Write,
{
    socket: S,
    conn: rustls::server::UnbufferedServerConnection,
    incoming_tls: &'s mut [u8],
    outgoing_tls: &'s mut [u8],
    incoming_used: usize,
    outgoing_used: usize,

    plaintext_in: &'s mut [u8],
    plaintext_in_used: usize,
    plaintext_out: &'s mut [u8],
    plaintext_out_used: usize,
}

impl<'s, S> ServerConnection<'s, S>
where
    S: embedded_io::Read + embedded_io::Write,
{
    pub fn new(
        config: alloc::sync::Arc<rustls::server::ServerConfig>,
        socket: S,
        incoming_tls: &'s mut [u8],
        outgoing_tls: &'s mut [u8],
        plaintext_in: &'s mut [u8],
        plaintext_out: &'s mut [u8],
    ) -> Result<Self, (S, ConnectionError<S::Error>)> {
        match rustls::server::UnbufferedServerConnection::new(config) {
            Ok(conn) => {
                let mut this = Self {
                    socket,
                    conn,
                    incoming_tls,
                    outgoing_tls,
                    incoming_used: 0,
                    outgoing_used: 0,

                    plaintext_in,
                    plaintext_in_used: 0,
                    plaintext_out,
                    plaintext_out_used: 0,
                };

                match this.work() {
                    Ok(_) => Ok(this),
                    Err(err) => Err((this.socket, err)),
                }
            }
            Err(err) => Err((socket, ConnectionError::Rustls(err))),
        }
    }

    pub fn free(self) -> S {
        self.socket
    }

    fn work(&mut self) -> Result<(), ConnectionError<S::Error>> {
        use rustls::unbuffered::{AppDataRecord, ConnectionState};

        let mut done = false;
        loop {
            if done {
                log::debug!("Done work for now");
                break;
            }

            log::debug!(
                "Incoming used {}, outgoing used {}, plaintext_in used {}, plaintext_out used {}",
                self.incoming_used,
                self.outgoing_used,
                self.plaintext_in_used,
                self.plaintext_out_used
            );

            let rustls::unbuffered::UnbufferedStatus { mut discard, state } = self
                .conn
                .process_tls_records(&mut self.incoming_tls[..self.incoming_used]);

            log::debug!("State {:?}", state);

            match state.map_err(ConnectionError::Rustls)? {
                ConnectionState::ReadTraffic(mut state) => {
                    while let Some(res) = state.next_record() {
                        let AppDataRecord {
                            discard: new_discard,
                            payload,
                        } = res.map_err(ConnectionError::Rustls)?;
                        discard += new_discard;

                        self.plaintext_in[self.plaintext_in_used..][..payload.len()]
                            .copy_from_slice(payload);
                        self.plaintext_in_used += payload.len();

                        done = true;
                    }
                }

                ConnectionState::ReadEarlyData(_state) => {
                    panic!("Unsupported early-data");
                }

                ConnectionState::EncodeTlsData(mut state) => {
                    let written = state
                        .encode(&mut self.outgoing_tls[self.outgoing_used..])
                        .map_err(ConnectionError::RustlsEncodeError)?;
                    self.outgoing_used += written;
                }

                ConnectionState::TransmitTlsData(state) => {
                    log::debug!("Send tls");
                    self.socket
                        .write_all(&self.outgoing_tls[..self.outgoing_used])?;
                    self.socket.flush()?;
                    self.outgoing_used = 0;
                    state.done();
                }

                ConnectionState::BlockedHandshake { .. } => {
                    log::debug!("Receive tls");

                    let read = self
                        .socket
                        .read(&mut self.incoming_tls[self.incoming_used..])?;
                    log::debug!("Received {read}B of data");
                    self.incoming_used += read;

                    if read == 0 {
                        return Ok(());
                    }
                }

                ConnectionState::WriteTraffic(mut may_encrypt) => {
                    if self.plaintext_out_used != 0 {
                        let written = may_encrypt
                            .encrypt(
                                &self.plaintext_out[..self.plaintext_out_used],
                                &mut self.outgoing_tls[self.outgoing_used..],
                            )
                            .expect("encrypted request does not fit in `outgoing_tls`");
                        self.outgoing_used += written;
                        self.plaintext_out_used = 0;

                        log::debug!("Send tls");
                        self.socket
                            .write_all(&self.outgoing_tls[..self.outgoing_used])?;
                        self.socket.flush()?;
                        self.outgoing_used = 0;
                    }
                    done = true;
                }

                ConnectionState::Closed => {
                    // handle this state
                }

                // other states are not expected here
                _ => unreachable!(),
            }

            if discard != 0 {
                assert!(discard <= self.incoming_used);

                self.incoming_tls
                    .copy_within(discard..self.incoming_used, 0);
                self.incoming_used -= discard;

                log::debug!("Discarded {discard}B from `incoming_tls`");
            }
        }

        Ok(())
    }
}

impl<S> embedded_io::ErrorType for ServerConnection<'_, S>
where
    S: embedded_io::Read + embedded_io::Write,
{
    type Error = ConnectionError<S::Error>;
}

impl<S> embedded_io::Read for ServerConnection<'_, S>
where
    S: embedded_io::Read + embedded_io::Write + embedded_io::ReadReady,
{
    fn read(&mut self, buf: &mut [u8]) -> Result<usize, Self::Error> {
        let tls_read_res = if let Ok(true) = self.socket.read_ready() {
            self.socket
                .read(&mut self.incoming_tls[self.incoming_used..])
        } else {
            Ok(0)
        };

        let tls_read = if let Err(err) = tls_read_res {
            if self.plaintext_in_used == 0 {
                Err(err)
            } else {
                Ok(0)
            }
        } else {
            tls_read_res
        }?;
        self.incoming_used += tls_read;

        self.work()?;

        let l = usize::min(buf.len(), self.plaintext_in_used);
        buf[0..l].copy_from_slice(&self.plaintext_in[0..l]);

        self.plaintext_in.copy_within(l..self.plaintext_in_used, 0);
        self.plaintext_in_used -= l;

        Ok(l)
    }
}

impl<S> embedded_io::Write for ServerConnection<'_, S>
where
    S: embedded_io::Read + embedded_io::Write,
{
    fn write(&mut self, buf: &[u8]) -> Result<usize, Self::Error> {
        self.plaintext_out[self.plaintext_out_used..][..buf.len()].copy_from_slice(buf);
        self.plaintext_out_used += buf.len();
        self.work()?;
        Ok(buf.len())
    }

    fn flush(&mut self) -> Result<(), Self::Error> {
        self.socket.flush()?;
        self.work()?;
        Ok(())
    }
}
