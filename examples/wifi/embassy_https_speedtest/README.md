# Embassy HTTPS speed test

An  wifi speed test that measures download and upload throughput against a
local test server over both HTTPS and plain HTTP. The server is a local
[go-httpbin](https://github.com/mccutchen/go-httpbin) instance, using
self-signed certs that the the firmware verifies.

## Running over HTTPS

```sh
make install-server
make server 
# in a new tmux pane/terminal
make flash SSID=mywifi PASSWORD=secret
```

## Running over plain HTTP

```sh
make server-http
# in a new tmux pane/terminal
make flash-plain SSID=mywifi PASSWORD=secret
```

## Troubleshooting

- Only tested on Linux.
- Auto-guesses the correct LAN IP of the server/host, set with SERVER_IP if the
  guess is wrong
- Make sure the ESP32 is on the same network as the host and that the host's
  firewall allows connections to ports 8443/8080.
