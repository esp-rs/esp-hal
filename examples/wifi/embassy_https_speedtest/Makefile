SERVER_IP ?= $(shell (hostname -I 2>/dev/null || ip -4 -o addr show scope global 2>/dev/null) | awk '{print $$1}' | cut -d/ -f1 | head -1)
TLS_PORT ?= 8443
HTTP_PORT ?= 8080
SSID ?= SSID
PASSWORD ?= PASSWORD

GO_HTTPBIN_VERSION ?= v2.25.0
GO_HTTPBIN_URL ?= https://github.com/mccutchen/go-httpbin/releases/download/$(GO_HTTPBIN_VERSION)/go-httpbin-linux-amd64.tar.gz
GO_HTTPBIN_DIR ?= .tools
GO_HTTPBIN ?= $(GO_HTTPBIN_DIR)/go-httpbin
# 0 disables go-httpbin's request read timeouts
GO_HTTPBIN_ARGS ?= -max-body-size 10000000 -max-duration 60s -srv-read-timeout 0 -srv-read-header-timeout 0

export RUSTUP_TOOLCHAIN ?= esp

CERT_FILES = server-cert.pem server-key.pem server-cert.der

.PHONY: help cert serve serve-http run run-http

help:
	@echo "Targets: cert, serve, serve-http, run, run-http"
	@echo "Detected SERVER_IP: $(SERVER_IP)"

server-key.pem server-cert.pem server-cert.der:
	openssl req -x509 -newkey ec -pkeyopt ec_paramgen_curve:P-256 \
		-keyout server-key.pem -out server-cert.pem -days 3650 -nodes \
		-subj "/CN=$(SERVER_IP)" -addext "subjectAltName=DNS:$(SERVER_IP)"
	openssl x509 -in server-cert.pem -outform DER -out server-cert.der
	@echo "Certificate generated for $(SERVER_IP)"

cert: server-key.pem server-cert.pem server-cert.der

$(GO_HTTPBIN):
	mkdir -p $(GO_HTTPBIN_DIR)
	wget -q -O $(GO_HTTPBIN_DIR)/go-httpbin.tar.gz $(GO_HTTPBIN_URL)
	tar -xzf $(GO_HTTPBIN_DIR)/go-httpbin.tar.gz -C $(GO_HTTPBIN_DIR)

serve: cert $(GO_HTTPBIN)
	$(GO_HTTPBIN) $(GO_HTTPBIN_ARGS) -host 0.0.0.0 -port $(TLS_PORT) \
		-https-cert-file server-cert.pem -https-key-file server-key.pem

serve-http: $(GO_HTTPBIN)
	$(GO_HTTPBIN) $(GO_HTTPBIN_ARGS) -host 0.0.0.0 -port $(HTTP_PORT)

run: cert
	SERVER_IP=$(SERVER_IP) SSID=$(SSID) PASSWORD=$(PASSWORD) \
		cargo build --release --features esp32s3
	espflash flash --monitor target/xtensa-esp32s3-none-elf/release/embassy-https-speedtest

run-http: cert
	SERVER_IP=$(SERVER_IP) SSID=$(SSID) PASSWORD=$(PASSWORD) \
		cargo build --release --features esp32s3,plain-http
	espflash flash --monitor target/xtensa-esp32s3-none-elf/release/embassy-https-speedtest
