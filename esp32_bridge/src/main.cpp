#include <Arduino.h>
#include <WiFi.h>

namespace {

constexpr const char *kApSsid = "FC_TELEM";
constexpr uint16_t kTcpPort = 5761;

#ifndef BRIDGE_WIFI_STA_SSID
#define BRIDGE_WIFI_STA_SSID ""
#endif

#ifndef BRIDGE_WIFI_STA_PASS
#define BRIDGE_WIFI_STA_PASS ""
#endif

#ifndef BRIDGE_WIFI_STA_TIMEOUT_MS
#define BRIDGE_WIFI_STA_TIMEOUT_MS 15000
#endif

constexpr int kUartRxPin = 16;
constexpr int kUartTxPin = 17;
constexpr uint32_t kUartBaud = 921600;

constexpr size_t kFcToTcpRingSize = 8192;
constexpr size_t kTcpToFcRingSize = 8192;
constexpr size_t kIoChunk = 256;

class ByteRing {
 public:
  ByteRing(uint8_t *storage, size_t capacity)
      : data_(storage), capacity_(capacity), head_(0), tail_(0), count_(0) {}

  size_t push(const uint8_t *src, size_t len) {
    size_t pushed = 0;
    while (pushed < len && count_ < capacity_) {
      data_[head_] = src[pushed++];
      head_ = (head_ + 1) % capacity_;
      count_++;
    }
    return pushed;
  }

  size_t pop(uint8_t *dst, size_t len) {
    size_t popped = 0;
    while (popped < len && count_ > 0) {
      dst[popped++] = data_[tail_];
      tail_ = (tail_ + 1) % capacity_;
      count_--;
    }
    return popped;
  }

  size_t size() const { return count_; }

 private:
  uint8_t *data_;
  size_t capacity_;
  size_t head_;
  size_t tail_;
  size_t count_;
};

struct BridgeCounters {
  uint64_t bytesFcToLaptop = 0;
  uint64_t bytesLaptopToFc = 0;
  uint32_t uartOverflowCount = 0;
  uint32_t tcpWriteFailureCount = 0;
};

HardwareSerial fcUart(1);
WiFiServer tcpServer(kTcpPort);
WiFiClient tcpClient;

uint8_t fcToTcpStorage[kFcToTcpRingSize];
uint8_t tcpToFcStorage[kTcpToFcRingSize];
ByteRing fcToTcp(fcToTcpStorage, kFcToTcpRingSize);
ByteRing tcpToFc(tcpToFcStorage, kTcpToFcRingSize);

BridgeCounters counters;
unsigned long lastStatusMs = 0;
bool hadClient = false;

bool startWifi() {
  const char *staSsid = BRIDGE_WIFI_STA_SSID;
  const char *staPass = BRIDGE_WIFI_STA_PASS;

  if (staSsid != nullptr && staSsid[0] != '\0') {
    WiFi.mode(WIFI_STA);
    WiFi.begin(staSsid, staPass);
    Serial.printf("[bridge] Connecting STA: SSID=%s\n", staSsid);

    const unsigned long startMs = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - startMs) < BRIDGE_WIFI_STA_TIMEOUT_MS) {
      delay(200);
      Serial.print('.');
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
      Serial.printf("[bridge] STA connected: IP=%s RSSI=%d dBm\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());
      return true;
    }

    Serial.println("[bridge] STA connect failed, falling back to AP mode");
  }

  WiFi.mode(WIFI_AP);
  bool apOk = WiFi.softAP(kApSsid);
  if (!apOk) {
    Serial.println("[bridge] Failed to start AP");
    return false;
  }

  Serial.printf("[bridge] AP up: SSID=%s IP=%s\n", kApSsid, WiFi.softAPIP().toString().c_str());
  return true;
}

void acceptClientIfNeeded() {
  if (tcpClient && tcpClient.connected()) {
    return;
  }

  if (tcpClient) {
    tcpClient.stop();
  }

  WiFiClient incoming = tcpServer.available();
  if (incoming) {
    tcpClient = incoming;
    Serial.printf("[bridge] TCP client connected: %s\n", tcpClient.remoteIP().toString().c_str());
  }
}

void readFromFcUart() {
  uint8_t buf[kIoChunk];
  while (fcUart.available() > 0) {
    size_t toRead = static_cast<size_t>(fcUart.available());
    if (toRead > sizeof(buf)) {
      toRead = sizeof(buf);
    }

    size_t got = fcUart.readBytes(buf, toRead);
    if (got == 0) {
      break;
    }

    size_t pushed = fcToTcp.push(buf, got);
    if (pushed < got) {
      counters.uartOverflowCount++;
    }
  }
}

void readFromTcpClient() {
  if (!(tcpClient && tcpClient.connected())) {
    return;
  }

  uint8_t buf[kIoChunk];
  while (tcpClient.available() > 0) {
    size_t toRead = static_cast<size_t>(tcpClient.available());
    if (toRead > sizeof(buf)) {
      toRead = sizeof(buf);
    }

    int got = tcpClient.read(buf, toRead);
    if (got <= 0) {
      break;
    }

    size_t pushed = tcpToFc.push(buf, static_cast<size_t>(got));
    if (pushed < static_cast<size_t>(got)) {
      counters.uartOverflowCount++;
    }
  }
}

void writeFcToTcp() {
  if (!(tcpClient && tcpClient.connected())) {
    return;
  }

  uint8_t buf[kIoChunk];
  while (fcToTcp.size() > 0) {
    size_t popped = fcToTcp.pop(buf, sizeof(buf));
    if (popped == 0) {
      break;
    }

    size_t written = tcpClient.write(buf, popped);
    if (written != popped) {
      counters.tcpWriteFailureCount++;
      if (written > 0 && written < popped) {
        size_t restore = popped - written;
        (void)fcToTcp.push(&buf[written], restore);
      }
      break;
    }

    counters.bytesFcToLaptop += written;
  }
}

void writeTcpToFc() {
  uint8_t buf[kIoChunk];
  while (tcpToFc.size() > 0) {
    size_t popped = tcpToFc.pop(buf, sizeof(buf));
    if (popped == 0) {
      break;
    }

    size_t written = fcUart.write(buf, popped);
    counters.bytesLaptopToFc += written;

    if (written < popped) {
      size_t restore = popped - written;
      (void)tcpToFc.push(&buf[written], restore);
      break;
    }
  }
}

void printPeriodicStatus() {
  unsigned long now = millis();
  if ((now - lastStatusMs) < 1000UL) {
    return;
  }
  lastStatusMs = now;

  bool connected = (tcpClient && tcpClient.connected());
  if (connected != hadClient) {
    hadClient = connected;
    Serial.printf("[bridge] TCP client %s\n", connected ? "connected" : "disconnected");
  }

  Serial.printf(
      "[bridge] FC->Laptop=%llu bytes, Laptop->FC=%llu bytes, UART_overflow=%lu, TCP_write_fail=%lu, q_fc2tcp=%u, q_tcp2fc=%u\n",
      static_cast<unsigned long long>(counters.bytesFcToLaptop),
      static_cast<unsigned long long>(counters.bytesLaptopToFc),
      static_cast<unsigned long>(counters.uartOverflowCount),
      static_cast<unsigned long>(counters.tcpWriteFailureCount),
      static_cast<unsigned>(fcToTcp.size()),
      static_cast<unsigned>(tcpToFc.size()));
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(200);

  Serial.println("[bridge] Starting ESP32-S3 FC telemetry bridge");

  fcUart.begin(kUartBaud, SERIAL_8N1, kUartRxPin, kUartTxPin);
  Serial.printf("[bridge] UART started @ %lu on RX=%d TX=%d\n", static_cast<unsigned long>(kUartBaud), kUartRxPin, kUartTxPin);

  (void)startWifi();

  tcpServer.begin();
  tcpServer.setNoDelay(true);
  Serial.printf("[bridge] TCP server listening on port %u\n", static_cast<unsigned>(kTcpPort));
}

void loop() {
  acceptClientIfNeeded();
  readFromFcUart();
  readFromTcpClient();
  writeFcToTcp();
  writeTcpToFc();
  printPeriodicStatus();
  delay(1);
}
