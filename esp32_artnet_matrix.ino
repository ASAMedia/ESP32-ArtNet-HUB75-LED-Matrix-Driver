//----------------------------------------
//         Library Includes
//----------------------------------------
#include <Arduino.h>
#include <WiFi.h>
#include <ESP32-HUB75-MatrixPanel-I2S-DMA.h>
#include <AsyncUDP.h>

//----------------------------------------
//         Wi-Fi Configuration
//----------------------------------------
const char* ssid     = "AROS-IOT";                  // Your Wi-Fi SSID
const char* password = "AX3@73AROSoftCuM@";         // Your Wi-Fi password
const char* hostname = "esp32-LED-Matrix-Controller";// Optional mDNS/hostname

//----------------------------------------
//         Matrix Panel Pins
//----------------------------------------
#define R1_PIN   19
#define G1_PIN   13
#define B1_PIN   18
#define R2_PIN    5
#define G2_PIN   12
#define B2_PIN   17

#define A_PIN    16
#define B_PIN    14
#define C_PIN     4
#define D_PIN    27
#define E_PIN    -1   // Not used on 1/16-scan panels

#define LAT_PIN  26
#define OE_PIN   15
#define CLK_PIN   2

//----------------------------------------
//         Panel Dimensions
//----------------------------------------
#define PANEL_RES_X  64   // Width of one module
#define PANEL_RES_Y  32   // Height of one module
#define PANEL_CHAIN   1   // Number of modules daisy-chained

//----------------------------------------
//         Globals & Framebuffer
//----------------------------------------
static uint16_t frame[2048] __attribute__((aligned(32)));
// Lookup tables to speed up RGB→565 conversion
static uint16_t R_LUT[256], G_LUT[256], B_LUT[256];

AsyncUDP udp;                          // UDP listener for Art-Net DMX
MatrixPanel_I2S_DMA* dma_display = nullptr; // Pointer to our panel driver

//----------------------------------------
//         Wi-Fi Event Handlers
//----------------------------------------
void WiFiStationConnected(WiFiEvent_t, WiFiEventInfo_t) {
  Serial.println("Connected to AP successfully!");
}

void WiFiGotIP(WiFiEvent_t, WiFiEventInfo_t) {
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void WiFiStationDisconnected(WiFiEvent_t, WiFiEventInfo_t info) {
  Serial.println("Disconnected from WiFi");
  Serial.print("Reason: ");
  Serial.println(info.wifi_sta_disconnected.reason);
  Serial.println("Reconnecting...");
  WiFi.begin(ssid, password);
}

//----------------------------------------
//         Color Lookup Initialization
//----------------------------------------
// Precompute bit-masked & shifted values for each 8-bit channel
// so we can do three table lookups instead of shifts/masks in ISR.
void IRAM_ATTR initColorLUTs() {
  for (int i = 0; i < 256; i++) {
    R_LUT[i] = (i & 0xF8) << 8;  // top 5 bits → bits 11–15
    G_LUT[i] = (i & 0xFC) << 3;  // top 6 bits → bits 5–10
    B_LUT[i] =  i >> 3;          // top 5 bits → bits 0–4
  }
}

//----------------------------------------
//         DMX Frame Handler (ISR-safe)
//----------------------------------------
// Called whenever a full Art-Net DMX packet arrives.
// Packs 3×8-bit RGB → 16-bit 565 and writes into framebuffer.
void IRAM_ATTR onDmxFrame(uint16_t universe,
                          uint16_t length,
                          uint8_t  sequence,
                          uint8_t* data)
{
  // We only care about universes 5…20 → 128 pixels each
  if (universe < 5 || universe > 20) return;

  uint16_t* out = frame + (universe - 5) * 128;
  for (int i = 0; i < 128; i++, data += 3) {
    out[i] = R_LUT[data[0]] | G_LUT[data[1]] | B_LUT[data[2]];
  }
}

//----------------------------------------
//         Utility: RGB→565 Inline
//----------------------------------------
static inline uint16_t to_rgb565(uint8_t r, uint8_t g, uint8_t b) {
  return (uint16_t)(
      ((r & 0xF8) << 8) |
      ((g & 0xFC) << 3) |
       (b >> 3)
  );
}

//----------------------------------------
//         UDP Packet Callback
//----------------------------------------
IRAM_ATTR void udpRXCallBack(AsyncUDPPacket &packet) {
  auto* d   = packet.data();
  auto  len = packet.length();

  // Minimum header + length field = 18 bytes
  if (len > 18
   && memcmp(d, "Art-Net\0", 8) == 0
   && (uint16_t(d[8]) | (uint16_t(d[9]) << 8)) == 0x5000  // OpDmx
  ) {
    uint8_t seq    = d[12];
    uint8_t subUni = d[14];
    uint8_t net    = d[15];

    // Universe = (Net << 8) | SubUni
    uint16_t universe = (uint16_t(net) << 8) | subUni;
    // DMX data length is big-endian at bytes 16–17
    uint16_t dmxlen   = (uint16_t(d[16]) << 8) | d[17];

    if (len >= 18 + dmxlen) {
      onDmxFrame(universe, dmxlen, seq, d + 18);
    }
  }
}

//----------------------------------------
//         Draw Task (Core-1)
//----------------------------------------
void drawTask(void*) {
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    // Draw the full framebuffer
    dma_display->drawRGBBitmap(0, 0, frame, PANEL_RES_X, PANEL_RES_Y);
    // Maintain a steady 50 ms cadence (≈20 FPS)
    vTaskDelayUntil(&last, pdMS_TO_TICKS(50));
  }
}

//----------------------------------------
//         Arduino setup()
//----------------------------------------
void setup() {
  Serial.begin(115200);
  delay(500);

  // Initialize color lookup tables
  initColorLUTs();

  // Configure Wi-Fi
  WiFi.setHostname(hostname);
  WiFi.disconnect(true);
  delay(200);
  WiFi.onEvent(WiFiStationConnected,         ARDUINO_EVENT_WIFI_STA_CONNECTED);
  WiFi.onEvent(WiFiGotIP,                    ARDUINO_EVENT_WIFI_STA_GOT_IP);
  WiFi.onEvent(WiFiStationDisconnected,      ARDUINO_EVENT_WIFI_STA_DISCONNECTED);
  WiFi.begin(ssid, password);
  Serial.println("Waiting for Wi-Fi...");

  // Start listening for Art-Net on UDP port 6454
  if (udp.listen(6454)) {
    udp.onPacket(udpRXCallBack);
  }

  // Configure HUB75 panel
  HUB75_I2S_CFG::i2s_pins pins = {
    R1_PIN, G1_PIN, B1_PIN,
    R2_PIN, G2_PIN, B2_PIN,
    A_PIN,  B_PIN,  C_PIN, D_PIN, E_PIN,
    LAT_PIN, OE_PIN, CLK_PIN
  };
  HUB75_I2S_CFG cfg(
    PANEL_RES_X,
    PANEL_RES_Y,
    PANEL_CHAIN,
    pins
  );
  cfg.i2sspeed    = HUB75_I2S_CFG::HZ_10M;
  cfg.clkphase    = false;
  cfg.double_buff = false;          // enable double buffering

  // Create & initialize the panel driver
  dma_display = new MatrixPanel_I2S_DMA(cfg);
  dma_display->begin();
  dma_display->setBrightness8(128);

  //testimage -> Fill screen red
  dma_display->fillScreen(dma_display->color565(255, 0, 0));
  delay(1000);

  // Kick off the draw task on core 1
  xTaskCreatePinnedToCore(
    drawTask,
    "Draw",
    4096,
    nullptr,
    1,    // low priority
    nullptr,
    0     // core 0
  );
}

//----------------------------------------
//         Arduino loop()
//----------------------------------------
void loop() {
  // Nothing to do here – drawing and UDP live on separate tasks!
}
