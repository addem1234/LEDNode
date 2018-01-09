// Network settings
#define NETWORK_SSID "test_network_dont_use"
#define NETWORK_PASS "test_key_dont_usee"
#define NETWORK_HOST "ArtNet-LedCube-1"
// Number of leds per strip
#define PER_STRIP  100
// Number of strips
#define NUM_STRIPS 3

// First patched universe
#define START_UNIVERSE  0
// Patched with one universe per strip, or continously
#define CONTINOUS_MODE true

#include <Arduino.h>

#if defined(ARDUINO_ARCH_ESP32)
  #pragma message "ESP32 imports"
  #include <WiFi.h>
  #include <WiFiUdp.h>
#elif defined(ARDUINO_ARCH_ESP8266)
  #pragma message "ESP8266 imports"
  #include <ESP8266WiFi.h>
  #include <WiFiUdp.h>
#elif defined(ARDUINO_AVR_MEGA2560)
  #pragma message "MEGA imports"
  #include <Ethernet.h>
  #include <EthernetUdp.h>
#endif

#include <FastLED.h>
#include <artnet_node.h>

// Internal valiables
#if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
WiFiUDP udpClient;
#else
EthernetUDP udpClient;
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
};
#endif
bool network_connected = false;

const int numLeds     = PER_STRIP * NUM_STRIPS;
const int numChannels = numLeds * 3;

CRGB leds[NUM_STRIPS][PER_STRIP];

// How many universes do we need?
const int maxUniverses = CONTINOUS_MODE ? (numChannels / 512 + ((numChannels % 512) ? 1 : 0)) : NUM_STRIPS;
// Keep track of recieved universes
bool universeReceived[maxUniverses];

// Callback for DMX data, this is what does the heavy lifting
// Takes DMX data and sets led colors accordingly.
void onDmxFrame(artnet_dmx_t *packetBuffer) {
  uint16_t universe = packetBuffer->universe;
  uint16_t length   = bytes_to_short(packetBuffer->lengthHi, packetBuffer->length);
  //uint8_t  sequence = packetBuffer->sequence;
  uint8_t* data     = packetBuffer->data;

  //Serial.printf("Got DMX data. Universe: %d, length: %d, sequence: %d\n", universe, length, sequence);

  int universeIndex = universe - START_UNIVERSE;

  if (universeIndex < 0 || universeIndex > maxUniverses) return;


  unsigned int byteOffset = 0;
  if (CONTINOUS_MODE) {
    byteOffset = universeIndex * numChannels;
    if (byteOffset + length > numChannels) {
      length = numChannels - byteOffset;
    }
  } else {
    byteOffset = universeIndex * PER_STRIP * 3;
    if(PER_STRIP * 3 < length) {
      length = PER_STRIP * 3;
    }
  }

  if (byteOffset + length > numChannels) {
    Serial.printf("Out or range when byteOffset: %d, length: %d\n", byteOffset, length);
    // This is bad! Lets not try to memcpy this...
    // Should check that logic...
    return;
  }

  // Finally, its time flip some bytes!
  memcpy(((uint8_t*) leds) + byteOffset, data, length * sizeof(uint8_t));

  universeReceived[universeIndex] = true;

  // Have we gotten data for the other universes?
  for (int i = 0 ; i < maxUniverses ; i++) {
    if(!universeReceived[i]) return; // No? Then lets not show the change.
  }

  // Seems we have all the universes, lets show the changes.
  FastLED.show();
  // Reset this to false for every universe
  memset(universeReceived, false, maxUniverses);
}

int counter = 0;
// A not so fancy animation to show that we have started. Nice for diagnostic purposes
void simplerainbow() {
  for(int strip = 0; strip < NUM_STRIPS; strip++) {
    for(int i = 0; i < PER_STRIP; i++) {
      hsv2rgb_rainbow(CHSV((counter + i) % 255, 255, 127), leds[strip][i]);
    }
  }
  FastLED.show();
  counter++;
  delay(100);
}


// Callback for poll request
void onPollRequest(artnet_poll_t *packetBuffer) {
  // create a reply
  artnet_reply_t poll_reply;
  fill_art_poll_reply(&poll_reply);

  // local ip, but ending in 255, so we broadcast to the network
  #if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
  IPAddress ip = WiFi.localIP();
  #elif defined(ARDUINO_AVR_MEGA2560)
  IPAddress ip = Ethernet.localIP();
  #endif
  ip[3] = 255;

  udpClient.beginPacket(ip, ARTNET_PORT);
  udpClient.write((uint8_t*) &poll_reply, sizeof(artnet_reply_t));
  udpClient.endPacket();
}

void setupArtNet() {
  // Setup callbacks
  set_dmx_callback(onDmxFrame);
  set_poll_callback(onPollRequest);

  // Listen on ARTNET_PORT (6454)
  udpClient.begin(ARTNET_PORT);

  // Get network info
  #if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
  uint8_t mac[6];
  WiFi.macAddress(mac);
  uint32_t ip = WiFi.localIP();
  uint32_t gw = WiFi.gatewayIP();
  uint32_t subnet = WiFi.subnetMask();
  #else
  uint32_t ip = Ethernet.localIP();
  uint32_t gw = Ethernet.gatewayIP();
  uint32_t subnet = Ethernet.subnetMask();
  #endif

  // Add network info to internal struct
  fill_art_node(mac, &ip, &gw, &subnet);

  // Broadcast our existance
  artnet_poll_t* null;
  onPollRequest(null);
}

#if defined(ARDUINO_ARCH_ESP32)
void onWiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case SYSTEM_EVENT_STA_START:
      Serial.printf("Setting host to %s\n", NETWORK_HOST);
      WiFi.setHostname(NETWORK_HOST);
      break;
    case SYSTEM_EVENT_STA_GOT_IP:
      setupArtNet();
      Serial.printf("WiFi connected, ip[3]: %d\n", WiFi.localIP()[3]);
      network_connected = true;
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi disconnected");
      network_connected = false;
      break;
    default:
      break;
  }
}
#elif defined(ARDUINO_ARCH_ESP8266)
void onWiFiEvent(WiFiEvent_t event) {
  switch(event) {
    case WIFI_EVENT_STAMODE_CONNECTED:
      Serial.printf("Setting host to %s\n", NETWORK_HOST);
      WiFi.hostname(NETWORK_HOST);
      break;
    case WIFI_EVENT_STAMODE_GOT_IP:
      setupArtNet();
      Serial.printf("WiFi connected, ip[3]: %d\n", WiFi.localIP()[3]);
      network_connected = true;
      break;
    case WIFI_EVENT_STAMODE_DISCONNECTED:
      Serial.println("WiFi disconnected");
      network_connected = false;
      break;
    default:
      break;
  }
}
#endif

void setupNetwork() {
  Serial.printf("Connecting to %s\n", NETWORK_SSID);
  #if defined(ARDUINO_ARCH_ESP32) || defined(ARDUINO_ARCH_ESP8266)
  //WiFi.mode(WIFI_MODE_STA);
  WiFi.onEvent(onWiFiEvent);
  wl_status_t status = WiFi.begin(NETWORK_SSID, NETWORK_PASS);
  switch (status) {
    case WL_NO_SHIELD: Serial.println("WL_NO_SHIELD"); break;
    case WL_IDLE_STATUS: Serial.println("WL_IDLE_STATUS"); break;
    case WL_NO_SSID_AVAIL: Serial.println("WL_NO_SSID_AVAIL"); break;
    case WL_SCAN_COMPLETED: Serial.println("WL_SCAN_COMPLETED"); break;
    case WL_CONNECTED: Serial.println("WL_CONNECTED"); break;
    case WL_CONNECT_FAILED: Serial.println("WL_CONNECT_FAILED"); break;
    case WL_CONNECTION_LOST: Serial.println("WL_CONNECTION_LOST"); break;
    case WL_DISCONNECTED: Serial.println("WL_DISCONNECTED"); break;
    default: Serial.printf("Connection status %d\n", status);
  }
  #else
  if(Ethernet.begin(mac)) {
    network_connected = true;
    Serial.printf("Ethernet connected, ip[3]: %d\n", Ethernet.localIP()[3]);
  }
  #endif
}

void setup() {
  Serial.begin(115200);

#if defined(ARDUINO_ARCH_ESP32)
  // templates make this ugly?
  // not sure if it can be better
  FastLED.addLeds<NEOPIXEL, 12>(leds[0], PER_STRIP);
  #if NUM_STRIPS > 1
  FastLED.addLeds<NEOPIXEL, 13>(leds[1], PER_STRIP);
  #endif
  #if NUM_STRIPS > 2
  FastLED.addLeds<NEOPIXEL, 14>(leds[2], PER_STRIP);
  #endif
  #if NUM_STRIPS > 3
  FastLED.addLeds<NEOPIXEL, 15>(leds[3], PER_STRIP);
  #endif
  #if NUM_STRIPS > 4
  FastLED.addLeds<NEOPIXEL, 16>(leds[4], PER_STRIP);
  #endif
  #if NUM_STRIPS > 5
  FastLED.addLeds<NEOPIXEL, 17>(leds[5], PER_STRIP);
  #endif
#elif defined(ARDUINO_ARCH_ESP8266)
  // pins 12-15, 4-5 on esp8266
  FastLED.addLeds<WS2811_PORTA, NUM_STRIPS>((CRGB*) leds, numLeds);
#else
  FastLED.addLeds<NEOPIXEL, 7>(leds[0], numLeds);
#endif

  setupNetwork();
  simplerainbow();
}

void loop() {
  if (network_connected) {
    // Read artnet data
    uint8_t* buffer = get_packet_buffer();
    int buffer_size = get_max_buffer_size();
    int packet_size = udpClient.parsePacket();
    // Only if the packet seems sane
    if (packet_size && packet_size <= buffer_size) {
      udpClient.read(buffer, buffer_size);
      //let artnet_node.c handle the art-net specifics
      handle_packet();
    }

    udpClient.flush();
  } else if (!network_connected) {
    //Serial.println("network_connected = false; showing simplerainbow.");
    simplerainbow();
  }
}

