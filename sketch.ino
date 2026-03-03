// ==========================================
//  REAL-TIME CO2 MONITORING SYSTEM — ESP32
//  MQ-2 Gas Sensor + WS2812 LED Strip
// ==========================================

#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <DHT.h>
#include <Adafruit_NeoPixel.h>

// ====== PIN DEFINITIONS ======
#define MQ2_PIN        34
#define DHT_PIN        4
#define US1_TRIG       5
#define US1_ECHO       18
#define US2_TRIG       17
#define US2_ECHO       16
#define BUZZER_PIN     25
#define LED_STRIP_PIN  26

// ====== LED STRIP CONFIGURATION ======
#define NUM_LEDS       8
Adafruit_NeoPixel strip(NUM_LEDS, LED_STRIP_PIN, NEO_GRB + NEO_KHZ800);

// ====== OLED CONFIGURATION ======
#define SCREEN_W 128
#define SCREEN_H 64
Adafruit_SSD1306 display(SCREEN_W, SCREEN_H, &Wire, -1);

// ====== DHT22 CONFIGURATION ======
DHT dht(DHT_PIN, DHT22);

// ====== WIFI CONFIGURATION ======
const char* ssid     = "Wokwi-GUEST";
const char* password = "";
const char* apiURL   = "https://api.example.com/data";

// ====== GLOBAL VARIABLES ======
int   co2_ppm      = 400;
int   targetPPM    = 400;           // <<< NEW: raw potentiometer target
const int PPM_STEP = 5;             // <<< NEW: ppm change per loop (~50 ppm/sec)
const int PPM_MAX  = 2000;          // <<< NEW: max range changed from 5000

float temperature   = 0.0;
float humidity      = 0.0;
int   peopleCount   = 0;
String alertStatus  = "good";

// People counting state
bool  s1Triggered   = false;
bool  s2Triggered   = false;
unsigned long s1Time = 0;
unsigned long s2Time = 0;
const unsigned long SEQ_TIMEOUT = 2000;

// Timing
unsigned long lastDHT   = 0;
unsigned long lastHTTP  = 0;

// Detection threshold in cm
const float DETECT_CM = 50.0;

// ==========================================
//                  SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  Serial.println("=== CO2 Monitor Starting ===");

  pinMode(MQ2_PIN, INPUT);
  pinMode(US1_TRIG, OUTPUT);
  pinMode(US1_ECHO, INPUT);
  pinMode(US2_TRIG, OUTPUT);
  pinMode(US2_ECHO, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  strip.begin();
  strip.setBrightness(150);
  strip.show();

  dht.begin();

  Wire.begin(21, 22);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED init FAILED");
    while (true);
  }
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("CO2 Monitor v2.0");
  display.println("Booting...");
  display.display();
  delay(1000);

  WiFi.begin(ssid, password);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Connecting WiFi...");
  display.display();

  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 20) {
    delay(500);
    Serial.print(".");
    tries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi OK: " + WiFi.localIP().toString());
    display.println("WiFi Connected!");
  } else {
    Serial.println("\nWiFi FAILED");
    display.println("WiFi Failed");
  }
  display.display();
  delay(1000);
}

// ==========================================
//                MAIN LOOP
// ==========================================
void loop() {
  unsigned long now = millis();

  // --- READ MQ-2 (potentiometer) ---
  int raw = analogRead(MQ2_PIN);
  targetPPM = map(raw, 0, 4095, 400, PPM_MAX);  // <<< CHANGED: max is now 2000

  // --- SMOOTH PPM TRANSITION ---                // <<< NEW BLOCK
  if (co2_ppm < targetPPM) {
    co2_ppm = min(co2_ppm + PPM_STEP, targetPPM);
  } else if (co2_ppm > targetPPM) {
    co2_ppm = max(co2_ppm - PPM_STEP, targetPPM);
  }

  // --- READ DHT22 every 2 seconds ---
  if (now - lastDHT >= 2000) {
    lastDHT = now;
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    if (!isnan(t)) temperature = t;
    if (!isnan(h)) humidity = h;
  }

  // --- ULTRASONIC PEOPLE COUNTING ---
  float d1 = measureDistance(US1_TRIG, US1_ECHO);
  float d2 = measureDistance(US2_TRIG, US2_ECHO);

  if (d1 < DETECT_CM && !s1Triggered) {
    s1Triggered = true;
    s1Time = now;
  }
  if (d2 < DETECT_CM && !s2Triggered) {
    s2Triggered = true;
    s2Time = now;
  }

  if (s1Triggered && s2Triggered) {
    if (s1Time < s2Time && (s2Time - s1Time) <= SEQ_TIMEOUT) {
      peopleCount++;
      Serial.println(">> ENTRY | Count: " + String(peopleCount));
    }
    else if (s2Time < s1Time && (s1Time - s2Time) <= SEQ_TIMEOUT) {
      if (peopleCount > 0) peopleCount--;
      Serial.println("<< EXIT  | Count: " + String(peopleCount));
    }
    s1Triggered = false;
    s2Triggered = false;
  }

  if (s1Triggered && (now - s1Time > SEQ_TIMEOUT)) s1Triggered = false;
  if (s2Triggered && (now - s2Time > SEQ_TIMEOUT)) s2Triggered = false;

  // --- ALERT HANDLING + LED STRIP ---
  handleAlerts();

  // --- UPDATE OLED ---
  updateOLED();

  // --- HTTP POST every 10 seconds ---
  if (now - lastHTTP >= 10000) {
    lastHTTP = now;
    postData();
  }

  delay(100);
}

// ==========================================
//     MEASURE DISTANCE (HC-SR04)
// ==========================================
float measureDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long dur = pulseIn(echoPin, HIGH, 30000);
  float cm = dur * 0.034 / 2.0;
  return (cm == 0) ? 999.0 : cm;
}

// ==========================================
//  ALERT LOGIC (LED STRIP + BUZZER)
// ==========================================
void handleAlerts() {                           // <<< CHANGED ENTIRELY

  // Determine alert status (thresholds adjusted for 400-2000 range)
  if (co2_ppm < 600) {
    alertStatus = "good";
  }
  else if (co2_ppm < 1000) {
    alertStatus = "moderate";
  }
  else if (co2_ppm < 1500) {
    alertStatus = "high";
  }
  else {
    alertStatus = "critical";
  }

  // Buzzer: starts at 1200 ppm, frequency rises with PPM
  if (co2_ppm >= 1000) {
    int freq = map(co2_ppm, 1100, PPM_MAX, 1000, 4000);
    freq = constrain(freq, 1000, 4000);
    tone(BUZZER_PIN, freq);
  } else {
    noTone(BUZZER_PIN);
  }

  updateLEDStrip();
}

// ==========================================
//     UPDATE LED STRIP WITH GRADIENT
// ==========================================
void updateLEDStrip() {
  int numLit = map(co2_ppm, 400, PPM_MAX, 1, NUM_LEDS);  // <<< CHANGED: uses PPM_MAX
  numLit = constrain(numLit, 1, NUM_LEDS);

  strip.clear();

  for (int i = 0; i < numLit; i++) {
    uint32_t color = getGradientColor(i, NUM_LEDS);
    strip.setPixelColor(i, color);
  }

  strip.show();
}

// ==========================================
//   GET GRADIENT COLOR (GREEN → YELLOW → ORANGE → RED)
// ==========================================
uint32_t getGradientColor(int ledIndex, int totalLeds) {
  if (totalLeds <= 1) totalLeds = 2;

  float position = (float)ledIndex / (float)(totalLeds - 1);

  uint8_t r, g, b;

  if (position < 0.33) {
    float t = position / 0.33;
    r = (uint8_t)(t * 255);
    g = 255;
    b = 0;
  }
  else if (position < 0.66) {
    float t = (position - 0.33) / 0.33;
    r = 255;
    g = (uint8_t)(255 - (t * 100));
    b = 0;
  }
  else {
    float t = (position - 0.66) / 0.34;
    r = 255;
    g = (uint8_t)(155 - (t * 155));
    b = 0;
  }

  return strip.Color(r, g, b);
}

// ==========================================
//          UPDATE OLED DISPLAY
// ==========================================
void updateOLED() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);

  display.print("CO2:    ");
  display.print(co2_ppm);
  display.println(" ppm");

  display.print("Temp:   ");
  display.print(temperature, 1);
  display.println(" C");

  display.print("Humid:  ");
  display.print(humidity, 1);
  display.println(" %");

  display.print("People: ");
  display.println(peopleCount);

  display.println();
  display.print("Status: ");
  display.println(alertStatus);

  display.drawLine(0, 53, 127, 53, SSD1306_WHITE);
  display.setCursor(0, 56);
  display.print("WiFi:");
  display.println(WiFi.status() == WL_CONNECTED ? "OK" : "NO");

  display.display();
}

// ==========================================
//        HTTP POST SENSOR DATA
// ==========================================
void postData() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("No WiFi — skip POST");
    return;
  }

  HTTPClient http;
  http.begin(apiURL);
  http.addHeader("Content-Type", "application/json");

  String json = "{";
  json += "\"co2\":" + String(co2_ppm) + ",";
  json += "\"temperature\":" + String(temperature, 1) + ",";
  json += "\"humidity\":" + String(humidity, 1) + ",";
  json += "\"people_count\":" + String(peopleCount) + ",";
  json += "\"alert\":\"" + alertStatus + "\"";
  json += "}";

  Serial.println("POST: " + json);
  int code = http.POST(json);

  if (code > 0) {
    Serial.println("Response: " + String(code));
  } else {
    Serial.println("POST failed: " + http.errorToString(code));
  }
  http.end();
}