#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <ESPSupabase.h>
#include <WiFiClientSecure.h>
#include <Adafruit_MQTT.h>
#include <Adafruit_MQTT_Client.h>
#include <ZMPT101B.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH1106.h>
#include <time.h>

// OLED Display
#define OLED_RESET -1
Adafruit_SH1106 display(OLED_RESET);

#define TRIGGER_PIN 0
bool shouldStartAP = false;

// Device Configuration
#define DEVICE_ID         "SmartSwitch-001"
#define TRIGGER_PIN       0
#define OUTPUT_PIN        2
#define VOLTAGE_PIN       36
#define CURRENT_PIN       39
#define TOGGLE_PIN       19

// Sensor Configuration
#define VOLTAGE_SENSITIVITY 500.0f
#define ACS_SENSITIVITY     0.066f
#define THRESHOLD_VOLTAGE   10.0
#define THRESHOLD_CURRENT   0.2

// Time Configuration
#define NTP_OFFSET       19800  // IST offset (5h 30m)
#define NTP_INTERVAL     60000
#define NTP_ADDRESS      "pool.ntp.org"



// Configuration Strings
const String supabase_url = "<SUPABASE_URL_link>";
const String supabase_key = "<SUPABASE_SECRETE_KEY>";
const String AIO_SERVER = "io.adafruit.com";
const String AIO_USER = "Omkar1705";
const String AIO_KEY = "<adafruit_API_KEY>";
const String MQTT_FEED = AIO_USER + "/feeds/led";



// MQTT Root CA Certificate
const char* adafruitio_root_ca = \
      "-----BEGIN CERTIFICATE-----\n"
      "MIIEjTCCA3WgAwIBAgIQDQd4KhM/xvmlcpbhMf/ReTANBgkqhkiG9w0BAQsFADBh\n"
      "MQswCQYDVQQGEwJVUzEVMBMGA1UEChMMRGlnaUNlcnQgSW5jMRkwFwYDVQQLExB3\n"
      "d3cuZGlnaWNlcnQuY29tMSAwHgYDVQQDExdEaWdpQ2VydCBHbG9iYWwgUm9vdCBH\n"
      "MjAeFw0xNzExMDIxMjIzMzdaFw0yNzExMDIxMjIzMzdaMGAxCzAJBgNVBAYTAlVT\n"
      "MRUwEwYDVQQKEwxEaWdpQ2VydCBJbmMxGTAXBgNVBAsTEHd3dy5kaWdpY2VydC5j\n"
      "b20xHzAdBgNVBAMTFkdlb1RydXN0IFRMUyBSU0EgQ0EgRzEwggEiMA0GCSqGSIb3\n"
      "DQEBAQUAA4IBDwAwggEKAoIBAQC+F+jsvikKy/65LWEx/TMkCDIuWegh1Ngwvm4Q\n"
      "yISgP7oU5d79eoySG3vOhC3w/3jEMuipoH1fBtp7m0tTpsYbAhch4XA7rfuD6whU\n"
      "gajeErLVxoiWMPkC/DnUvbgi74BJmdBiuGHQSd7LwsuXpTEGG9fYXcbTVN5SATYq\n"
      "DfbexbYxTMwVJWoVb6lrBEgM3gBBqiiAiy800xu1Nq07JdCIQkBsNpFtZbIZhsDS\n"
      "fzlGWP4wEmBQ3O67c+ZXkFr2DcrXBEtHam80Gp2SNhou2U5U7UesDL/xgLK6/0d7\n"
      "6TnEVMSUVJkZ8VeZr+IUIlvoLrtjLbqugb0T3OYXW+CQU0kBAgMBAAGjggFAMIIB\n"
      "PDAdBgNVHQ4EFgQUlE/UXYvkpOKmgP792PkA76O+AlcwHwYDVR0jBBgwFoAUTiJU\n"
      "IBiV5uNu5g/6+rkS7QYXjzkwDgYDVR0PAQH/BAQDAgGGMB0GA1UdJQQWMBQGCCsG\n"
      "AQUFBwMBBggrBgEFBQcDAjASBgNVHRMBAf8ECDAGAQH/AgEAMDQGCCsGAQUFBwEB\n"
      "BCgwJjAkBggrBgEFBQcwAYYYaHR0cDovL29jc3AuZGlnaWNlcnQuY29tMEIGA1Ud\n"
      "HwQ7MDkwN6A1oDOGMWh0dHA6Ly9jcmwzLmRpZ2ljZXJ0LmNvbS9EaWdpQ2VydEds\n"
      "b2JhbFJvb3RHMi5jcmwwPQYDVR0gBDYwNDAyBgRVHSAAMCowKAYIKwYBBQUHAgEW\n"
      "HGh0dHBzOi8vd3d3LmRpZ2ljZXJ0LmNvbS9DUFMwDQYJKoZIhvcNAQELBQADggEB\n"
      "AIIcBDqC6cWpyGUSXAjjAcYwsK4iiGF7KweG97i1RJz1kwZhRoo6orU1JtBYnjzB\n"
      "c4+/sXmnHJk3mlPyL1xuIAt9sMeC7+vreRIF5wFBC0MCN5sbHwhNN1JzKbifNeP5\n"
      "ozpZdQFmkCo+neBiKR6HqIA+LMTMCMMuv2khGGuPHmtDze4GmEGZtYLyF8EQpa5Y\n"
      "jPuV6k2Cr/N3XxFpT3hRpt/3usU/Zb9wfKPtWpoznZ4/44c1p9rzFcZYrWkj3A+7\n"
      "TNBJE0GmP2fhXhP1D/XVfIW/h0yCJGEiV9Glm/uGOa3DXHlmbAcxSyCRraG+ZBkA\n"
      "7h4SeM6Y8l/7MBRpPCz6l8Y=\n"
      "-----END CERTIFICATE-----\n";

// Global Objects
Preferences preferences;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, NTP_ADDRESS, NTP_OFFSET, NTP_INTERVAL);
ZMPT101B voltageSensor(VOLTAGE_PIN, 50.0);
WiFiClientSecure client;
Adafruit_MQTT_Client mqtt(&client, AIO_SERVER.c_str(), 8883, AIO_USER.c_str(), AIO_KEY.c_str());
Adafruit_MQTT_Subscribe mqttSub(&mqtt, MQTT_FEED.c_str());
Supabase db;
WebServer server(80);

struct SensorData {
  float voltage;
  float current;
  unsigned long timestamp;
};





std::vector<SensorData> hourlyData;
bool apMode = false;
float zeroCurrentVoltage = 1.65;
bool outputState = false;
bool output;
unsigned long lastReconnectAttempt = 0;
static unsigned long lastMQTTCheck = 0;
bool lastState = false;



String getISTTimestamp(unsigned long epoch) {
  time_t rawtime = epoch;
  struct tm *ti;
  ti = gmtime(&rawtime);
  
  char buf[30];
  sprintf(buf, "%04d-%02d-%02dT%02d:%02d:%02d+05:30",
          ti->tm_year + 1900, ti->tm_mon + 1, ti->tm_mday,
          ti->tm_hour, ti->tm_min, ti->tm_sec);
  return String(buf);
}
void displayDisplayMessage(const char* message);
void setup() {
  Serial.begin(115200);
  
  // Initialize OLED
  display.begin(SH1106_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  displayDisplayMessage("Booting...");

  pinMode(OUTPUT_PIN, OUTPUT);
  pinMode(TRIGGER_PIN, INPUT_PULLUP);
  pinMode(TOGGLE_PIN, INPUT);

  preferences.begin("wifi", false);
  initWiFiConnection();

  if(!apMode) {
    initializeConnectedServices();
  }
}

void handleNormalOperation();
void handleNetworkReconnect();
void checkResetButton();
void displayAPModeScreen();


void loop() {
  if(apMode) {
    handleAPMode();
  } else {
    handleNormalOperation();
  }
  bool touched = touchRead(TOGGLE_PIN) < 30;
  if(touched != lastState) {
    lastState = touched;
    digitalWrite(OUTPUT_PIN, touched ? HIGH : LOW);
    updateStatus(touched);
    delay(100); // Simple debounce
  }
}





void updateStatus(bool isActive) {
  
String json = "{\"isOnline\":" + String(isActive ? "true" : "false") + "}";  
  int statusCode = db.from("paired_devices")
                  .eq("device_id", DEVICE_ID)
                  .doUpdate(json);
  
  Serial.print(isActive ? "Device ON" : "Device OFF");
  Serial.print(" | Status code: ");
  Serial.println(statusCode);
  
  if(statusCode >= 200 && statusCode < 300) {
    Serial.println("Update successful");
  } else {
    Serial.println("Update failed");
  }
}

// WiFi Management
void initWiFiConnection() {
  preferences.begin("wifi", true);  
  String ssid = preferences.getString("ssid", "");
  String pass = preferences.getString("password", "");
  String device_name = preferences.getString("device_name", "");
  String device_type = preferences.getString("device_type", "");
  String user_id = preferences.getString("user_id", "");
  preferences.end();

  if(!device_name.isEmpty() && !device_type.isEmpty()){
    DynamicJsonDocument doc(1024);
  doc["deviceId"] = DEVICE_ID;
  doc["userid"] = user_id;
  doc["deviceType"] = device_type;
  doc["deviceName"] = device_name;

  String json;
  serializeJson(doc, json);
  int code = db.insert("paired_devices", json, true);
  
  if(code >= 200 && code < 300){
    displayDisplayMessage("Successful");
  }
  else 
  displayDisplayMessage("something went wrong");
  }

  if(ssid.isEmpty() || pass.isEmpty()) {
    startAPMode();
    return;
  }

  WiFi.begin(ssid.c_str(), pass.c_str());
  displayDisplayMessage("Connecting to WiFi...");
  
  unsigned long start = millis();
  while(WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
    delay(500);
    Serial.print(".");
  }

  if(WiFi.status() != WL_CONNECTED) {
    startAPMode();
  } else {
    displayDisplayMessage("WiFi Connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  }
}

void startAPMode() {
  apMode = true;
  WiFi.mode(WIFI_AP);
  WiFi.softAP("SmartSwitch-AP", "12345678");
  server.on("/configure", HTTP_POST, handleConfig);
  server.begin();
  displayAPModeScreen();
}

// Connected Services
void initializeConnectedServices() {
  initializeTime();
  initializeMQTT();
  initializeSupabase();
  loadInitialState();
  calibrateSensors();
}

void initializeTime() {
  timeClient.begin();
  while(!timeClient.update()) {
    timeClient.forceUpdate();
    delay(100);
  }
}

void initializeMQTT() {
  client.setCACert(adafruitio_root_ca);
  mqtt.subscribe(&mqttSub);
  mqttSub.setCallback(mqttCallback);
  reconnectMQTT();
}

void initializeSupabase() {
  db.begin(supabase_url, supabase_key);
}

// Sensor Functions
void calibrateSensors() {
  displayDisplayMessage("Calibrating...");
  long total = 0;
  for(int i = 0; i < 500; i++) {
    total += analogRead(CURRENT_PIN);
    delay(2);
  }
  zeroCurrentVoltage = ((float)total / 500.0) * 3.3 / 4095.0;
  voltageSensor.setSensitivity(VOLTAGE_SENSITIVITY);
  displayDisplayMessage("Calibration Done!");
}

float readVoltage() {
  float total = 0;
  for(int i = 0; i < 10; i++) {
    total += voltageSensor.getRmsVoltage();
    delay(50);
  }
  return total / 10.0 < THRESHOLD_VOLTAGE ? 0.0 : total / 10.0;
}

float readCurrent() {
  float sumSq = 0;
  for(int i = 0; i < 1000; i++) {
    float voltage = (analogRead(CURRENT_PIN) / 4095.0) * 3.3;
    sumSq += pow(voltage - zeroCurrentVoltage, 2);
  }
  float current = sqrt(sumSq / 1000) / ACS_SENSITIVITY;
  
  static int stableCount = 0;
  if(current < 0.1 && ++stableCount < 3) current = 0.0;
  else stableCount = 0;
  
  return current < THRESHOLD_CURRENT ? 0.0 : current;
}

// Data Handling
void handleSensorData() {
  static unsigned long lastRead = 0;
  if(millis() - lastRead >= 1000) {
    hourlyData.push_back({
      readVoltage(),
      readCurrent(),
      timeClient.getEpochTime()
    });
    if(hourlyData.size() > 7200) hourlyData.erase(hourlyData.begin());
    lastRead = millis();
  }
}

void handleHourlyUpload() {
  static unsigned long lastUpload = 0;
  if(timeClient.getEpochTime() - lastUpload < 3600) return;

  for(int attempt = 0; attempt < 3; attempt++) {
    if(sendHourlyData()) {
      hourlyData.clear();
      lastUpload = timeClient.getEpochTime();
      return;
    }
    delay(5000);
  }
}

bool sendHourlyData() {
  if(hourlyData.empty()) return true;

  float avgVoltage = 0, avgCurrent = 0;
  for(auto& entry : hourlyData) {
    avgVoltage += entry.voltage;
    avgCurrent += entry.current;
  }

  DynamicJsonDocument doc(1024);
  doc["deviceId"] = DEVICE_ID;
  doc["power_consumption"] = ((avgVoltage / hourlyData.size())*(avgCurrent / hourlyData.size())) / 1000;
  doc["timestamp"] = getISTTimestamp(hourlyData[0].timestamp);

  String json;
  serializeJson(doc, json);
  int code = db.insert("device_power_logs", json, false);
  return (code >= 200 && code < 300);
}

// MQTT Callback
void mqttCallback(char* data, uint16_t len) {
  String payload(data);
  outputState = payload.equalsIgnoreCase("1");
  digitalWrite(OUTPUT_PIN, outputState);
  DynamicJsonDocument doc(128);
  doc["status"] = toupper(outputState);
  doc["device_id"] = DEVICE_ID;
  doc["requested_at"] = getISTTimestamp(timeClient.getEpochTime());
  
  String json;
  serializeJson(doc, json);
  db.insert("device_toggle_requests", json, true);
}

// Display Functions
void updateDisplay() {
  static unsigned long lastUpdate = 0;
  if(millis() - lastUpdate < 500) return;
  
  float currentVoltage = readVoltage();
  float currentCurrent = readCurrent();
  display.clearDisplay();
  display.setCursor(0,0);
  display.println(DEVICE_ID);
  display.println("-------------------");
  
  display.printf("Voltage: %.1f V\n", currentVoltage);
  display.printf("Current: %.2f A\n", currentCurrent);
  
  display.println("-------------------");
  display.printf("Status: %s\nWiFi: %s", 
                outputState ? "ON" : "OFF", 
                WiFi.SSID().c_str());
  display.display();
  lastUpdate = millis();
}

// ================== DISPLAY HELPER FUNCTIONS ==================
void displayAPModeScreen() {
  display.clearDisplay();
  display.setCursor(0,0);
  display.println("AP Mode: " DEVICE_ID);
  display.println("SSID: SmartSwitch-AP");
  display.print("IP: ");
  display.println(WiFi.softAPIP());
  display.display();
}

void displayDisplayMessage(const char* message) {
  display.clearDisplay();
  display.setCursor(5,30);
  display.setTextSize(1.5);
  display.println(message);
  display.display();
}

// ================== WIFI RECONNECTION HANDLING ==================
void handleNetworkReconnect() {
  static unsigned long lastAttempt = 0;
  if(millis() - lastAttempt > 3000) {
    displayDisplayMessage("Reconnecting WiFi...");
    WiFi.reconnect();
    lastAttempt = millis();
    
    // Wait for connection
    unsigned long start = millis();
    while(WiFi.status() != WL_CONNECTED && millis() - start < 5000) {
      delay(250);
    }
    
    if(WiFi.status() != WL_CONNECTED) {
      displayDisplayMessage("WiFi Connect Failed");
    }
  }
}

// ================== RESET BUTTON LOGIC ==================
void checkResetButton() {
  static unsigned long pressStart = 0;
  static bool buttonActive = false;

  if(digitalRead(TRIGGER_PIN) == LOW) {
    if(!buttonActive) {
      buttonActive = true;
      pressStart = millis();
    }
    
    if(millis() - pressStart > 3000) {
      preferences.begin("wifi", false);
      preferences.clear();
      preferences.end();
      displayDisplayMessage("Reset Complete!\nRebooting...");
      delay(1000);
      ESP.restart();
    }
  } else {
    buttonActive = false;
  }
}

// ================== AP MODE HANDLERS ==================
void handleAPMode() {
  server.handleClient();
  checkResetButton();
  
  // Update AP mode display every 2 seconds
  static unsigned long lastAPUpdate = 0;
  if(millis() - lastAPUpdate > 2000) {
    displayAPModeScreen();
    lastAPUpdate = millis();
  }
}

void handleConfig() {
  if(server.hasArg("plain")) {
    DynamicJsonDocument doc(256);
    DeserializationError error = deserializeJson(doc, server.arg("plain"));
    
    if(error) {
      server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
      return;
    }

    String ssid = doc["ssid"].as<String>();
    String password = doc["password"].as<String>();
    String device_type = doc["device_type"].as<String>();
    String device_name = doc["device_name"].as<String>();
    String user_id = doc["user_id"].as<String>();

    if(!ssid.isEmpty() && !password.isEmpty()) {
      WiFi.begin(ssid.c_str(), password.c_str());
      
      unsigned long startTime = millis();
      while(WiFi.status() != WL_CONNECTED && millis() - startTime < 15000) {
        delay(500);
        Serial.print(".");
      }

      if(WiFi.status() == WL_CONNECTED) {
        preferences.begin("wifi", false);
        preferences.putString("ssid", ssid);
        preferences.putString("password", password);
        preferences.putString("device_type", device_type);
        preferences.putString("device_name", device_name);
        preferences.putString("user_id", user_id);
        preferences.end();

        server.send(200, "application/json", "{\"status\":\"success\"}");
        delay(1000);  // Ensure response is sent
        ESP.restart();
        return;
      }
    }
    server.send(400, "application/json", "{\"error\":\"Connection failed\"}");
  } else {
    server.send(400, "application/json", "{\"error\":\"No data received\"}");
  }
}

// ================== INITIAL STATE LOADING ==================
void loadInitialState() {
  String response = db.from("paired_devices")
                     .eq("deviceId", DEVICE_ID)
                     .doSelect();

  DynamicJsonDocument doc(256);
  DeserializationError error = deserializeJson(doc, response);
  
  if(!error && doc.size() > 0) {
    outputState = doc[0]["isOnline"];
    digitalWrite(OUTPUT_PIN, outputState);
    
    // Update local timestamp
    if(doc[0].containsKey("last_update")) {
      String timestamp = doc[0]["last_update"];
      // Here you would add code to parse timestamp if needed
    }
  }
}

// ================== MQTT RECONNECTION HANDLING ==================
void reconnectMQTT() {
  static unsigned long lastAttempt = 0;
  if(millis() - lastAttempt < 5000) return;

  displayDisplayMessage("MQTT Reconnecting...");
  
  int retries = 3;
  while(retries-- > 0) {
    int8_t ret = mqtt.connect();
    if(ret == 0) {
      mqtt.setKeepAliveInterval(15);
      mqtt.subscribe(&mqttSub);
      displayDisplayMessage("MQTT Connected!");
      lastAttempt = 0;
      return;
    }
    
    Serial.print("MQTT Connect Failed: ");
    Serial.println(mqtt.connectErrorString(ret));
    delay(3000);
  }
  
  lastAttempt = millis();
  displayDisplayMessage("MQTT Connect Failed");
}


void handleNormalOperation() {
  if(WiFi.status() != WL_CONNECTED) {
    handleNetworkReconnect();
    return;
  }
  
  if(millis() - lastMQTTCheck >= 50) {
    if(!mqtt.connected()) {
      reconnectMQTT();
    }
    mqtt.processPackets(10); 
    lastMQTTCheck = millis();
  }
  
  mqtt.processPackets(100);
  handleSensorData();
  updateDisplay();
  handleHourlyUpload();
}