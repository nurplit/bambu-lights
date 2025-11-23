/*
 * ESP32 Bambu Lab MQTT Sniffer
 * 
 * This sketch connects to your Bambu Lab printer's MQTT broker and dumps
 * all messages to Serial for capture and analysis.
 * 
 * Instructions:
 * 1. Update WiFi credentials below
 * 2. Update printer IP, access code, and serial number
 * 3. Upload to ESP32 (any variant - NodeMCU-32S, DevKitC, etc.)
 * 4. Open Serial Monitor at 115200 baud
 * 5. Use a serial capture tool to save output to .txt file
 * 
 * Hardware: ESP32 (any variant)
 * 
 * Required Libraries (install via Arduino Library Manager):
 * - WiFi (built-in with ESP32 board support)
 * - PubSubClient by Nick O'Leary
 * 
 * Board Selection: Tools -> Board -> ESP32 Arduino -> ESP32 Dev Module (or your board)
 */

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>

// ======================== CONFIGURATION ========================
// WiFi Settings
const char* WIFI_SSID = "YourWiFiSSID";           // Your WiFi network name
const char* WIFI_PASSWORD = "YourWiFiPassword";   // Your WiFi password

// Bambu Lab Printer Settings
const char* MQTT_SERVER = "192.168.0.100";        // Your printer's IP address
const int MQTT_PORT = 8883;                        // SSL port (8883) or non-SSL (1883)
const char* MQTT_USERNAME = "bblp";                // Always "bblp" for Bambu Lab
const char* MQTT_PASSWORD = "12345678";            // Your printer's access code
const char* DEVICE_SERIAL = "01S00A123456789";     // Your printer's serial number

const bool USE_SSL = true;                         // Use SSL/TLS (true for port 8883)
// ===============================================================

WiFiClientSecure wifiClientSecure;
WiFiClient wifiClient;
PubSubClient mqttClient;

unsigned long lastReconnect = 0;
unsigned long messageCount = 0;

void setup() {
  // Start serial at high speed for fast capture
  Serial.begin(115200);
  delay(100);
  
  Serial.println("\n\n");
  Serial.println("========================================");
  Serial.println("  ESP8266 Bambu Lab MQTT Sniffer");
  Serial.println("========================================");
  Serial.println();
  
  // Connect to WiFi
  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println();
  Serial.println("WiFi connected!");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println();
  
  // Configure MQTT client
  if (USE_SSL) {
    wifiClientSecure.setInsecure(); // Skip certificate verification (like Bambu does)
    mqttClient.setClient(wifiClientSecure);
  } else {
    mqttClient.setClient(wifiClient);
  }
  
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(32768); // 32KB buffer (ESP32 has plenty of RAM)
  
  Serial.println("Configuration:");
  Serial.print("  MQTT Server: ");
  Serial.println(MQTT_SERVER);
  Serial.print("  MQTT Port: ");
  Serial.println(MQTT_PORT);
  Serial.print("  SSL/TLS: ");
  Serial.println(USE_SSL ? "Enabled" : "Disabled");
  Serial.print("  Device Serial: ");
  Serial.println(DEVICE_SERIAL);
  Serial.println();
  Serial.println("========================================");
  Serial.println();
  
  // Initial connection
  connectMQTT();
}

void loop() {
  // Maintain MQTT connection
  if (!mqttClient.connected()) {
    unsigned long now = millis();
    if (now - lastReconnect > 5000) { // Try reconnect every 5 seconds
      lastReconnect = now;
      connectMQTT();
    }
  } else {
    mqttClient.loop();
  }
  
  // Yield to allow ESP8266 background tasks
  yield();
}

void connectMQTT() {
  Serial.println("Connecting to MQTT broker...");
  Serial.print("  Server: ");
  Serial.print(MQTT_SERVER);
  Serial.print(":");
  Serial.println(MQTT_PORT);
  
  // Generate unique client ID
  String clientId = "ESP8266-Sniffer-";
  clientId += String(random(0xffff), HEX);
  
  // Connect with username and password
  if (mqttClient.connect(clientId.c_str(), MQTT_USERNAME, MQTT_PASSWORD)) {
    Serial.println("MQTT Connected!");
    
    // Subscribe to all printer topics
    String reportTopic = "device/" + String(DEVICE_SERIAL) + "/report";
    
    if (mqttClient.subscribe(reportTopic.c_str(), 0)) {
      Serial.print("Subscribed to: ");
      Serial.println(reportTopic);
    } else {
      Serial.println("ERROR: Failed to subscribe!");
    }
    
    Serial.println();
    Serial.println("========================================");
    Serial.println("  Listening for MQTT messages...");
    Serial.println("  Start your timelapse print now!");
    Serial.println("========================================");
    Serial.println();
    
  } else {
    Serial.print("MQTT connection failed, rc=");
    Serial.println(mqttClient.state());
    Serial.println("Error codes:");
    Serial.println("  -4 : MQTT_CONNECTION_TIMEOUT");
    Serial.println("  -3 : MQTT_CONNECTION_LOST");
    Serial.println("  -2 : MQTT_CONNECT_FAILED");
    Serial.println("  -1 : MQTT_DISCONNECTED");
    Serial.println("   1 : MQTT_CONNECT_BAD_PROTOCOL");
    Serial.println("   2 : MQTT_CONNECT_BAD_CLIENT_ID");
    Serial.println("   3 : MQTT_CONNECT_UNAVAILABLE");
    Serial.println("   4 : MQTT_CONNECT_BAD_CREDENTIALS");
    Serial.println("   5 : MQTT_CONNECT_UNAUTHORIZED");
    Serial.println();
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  messageCount++;
  
  // Print message separator
  Serial.println();
  Serial.println("╔═══════════════════════════════════════════════════════════");
  Serial.print("║ MESSAGE #");
  Serial.println(messageCount);
  Serial.println("╟───────────────────────────────────────────────────────────");
  
  // Print timestamp
  Serial.print("║ Time: ");
  Serial.print(millis() / 1000);
  Serial.println(" seconds");
  
  // Print topic
  Serial.print("║ Topic: ");
  Serial.println(topic);
  
  // Print payload length
  Serial.print("║ Length: ");
  Serial.print(length);
  Serial.println(" bytes");
  
  Serial.println("╟───────────────────────────────────────────────────────────");
  Serial.println("║ PAYLOAD:");
  Serial.println("╟───────────────────────────────────────────────────────────");
  
  // Print the full payload
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  
  Serial.println();
  Serial.println("╚═══════════════════════════════════════════════════════════");
  Serial.println();
  
  // Flush serial buffer to ensure everything is sent
  Serial.flush();
  
  // Print a summary line for easy grepping
  Serial.print("SUMMARY: MSG#");
  Serial.print(messageCount);
  Serial.print(" | Time=");
  Serial.print(millis() / 1000);
  Serial.print("s | Size=");
  Serial.print(length);
  Serial.print("B | Topic=");
  Serial.println(topic);
  Serial.println();
}
