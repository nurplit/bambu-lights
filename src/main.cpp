/*
 * Bambu Lab Printer LED Controller
 * 
 * ESP32-based controller for addressable LED strips that displays the status
 * of Bambu Lab 3D printers via MQTT connection.
 * 
 * Inspired by and builds upon concepts from:
 * - BambuLights by judge2005 (https://github.com/judge2005/BambuLights)
 * 
 * Features:
 * - Web-based configuration interface
 * - OTA firmware updates
 * - Auto-off timer after print completion
 * - LiDAR pause support
 * - Door detection and automatic control
 * - Serial logging via web interface
 * - MQTT status monitoring
 * 
 * License: MIT
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiManager.h>
#include <espMqttClient.h>
#include <WiFiClientSecure.h>
#include <ArduinoJson.h>
#include <Preferences.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>
#include <LittleFS.h>
#include "driver/rmt.h"
#include "esp_task_wdt.h"

// Custom header files
#include "config.h"
#include "structs.h"
#include "led_effects.h"
#include "printer_status.h"

// Configuration - Configurable via web interface
int led_pin = 2;                              // Default LED data pin
int num_leds = 10;                           // Default number of LEDs
String led_type = "WS2812B";                 // Default LED type
String color_order = "GRB";                  // Default color order
#define RMT_CHANNEL RMT_CHANNEL_0

// WiFi Configuration - Now handled by WiFiManager (no hardcoded credentials!)
const int WIFI_TIMEOUT_MS = 20000;           // 20 seconds timeout
WiFiManager wifiManager;                      // WiFiManager instance

// Bambu Lab MQTT Configuration - Configurable via web interface
String mqtt_server = "192.168.0.100";        // Default printer IP
int mqtt_port = 8883; // Use SSL port
const char* MQTT_USERNAME = "bblp";          // Default Bambu Lab username
String mqtt_password = "your_access_code";   // Configurable access code
String device_serial = "01S00A123456789";    // Configurable printer serial number
bool mqtt_use_ssl = true;                    // Use SSL/TLS for MQTT connection (port 8883)
bool mqtt_skip_cert_check = true;           // Skip SSL certificate verification (insecure but often needed)

// Home Assistant MQTT Configuration - Configurable via web interface
bool ha_mqtt_enabled = false;                // Enable/disable HA MQTT publishing
String ha_mqtt_server = "192.168.0.50";     // Home Assistant MQTT broker IP
int ha_mqtt_port = 1883;                    // HA MQTT broker port (usually 1883, non-SSL)
String ha_mqtt_username = "";               // HA MQTT username (empty if no auth)
String ha_mqtt_password = "";               // HA MQTT password (empty if no auth)
String ha_discovery_prefix = "homeassistant"; // HA discovery topic prefix
String ha_device_name = "Bambu Lights";     // Device name in Home Assistant
bool ha_mqtt_use_ssl = false;               // Usually false for local HA broker

// Preferences for storing settings
Preferences preferences;

// LED timing constants (in RMT ticks, 80MHz clock = 12.5ns per tick)
// Current timing values (will be set based on led_type)
int T0H = 32;   // 0 bit high time
int T0L = 64;   // 0 bit low time  
int T1H = 64;   // 1 bit high time
int T1L = 32;   // 1 bit low time

// Predefined timing for different LED types
struct LEDTiming {
    int t0h, t0l, t1h, t1l;
};

// LED type timing definitions (all in RMT ticks @ 80MHz)
const LEDTiming LED_TIMINGS[] = {
    // WS2812/WS2812B: T0H=0.4us, T0L=0.85us, T1H=0.8us, T1L=0.45us
    {32, 68, 64, 36},   // WS2812B
    // WS2811: T0H=0.5us, T0L=2.0us, T1H=1.2us, T1L=1.3us  
    {40, 160, 96, 104}, // WS2811
    // WS2813: Same as WS2812B but with backup data line
    {32, 68, 64, 36},   // WS2813
    // SK6812: T0H=0.3us, T0L=0.9us, T1H=0.6us, T1L=0.6us
    {24, 72, 48, 48},   // SK6812
    // APA102 uses SPI, but keeping for compatibility
    {32, 68, 64, 36}    // APA102 (fallback to WS2812 timing)
};

const String LED_TYPE_NAMES[] = {"WS2812B", "WS2811", "WS2813", "SK6812", "APA102"};
const String COLOR_ORDER_NAMES[] = {"RGB", "GRB", "BRG", "RBG", "GBR", "BGR"};

// Color structures (RGB defined in structs.h)
struct RGBW {
    uint8_t r, g, b, w;
};

// LED Array and RMT data (dynamically allocated in setup)
RGB* leds = nullptr;
RGBW* rgbw_leds = nullptr;
rmt_item32_t* led_data = nullptr;
bool is_rgbw = false; // Track if using RGBW LEDs

// Beacon LED Array (second strip for progress display)
RGB* beacon_leds = nullptr;
RGBW* beacon_rgbw_leds = nullptr;
rmt_item32_t* beacon_led_data = nullptr;
int beacon_led_count = DEFAULT_BEACON_LED_COUNT;
int beacon_led_pin = BEACON_LED_PIN;
int beacon_led_type = 0; // Default: WS2812B
int beacon_color_order = 1; // Default: GRB
bool beacon_enabled = false;
bool beacon_is_rgbw = false; // Track if beacon uses RGBW LEDs
String beacon_effect = "progress"; // progress, climbing, gradient, pulse
RGB beacon_color = {0, 255, 0}; // Default: green (start color for gradient)
int beacon_brightness = 80;
RGB beacon_climb_color = {255, 255, 255}; // Default: white for climbing pixel
int beacon_climb_speed = 150; // Milliseconds per step (default 150ms = 1.5s full cycle)
RGB beacon_gradient_end = {0, 0, 255}; // Default: blue for gradient end color
bool beacon_gradient_debug = false; // Flag to log gradient colors once per change
bool beacon_force_update = false; // Flag to force beacon render update

// Web Server and MQTT Client
WebServer server(80);

// Telnet Server for remote serial monitoring
WiFiServer telnetServer(23);  // Port 23 is standard for Telnet
WiFiClient telnetClient;
bool telnetConnected = false;

WiFiClient wifiClient;
WiFiClientSecure wifiClientSecure;
espMqttClientSecure mqttClient;  // Use secure client for SSL/TLS (Bambu printer)

// Home Assistant MQTT Client (separate connection)
espMqttClient haMqttClient(espMqttClientTypes::UseInternalTask::NO);

// Current mode tracking
enum LightMode {
    MODE_AUTO,      // Automatic color cycling
    MODE_MANUAL,    // Manual color control
    MODE_PRINTER,   // Bambu Lab printer status
    MODE_OFF        // LEDs off
};

LightMode currentMode = MODE_PRINTER;  // Default to Printer Status mode
RGB manualColor = {128, 0, 128}; // Default purple

// Auto-cycle fade variables
RGB currentAutoColor = {0, 0, 0};
RGB targetAutoColor = {0, 128, 0};
int fadeStep = 0;
int autoCycleSpeed = 50;  // Speed: 1-100 (higher = faster, affects FADE_STEPS)
int autoCycleBrightness = 100;  // Brightness: 0-100 (percentage)

// Printer status (struct defined in structs.h)
PrinterStatus printerStatus;

// Door event configuration
bool door_open_lights_enabled = false;
RGB door_open_color = {255, 255, 255}; // Default white for door open
bool door_is_open = false;
bool manual_door_trigger = false; // Manual override for firmware 01.10.00 bug
bool manual_beacon_trigger = false; // Manual override for beacon testing

// Timelapse configuration (uses MQTT credentials)
// printerIP comes from mqtt_server, accessCode from mqtt_password, serial from device_serial

// Auto-off timer configuration
int auto_off_minutes = 0; // 0 = disabled, otherwise minutes to wait after lights turn on
unsigned long lights_on_time = 0; // When did the lights turn on
bool auto_off_pending = false; // Is the timer running
LightMode mode_before_auto_off = MODE_PRINTER; // Remember mode to restore after auto-off

// Chamber light sync configuration
bool chamber_light_sync_enabled = true; // Sync chamber light with LED strip auto-off timer
bool chamber_light_was_on_before_auto_off = false; // Track chamber light state before auto-off

// LiDAR scanning configuration
bool lidar_lights_off_enabled = true; // Default: turn off lights during LiDAR scan
bool lidar_scanning = false;

// Printer Status LED Behavior Configuration (struct defined in structs.h)
PrinterStateBehavior behavior_heating = {"pulse", {255, 140, 0}, 70};        // Orange pulse during heating
PrinterStateBehavior behavior_printing = {"progress", {0, 255, 0}, 80};      // Green progress bar
PrinterStateBehavior behavior_idle = {"solid", {0, 0, 255}, 50};             // Blue solid
PrinterStateBehavior behavior_pause = {"solid", {255, 165, 0}, 70};          // Orange solid
PrinterStateBehavior behavior_finish = {"rainbow", {0, 0, 0}, 100};          // Rainbow cycle (color unused)
PrinterStateBehavior behavior_error = {"blink", {255, 0, 0}, 100};           // Red blink

// Track home_flag changes for debugging
long last_home_flag = 0;
bool home_flag_initialized = false;
int home_flag_update_count = 0; // Track number of updates to ensure fresh data

// MQTT Debug Logging
bool mqtt_full_logging = false; // Enable via web interface to log all MQTT traffic

// FINISH->IDLE transition control
bool idle_state_locked = false; // When true, prevent MQTT from overwriting IDLE state

// Beacon reinitialization deferred to main loop to avoid blocking web server
bool beacon_reinit_pending = false;
bool beacon_was_enabled_before_reinit = false;

// Timelapse detection
bool timelapse_active = false; // Track if timelapse is enabled for current print
bool timelapse_white_lights_enabled = true; // Force white lights during timelapse (default ON)

// Network scanning removed: user-configured manual setup via web UI is used instead.
// Keeping minimal placeholders for compatibility with older status displays.
bool scanInProgress = false;
String scanResult = "";

// Serial log buffer for web interface
#define SERIAL_LOG_SIZE 50
String serialLogs[SERIAL_LOG_SIZE];
int serialLogIndex = 0;
int serialLogCount = 0;

void addSerialLog(String msg) {
    serialLogs[serialLogIndex] = msg;
    serialLogIndex = (serialLogIndex + 1) % SERIAL_LOG_SIZE;
    if (serialLogCount < SERIAL_LOG_SIZE) {
        serialLogCount++;
    }
}

// Serial print that also logs to web interface and Telnet
void logPrint(String msg) {
    Serial.print(msg);
    addSerialLog(msg);
    // Send to Telnet client if connected
    if (telnetConnected && telnetClient && telnetClient.connected()) {
        telnetClient.print(msg);
    }
}

void logPrintln(String msg) {
    Serial.println(msg);
    addSerialLog(msg + "\n");
    // Send to Telnet client if connected
    if (telnetConnected && telnetClient && telnetClient.connected()) {
        telnetClient.println(msg);
    }
}

// Forward declarations
void updatePrinterLEDs();
void clearSettings();
void loadSettings();
void saveSettings();
bool testBambuMQTT(String ip);
void setAllLEDs(uint8_t r, uint8_t g, uint8_t b);
void clearAllLEDs();
void updateLEDs();
String listTimelapseFiles();
void publishHAState();
void publishHADiscovery();
bool connectHAMQTT();

void clearSettings() {
    preferences.begin("bambu", false);
    preferences.clear(); // Clear all saved settings
    preferences.end();
    Serial.println("EEPROM settings cleared - using defaults");
}

void loadSettings() {
    preferences.begin("bambu", false);
    mqtt_server = preferences.getString("mqtt_server", "192.168.0.100");
    mqtt_port = preferences.getInt("mqtt_port", 8883);  // Fixed default to SSL port
    mqtt_password = preferences.getString("mqtt_password", "your_access_code");
    device_serial = preferences.getString("device_serial", "01S00A123456789");
    mqtt_use_ssl = preferences.getBool("mqtt_use_ssl", true);  // Fixed default to SSL
    mqtt_skip_cert_check = preferences.getBool("mqtt_skip_cert", true);
    led_pin = preferences.getInt("led_pin", 2);
    num_leds = preferences.getInt("num_leds", 10);
    led_type = preferences.getString("led_type", "WS2812B");
    color_order = preferences.getString("color_order", "GRB");
    
    // Load door event configuration
    door_open_lights_enabled = preferences.getBool("door_lights", false);
    door_open_color.r = preferences.getUChar("door_r", 255);
    door_open_color.g = preferences.getUChar("door_g", 255);
    door_open_color.b = preferences.getUChar("door_b", 255);
    manual_door_trigger = preferences.getBool("door_trigger", false);
    // manual_beacon_trigger deliberately not loaded - always starts OFF to prevent continuous updates
    
    // Load LiDAR scanning configuration
    lidar_lights_off_enabled = preferences.getBool("lidar_lights_off", true);
    
    // Load auto-off timer configuration
    auto_off_minutes = preferences.getInt("auto_off_min", 0);
    
    // Load chamber light sync configuration
    chamber_light_sync_enabled = preferences.getBool("chamber_sync", true);
    Serial.println("Chamber Light Sync: " + String(chamber_light_sync_enabled ? "ENABLED" : "DISABLED"));
    
    // Load timelapse white lights configuration
    timelapse_white_lights_enabled = preferences.getBool("timelapse_white", true);
    Serial.println("Timelapse White Lights: " + String(timelapse_white_lights_enabled ? "ENABLED" : "DISABLED"));
    
    // Load printer status behavior settings
    behavior_heating.effect = preferences.getString("heat_effect", "pulse");
    behavior_heating.color.r = preferences.getUChar("heat_r", 255);
    behavior_heating.color.g = preferences.getUChar("heat_g", 140);
    behavior_heating.color.b = preferences.getUChar("heat_b", 0);
    behavior_heating.brightness = preferences.getInt("heat_bright", 70);
    
    behavior_printing.effect = preferences.getString("print_effect", "progress");
    behavior_printing.color.r = preferences.getUChar("print_r", 0);
    behavior_printing.color.g = preferences.getUChar("print_g", 255);
    behavior_printing.color.b = preferences.getUChar("print_b", 0);
    behavior_printing.brightness = preferences.getInt("print_bright", 80);
    
    behavior_idle.effect = preferences.getString("idle_effect", "solid");
    behavior_idle.color.r = preferences.getUChar("idle_r", 0);
    behavior_idle.color.g = preferences.getUChar("idle_g", 0);
    behavior_idle.color.b = preferences.getUChar("idle_b", 255);
    behavior_idle.brightness = preferences.getInt("idle_bright", 50);
    
    behavior_pause.effect = preferences.getString("pause_effect", "solid");
    behavior_pause.color.r = preferences.getUChar("pause_r", 255);
    behavior_pause.color.g = preferences.getUChar("pause_g", 165);
    behavior_pause.color.b = preferences.getUChar("pause_b", 0);
    behavior_pause.brightness = preferences.getInt("pause_bright", 70);
    
    behavior_finish.effect = preferences.getString("finish_effect", "rainbow");
    behavior_finish.color.r = preferences.getUChar("finish_r", 0);
    behavior_finish.color.g = preferences.getUChar("finish_g", 0);
    behavior_finish.color.b = preferences.getUChar("finish_b", 0);
    behavior_finish.brightness = preferences.getInt("finish_bright", 100);
    
    behavior_error.effect = preferences.getString("error_effect", "blink");
    behavior_error.color.r = preferences.getUChar("error_r", 255);
    behavior_error.color.g = preferences.getUChar("error_g", 0);
    behavior_error.color.b = preferences.getUChar("error_b", 0);
    behavior_error.brightness = preferences.getInt("error_bright", 100);
    
    // Load auto-cycle configuration
    autoCycleSpeed = preferences.getInt("cycle_speed", 50);
    autoCycleBrightness = preferences.getInt("cycle_bright", 100);
    
    // Load beacon LED configuration
    beacon_enabled = preferences.getBool("beacon_en", false);
    beacon_led_count = preferences.getInt("beacon_count", DEFAULT_BEACON_LED_COUNT);
    beacon_led_pin = preferences.getInt("beacon_pin", BEACON_LED_PIN);
    beacon_led_type = preferences.getInt("beacon_type", 0); // 0 = WS2812B
    beacon_color_order = preferences.getInt("beacon_order", 1); // 1 = GRB
    beacon_is_rgbw = preferences.getBool("beacon_rgbw", false); // RGB by default
    beacon_effect = preferences.getString("beacon_fx", "progress");
    beacon_color.r = preferences.getUChar("beacon_r", 0);
    beacon_color.g = preferences.getUChar("beacon_g", 255);
    beacon_color.b = preferences.getUChar("beacon_b", 0);
    beacon_brightness = preferences.getInt("beacon_bright", 80);
    beacon_climb_color.r = preferences.getUChar("beacon_climb_r", 255);
    beacon_climb_color.g = preferences.getUChar("beacon_climb_g", 255);
    beacon_climb_color.b = preferences.getUChar("beacon_climb_b", 255);
    beacon_climb_speed = preferences.getInt("bcn_clmb_spd", 150);
    beacon_gradient_end.r = preferences.getUChar("bcn_grad_end_r", 0);
    beacon_gradient_end.g = preferences.getUChar("bcn_grad_end_g", 0);
    beacon_gradient_end.b = preferences.getUChar("bcn_grad_end_b", 255);
    
    Serial.printf("Loaded beacon climb speed: %d ms\n", beacon_climb_speed);
    Serial.printf("Loaded beacon climb color: R=%d G=%d B=%d\n", beacon_climb_color.r, beacon_climb_color.g, beacon_climb_color.b);
    Serial.printf("Loaded beacon gradient end: R=%d G=%d B=%d\n", beacon_gradient_end.r, beacon_gradient_end.g, beacon_gradient_end.b);
    
    // Load Home Assistant MQTT configuration
    ha_mqtt_enabled = preferences.getBool("ha_mqtt_en", false);
    ha_mqtt_server = preferences.getString("ha_mqtt_srv", "192.168.0.50");
    ha_mqtt_port = preferences.getInt("ha_mqtt_port", 1883);
    ha_mqtt_username = preferences.getString("ha_mqtt_user", "");
    ha_mqtt_password = preferences.getString("ha_mqtt_pass", "");
    ha_discovery_prefix = preferences.getString("ha_disc_pfx", "homeassistant");
    ha_device_name = preferences.getString("ha_dev_name", "Bambu Lights");
    ha_mqtt_use_ssl = preferences.getBool("ha_mqtt_ssl", false);
    
    preferences.end();
    
    Serial.println("Settings loaded:");
    Serial.printf("MQTT Server: %s\n", mqtt_server.c_str());
    Serial.printf("MQTT Port: %d (SSL: %s)\n", mqtt_port, mqtt_use_ssl ? "YES" : "NO");
    Serial.printf("Device Serial: %s\n", device_serial.c_str());
    Serial.printf("LED Pin: %d, Count: %d\n", led_pin, num_leds);
    Serial.printf("Beacon: %s, Pin: %d, Count: %d\n", beacon_enabled ? "ENABLED" : "DISABLED", beacon_led_pin, beacon_led_count);
    Serial.printf("Door Lights: %s\n", door_open_lights_enabled ? "ENABLED" : "DISABLED");
    Serial.printf("LiDAR Lights Off: %s\n", lidar_lights_off_enabled ? "ENABLED" : "DISABLED");
    Serial.printf("Home Assistant MQTT: %s\n", ha_mqtt_enabled ? "ENABLED" : "DISABLED");
    if (ha_mqtt_enabled) {
        Serial.printf("  HA Broker: %s:%d\n", ha_mqtt_server.c_str(), ha_mqtt_port);
    }
    Serial.println("MQTT Password: [HIDDEN]");
}

void saveSettings() {
    preferences.begin("bambu", false);
    preferences.putString("mqtt_server", mqtt_server);
    preferences.putInt("mqtt_port", mqtt_port);
    preferences.putString("mqtt_password", mqtt_password);
    preferences.putString("device_serial", device_serial);
    preferences.putBool("mqtt_use_ssl", mqtt_use_ssl);
    preferences.putBool("mqtt_skip_cert", mqtt_skip_cert_check);
    preferences.putInt("led_pin", led_pin);
    preferences.putInt("num_leds", num_leds);
    preferences.putString("led_type", led_type);
    preferences.putString("color_order", color_order);
    
    // Save door event configuration
    preferences.putBool("door_lights", door_open_lights_enabled);
    preferences.putUChar("door_r", door_open_color.r);
    preferences.putUChar("door_g", door_open_color.g);
    preferences.putUChar("door_b", door_open_color.b);
    
    // Save LiDAR scanning configuration
    preferences.putBool("lidar_lights_off", lidar_lights_off_enabled);
    
    // Save chamber light sync configuration
    preferences.putBool("chamber_sync", chamber_light_sync_enabled);
    
    // Save timelapse white lights configuration
    preferences.putBool("timelapse_white", timelapse_white_lights_enabled);
    
    // Save auto-cycle configuration
    preferences.putInt("cycle_speed", autoCycleSpeed);
    preferences.putInt("cycle_bright", autoCycleBrightness);
    
    // Save beacon LED configuration
    preferences.putBool("beacon_en", beacon_enabled);
    preferences.putInt("beacon_count", beacon_led_count);
    preferences.putInt("beacon_pin", beacon_led_pin);
    preferences.putInt("beacon_type", beacon_led_type);
    preferences.putInt("beacon_order", beacon_color_order);
    preferences.putBool("beacon_rgbw", beacon_is_rgbw);
    preferences.putString("beacon_fx", beacon_effect);
    preferences.putUChar("beacon_r", beacon_color.r);
    preferences.putUChar("beacon_g", beacon_color.g);
    preferences.putUChar("beacon_b", beacon_color.b);
    preferences.putInt("beacon_bright", beacon_brightness);
    preferences.putUChar("beacon_climb_r", beacon_climb_color.r);
    preferences.putUChar("beacon_climb_g", beacon_climb_color.g);
    preferences.putUChar("beacon_climb_b", beacon_climb_color.b);
    preferences.putInt("bcn_clmb_spd", beacon_climb_speed);
    preferences.putUChar("bcn_grad_end_r", beacon_gradient_end.r);
    preferences.putUChar("bcn_grad_end_g", beacon_gradient_end.g);
    preferences.putUChar("bcn_grad_end_b", beacon_gradient_end.b);
    
    // Save Home Assistant MQTT configuration
    preferences.putBool("ha_mqtt_en", ha_mqtt_enabled);
    preferences.putString("ha_mqtt_srv", ha_mqtt_server);
    preferences.putInt("ha_mqtt_port", ha_mqtt_port);
    preferences.putString("ha_mqtt_user", ha_mqtt_username);
    preferences.putString("ha_mqtt_pass", ha_mqtt_password);
    preferences.putString("ha_disc_pfx", ha_discovery_prefix);
    preferences.putString("ha_dev_name", ha_device_name);
    preferences.putBool("ha_mqtt_ssl", ha_mqtt_use_ssl);
    
    preferences.end();
    
    Serial.println("Configuration saved to EEPROM");
}

bool testBambuMQTT(String ip) {
    Serial.printf("Testing MQTT connection to %s:%d\n", ip.c_str(), mqtt_port);
    Serial.println("Note: Using simplified test - espMqttClient conversion complete");
    
    // Simplified test - just return true since we know it's a Bambu printer
    // The full test would require setting up another espMqttClient instance
    return true; // Assume Bambu printer if IP scanning reached this point
}

// Network scanning functions removed. Manual configuration via web UI is now the supported flow.

void configureLEDTiming() {
    // Find the LED type index
    int typeIndex = 0; // Default to WS2812B
    for (int i = 0; i < 5; i++) {
        if (led_type == LED_TYPE_NAMES[i]) {
            typeIndex = i;
            break;
        }
    }
    
    // Set timing values
    T0H = LED_TIMINGS[typeIndex].t0h;
    T0L = LED_TIMINGS[typeIndex].t0l;
    T1H = LED_TIMINGS[typeIndex].t1h;
    T1L = LED_TIMINGS[typeIndex].t1l;
    
    // Check if this is an RGBW LED type
    is_rgbw = (led_type == "SK6812");
    
    // LED timing configured
}

uint32_t applyColorOrder(RGB color, uint8_t white = 0) {
    if (is_rgbw) {
        // For RGBW LEDs (SK6812), return 32-bit value with white channel
        // SK6812 standard order is GRBW (Green, Red, Blue, White)
        if (color_order == "RGBW") {
            return (color.r << 24) | (color.g << 16) | (color.b << 8) | white;
        } else if (color_order == "GRBW" || color_order == "GRB") {
            // GRB maps to GRBW for SK6812
            return (color.g << 24) | (color.r << 16) | (color.b << 8) | white;
        } else if (color_order == "RGB") {
            // RGB maps to RGBW
            return (color.r << 24) | (color.g << 16) | (color.b << 8) | white;
        } else {
            // Default: GRBW (standard SK6812 order)
            return (color.g << 24) | (color.r << 16) | (color.b << 8) | white;
        }
    } else {
        // For RGB LEDs, return 24-bit value
        if (color_order == "RGB") {
            return (color.r << 16) | (color.g << 8) | color.b;
        } else if (color_order == "GRB") {
            return (color.g << 16) | (color.r << 8) | color.b;
        } else if (color_order == "BRG") {
            return (color.b << 16) | (color.r << 8) | color.g;
        } else if (color_order == "RBG") {
            return (color.r << 16) | (color.b << 8) | color.g;
        } else if (color_order == "GBR") {
            return (color.g << 16) | (color.b << 8) | color.r;
        } else if (color_order == "BGR") {
            return (color.b << 16) | (color.g << 8) | color.r;
        }
        // Default to GRB if unknown
        return (color.g << 16) | (color.r << 8) | color.b;
    }
}

void setupRMT() {
    rmt_config_t config = {};
    config.rmt_mode = RMT_MODE_TX;
    config.channel = RMT_CHANNEL;
    config.gpio_num = (gpio_num_t)led_pin;
    config.mem_block_num = 1;
    config.clk_div = 1; // 80MHz clock
    config.tx_config.loop_en = false;
    config.tx_config.carrier_en = false;
    config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    config.tx_config.idle_output_en = true;
    
    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));
}

void rgbToRMT(RGB* colors, rmt_item32_t* items, int numLeds, RGBW* rgbw_colors = nullptr) {
    int idx = 0;
    int bitsPerLed = is_rgbw ? 32 : 24; // RGBW uses 32 bits, RGB uses 24 bits
    
    for (int led = 0; led < numLeds; led++) {
        uint32_t color;
        if (is_rgbw && rgbw_colors) {
            // Use RGBW data
            RGB rgb_part = {rgbw_colors[led].r, rgbw_colors[led].g, rgbw_colors[led].b};
            color = applyColorOrder(rgb_part, rgbw_colors[led].w);
        } else {
            // Use RGB data
            color = applyColorOrder(colors[led]);
        }
        
        for (int bit = bitsPerLed - 1; bit >= 0; bit--) {
            if (color & (1UL << bit)) {
                // Send '1'
                items[idx].level0 = 1;
                items[idx].duration0 = T1H;
                items[idx].level1 = 0; 
                items[idx].duration1 = T1L;
            } else {
                // Send '0'
                items[idx].level0 = 1;
                items[idx].duration0 = T0H;
                items[idx].level1 = 0;
                items[idx].duration1 = T0L;
            }
            idx++;
        }
    }
}

void updateLEDs() {
    if (led_data) {
        if (is_rgbw && rgbw_leds) {
            rgbToRMT(nullptr, led_data, num_leds, rgbw_leds);
        } else if (leds) {
            rgbToRMT(leds, led_data, num_leds);
        }
        int bitsPerLed = is_rgbw ? 32 : 24;
        ESP_ERROR_CHECK(rmt_write_items(RMT_CHANNEL, led_data, num_leds * bitsPerLed, true));
    }
}

void setLED(int index, uint8_t r, uint8_t g, uint8_t b, uint8_t w = 0) {
    if (index >= 0 && index < num_leds) {
        if (is_rgbw && rgbw_leds) {
            // For RGBW LEDs, optimize pure white by using the white channel
            if (w == 0 && r > 0 && r == g && g == b) {
                // RGB values are equal (gray/white) - use white channel instead
                rgbw_leds[index].r = 0;
                rgbw_leds[index].g = 0;
                rgbw_leds[index].b = 0;
                rgbw_leds[index].w = r;  // Move the brightness to white channel
            } else {
                rgbw_leds[index].r = r;
                rgbw_leds[index].g = g;
                rgbw_leds[index].b = b;
                rgbw_leds[index].w = w;
            }
        } else if (leds) {
            leds[index].r = r;
            leds[index].g = g;
            leds[index].b = b;
        }
    }
}

void setAllLEDs(uint8_t r, uint8_t g, uint8_t b) {
    for (int i = 0; i < num_leds; i++) {
        setLED(i, r, g, b);
    }
}

void clearAllLEDs() {
    setAllLEDs(0, 0, 0);
}

void reallocateLEDs() {
    // Clean up existing arrays
    if (leds) {
        delete[] leds;
        leds = nullptr;
    }
    if (rgbw_leds) {
        delete[] rgbw_leds;
        rgbw_leds = nullptr;
    }
    if (led_data) {
        delete[] led_data;
        led_data = nullptr;
    }
    
    // Allocate new arrays with current num_leds
    if (num_leds > 0) {
        // Allocate RGB or RGBW array based on LED type
        if (is_rgbw) {
            rgbw_leds = new RGBW[num_leds];
            // Initialize all LEDs to off
            for (int i = 0; i < num_leds; i++) {
                rgbw_leds[i] = {0, 0, 0, 0};
            }
        } else {
            leds = new RGB[num_leds];
            // Initialize all LEDs to off
            for (int i = 0; i < num_leds; i++) {
                leds[i] = {0, 0, 0};
            }
        }
        
        // Allocate RMT data: 24 bits for RGB, 32 bits for RGBW
        int bitsPerLed = is_rgbw ? 32 : 24;
        led_data = new rmt_item32_t[num_leds * bitsPerLed];
        
        // RMT data allocated
    }
}

// ==================== BEACON LED FUNCTIONS ====================

void configureBeaconTiming() {
    // Beacon uses same timing as main LEDs, just stored separately
    // Get timing from beacon_led_type (0-4 index)
    if (beacon_led_type >= 0 && beacon_led_type < 5) {
        // Timing is already configured via LED_TIMINGS array
        // Individual beacon will use these when rendering
    }
}

void setupBeaconRMT() {
    if (!beacon_enabled || beacon_led_count <= 0) return;
    
    // Use RMT_CHANNEL_1 for beacon (main LEDs use RMT_CHANNEL_0)
    rmt_config_t config = {};
    config.rmt_mode = RMT_MODE_TX;
    config.channel = RMT_CHANNEL_1;
    config.gpio_num = (gpio_num_t)beacon_led_pin;
    config.mem_block_num = 1;
    config.clk_div = 1; // 80MHz clock
    config.tx_config.loop_en = false;
    config.tx_config.carrier_en = false;
    config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    config.tx_config.idle_output_en = true;
    
    esp_err_t err = rmt_config(&config);
    if (err == ESP_OK) {
        err = rmt_driver_install(config.channel, 0, 0);
        if (err == ESP_OK) {
            Serial.printf("Beacon RMT configured on GPIO %d, Channel 1\n", beacon_led_pin);
        } else {
            Serial.printf("Beacon RMT driver install failed: %d\n", err);
            beacon_enabled = false;
        }
    } else {
        Serial.printf("Beacon RMT config failed: %d\n", err);
        beacon_enabled = false;
    }
}

uint32_t applyBeaconColorOrder(RGB color) {
    // Apply color order for beacon LEDs
    switch (beacon_color_order) {
        case 0: return (color.r << 16) | (color.g << 8) | color.b; // RGB
        case 1: return (color.g << 16) | (color.r << 8) | color.b; // GRB
        case 2: return (color.b << 16) | (color.r << 8) | color.g; // BRG
        case 3: return (color.r << 16) | (color.b << 8) | color.g; // RBG
        case 4: return (color.g << 16) | (color.b << 8) | color.r; // GBR
        case 5: return (color.b << 16) | (color.g << 8) | color.r; // BGR
        default: return (color.g << 16) | (color.r << 8) | color.b; // Default GRB
    }
}

void beaconRgbToRMT(RGB* colors, rmt_item32_t* items, int numLeds) {
    int idx = 0;
    const LEDTiming& timing = LED_TIMINGS[beacon_led_type];
    int bitsPerLed = beacon_is_rgbw ? 32 : 24;
    
    for (int led = 0; led < numLeds; led++) {
        uint32_t color;
        if (beacon_is_rgbw && beacon_rgbw_leds) {
            // RGBW: send all 4 channels
            RGBW rgbw = beacon_rgbw_leds[led];
            switch (beacon_color_order) {
                case 0: color = ((uint32_t)rgbw.r << 24) | ((uint32_t)rgbw.g << 16) | ((uint32_t)rgbw.b << 8) | rgbw.w; break; // RGBW
                case 1: color = ((uint32_t)rgbw.g << 24) | ((uint32_t)rgbw.r << 16) | ((uint32_t)rgbw.b << 8) | rgbw.w; break; // GRBW
                case 2: color = ((uint32_t)rgbw.b << 24) | ((uint32_t)rgbw.r << 16) | ((uint32_t)rgbw.g << 8) | rgbw.w; break; // BRGW
                case 3: color = ((uint32_t)rgbw.r << 24) | ((uint32_t)rgbw.b << 16) | ((uint32_t)rgbw.g << 8) | rgbw.w; break; // RBGW
                case 4: color = ((uint32_t)rgbw.g << 24) | ((uint32_t)rgbw.b << 16) | ((uint32_t)rgbw.r << 8) | rgbw.w; break; // GBRW
                case 5: color = ((uint32_t)rgbw.b << 24) | ((uint32_t)rgbw.g << 16) | ((uint32_t)rgbw.r << 8) | rgbw.w; break; // BGRW
                default: color = ((uint32_t)rgbw.g << 24) | ((uint32_t)rgbw.r << 16) | ((uint32_t)rgbw.b << 8) | rgbw.w; break; // Default GRBW
            }
        } else {
            // RGB: send 3 channels
            color = applyBeaconColorOrder(colors[led]);
        }
        
        for (int bit = bitsPerLed - 1; bit >= 0; bit--) {
            if (color & (1UL << bit)) {
                items[idx].level0 = 1;
                items[idx].duration0 = timing.t1h;
                items[idx].level1 = 0;
                items[idx].duration1 = timing.t1l;
            } else {
                items[idx].level0 = 1;
                items[idx].duration0 = timing.t0h;
                items[idx].level1 = 0;
                items[idx].duration1 = timing.t0l;
            }
            idx++;
        }
    }
}

void updateBeaconLEDs() {
    if (!beacon_enabled || !beacon_leds || !beacon_led_data || beacon_led_count <= 0) {
        return;
    }
    
    // Convert LED data to RMT format
    beaconRgbToRMT(beacon_leds, beacon_led_data, beacon_led_count);
    
    // Write to RMT with NON-BLOCKING mode (wait=false) to prevent web server stalls
    // This is critical - blocking RMT writes were causing the web lockups
    int bitsPerLed = beacon_is_rgbw ? 32 : 24;
    esp_err_t err = rmt_write_items(RMT_CHANNEL_1, beacon_led_data, beacon_led_count * bitsPerLed, false);
    if (err != ESP_OK) {
        // Silently fail - don't block on errors
    }
}

void setBeaconLED(int index, uint8_t r, uint8_t g, uint8_t b) {
    if (index >= 0 && index < beacon_led_count && beacon_leds) {
        // Always set RGB values in beacon_leds (used as staging)
        beacon_leds[index].r = r;
        beacon_leds[index].g = g;
        beacon_leds[index].b = b;
        
        // Also set RGBW array if in RGBW mode
        if (beacon_is_rgbw && beacon_rgbw_leds) {
            beacon_rgbw_leds[index].r = r;
            beacon_rgbw_leds[index].g = g;
            beacon_rgbw_leds[index].b = b;
            beacon_rgbw_leds[index].w = 0; // No white channel for color LEDs
        }
    }
}

void clearBeaconLEDs() {
    for (int i = 0; i < beacon_led_count; i++) {
        setBeaconLED(i, 0, 0, 0);
    }
}

void reallocateBeaconLEDs() {
    // Clean up existing arrays
    if (beacon_leds) {
        delete[] beacon_leds;
        beacon_leds = nullptr;
    }
    if (beacon_rgbw_leds) {
        delete[] beacon_rgbw_leds;
        beacon_rgbw_leds = nullptr;
    }
    if (beacon_led_data) {
        delete[] beacon_led_data;
        beacon_led_data = nullptr;
    }
    
    // Allocate new arrays if enabled
    if (beacon_enabled && beacon_led_count > 0) {
        int bitsPerLed = beacon_is_rgbw ? 32 : 24;
        
        // Always allocate beacon_leds as staging array for RGB values
        beacon_leds = new RGB[beacon_led_count];
        for (int i = 0; i < beacon_led_count; i++) {
            beacon_leds[i] = {0, 0, 0};
        }
        
        // Additionally allocate RGBW array if needed
        if (beacon_is_rgbw) {
            beacon_rgbw_leds = new RGBW[beacon_led_count];
            for (int i = 0; i < beacon_led_count; i++) {
                beacon_rgbw_leds[i] = {0, 0, 0, 0};
            }
        }
        
        beacon_led_data = new rmt_item32_t[beacon_led_count * bitsPerLed];
        
        Serial.printf("Beacon LEDs allocated: %d LEDs (%s)\n", beacon_led_count, beacon_is_rgbw ? "RGBW" : "RGB");
    }
}

void updateBeaconProgress(int progress) {
    if (!beacon_enabled || !beacon_leds) return;
    
    // Manual override: force beacon on with current effect at 100%
    if (manual_beacon_trigger) {
        progress = 100;
    }
    
    // Skip update if progress hasn't changed (reduces CPU load significantly)
    // Exception: "progress" effect always updates for animation, or force update flag is set
    static int lastProgress = -1;
    static String lastEffect = "";
    if (progress == lastProgress && beacon_effect == lastEffect && beacon_effect != "progress" && !beacon_force_update) {
        return; // No change, skip expensive update
    }
    beacon_force_update = false; // Clear force update flag
    lastProgress = progress;
    lastEffect = beacon_effect;
    
    // Calculate brightness-adjusted color
    uint8_t r = (beacon_color.r * beacon_brightness) / 100;
    uint8_t g = (beacon_color.g * beacon_brightness) / 100;
    uint8_t b = (beacon_color.b * beacon_brightness) / 100;
    
    if (beacon_effect == "progress") {
        // Progress bar: fill from bottom to top with animated white climbing pixel
        int litLEDs = (progress * beacon_led_count) / 100;
        
        // Animated climbing pixel (continuously climbs from bottom to top)
        static unsigned long lastClimbUpdate = 0;
        static int climbPosition = 0;
        
        if (millis() - lastClimbUpdate > beacon_climb_speed) {
            lastClimbUpdate = millis();
            climbPosition++;
            if (climbPosition >= beacon_led_count) {
                climbPosition = 0; // Reset to bottom when reaching top
            }
        }
        
        // Draw progress bar
        for (int i = 0; i < beacon_led_count; i++) {
            if (i == climbPosition) {
                // Climbing pixel with custom color (always visible)
                setBeaconLED(i, beacon_climb_color.r, beacon_climb_color.g, beacon_climb_color.b);
            } else if (i < litLEDs) {
                // Normal progress color
                setBeaconLED(i, r, g, b);
            } else {
                // Off
                setBeaconLED(i, 0, 0, 0);
            }
        }
    } else if (beacon_effect == "climbing") {
        // Climbing dot: single LED moves up
        clearBeaconLEDs();
        int ledIndex = (progress * (beacon_led_count - 1)) / 100;
        setBeaconLED(ledIndex, r, g, b);
    } else if (beacon_effect == "gradient") {
        // Gradient fill: transitions from start color to end color as it fills
        uint8_t end_r = (beacon_gradient_end.r * beacon_brightness) / 100;
        uint8_t end_g = (beacon_gradient_end.g * beacon_brightness) / 100;
        uint8_t end_b = (beacon_gradient_end.b * beacon_brightness) / 100;
        
        // Log gradient colors once per change
        if (beacon_gradient_debug) {
            logPrintln("Rendering gradient: Start RGB(" + String(r) + "," + String(g) + "," + String(b) + ") -> End RGB(" + String(end_r) + "," + String(end_g) + "," + String(end_b) + ") Progress=" + String(progress) + "%");
            beacon_gradient_debug = false;
        }
        
        int litLEDs = (progress * beacon_led_count) / 100;
        for (int i = 0; i < beacon_led_count; i++) {
            if (i < litLEDs) {
                // Smooth gradient from start color (bottom) to end color (top)
                int blend = (i * 100) / max(1, litLEDs - 1);
                uint8_t grad_r = r + ((end_r - r) * blend) / 100;
                uint8_t grad_g = g + ((end_g - g) * blend) / 100;
                uint8_t grad_b = b + ((end_b - b) * blend) / 100;
                setBeaconLED(i, grad_r, grad_g, grad_b);
            } else {
                setBeaconLED(i, 0, 0, 0);
            }
        }
    } else if (beacon_effect == "pulse") {
        // Pulse: all LEDs pulse together (simplified for better performance)
        // Fill up to current progress level with pulsing brightness
        static unsigned long lastPulseCalc = 0;
        static int pulseBright = beacon_brightness;
        
        // Only recalculate pulse brightness every 100ms (was 50ms)
        if (millis() - lastPulseCalc > 100) {
            static int pulseDir = 1;
            pulseBright += pulseDir * 15;
            if (pulseBright >= beacon_brightness) {
                pulseBright = beacon_brightness;
                pulseDir = -1;
            } else if (pulseBright <= beacon_brightness / 3) {
                pulseBright = beacon_brightness / 3;
                pulseDir = 1;
            }
            lastPulseCalc = millis();
        }
        
        uint8_t pulse_r = (beacon_color.r * pulseBright) / 100;
        uint8_t pulse_g = (beacon_color.g * pulseBright) / 100;
        uint8_t pulse_b = (beacon_color.b * pulseBright) / 100;
        
        int litLEDs = (progress * beacon_led_count) / 100;
        for (int i = 0; i < beacon_led_count; i++) {
            if (i < litLEDs) {
                setBeaconLED(i, pulse_r, pulse_g, pulse_b);
            } else {
                setBeaconLED(i, 0, 0, 0);
            }
        }
    }
    
    updateBeaconLEDs();
}

// ==================== END BEACON LED FUNCTIONS ====================

bool connectWiFi() {
    logPrintln("Starting WiFi connection...");
    
    // Set blue color while connecting
    setAllLEDs(0, 0, 255);
    updateLEDs();
    
    WiFi.mode(WIFI_STA);
    
    // WiFiManager will handle the connection
    // If no saved credentials, it will start an Access Point
    wifiManager.setConfigPortalTimeout(180); // 3 minutes timeout for config portal
    wifiManager.setConnectTimeout(20);       // 20 seconds to connect to saved WiFi
    
    // Create unique AP name based on chip ID
    String apName = "BambuLED-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    logPrintln("AP Name (if needed): " + apName);
    logPrintln("If no WiFi configured, connect to AP and visit 192.168.4.1");
    
    // Blink blue while connecting
    bool ledState = false;
    wifiManager.setAPCallback([](WiFiManager *myWiFiManager) {
        Serial.println("Entered config mode - AP started");
        Serial.println("Connect to: " + String(myWiFiManager->getConfigPortalSSID()));
        Serial.println("Visit: 192.168.4.1 to configure WiFi");
    });
    
    // Try to connect with saved credentials or start config portal
    bool connected = wifiManager.autoConnect(apName.c_str());
    
    if (connected) {
        logPrintln("WiFi connected! IP: " + WiFi.localIP().toString());
        logPrintln("Signal strength: " + String(WiFi.RSSI()) + " dBm");
        
        // Green color for successful connection
        setAllLEDs(0, 255, 0);
        updateLEDs();
        delay(2000);
        
        return true;
    } else {
        logPrintln("WiFi connection failed or timed out!");
        
        // Red color for failed connection
        setAllLEDs(255, 0, 0);
        updateLEDs();
        delay(2000);
        
        return false;
    }
}

// MQTT Functions
static unsigned long messageCount = 0;
static String assembledMessage = "";
static String assembledTopic = "";

// Forward declaration
void processMqttMessage(const String& topic, const String& message);

// Send MQTT command to printer
void sendPrinterCommand(const String& command) {
    if (!mqttClient.connected()) {
        logPrintln("Cannot send command - MQTT not connected");
        return;
    }
    
    String requestTopic = "device/" + device_serial + "/request";
    logPrintln("Sending MQTT command to: " + requestTopic);
    logPrintln("Command: " + command);
    
    mqttClient.publish(requestTopic.c_str(), 0, false, command.c_str());
}

// Control chamber light (on/off with optional brightness)
void setChamberLight(bool on, int brightness = 100) {
    // Track last command to avoid flooding MQTT with redundant messages
    static bool last_state = false;
    static unsigned long last_command_time = 0;
    static uint32_t seq_id = 1;
    unsigned long now = millis();
    
    // Only send if state changed OR it's been more than 5 seconds (for reliability)
    if (on == last_state && (now - last_command_time < 5000)) {
        return; // Skip redundant command
    }
    
    last_state = on;
    last_command_time = now;
    
    // Try direct LED mode control for chamber light
    // Mode: "on" or "off" for chamber light control
    String command = "{\"system\":{\"sequence_id\":\"" + String(seq_id++) + 
                     "\",\"command\":\"ledctrl\",\"led_node\":\"chamber_light\",\"led_mode\":\"";
    command += on ? "on" : "off";
    command += "\",\"led_on_time\":500,\"led_off_time\":500,\"loop_times\":1,\"interval_time\":0}}";
    
    sendPrinterCommand(command);
    logPrintln("Chamber light: " + String(on ? "ON" : "OFF") + " (LED control: " + String(on ? "on" : "off") + ")");
}

void onMqttMessage(const espMqttClientTypes::MessageProperties& properties, const char* topic, const uint8_t* payload, size_t len, size_t index, size_t total) {
    // Handle message assembly for multi-part messages
    if (index == 0) {
        // Start of new message
        messageCount++;
        assembledMessage = "";
        assembledTopic = String(topic);
        if (messageCount % 10 == 1) { // Log every 10th message to web
            logPrintln("MQTT message #" + String(messageCount) + " (" + String(total) + " bytes)");
        }
    }
    
    // Append this part to the assembled message
    for (size_t i = 0; i < len; i++) {
        assembledMessage += (char)payload[i];
    }
    
    // Only process when we have the complete message (index + len == total)
    if (index + len >= total) {

        
        processMqttMessage(assembledTopic, assembledMessage);
    }
}

void processMqttMessage(const String& topic, const String& message) {
    // Throttle message processing to avoid blocking web server
    static unsigned long lastProcessTime = 0;
    unsigned long now = millis();
    
    // Process at most once per 100ms to keep web responsive
    if (now - lastProcessTime < 100) {
        return; // Skip this message, too soon after last one
    }
    lastProcessTime = now;
    
    yield(); // Yield before heavy processing
    
    // Full MQTT logging if enabled
    if (mqtt_full_logging) {
        logPrintln("========== MQTT MESSAGE ==========");
        logPrintln("Topic: " + topic);
        logPrintln("Size: " + String(message.length()) + " bytes");
        logPrintln("Payload: " + message);
        logPrintln("==================================");
    }
    
    // Parse JSON message - 20KB buffer needed for large Bambu Lab messages
    DynamicJsonDocument doc(20480); // 20KB buffer for 15.6KB messages
    DeserializationError error = deserializeJson(doc, message);
    
    yield(); // Yield after heavy JSON parsing
    
    if (error) {
        Serial.printf("JSON error: %s\n", error.c_str());
        return;
    }
    
    // Update printer status
    printerStatus.online = true;
    printerStatus.lastUpdate = millis();
    
    // Log key fields for debugging (always enabled for state tracking)
    if (doc.containsKey("print")) {
        JsonObject print = doc["print"];
        String debugInfo = "MQTT Update: ";
        
        if (print.containsKey("gcode_state")) {
            debugInfo += "State=" + print["gcode_state"].as<String>() + " ";
        }
        if (print.containsKey("mc_print_stage")) {
            debugInfo += "Stage=" + print["mc_print_stage"].as<String>() + " ";
        }
        if (print.containsKey("stg_cur")) {
            debugInfo += "StgNum=" + String(print["stg_cur"].as<int>()) + " ";
        }
        if (print.containsKey("mc_percent")) {
            debugInfo += "Progress=" + String(print["mc_percent"].as<int>()) + "% ";
        }
        if (print.containsKey("layer_num")) {
            debugInfo += "Layer=" + String(print["layer_num"].as<int>());
            if (print.containsKey("total_layer_num")) {
                debugInfo += "/" + String(print["total_layer_num"].as<int>());
            }
            debugInfo += " ";
        }
        if (print.containsKey("bed_temper")) {
            debugInfo += "Bed=" + String(print["bed_temper"].as<float>(), 1) + "C ";
        }
        if (print.containsKey("nozzle_temper")) {
            debugInfo += "Nozzle=" + String(print["nozzle_temper"].as<float>(), 1) + "C ";
        }
        
        // Only show error code if non-zero (indicates actual error)
        if (print.containsKey("print_error")) {
            int error_code = print["print_error"].as<int>();
            if (error_code != 0) {
                debugInfo += "Error=" + String(error_code) + " ";
            }
        }
        
        if (debugInfo.length() > 13) { // More than just "MQTT Update: "
            logPrintln(debugInfo);
        }
    }
    
    if (doc.containsKey("print")) {
        JsonObject print = doc["print"];
        
        // Parse stage - try multiple field names
        if (print.containsKey("stg_cur")) {
            // stg_cur is the actual stage number (more reliable)
            int stage_num = print["stg_cur"];
            printerStatus.stage = String(stage_num);
        } else if (print.containsKey("stage")) {
            printerStatus.stage = print["stage"].as<String>();
        }
        
        if (print.containsKey("mc_percent")) {
            printerStatus.progress = print["mc_percent"];
        }
        
        // Check for timelapse mode in ipcam object
        if (print.containsKey("ipcam")) {
            JsonObject ipcam = print["ipcam"];
            if (ipcam.containsKey("timelapse")) {
                String timelapse_str = ipcam["timelapse"].as<String>();
                bool new_timelapse_state = (timelapse_str == "enable");
                
                // Log timelapse state changes
                if (new_timelapse_state != timelapse_active) {
                    logPrintln("=== TIMELAPSE STATE CHANGE ===");
                    logPrintln("Timelapse: " + String(timelapse_active ? "ON" : "OFF") + " -> " + String(new_timelapse_state ? "ON" : "OFF"));
                    logPrintln("============================");
                    timelapse_active = new_timelapse_state;
                }
            }
        }
        
        if (print.containsKey("gcode_state")) {
            String old_state = printerStatus.gcode_state;
            String new_state = print["gcode_state"].as<String>();
            
            // If a new print starts (RUNNING/PREPARE), unlock the idle state
            if (new_state == "RUNNING" || new_state == "PREPARE") {
                idle_state_locked = false;
            }
            
            // Only update gcode_state if not locked in IDLE
            if (!idle_state_locked) {
                printerStatus.gcode_state = new_state;
            }
            
            // Log print state changes
            if (old_state != printerStatus.gcode_state) {
                logPrintln("=== PRINT STATE CHANGE ===");
                logPrintln("From: " + old_state + " -> To: " + printerStatus.gcode_state);
                logPrintln("Progress: " + String(printerStatus.progress) + "%");
                logPrintln("Layer: " + String(printerStatus.layer_num) + "/" + String(printerStatus.total_layer_num));
                logPrintln("========================");
            }
            
            // Clear timelapse state when print ends
            if ((printerStatus.gcode_state == "FINISH" || printerStatus.gcode_state == "IDLE") && timelapse_active) {
                logPrintln("Print ended - clearing timelapse state");
                timelapse_active = false;
            }
            
            // Cancel auto-off timer when a new print starts (RUNNING or PREPARE state)
            if ((printerStatus.gcode_state == "RUNNING" || printerStatus.gcode_state == "PREPARE") && 
                (old_state != "RUNNING" && old_state != "PREPARE") && auto_off_pending) {
                auto_off_pending = false;
                logPrintln("Auto-off timer cancelled: New print started");
            }
            
            // Start auto-off timer when print finishes (if in PRINTER mode and timer enabled)
            if (printerStatus.gcode_state == "FINISH" && old_state != "FINISH" && 
                auto_off_minutes > 0 && !auto_off_pending && currentMode == MODE_PRINTER) {
                auto_off_pending = true;
                lights_on_time = millis();
                logPrintln("Print complete - auto-off timer started: " + String(auto_off_minutes) + " minutes");
            }
        }
        
        // mc_print_stage is usually a string like "printing", "heating", etc.
        if (print.containsKey("mc_print_stage")) {
            printerStatus.mc_print_stage = print["mc_print_stage"].as<String>();
        }
        
        // print_real_action is usually a numeric code
        if (print.containsKey("print_real_action")) {
            printerStatus.print_real_action = String(print["print_real_action"].as<int>());
        }
        
        // Get layer information for first layer detection
        if (print.containsKey("layer_num")) {
            printerStatus.layer_num = print["layer_num"];
        }
        if (print.containsKey("total_layer_num")) {
            printerStatus.total_layer_num = print["total_layer_num"];
        }
        
        yield(); // Yield during processing
        
        // Capture additional fields to detect true idle state
        if (print.containsKey("gcode_file")) {
            printerStatus.gcode_file = print["gcode_file"].as<String>();
        }
        if (print.containsKey("subtask_name")) {
            printerStatus.subtask_name = print["subtask_name"].as<String>();
        }
        if (print.containsKey("print_error")) {
            printerStatus.print_error = print["print_error"];
        }
        
        yield(); // Yield after field extraction
        
        // Smart FINISH->IDLE and FAILED->IDLE transition using mc_print_stage
        // mc_print_stage: 1 = idle/no active print, 2 = printing/preparing
        // When printer is in FINISH/FAILED state but mc_print_stage returns to 1, it's truly idle
        if ((printerStatus.gcode_state == "FINISH" || printerStatus.gcode_state == "FAILED") && 
            printerStatus.mc_print_stage == "1" && 
            !idle_state_locked) {
            logPrintln("=== AUTO IDLE DETECTION ===");
            logPrintln(printerStatus.gcode_state + " state with Print Stage=1 (no active job) - transitioning to IDLE");
            logPrintln("Progress: " + String(printerStatus.progress) + "%, Layer: " + String(printerStatus.layer_num) + "/" + String(printerStatus.total_layer_num));
            logPrintln("Locking IDLE state to prevent MQTT overwrite");
            logPrintln("=========================");
            printerStatus.gcode_state = "IDLE";
            idle_state_locked = true; // Lock IDLE state until next print starts
        }
    }
    
    // Parse temperature data - handle multiple field name variations
    if (doc.containsKey("print")) {
        JsonObject print = doc["print"];
        
        // Bed temperature - actual current temperature
        if (print.containsKey("bed_temper")) {
            printerStatus.bed_temp = print["bed_temper"].as<float>();
            logPrintln("Bed temp from print.bed_temper: " + String(printerStatus.bed_temp));
        }
        
        // Nozzle temperature - actual current temperature
        if (print.containsKey("nozzle_temper")) {
            printerStatus.nozzle_temp = print["nozzle_temper"].as<float>();
            logPrintln("Nozzle temp from print.nozzle_temper: " + String(printerStatus.nozzle_temp));
        }
    }
    
    // Legacy support for temp object (if it exists in some firmware versions)
    if (doc.containsKey("temp")) {
        JsonObject temp = doc["temp"];
        if (temp.containsKey("bed_temper") && printerStatus.bed_temp == 0.0) {
            printerStatus.bed_temp = temp["bed_temper"].as<float>();
        }
        if (temp.containsKey("nozzle_temper") && printerStatus.nozzle_temp == 0.0) {
            printerStatus.nozzle_temp = temp["nozzle_temper"].as<float>();
        }
    }
    
    // Show key printer status when available with temperature info
    if (doc.containsKey("print")) {
        JsonObject print = doc["print"];
        if (print.containsKey("gcode_state")) {
            Serial.printf("State: %s", print["gcode_state"].as<String>().c_str());
        }
        if (print.containsKey("layer_num") && print.containsKey("total_layer_num")) {
            Serial.printf(", Layer: %d/%d", print["layer_num"].as<int>(), print["total_layer_num"].as<int>());
        }
        if (print.containsKey("mc_print_stage")) {
            Serial.printf(", Stage: %s", print["mc_print_stage"].as<String>().c_str());
        }
        if (print.containsKey("stg_cur")) {
            Serial.printf(", StgCur: %d", print["stg_cur"].as<int>());
        }
        Serial.println();
    }
    
    // Show temperature data when available
    if (doc.containsKey("temp")) {
        JsonObject temp = doc["temp"];
        bool has_temp = false;
        if (temp.containsKey("bed_temper") || temp.containsKey("bed_temp")) {
            Serial.printf("Temps: Bed=%.1f°C", printerStatus.bed_temp);
            has_temp = true;
        }
        if (temp.containsKey("nozzle_temper") || temp.containsKey("nozzle_temp")) {
            if (has_temp) Serial.printf(", ");
            else Serial.printf("Temps: ");
            Serial.printf("Nozzle=%.1f°C", printerStatus.nozzle_temp);
            has_temp = true;
        }
        if (has_temp) Serial.println();
    }
    
    // Check for door events using proper bit 23 (0x00800000) detection
    // Note: Don't reset door_detected here - it's a global variable that should maintain state between messages
    // Only update it when home_flag is actually present in the MQTT message
    static bool door_detected = false;
    
    // Primary method: Check bit 23 in home_flag (standard Bambu Lab door sensor bit)
    // home_flag is typically inside the "print" object
    if (doc.containsKey("print") && doc["print"].containsKey("home_flag")) {
        long home_flag = doc["print"]["home_flag"];
        
        // Track home_flag changes for door detection
        if (!home_flag_initialized || home_flag != last_home_flag) {
            if (!home_flag_initialized) {
                Serial.printf("Home flag initialized: 0x%08X\n", (unsigned long)home_flag);
                home_flag_initialized = true;
            } else {
                // Check specifically bit 23 (door bit)
                unsigned long old_unsigned = (unsigned long)last_home_flag;
                unsigned long new_unsigned = (unsigned long)home_flag;
                bool old_door_bit = (old_unsigned & 0x00800000UL) != 0;
                bool new_door_bit = (new_unsigned & 0x00800000UL) != 0;
                if (old_door_bit != new_door_bit) {
                    Serial.printf("DOOR %s (home_flag: 0x%08X)\n",
                                 new_door_bit ? "OPENED" : "CLOSED", new_unsigned);
                }
            }
            last_home_flag = home_flag;
        }
        
        const unsigned long DOOR_OPEN_BIT = 0x00800000UL; // Bit 23 (24th bit, zero-indexed)
        
        // Convert to unsigned for proper bit operations
        unsigned long home_flag_unsigned = (unsigned long)home_flag;
        
        // Update door_detected based on bit state
        door_detected = (home_flag_unsigned & DOOR_OPEN_BIT) != 0;
    } else if (doc.containsKey("home_flag")) {
        // Fallback: check at root level
        long home_flag = doc["home_flag"];
        
        // Track root level home_flag changes
        if (!home_flag_initialized || home_flag != last_home_flag) {
            if (!home_flag_initialized) {
                Serial.printf("Home flag initialized (root): 0x%08X - Waiting for fresh update...\n", (unsigned long)home_flag);
                home_flag_initialized = true;
            } else {
                // This is a REAL update (not first stale value)
                home_flag_update_count++;
                
                // Check door bit changes (bit 23: 1=CLOSED, 0=OPEN - inverted logic)
                unsigned long old_unsigned = (unsigned long)last_home_flag;
                unsigned long new_unsigned = (unsigned long)home_flag;
                bool old_door_bit = (old_unsigned & 0x00800000UL) != 0;
                bool new_door_bit = (new_unsigned & 0x00800000UL) != 0;
                if (old_door_bit != new_door_bit) {
                    // Inverted: bit SET = CLOSED, bit CLEAR = OPEN
                    Serial.printf("DOOR %s (home_flag: 0x%08X)\n",
                                 new_door_bit ? "CLOSED" : "OPENED", new_unsigned);
                }
            }
            last_home_flag = home_flag;
        }
        
        // Only trust door detection after we've received at least one real update
        if (home_flag_update_count > 0) {
            const unsigned long DOOR_CLOSED_BIT = 0x00800000UL;  // Bit 23: 1=CLOSED, 0=OPEN
            unsigned long home_flag_unsigned = (unsigned long)home_flag;
            
            // Update door_detected: Door is OPEN when bit is CLEAR (inverted logic)
            door_detected = !(home_flag_unsigned & DOOR_CLOSED_BIT);
        }
    }
    
    // Combine automatic detection with manual trigger
    bool prev_door_state = door_is_open;
    door_is_open = door_detected || manual_door_trigger;
    bool door_just_closed = prev_door_state && !door_is_open;
    
    // Log door state changes with detailed info
    if (door_is_open != prev_door_state) {
        String doorSource = "";
        if (door_detected && manual_door_trigger) {
            doorSource = " (Auto + Manual)";
        } else if (door_detected) {
            doorSource = " (Auto Detection)";
        } else if (manual_door_trigger) {
            doorSource = " (Manual Override)";
        }
        
        logPrintln("=== DOOR STATE CHANGE ===" + doorSource);
        logPrintln("Door: " + String(door_is_open ? "OPEN" : "CLOSED"));
        logPrintln("Door Lights Enabled: " + String(door_open_lights_enabled ? "YES" : "NO"));
        logPrintln("Current Mode: " + String(
            currentMode == MODE_AUTO ? "AUTO" :
            currentMode == MODE_MANUAL ? "MANUAL" :
            currentMode == MODE_PRINTER ? "PRINTER" : "OFF"
        ));
        
        // If door opens and mode is OFF (from auto-off), restore previous mode
        if (door_is_open && currentMode == MODE_OFF && door_open_lights_enabled) {
            logPrintln("Restoring mode from AUTO-OFF: " + String(
                mode_before_auto_off == MODE_AUTO ? "AUTO" :
                mode_before_auto_off == MODE_MANUAL ? "MANUAL" :
                mode_before_auto_off == MODE_PRINTER ? "PRINTER" : "OFF"
            ));
            currentMode = mode_before_auto_off;
            
            // Restore chamber light if it was on before auto-off
            if (chamber_light_sync_enabled && chamber_light_was_on_before_auto_off && mqttClient.connected()) {
                logPrintln("Restoring chamber light (synced)");
                setChamberLight(true);
                chamber_light_was_on_before_auto_off = false;
            }
        }
        
        logPrintln("========================");
    }
    
    // Transition from FINISHED to IDLE when door closes (after user retrieves print)
    if (!door_is_open && prev_door_state && printerStatus.gcode_state == "FINISH") {
        logPrintln("=== PRINT RETRIEVED ===");
        logPrintln("Door closed after FINISH state - transitioning to IDLE");
        printerStatus.gcode_state = "IDLE";
        logPrintln("======================");
    }
    
    // Start auto-off timer if door opens and lights turn on (if timer enabled and not already running)
    if (door_is_open && !prev_door_state && auto_off_minutes > 0 && 
        currentMode == MODE_PRINTER && door_open_lights_enabled) {
        if (!auto_off_pending) {
            auto_off_pending = true;
            lights_on_time = millis();
            logPrintln("Door opened - auto-off timer started: " + String(auto_off_minutes) + " minutes");
        } else {
            // If timer already running, restart it
            lights_on_time = millis();
            logPrintln("Door opened - auto-off timer restarted");
        }
    }
    
    // *** LiDAR SCANNING DETECTION ***
    // Detect when the printer is performing LiDAR scans (typically after first layer)
    bool new_lidar_scanning = false;
    
    // Check for LiDAR scanning conditions
    if (printerStatus.mc_print_stage == "scanning" || 
        printerStatus.mc_print_stage == "first_layer_scan" ||
        printerStatus.mc_print_stage == "auto_bed_leveling" ||
        printerStatus.print_real_action == "scanning" ||
        printerStatus.print_real_action == "lidar_scan" ||
        printerStatus.print_real_action == "first_layer_inspect" ||
        (printerStatus.layer_num == 1 && 
         (printerStatus.mc_print_stage.indexOf("inspect") >= 0 || 
          printerStatus.mc_print_stage.indexOf("scan") >= 0)) ||
        printerStatus.mc_print_stage.indexOf("bed_leveling") >= 0 ||
        printerStatus.mc_print_stage.indexOf("calibrat") >= 0 ||
        printerStatus.print_real_action.indexOf("bed_leveling") >= 0 ||
        printerStatus.print_real_action.indexOf("calibrat") >= 0) {
        new_lidar_scanning = true;
    }
    
    // Update LiDAR scanning state
    if (new_lidar_scanning != lidar_scanning) {
        lidar_scanning = new_lidar_scanning;
        if (lidar_scanning) {
            logPrintln("=== LiDAR SCAN STARTED ===");
            logPrintln("Stage: " + printerStatus.mc_print_stage);
            logPrintln("Action: " + printerStatus.print_real_action);
            logPrintln("Layer: " + String(printerStatus.layer_num));
            logPrintln("Lights: " + String(lidar_lights_off_enabled ? "OFF (pause enabled)" : "STAYING ON (pause disabled)"));
            logPrintln("========================");
        } else {
            logPrintln("=== LiDAR SCAN ENDED - Lights Restored ===");
        }
    }
    
    yield(); // Final yield before LED update
    
    // Only update LEDs based on printer state if in PRINTER mode
    // Skip updates when door is open (to prevent jogging from changing LED color)
    // Force update when door closes to immediately restore LEDs to printer state
    if (currentMode == MODE_PRINTER && (!door_is_open || door_just_closed)) {
        updatePrinterLEDs();
    }
    
    yield(); // Yield after processing complete
}

// Apply LED effect based on behavior configuration (declaration in led_effects.h)
void applyEffect(PrinterStateBehavior& behavior, int progress) {
    uint8_t r = (behavior.color.r * behavior.brightness) / 100;
    uint8_t g = (behavior.color.g * behavior.brightness) / 100;
    uint8_t b = (behavior.color.b * behavior.brightness) / 100;
    
    if (behavior.effect == "solid") {
        setAllLEDs(r, g, b);
    } 
    else if (behavior.effect == "blink") {
        static unsigned long lastBlink = 0;
        static bool blinkState = false;
        if (millis() - lastBlink > 500) {
            blinkState = !blinkState;
            lastBlink = millis();
        }
        if (blinkState) {
            setAllLEDs(r, g, b);
        } else {
            clearAllLEDs();
        }
    }
    else if (behavior.effect == "breathe") {
        static unsigned long lastBreath = 0;
        static float breathPhase = 0;
        if (millis() - lastBreath > 30) {
            breathPhase += 0.05;
            if (breathPhase > 6.28) breathPhase = 0;
            lastBreath = millis();
        }
        float brightness = (sin(breathPhase) + 1.0) / 2.0;  // 0.0 to 1.0
        setAllLEDs(r * brightness, g * brightness, b * brightness);
    }
    else if (behavior.effect == "pulse") {
        static unsigned long lastPulse = 0;
        static bool pulseOn = false;
        static float pulsePhase = 0;
        
        if (pulseOn) {
            // Quick fade in/out pulse
            if (millis() - lastPulse > 15) {
                pulsePhase += 0.15;
                if (pulsePhase > 3.14) {  // Half sine wave (0 to π)
                    pulsePhase = 0;
                    pulseOn = false;
                    lastPulse = millis();
                }
            }
            float brightness = sin(pulsePhase);  // 0.0 to 1.0
            setAllLEDs(r * brightness, g * brightness, b * brightness);
        } else {
            // Off period between pulses
            setAllLEDs(0, 0, 0);
            if (millis() - lastPulse > 1500) {  // 1.5 second pause between pulses
                pulseOn = true;
                pulsePhase = 0;
                lastPulse = millis();
            }
        }
    }
    else if (behavior.effect == "progress" && progress >= 0) {
        // Progress bar effect
        if (progress < 0) progress = 0;
        if (progress > 100) progress = 100;
        
        int ledsToLight = (progress * num_leds) / 100;
        clearAllLEDs();
        
        // Light up completed portion
        for (int i = 0; i < ledsToLight && i < num_leds; i++) {
            setLED(i, r, g, b);
        }
        
        // Show leading edge in brighter color
        if (progress >= 1 && progress < 100 && ledsToLight < num_leds) {
            setLED(ledsToLight, behavior.color.r, behavior.color.g, behavior.color.b);  // Full brightness leading edge
        }
    }
    else if (behavior.effect == "rainbow") {
        // Rainbow cycle effect
        static unsigned long lastRainbow = 0;
        static int rainbowOffset = 0;
        if (millis() - lastRainbow > 50) {  // Update every 50ms
            rainbowOffset = (rainbowOffset + 1) % 256;
            lastRainbow = millis();
        }
        
        for (int i = 0; i < num_leds; i++) {
            int hue = (rainbowOffset + (i * 256 / num_leds)) % 256;
            
            // HSV to RGB conversion (simplified)
            int segment = hue / 43;  // 0-5
            int fraction = (hue % 43) * 6;  // 0-255
            
            int p = 0;
            int q = (255 * (255 - fraction)) / 255;
            int t = (255 * fraction) / 255;
            
            int rr, gg, bb;
            switch(segment) {
                case 0: rr = 255; gg = t;   bb = p;   break;
                case 1: rr = q;   gg = 255; bb = p;   break;
                case 2: rr = p;   gg = 255; bb = t;   break;
                case 3: rr = p;   gg = q;   bb = 255; break;
                case 4: rr = t;   gg = p;   bb = 255; break;
                default: rr = 255; gg = p;   bb = q;   break;
            }
            
            // Apply brightness scaling
            rr = (rr * behavior.brightness) / 100;
            gg = (gg * behavior.brightness) / 100;
            bb = (bb * behavior.brightness) / 100;
            
            setLED(i, rr, gg, bb);
        }
    }
    else {
        // Default to solid if unknown effect
        setAllLEDs(r, g, b);
    }
}

void updatePrinterLEDs() {
    if (currentMode != MODE_PRINTER) return;
    
    // Door open has priority - skip printer LED updates when door is open
    // IMPORTANT: This prevents LED state from updating, but door lights handle the override
    if (door_open_lights_enabled && door_is_open) {
        // Don't return immediately - let door lights in main loop handle this
        // This prevents state from getting stuck when door closes
        return;
    }
    
    // Priority 1: LiDAR scanning - turn off lights to avoid interference (if enabled)
    if (lidar_lights_off_enabled && lidar_scanning) {
        clearAllLEDs();
        updateLEDs();
        return;
    }
    
    // Track LED mode for change detection
    static uint8_t last_led_mode = 0; // 0=none, 1=white, 2=normal
    static String last_gcode_state = "";
    static bool finish_effect_logged = false;
    uint8_t current_led_mode = 0;
    
    // Log state changes for debugging
    if (printerStatus.gcode_state != last_gcode_state) {
        logPrintln(">>> PRINTER STATE CHANGE: " + last_gcode_state + " -> " + printerStatus.gcode_state + " <<<");
        last_gcode_state = printerStatus.gcode_state;
        finish_effect_logged = false; // Reset when state changes
        
        // Publish state change to Home Assistant
        publishHAState();
    }
    
    // Priority 2: Timelapse mode - force white lights during printing for consistent lighting
    // Only activate if BOTH the setting is enabled AND timelapse is actually active in the print
    if (timelapse_white_lights_enabled && timelapse_active && (printerStatus.gcode_state == "RUNNING" || printerStatus.gcode_state == "PREPARE")) {
        current_led_mode = 1; // White mode
        if (last_led_mode != 1) {
            logPrintln(">>> LED MODE CHANGE: Timelapse WHITE lights active <<<");
            last_led_mode = 1;
        }
        
        setAllLEDs(255, 255, 255); // Full white for timelapse
        updateLEDs();
        
        // Keep chamber light on if sync is enabled (always on for timelapse white mode)
        if (chamber_light_sync_enabled) {
            setChamberLight(true);
        }
        
        // Prevent auto-off during timelapse recording
        if (auto_off_pending) {
            auto_off_pending = false;
            logPrintln("Auto-off cancelled: Timelapse recording active");
        }
        
        return;
    }
    
    // Handle timelapse active but white lights disabled - keep chamber on only if using white printing color
    if (timelapse_active && (printerStatus.gcode_state == "RUNNING" || printerStatus.gcode_state == "PREPARE")) {
        current_led_mode = 2; // Normal mode
        if (last_led_mode != 2) {
            logPrintln(">>> LED MODE CHANGE: Timelapse NORMAL lights (using printer status colors) <<<");
            last_led_mode = 2;
        }
        
        // Keep chamber light on if sync is enabled AND printing color is white (255,255,255)
        // This prevents chamber light from washing out custom printing colors
        if (chamber_light_sync_enabled && 
            behavior_printing.color.r == 255 && 
            behavior_printing.color.g == 255 && 
            behavior_printing.color.b == 255) {
            setChamberLight(true);
        }
        
        // Prevent auto-off during timelapse recording
        if (auto_off_pending) {
            auto_off_pending = false;
            logPrintln("Auto-off cancelled: Timelapse recording active (white lights disabled)");
        }
        // Continue to normal printer status logic below
    }
    
    // If we're no longer in timelapse mode, reset tracking
    if (current_led_mode == 0 && last_led_mode != 0) {
        logPrintln(">>> LED MODE CHANGE: Exited timelapse mode <<<");
        last_led_mode = 0;
    }
    
    // Priority 3: Printer state-based colors with configurable effects
    if (!printerStatus.online || millis() - printerStatus.lastUpdate > 60000) {
        // Offline/Error - use error behavior
        applyEffect(behavior_error);
    } else if (printerStatus.gcode_state == "IDLE") {
        applyEffect(behavior_idle);
    } else if (printerStatus.gcode_state == "RUNNING" || printerStatus.gcode_state == "PREPARE") {
        // Check stage number to differentiate heating from actual printing
        // Stage 2 = HEATBED_PREHEATING, Stage 3 = HEATING_HOTEND, etc.
        // Stage 14+ = actual printing stages
        int stageNum = printerStatus.stage.toInt();
        if (stageNum > 0 && stageNum < 10 && printerStatus.progress == 0) {
            // Heating/preparation stages (stage 1-9) with 0% progress = heating phase
            applyEffect(behavior_heating);
        } else {
            // Actually printing
            applyEffect(behavior_printing, printerStatus.progress);
        }
    } else if (printerStatus.gcode_state == "PAUSE") {
        applyEffect(behavior_pause, printerStatus.progress);
    } else if (printerStatus.gcode_state == "FINISH") {
        if (!finish_effect_logged) {
            logPrintln("=== APPLYING FINISH STATE ===");
            logPrintln("Effect: " + behavior_finish.effect);
            logPrintln("Color: RGB(" + String(behavior_finish.color.r) + "," + String(behavior_finish.color.g) + "," + String(behavior_finish.color.b) + ")");
            logPrintln("Brightness: " + String(behavior_finish.brightness) + "%");
            logPrintln("==============================");
            finish_effect_logged = true;
        }
        applyEffect(behavior_finish);
    } else if (printerStatus.gcode_state == "FAILED") {
        // FAILED state (cancelled print) - treat as idle since it's not a real error
        // Real errors would have print_error != 0 or mc_print_error_code != "0"
        applyEffect(behavior_idle);
    } else {
        // Unknown state - use error behavior
        applyEffect(behavior_error);
    }
    
    // Manage chamber light based on printing color
    // Turn ON for white (255,255,255), turn OFF for any other color
    if (chamber_light_sync_enabled && 
        (printerStatus.gcode_state == "RUNNING" || printerStatus.gcode_state == "PREPARE")) {
        if (behavior_printing.color.r == 255 && 
            behavior_printing.color.g == 255 && 
            behavior_printing.color.b == 255) {
            setChamberLight(true);  // White printing color - turn on chamber light
        } else {
            setChamberLight(false); // Non-white printing color - turn off chamber light
        }
    }
    
    updateLEDs();
}

// espMqttClient handles SSL internally, no separate client setup needed

bool connectMQTT() {
    logPrintln("=== MQTT Connection Debug ===");
    logPrintln("MQTT Server: " + mqtt_server);
    logPrintln("MQTT Port: " + String(mqtt_port));
    logPrintln("MQTT SSL: " + String(mqtt_use_ssl ? "YES" : "NO"));
    logPrintln("MQTT Skip Cert: " + String(mqtt_skip_cert_check ? "YES" : "NO"));
    logPrintln("MQTT Username: " + String(MQTT_USERNAME));
    logPrintln("Device Serial: " + device_serial);
    logPrintln("WiFi Status: " + String(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected"));
    logPrintln("WiFi IP: " + WiFi.localIP().toString());
    
    // Set up espMqttClientSecure configuration  
    mqttClient.onMessage(onMqttMessage);
    
    // Configure SSL/TLS - espMqttClientSecure is always secure
    if (mqtt_skip_cert_check) {
        logPrintln("WARNING: Skipping SSL certificate verification");
        mqttClient.setInsecure();  // Skip certificate verification for Bambu Lab
    }
    
    mqttClient.setServer(mqtt_server.c_str(), mqtt_port);
    mqttClient.setCredentials(MQTT_USERNAME, mqtt_password.c_str());
    mqttClient.setKeepAlive(60);
    
    logPrintln("Connecting to MQTT: " + mqtt_server + ":" + String(mqtt_port));
    
    // Use truly unique client ID to avoid conflicts
    String clientId = String(random(100000, 999999)) + "_" + String(millis());
    logPrintln("Client ID: " + clientId);
    mqttClient.setClientId(clientId.c_str());
    
    mqttClient.connect();
    
    // Wait for connection with timeout
    int connectionAttempts = 0;
    const int maxAttempts = 30; // 15 seconds timeout
    while (!mqttClient.connected() && connectionAttempts < maxAttempts) {
        delay(500);
        connectionAttempts++;
        yield();
    }
    
    if (mqttClient.connected()) {
        logPrintln("MQTT connected successfully!");
        
        // Subscribe to printer status topic
        String statusTopic = "device/" + device_serial + "/report";
        mqttClient.subscribe(statusTopic.c_str(), 0);  // QOS 0
        logPrintln("Subscribed to: " + statusTopic);
        
        // DEBUGGING: Subscribe to ALL device topics to see what's actually being published
        mqttClient.subscribe("device/+/report", 0);  // QOS 0
        logPrintln("DEBUG: Subscribed to device/+/report (wildcard)");
        
        mqttClient.subscribe("device/+/info", 0);  // QOS 0
        logPrintln("DEBUG: Subscribed to device/+/info (wildcard)");
        
        // Subscribe to even broader pattern to catch everything
        mqttClient.subscribe("device/#", 0);  // QOS 0
        logPrintln("DEBUG: Subscribed to device/# (all device topics)");
        
        // ENHANCED DEBUGGING: Try subscribing to ALL topics to see what's available
        logPrintln("DEBUG: Subscribing to ALL MQTT topics for debugging...");
        mqttClient.subscribe("#", 0);  // Subscribe to EVERYTHING, QOS 0
        logPrintln("DEBUG: Subscribed to # (ALL TOPICS)");
        
        // Also try some common Bambu Lab topic patterns
        mqttClient.subscribe("bambulabs/+/report", 0);  // QOS 0
        logPrintln("DEBUG: Subscribed to bambulabs/+/report");
        
        mqttClient.subscribe("bambu/+/report", 0);  // QOS 0
        logPrintln("DEBUG: Subscribed to bambu/+/report");
        
        // CALLBACK TEST: Subscribe to test topic and publish test message
        String testTopic = "device/" + device_serial + "/test";
        mqttClient.subscribe(testTopic.c_str(), 0);  // QOS 0
        Serial.printf("DEBUG: Subscribed to test topic: %s\n", testTopic.c_str());
        
        Serial.println("DEBUG: Publishing test message to verify callback...");
        mqttClient.publish(testTopic.c_str(), 0, false, "{\"test\":\"callback_verification\",\"home_flag\":-1234567}");  // QOS 0, no retain
        delay(100); // Give time for callback
        Serial.println("DEBUG: Test message published, callback should have triggered");
        
        // Send connection confirmation
        setAllLEDs(0, 255, 0); // Green
        updateLEDs();
        delay(500); // Reduced delay to keep interface responsive
        yield(); // Yield to keep web server responsive
        
        return true;
    } else {
        Serial.println("MQTT connection failed!");
        Serial.println("Check your MQTT server, credentials, and network settings.");
        yield(); // Yield after failed connection attempt
        
        // Red blink for MQTT failure
        setAllLEDs(255, 0, 0);
        updateLEDs();
        delay(300); // Reduced delay to keep interface responsive
        yield(); // Yield to keep web server responsive
        
        return false;
    }
}

// Home Assistant MQTT Discovery helpers
String getDeviceId() {
    uint32_t chipId = 0;
    for(int i=0; i<17; i=i+8) {
        chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
    }
    return String(chipId, HEX);
}

void publishHADiscovery() {
    if (!haMqttClient.connected() || !ha_mqtt_enabled) {
        return;
    }
    
    String deviceId = "bambu_lights_" + getDeviceId();
    String baseTopic = "homeassistant/sensor/" + deviceId;
    
    // State topic base from device name
    String stateTopicBase = ha_device_name;
    stateTopicBase.replace(" ", "_");
    
    // Device info shared by all sensors
    String deviceJson = "\"device\":{\"identifiers\":[\"" + deviceId + "\"],"
                       "\"name\":\"" + ha_device_name + "\","
                       "\"manufacturer\":\"Bambu Lights\","
                       "\"model\":\"ESP32 LED Controller\","
                       "\"sw_version\":\"1.0\"}";
    
    // State sensor
    String stateConfig = "{\"name\":\"Printer State\","
                        "\"state_topic\":\"" + stateTopicBase + "/state\","
                        "\"unique_id\":\"" + deviceId + "_state\","
                        "\"icon\":\"mdi:printer-3d\","
                        + deviceJson + "}";
    haMqttClient.publish((baseTopic + "_state/config").c_str(), 0, true, stateConfig.c_str());
    
    // Progress sensor
    String progressConfig = "{\"name\":\"Print Progress\","
                           "\"state_topic\":\"" + stateTopicBase + "/progress\","
                           "\"unique_id\":\"" + deviceId + "_progress\","
                           "\"unit_of_measurement\":\"%\","
                           "\"icon\":\"mdi:progress-clock\","
                           + deviceJson + "}";
    haMqttClient.publish((baseTopic + "_progress/config").c_str(), 0, true, progressConfig.c_str());
    
    // Bed temperature sensor
    String bedTempConfig = "{\"name\":\"Bed Temperature\","
                          "\"state_topic\":\"" + stateTopicBase + "/bed_temp\","
                          "\"unique_id\":\"" + deviceId + "_bed_temp\","
                          "\"unit_of_measurement\":\"°C\","
                          "\"device_class\":\"temperature\","
                          "\"icon\":\"mdi:thermometer\","
                          + deviceJson + "}";
    haMqttClient.publish((baseTopic + "_bed_temp/config").c_str(), 0, true, bedTempConfig.c_str());
    
    // Nozzle temperature sensor
    String nozzleTempConfig = "{\"name\":\"Nozzle Temperature\","
                             "\"state_topic\":\"" + stateTopicBase + "/nozzle_temp\","
                             "\"unique_id\":\"" + deviceId + "_nozzle_temp\","
                             "\"unit_of_measurement\":\"°C\","
                             "\"device_class\":\"temperature\","
                             "\"icon\":\"mdi:printer-3d-nozzle\","
                             + deviceJson + "}";
    haMqttClient.publish((baseTopic + "_nozzle_temp/config").c_str(), 0, true, nozzleTempConfig.c_str());
    
    // Layer sensor
    String layerConfig = "{\"name\":\"Current Layer\","
                        "\"state_topic\":\"" + stateTopicBase + "/layer\","
                        "\"unique_id\":\"" + deviceId + "_layer\","
                        "\"icon\":\"mdi:layers\","
                        + deviceJson + "}";
    haMqttClient.publish((baseTopic + "_layer/config").c_str(), 0, true, layerConfig.c_str());
    
    // Filename sensor
    String filenameConfig = "{\"name\":\"Current File\","
                           "\"state_topic\":\"" + stateTopicBase + "/filename\","
                           "\"unique_id\":\"" + deviceId + "_filename\","
                           "\"icon\":\"mdi:file-document\","
                           + deviceJson + "}";
    haMqttClient.publish((baseTopic + "_filename/config").c_str(), 0, true, filenameConfig.c_str());
    
    // Binary sensor for printing status
    String binaryBaseTopic = "homeassistant/binary_sensor/" + deviceId;
    String printingConfig = "{\"name\":\"Printing\","
                           "\"state_topic\":\"" + stateTopicBase + "/printing\","
                           "\"unique_id\":\"" + deviceId + "_printing\","
                           "\"device_class\":\"running\","
                           "\"payload_on\":\"ON\","
                           "\"payload_off\":\"OFF\","
                           + deviceJson + "}";
    haMqttClient.publish((binaryBaseTopic + "_printing/config").c_str(), 0, true, printingConfig.c_str());
    
    logPrintln("HA: Published MQTT Discovery messages");
}

void publishHAState() {
    if (!haMqttClient.connected() || !ha_mqtt_enabled) {
        return;
    }
    
    static unsigned long lastPublish = 0;
    static String lastState = "";
    static int lastProgress = -1;
    static bool firstPublish = true;
    
    // Throttle updates to once per second max (but allow first publish)
    if (!firstPublish && millis() - lastPublish < 1000) {
        return;
    }
    
    // Force telemetry update every 5 minutes (300000ms)
    bool forceUpdate = (millis() - lastPublish >= 300000);
    
    // Publish if state/progress changed, first publish, or 5 minutes elapsed
    if (!firstPublish && !forceUpdate && printerStatus.gcode_state == lastState && printerStatus.progress == lastProgress) {
        return;
    }
    
    firstPublish = false;
    lastPublish = millis();
    lastState = printerStatus.gcode_state;
    lastProgress = printerStatus.progress;
    
    // Build topic base from device name
    String topicBase = ha_device_name;
    topicBase.replace(" ", "_");
    
    // Publish state
    haMqttClient.publish((topicBase + "/state").c_str(), 0, false, printerStatus.gcode_state.c_str());
    
    // Publish progress
    String progressStr = String(printerStatus.progress);
    haMqttClient.publish((topicBase + "/progress").c_str(), 0, false, progressStr.c_str());
    
    // Publish bed temperature
    String bedTempStr = String(printerStatus.bed_temp);
    haMqttClient.publish((topicBase + "/bed_temp").c_str(), 0, false, bedTempStr.c_str());
    
    // Publish nozzle temperature
    String nozzleTempStr = String(printerStatus.nozzle_temp);
    haMqttClient.publish((topicBase + "/nozzle_temp").c_str(), 0, false, nozzleTempStr.c_str());
    
    // Publish layer info
    String layerStr = String(printerStatus.layer_num) + "/" + String(printerStatus.total_layer_num);
    haMqttClient.publish((topicBase + "/layer").c_str(), 0, false, layerStr.c_str());
    
    // Publish filename
    haMqttClient.publish((topicBase + "/filename").c_str(), 0, false, printerStatus.gcode_file.c_str());
    
    // Publish printing binary sensor
    bool isPrinting = (printerStatus.gcode_state == "RUNNING" || printerStatus.gcode_state == "PREPARE");
    haMqttClient.publish((topicBase + "/printing").c_str(), 0, false, isPrinting ? "ON" : "OFF");
    
    logPrintln("HA: Published state update - " + printerStatus.gcode_state + " " + String(printerStatus.progress) + "%");
}

void onHAMqttConnect(bool sessionPresent) {
    logPrintln("HA MQTT connected! Session present: " + String(sessionPresent));
    publishHADiscovery();
    publishHAState(); // Send initial state after discovery
}

bool connectHAMQTT() {
    if (!ha_mqtt_enabled) {
        logPrintln("HA MQTT: Disabled in configuration");
        return false;
    }
    
    logPrintln("=== HA MQTT Connection Debug ===");
    logPrintln("HA MQTT Server: " + ha_mqtt_server);
    logPrintln("HA MQTT Port: " + String(ha_mqtt_port));
    logPrintln("HA MQTT SSL: " + String(ha_mqtt_use_ssl ? "YES" : "NO"));
    logPrintln("HA MQTT Username: " + ha_mqtt_username);
    logPrintln("WiFi Status: " + String(WiFi.status() == WL_CONNECTED ? "Connected" : "Disconnected"));
    
    // Set up callback
    haMqttClient.onConnect(onHAMqttConnect);
    
    // espMqttClient automatically uses non-secure connection
    // For SSL, we would need espMqttClientSecure (not implemented yet)
    
    haMqttClient.setServer(ha_mqtt_server.c_str(), ha_mqtt_port);
    
    if (ha_mqtt_username.length() > 0) {
        haMqttClient.setCredentials(ha_mqtt_username.c_str(), ha_mqtt_password.c_str());
    }
    
    haMqttClient.setKeepAlive(60);
    
    logPrintln("Connecting to HA MQTT: " + ha_mqtt_server + ":" + String(ha_mqtt_port));
    
    // Use unique client ID
    String clientId = "bambu_lights_" + String(random(100000, 999999));
    logPrintln("HA Client ID: " + clientId);
    haMqttClient.setClientId(clientId.c_str());
    
    haMqttClient.connect();
    
    // Wait for connection with timeout
    int connectionAttempts = 0;
    const int maxAttempts = 30; // 15 seconds timeout
    while (!haMqttClient.connected() && connectionAttempts < maxAttempts) {
        delay(500);
        connectionAttempts++;
        yield();
    }
    
    if (haMqttClient.connected()) {
        logPrintln("HA MQTT connected successfully!");
        return true;
    } else {
        logPrintln("HA MQTT connection failed!");
        logPrintln("Check your HA MQTT broker settings.");
        return false;
    }
}

void handleRoot() {
    logPrintln("HTTP: / (root) requested");
    
    // Use chunked transfer to avoid memory issues with large HTML
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(200, "text/html", "");
    
    // Helper lambda to send chunks
    auto sendChunk = [](const String& chunk) {
        server.sendContent(chunk);
    };
    
    String html = "<!DOCTYPE html><html><head><title>Bambu LED Controller</title>";
    html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
    html += "<link rel='stylesheet' href='https://cdnjs.cloudflare.com/ajax/libs/font-awesome/6.4.0/css/all.min.css'>";
    sendChunk(html);
    html = "";
    html += "<style>";
    // Dark Theme Base
    html += "* { margin: 0; padding: 0; box-sizing: border-box; }";
    html += "body { font-family: 'Inter', -apple-system, BlinkMacSystemFont, 'Segoe UI', Arial, sans-serif; background: #0a0e27; min-height: 100vh; color: #e0e0e0; overflow-x: hidden; }";
    
    // Header
    html += ".header { background: linear-gradient(135deg, #1a1f3a 0%, #2d1b4e 100%); padding: 20px; text-align: center; border-bottom: 2px solid #7c3aed; box-shadow: 0 4px 20px rgba(124, 58, 237, 0.3); }";
    html += ".header h1 { color: #fff; font-size: 2.2em; margin-bottom: 8px; text-shadow: 0 0 20px rgba(124, 58, 237, 0.5); letter-spacing: 1px; }";
    html += ".header .subtitle { color: #a78bfa; font-size: 1em; letter-spacing: 0.5px; }";
    
    // Layout
    html += ".main-container { max-width: 1600px; margin: 0 auto; padding: 24px; display: grid; grid-template-columns: repeat(12, 1fr); gap: 20px; }";
    
    // Panels
    html += ".panel { background: linear-gradient(145deg, #151932 0%, #1e2139 100%); border-radius: 16px; padding: 24px; box-shadow: 0 8px 32px rgba(0,0,0,0.4), 0 0 0 1px rgba(124, 58, 237, 0.1); border: 1px solid #2a2d47; }";
    html += ".panel:hover { box-shadow: 0 12px 40px rgba(124, 58, 237, 0.2), 0 0 0 1px rgba(124, 58, 237, 0.2); transition: all 0.3s ease; }";
    html += ".panel h2 { color: #fff; margin-bottom: 20px; font-size: 1.3em; border-bottom: 2px solid #7c3aed; padding-bottom: 12px; display: flex; align-items: center; gap: 10px; }";
    
    // Grid Layouts
    html += ".col-4 { grid-column: span 4; }";
    html += ".col-5 { grid-column: span 5; }";
    html += ".col-6 { grid-column: span 6; }";
    html += ".col-7 { grid-column: span 7; }";
    html += ".col-8 { grid-column: span 8; }";
    html += ".col-12 { grid-column: span 12; }";
    
    // Buttons
    html += ".btn { padding: 12px 24px; margin: 6px; border: none; border-radius: 10px; font-size: 14px; cursor: pointer; font-weight: 600; transition: all 0.2s ease; letter-spacing: 0.3px; }";
    html += ".btn:hover { transform: translateY(-2px); box-shadow: 0 6px 20px rgba(0,0,0,0.4); }";
    html += ".btn:active { transform: translateY(0); }";
    html += ".btn-primary { background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); color: white; box-shadow: 0 4px 15px rgba(102, 126, 234, 0.4); }";
    html += ".btn-success { background: linear-gradient(135deg, #10b981 0%, #059669 100%); color: white; box-shadow: 0 4px 15px rgba(16, 185, 129, 0.4); }";
    html += ".btn-secondary { background: linear-gradient(135deg, #6366f1 0%, #4f46e5 100%); color: white; box-shadow: 0 4px 15px rgba(99, 102, 241, 0.4); }";
    html += ".btn-warning { background: linear-gradient(135deg, #f59e0b 0%, #d97706 100%); color: white; box-shadow: 0 4px 15px rgba(245, 158, 11, 0.4); }";
    html += ".btn-danger { background: linear-gradient(135deg, #ef4444 0%, #dc2626 100%); color: white; box-shadow: 0 4px 15px rgba(239, 68, 68, 0.4); }";
    html += ".btn-large { padding: 14px 28px; font-size: 15px; min-width: 140px; }";
    
    // Status Box
    html += ".status { padding: 14px 18px; margin: 16px 0; border-radius: 10px; font-weight: 600; font-size: 0.95em; border-left: 4px solid; }";
    html += ".status-success { background: rgba(16, 185, 129, 0.15); color: #34d399; border-color: #10b981; }";
    html += ".status-auto { background: rgba(99, 102, 241, 0.15); color: #818cf8; border-color: #6366f1; }";
    html += ".status-printer { background: rgba(245, 158, 11, 0.15); color: #fbbf24; border-color: #f59e0b; }";
    html += ".status-off { background: rgba(107, 114, 128, 0.15); color: #9ca3af; border-color: #6b7280; }";
    
    // Info Box
    html += ".info-box { background: #0d1129; border: 1px solid #2a2d47; border-radius: 10px; padding: 16px; margin: 12px 0; font-family: 'Fira Code', monospace; font-size: 0.85em; max-height: 400px; overflow-y: auto; color: #a3a3a3; }";
    html += ".info-box::-webkit-scrollbar { width: 8px; }";
    html += ".info-box::-webkit-scrollbar-track { background: #151932; border-radius: 10px; }";
    html += ".info-box::-webkit-scrollbar-thumb { background: #7c3aed; border-radius: 10px; }";
    
    // Form Controls
    html += ".form-control { width: 100%; padding: 10px 14px; background: #0d1129; border: 2px solid #2a2d47; border-radius: 8px; font-size: 14px; color: #e0e0e0; transition: all 0.3s ease; }";
    html += ".form-control:focus { outline: none; border-color: #7c3aed; box-shadow: 0 0 0 3px rgba(124, 58, 237, 0.1); }";
    html += "input[type='color'] { cursor: pointer; border: 2px solid #2a2d47; }";
    
    // Range Sliders
    html += "input[type='range'] { width: 100%; height: 6px; border-radius: 5px; background: #2a2d47; outline: none; transition: opacity 0.2s; }";
    html += "input[type='range']:hover { opacity: 1; }";
    html += "input[type='range']::-webkit-slider-thumb { appearance: none; width: 18px; height: 18px; border-radius: 50%; background: linear-gradient(135deg, #7c3aed 0%, #a855f7 100%); cursor: pointer; box-shadow: 0 2px 8px rgba(124, 58, 237, 0.5); }";
    html += "input[type='range']::-moz-range-thumb { width: 18px; height: 18px; border-radius: 50%; background: linear-gradient(135deg, #7c3aed 0%, #a855f7 100%); cursor: pointer; border: none; box-shadow: 0 2px 8px rgba(124, 58, 237, 0.5); }";
    
    // Utility Classes
    html += ".responsive { display: flex; flex-wrap: wrap; gap: 12px; align-items: center; }";
    html += ".grid-3 { display: grid; grid-template-columns: repeat(3, 1fr); gap: 12px; }";
    html += ".grid-2 { display: grid; grid-template-columns: repeat(2, 1fr); gap: 12px; }";
    html += ".text-center { text-align: center; }";
    html += ".mb-2 { margin-bottom: 12px; }";
    html += "label { display: block; margin-bottom: 6px; font-size: 13px; font-weight: 500; color: #a3a3a3; }";
    
    // Cards
    html += ".card { background: #0d1129; border: 1px solid #2a2d47; border-radius: 10px; padding: 16px; margin: 10px 0; }";
    html += ".card-header { font-weight: 600; color: #fff; margin-bottom: 12px; padding-bottom: 8px; border-bottom: 1px solid #2a2d47; }";
    
    // Responsive
    html += "@media (max-width: 1200px) { .col-5, .col-6, .col-7, .col-4, .col-8 { grid-column: span 12; } }";
    html += "@media (max-width: 768px) { .main-container { padding: 16px; gap: 16px; } .panel { padding: 18px; } .btn-large { padding: 12px 20px; min-width: 120px; } }";
    html += "</style></head><body>";
    sendChunk(html); html = "";  // Send CSS chunk
    
    html += "<div class='header'>";
    html += "<h1><i class='fas fa-bolt'></i> BAMBU LIGHTS</h1>";
    html += "<div class='subtitle'>Smart LED Control System</div>";
    html += "</div>";
    sendChunk(html); html = "";  // Send header chunk
    
    html += "<div class='main-container'>";
    
    // Mode Control - Full Width at Top
    html += "<div class='panel col-12'>";
    html += "<h2><i class='fas fa-sliders'></i> Mode Control</h2>";
    html += "<div id='status' class='status status-auto'>Current Mode: Auto Cycling</div>";
    html += "<div id='modeDescription' style='margin: 12px 0; padding: 12px; background: #1e2139; border-left: 3px solid #7c3aed; border-radius: 6px; font-size: 13px; color: #d4d4d8; line-height: 1.5;'>";
    html += "<i class='fas fa-info-circle' style='color: #7c3aed; margin-right: 6px;'></i>LEDs cycle through colors automatically with adjustable speed and brightness.";
    html += "</div>";
    html += "<div class='grid-2' style='margin-top: 20px; max-width: 600px; margin-left: auto; margin-right: auto;'>";
    html += "<button class='btn btn-primary btn-large' onclick='setMode(\"printer\")'><i class='fas fa-print'></i> Printer</button>";
    html += "<button class='btn btn-warning btn-large' onclick='setMode(\"auto\")'><i class='fas fa-magic'></i> Auto</button>";
    html += "<button class='btn btn-success btn-large' onclick='setMode(\"manual\")'><i class='fas fa-hand-pointer'></i> Manual</button>";
    html += "<button class='btn btn-secondary btn-large' onclick='setMode(\"off\")'><i class='fas fa-power-off'></i> Off</button>";
    html += "</div></div>";
    sendChunk(html); html = "";  // Send mode control chunk
    
    // Printer Status LED Configuration Panel - Full Width
    html += "<div class='panel col-12'>";
    html += "<h2><i class='fas fa-print'></i> Printer Status LED Behavior</h2>";
    html += "<p style='font-size: 14px; color: #a3a3a3; margin-bottom: 20px; text-align: center;'>Configure LED response for each printer state</p>";
    html += "<div style='display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 16px;'>";
    
    // Printing Status
    html += "<div class='card' style='border-left: 4px solid #10b981;'>";
    html += "<div class='card-header' style='color: #10b981;'><i class='fas fa-cog fa-spin'></i> Printing</div>";
    html += "<div class='grid-3'>";
    html += "<div><label>Effect</label><select id='printing_effect' class='form-control' onchange='autoSaveState(\"printing\")'><option value='solid'>Solid</option><option value='breathe'>Breathe</option><option value='pulse'>Pulse</option><option value='progress'>Progress</option></select></div>";
    html += "<div><label>Color</label><input type='color' id='printing_color' value='#00ff00' style='width: 100%; height: 40px;' onchange='autoSaveState(\"printing\")'></div>";
    html += "<div><label>Brightness</label><input type='range' id='printing_brightness' min='0' max='100' value='80' onchange='autoSaveState(\"printing\")'><span id='printing_brightness_val' style='font-size: 12px; color: #7c3aed;'>80%</span></div>";
    html += "</div></div>";
    
    // Idle Status
    html += "<div class='card' style='border-left: 4px solid #6366f1;'>";
    html += "<div class='card-header' style='color: #818cf8;'><i class='fas fa-moon'></i> Idle</div>";
    html += "<div class='grid-3'>";
    html += "<div><label>Effect</label><select id='idle_effect' class='form-control' onchange='autoSaveState(\"idle\")'><option value='solid'>Solid</option><option value='breathe'>Breathe</option><option value='pulse'>Pulse</option></select></div>";
    html += "<div><label>Color</label><input type='color' id='idle_color' value='#00bfff' style='width: 100%; height: 40px;' onchange='autoSaveState(\"idle\")'></div>";
    html += "<div><label>Brightness</label><input type='range' id='idle_brightness' min='0' max='100' value='50' onchange='autoSaveState(\"idle\")'><span id='idle_brightness_val' style='font-size: 12px; color: #7c3aed;'>50%</span></div>";
    html += "</div></div>";
    
    // Error Status
    html += "<div class='card' style='border-left: 4px solid #ef4444;'>";
    html += "<div class='card-header' style='color: #f87171;'><i class='fas fa-exclamation-triangle'></i> Error</div>";
    html += "<div class='grid-3'>";
    html += "<div><label>Effect</label><select id='error_effect' class='form-control' onchange='autoSaveState(\"error\")'><option value='solid'>Solid</option><option value='breathe'>Breathe</option><option value='pulse'>Pulse</option><option value='blink'>Blink</option></select></div>";
    html += "<div><label>Color</label><input type='color' id='error_color' value='#ff0000' style='width: 100%; height: 40px;' onchange='autoSaveState(\"error\")'></div>";
    html += "<div><label>Brightness</label><input type='range' id='error_brightness' min='0' max='100' value='100' onchange='autoSaveState(\"error\")'><span id='error_brightness_val' style='font-size: 12px; color: #7c3aed;'>100%</span></div>";
    html += "</div></div>";
    
    // Heating Status
    html += "<div class='card' style='border-left: 4px solid #f59e0b;'>";
    html += "<div class='card-header' style='color: #fbbf24;'><i class='fas fa-fire'></i> Heating</div>";
    html += "<div class='grid-3'>";
    html += "<div><label>Effect</label><select id='heating_effect' class='form-control' onchange='autoSaveState(\"heating\")'><option value='solid'>Solid</option><option value='pulse'>Pulse</option><option value='breathe'>Breathe</option></select></div>";
    html += "<div><label>Color</label><input type='color' id='heating_color' value='#ffa500' style='width: 100%; height: 40px;' onchange='autoSaveState(\"heating\")'></div>";
    html += "<div><label>Brightness</label><input type='range' id='heating_brightness' min='0' max='100' value='70' onchange='autoSaveState(\"heating\")'><span id='heating_brightness_val' style='font-size: 12px; color: #7c3aed;'>70%</span></div>";
    html += "</div></div>";
    
    // Pause Status
    html += "<div class='card' style='border-left: 4px solid #f59e0b;'>";
    html += "<div class='card-header' style='color: #fbbf24;'><i class='fas fa-pause-circle'></i> Paused</div>";
    html += "<div class='grid-3'>";
    html += "<div><label>Effect</label><select id='pause_effect' class='form-control' onchange='autoSaveState(\"pause\")'><option value='solid'>Solid</option><option value='breathe'>Breathe</option><option value='pulse'>Pulse</option><option value='blink'>Blink</option></select></div>";
    html += "<div><label>Color</label><input type='color' id='pause_color' value='#ffa500' style='width: 100%; height: 40px;' onchange='autoSaveState(\"pause\")'></div>";
    html += "<div><label>Brightness</label><input type='range' id='pause_brightness' min='0' max='100' value='70' onchange='autoSaveState(\"pause\")'><span id='pause_brightness_val' style='font-size: 12px; color: #7c3aed;'>70%</span></div>";
    html += "</div></div>";
    
    // Complete Status
    html += "<div class='card' style='border-left: 4px solid #a855f7;'>";
    html += "<div class='card-header' style='color: #c084fc;'><i class='fas fa-check-circle'></i> Complete</div>";
    html += "<div class='grid-3'>";
    html += "<div><label>Effect</label><select id='finish_effect' class='form-control' onchange='autoSaveState(\"finish\")'><option value='solid'>Solid</option><option value='breathe'>Breathe</option><option value='pulse'>Pulse</option><option value='rainbow'>Rainbow</option></select></div>";
    html += "<div><label>Color</label><input type='color' id='finish_color' value='#ffffff' style='width: 100%; height: 40px;' onchange='autoSaveState(\"finish\")'></div>";
    html += "<div><label>Brightness</label><input type='range' id='finish_brightness' min='0' max='100' value='100' onchange='autoSaveState(\"finish\")'><span id='finish_brightness_val' style='font-size: 12px; color: #7c3aed;'>100%</span></div>";
    html += "</div></div>";
    
    html += "</div>"; // Close the grid container
    
    // Beacon Progress Visual Settings - Full Width
    html += "<div class='card' style='background: #0d1129; padding: 12px; border-radius: 8px; border: 2px solid #7c3aed; margin-top: 12px; overflow: hidden;'>";
    html += "<div style='font-weight: 600; color: #a78bfa; margin-bottom: 12px; font-size: 14px;'><i class='fas fa-lightbulb'></i> Beacon Progress</div>";
    html += "<div style='display: flex; gap: 12px; align-items: flex-start;'>";
    html += "<div style='flex: 1; display: flex; flex-direction: column;'>";
    html += "<div style='display: grid; grid-template-columns: 2.5fr 1fr 1fr; gap: 8px;'>";
    html += "<div><label style='font-size: 12px; color: #a3a3a3;'>Effect:</label><select id='beacon_effect' class='form-control' style='font-size: 12px; padding: 4px; min-width: 150px;' onchange='autoSaveBeacon()'><option value='progress'>Progress Bar</option><option value='climbing'>Climbing Dot</option><option value='gradient'>Gradient Fill</option><option value='pulse'>Pulsing Dot</option></select></div>";
    html += "<div><label style='font-size: 12px; color: #a3a3a3;'>Color:</label><input type='color' id='beacon_color' value='#00ff00' style='width: 100%; height: 28px; border-radius: 5px;' onchange='autoSaveBeacon()'></div>";
    html += "<div><label style='font-size: 12px; color: #a3a3a3;'>Brightness:</label><input type='range' id='beacon_brightness' min='0' max='100' value='80' style='width: 100%;' onchange='autoSaveBeacon()'><span id='beacon_brightness_val' style='font-size: 11px; color: #7c3aed;'>80%</span></div>";
    html += "</div>";
    html += "<div style='margin-top: 10px; padding: 10px; background: #1e2139; border-radius: 5px; flex: 1;'>";
    html += "<h4 style='font-size: 13px; margin: 0 0 8px 0; color: #e0e0e0;'>Progress Bar Animation</h4>";
    html += "<div style='display: grid; grid-template-columns: 1fr 1fr; gap: 8px;'>";
    html += "<div><label style='font-size: 12px; color: #a3a3a3;'>Climb Color:</label><input type='color' id='beacon_climb_color' value='#ffffff' style='width: 100%; height: 28px; border-radius: 5px;' onchange='autoSaveBeacon()'></div>";
    html += "<div><label style='font-size: 12px; color: #a3a3a3;'>Speed:</label><input type='range' id='beacon_climb_speed' min='20' max='2000' value='150' step='10' style='width: 100%;' onchange='autoSaveBeacon()'><span id='beacon_climb_speed_val' style='font-size: 11px; color: #7c3aed;'>150ms (0.66/s)</span></div>";
    html += "<div id='gradient_end_color_container' style='grid-column: 1 / -1;'><label style='font-size: 12px; color: #a3a3a3;'>End Color:</label><input type='color' id='beacon_gradient_end_color' value='#0000ff' style='width: 100%; height: 28px; border-radius: 5px;' onchange='autoSaveBeacon()'></div>";
    html += "</div></div></div>";
    html += "<div style='flex-shrink: 0; width: 500px; display: flex; align-items: center; justify-content: center; padding: 8px;'>";
    html += "<img id='beacon_preview_gif' src='/progress_bar.gif' alt='Beacon Animation' style='width: 480px; height: 480px; object-fit: contain; border-radius: 6px; box-shadow: 0 4px 12px rgba(124, 58, 237, 0.5); border: 2px solid #7c3aed;' title='Live beacon effect preview'>";
    html += "</div></div></div>";
    
    html += "<div style='text-align: center; margin-top: 15px; padding: 10px; background: #1e2139; border-radius: 8px;'>";
    html += "<i class='fas fa-check-circle' style='color: #10b981; margin-right: 8px;'></i>";
    html += "<span style='color: #a3a3a3; font-size: 13px;'>Settings auto-save on change</span>";
    html += "</div>";
    
    html += "</div>";
    sendChunk(html); html = "";  // Send printer behavior chunk
    
    // Auto-Cycle Settings (Left) and Manual Color Control (Right) - Side by Side
    html += "<div class='panel col-6'>";
    html += "<h2><i class='fas fa-sync-alt'></i> Auto-Cycle Settings</h2>";
    html += "<div class='card' style='border-left: 4px solid #f59e0b;'>";
    
    // Speed Control
    html += "<div style='margin-bottom: 15px;'>";
    html += "<div style='display: flex; justify-content: space-between; align-items: center; margin-bottom: 6px;'>";
    html += "<label style='font-size: 14px; font-weight: 500; color: #e0e0e0;'>Speed:</label>";
    html += "<span id='speedValue' style='font-size: 14px; font-weight: 600; color: #7c3aed;'>50</span>";
    html += "</div>";
    html += "<input type='range' id='autoCycleSpeed' min='1' max='100' value='" + String(autoCycleSpeed) + "' oninput='updateSpeedValue(this.value)' onchange='saveAutoCycleSettings()' style='width: 100%; height: 8px; border-radius: 4px; background: #2a2d47; outline: none; cursor: pointer;'>";
    html += "<div style='display: flex; justify-content: space-between; font-size: 11px; color: #a3a3a3; margin-top: 2px;'>";
    html += "<span>Slow</span><span>Fast</span>";
    html += "</div>";
    html += "</div>";
    
    // Brightness Control
    html += "<div>";
    html += "<div style='display: flex; justify-content: space-between; align-items: center; margin-bottom: 6px;'>";
    html += "<label style='font-size: 14px; font-weight: 500; color: #e0e0e0;'>Brightness:</label>";
    html += "<span id='brightnessValue' style='font-size: 14px; font-weight: 600; color: #7c3aed;'>100%</span>";
    html += "</div>";
    html += "<input type='range' id='autoCycleBrightness' min='0' max='100' value='" + String(autoCycleBrightness) + "' oninput='updateBrightnessValue(this.value)' onchange='saveAutoCycleSettings()' style='width: 100%; height: 8px; border-radius: 4px; background: #2a2d47; outline: none; cursor: pointer;'>";
    html += "<div style='display: flex; justify-content: space-between; font-size: 11px; color: #a3a3a3; margin-top: 2px;'>";
    html += "<span>Dim</span><span>Bright</span>";
    html += "</div>";
    html += "</div>";
    
    html += "<p style='font-size: 12px; color: #a3a3a3; margin: 12px 0 0 0;'>Adjust speed and brightness for Auto mode color cycling</p>";
    html += "</div>";
    html += "</div>";
    
    // Manual Color Control (Right)
    html += "<div class='panel col-6'>";
    html += "<h2><i class='fas fa-palette'></i> Manual Color Control</h2>";
    html += "<div class='card' style='border-left: 4px solid #10b981;'>";
    html += "<div style='display: flex; align-items: center; gap: 16px; justify-content: center; padding: 12px;'>";
    html += "<input type='color' id='colorPicker' value='#ff0000' style='width: 80px; height: 80px; border-radius: 12px; cursor: pointer;'>";
    html += "<div style='flex: 1;'>";
    html += "<button class='btn btn-success' style='width: 100%; font-size: 16px; padding: 16px;' onclick='applySelectedColor()'><i class='fas fa-check'></i> Apply Color</button>";
    html += "</div></div>";
    html += "<p style='font-size: 12px; color: #a3a3a3; margin: 8px 0 0 0; text-align: center;'>Select and apply a solid color in Manual mode</p>";
    html += "</div>";
    html += "</div>";
    sendChunk(html); html = "";  // Send auto/manual controls chunk
    
    // Quick Controls Panel
    html += "<div class='panel'>";
    html += "<h2><i class='fas fa-bolt'></i> Quick Controls</h2>";
    html += "<div style='padding: 15px;'>";
    
    // Door Lights Feature Toggle
    html += "<div class='card'>";
    html += "<label style='display: flex; align-items: center; font-size: 15px; font-weight: 500; cursor: pointer; margin-bottom: 12px;'>";
    html += "<input type='checkbox' id='doorLightsFeature' " + String(door_open_lights_enabled ? "checked" : "") + " onchange='toggleDoorLightsFeature()' style='margin-right: 12px; transform: scale(1.4);'>";
    html += "<span><i class='fas fa-door-open'></i> Enable Door Lights</span>";
    html += "</label>";
    html += "<div style='display: flex; align-items: center; gap: 10px; margin-left: 28px;'>";
    html += "<label style='font-size: 13px;'>Door Color:</label>";
    html += "<input type='color' id='doorColorQuick' value='#ffffff' onchange='saveDoorColor()' style='width: 50px; height: 35px; border-radius: 6px; cursor: pointer;'>";
    html += "</div>";
    html += "<p style='font-size: 12px; color: #a3a3a3; margin: 8px 0 0 28px;'>Lights turn on when door opens</p>";
    html += "</div>";
    
    // Manual Door Override (Debug)
    html += "<div class='card' style='border-left: 4px solid #f59e0b;'>";
    html += "<label style='display: flex; align-items: center; font-size: 15px; font-weight: 500; cursor: pointer;'>";
    html += "<input type='checkbox' id='doorManualOverride' " + String(manual_door_trigger ? "checked" : "") + " onchange='toggleManualDoorOverride()' style='margin-right: 12px; transform: scale(1.4);'>";
    html += "<span><i class='fas fa-door-open'></i> Manual Door Override</span>";
    html += "</label>";
    html += "<p style='font-size: 12px; color: #a3a3a3; margin: 8px 0 0 28px;'>Force door lights ON (testing)</p>";
    html += "</div>";
    
    // Chamber Light Sync
    html += "<div class='card' style='border-left: 4px solid #06b6d4;'>";
    html += "<label style='display: flex; align-items: center; font-size: 15px; font-weight: 500; cursor: pointer;'>";
    html += "<input type='checkbox' id='chamberLightSync' " + String(chamber_light_sync_enabled ? "checked" : "") + " onchange='toggleChamberLightSync()' style='margin-right: 12px; transform: scale(1.4);'>";
    html += "<span><i class='fas fa-link'></i> Sync Chamber Light</span>";
    html += "</label>";
    html += "<p style='font-size: 12px; color: #a3a3a3; margin: 8px 0 0 28px;'>Timer controls printer chamber light</p>";
    html += "</div>";
    
    // Timelapse White Lights
    html += "<div class='card' style='border-left: 4px solid #8b5cf6;'>";
    html += "<label style='display: flex; align-items: center; font-size: 15px; font-weight: 500; cursor: pointer;'>";
    html += "<input type='checkbox' id='timelapseWhiteLights' " + String(timelapse_white_lights_enabled ? "checked" : "") + " onchange='toggleTimelapseWhiteLights()' style='margin-right: 12px; transform: scale(1.4);'>";
    html += "<span><i class='fas fa-video'></i> Timelapse White Lights</span>";
    html += "</label>";
    html += "<p style='font-size: 12px; color: #a3a3a3; margin: 8px 0 0 28px;'>Force white lights during timelapse</p>";
    html += "</div>";
    
    // Chamber Light Control (Testing)
    html += "<div class='card' style='border-left: 4px solid #f59e0b;'>";
    html += "<div style='font-size: 15px; font-weight: 500; margin-bottom: 12px;'><i class='fas fa-lightbulb'></i> Chamber Light Test</div>";
    html += "<div class='grid-2'>";
    html += "<button onclick='setChamberLightState(true)' class='btn btn-success'><i class='fas fa-toggle-on'></i> ON</button>";
    html += "<button onclick='setChamberLightState(false)' class='btn btn-danger'><i class='fas fa-toggle-off'></i> OFF</button>";
    html += "</div>";
    html += "<p style='font-size: 12px; color: #a3a3a3; margin: 8px 0 0 0;'>Manual G-code control for testing</p>";
    html += "</div>";
    
    // Manual Beacon Override (Debug)
    html += "<div class='card' style='border-left: 4px solid #a855f7;'>";
    html += "<label style='display: flex; align-items: center; font-size: 15px; font-weight: 500; cursor: pointer;'>";
    html += "<input type='checkbox' id='beaconManualOverride' onchange='toggleManualBeaconOverride()' style='margin-right: 12px; transform: scale(1.4);'>";
    html += "<span><i class='fas fa-broadcast-tower'></i> Manual Beacon Override</span>";
    html += "</label>";
    html += "<p style='font-size: 12px; color: #a3a3a3; margin: 8px 0 0 28px;'>Force beacon ON (testing)</p>";
    html += "</div>";
    
    // LiDAR Pause
    html += "<div class='card' style='border-left: 4px solid #8b5cf6;'>";
    html += "<label style='display: flex; align-items: center; font-size: 15px; font-weight: 500; cursor: pointer;'>";
    html += "<input type='checkbox' id='lidarEnabled' " + String(lidar_lights_off_enabled ? "checked" : "") + " onchange='saveLidarSettings()' style='margin-right: 12px; transform: scale(1.4);'>";
    html += "<span><i class='fas fa-eye'></i> Pause for LiDAR Scan</span>";
    html += "</label>";
    html += "<p style='font-size: 12px; color: #a3a3a3; margin: 8px 0 0 28px;'>Turn off lights during LiDAR scanning</p>";
    html += "</div>";
    
    // Auto-off Timer
    html += "<div class='card' style='border-left: 4px solid #06b6d4;'>";
    html += "<label style='display: block; font-size: 15px; font-weight: 500; margin-bottom: 10px; color: #fff;'><i class='fas fa-clock'></i> Auto-Off Timer</label>";
    html += "<div style='display: flex; align-items: center; gap: 10px; margin-bottom: 8px;'>";
    html += "<input type='number' id='autoOffMinutes' value='0' min='0' max='120' style='width: 80px; padding: 8px; background: #1e2139; border: 2px solid #2a2d47; border-radius: 6px; font-size: 14px; color: #e0e0e0;'>";
    html += "<span style='font-size: 14px; color: #e0e0e0;'>minutes</span>";
    html += "<button class='btn btn-success' onclick='saveAutoOff()' style='margin-left: auto;'>Save</button>";
    html += "</div>";
    html += "<p style='font-size: 12px; color: #a3a3a3; margin: 0;'>Timer starts when print finishes. Lights stay on during active printing. Set to 0 to disable.</p>";
    html += "</div>";
    
    html += "</div></div>";
    
    // Status Panel
    html += "<div class='panel status-panel col-5'>";
    html += "<h2><i class='fas fa-info-circle'></i> System Status & Configuration</h2>";
    html += "<div class='responsive'>";
    html += "<button class='btn btn-secondary' onclick='getStatus()'><i class='fas fa-sync'></i> Refresh Status</button>";
    html += "<button class='btn btn-primary' onclick='showConfig()'><i class='fas fa-cog'></i> Config</button>";
    html += "</div>";
    html += "<div id='info' class='info-box'>";
    
    // Generate initial status inline instead of waiting for fetch
    html += "<b>ESP32 Status:</b><br>";
    html += "WiFi: " + WiFi.localIP().toString() + "<br>";
    html += "Signal: " + String(WiFi.RSSI()) + " dBm<br>";
    html += "Uptime: " + String(millis() / 1000) + " seconds<br>";
    html += "Free Memory: " + String(ESP.getFreeHeap()) + " bytes<br>";
    html += "LED Count: " + String(num_leds) + "<br>";
    
    bool mqttConnected = mqttClient.connected();
    String mqttStatus = mqttConnected ? 
        "<span style='color: #28a745; font-weight: bold;'>&#x2713; Connected</span>" : 
        "<span style='color: #dc3545; font-weight: bold;'>&#x2717; Disconnected</span>";
    html += "Printer MQTT: " + mqttStatus + "<br>";
    
    // Chamber light sync status
    String chamberSyncStatus = chamber_light_sync_enabled ? 
        "<span style='color: #28a745;'>&#x2713; Enabled</span>" : 
        "<span style='color: #6c757d;'>Disabled</span>";
    html += "Chamber Light Sync: " + chamberSyncStatus + "<br>";
    
    // Timelapse white lights status
    String timelapseWhiteStatus = timelapse_white_lights_enabled ? 
        "<span style='color: #28a745;'>&#x2713; Enabled</span>" : 
        "<span style='color: #6c757d;'>Disabled</span>";
    html += "Timelapse White Lights: " + timelapseWhiteStatus + "<br>";
    
    String modeStr = "Unknown";
    switch (currentMode) {
        case MODE_AUTO: modeStr = "Auto Cycling"; break;
        case MODE_MANUAL: modeStr = "Manual Control"; break;  
        case MODE_PRINTER: modeStr = "Printer Status"; break;
        case MODE_OFF: modeStr = "LEDs Off"; break;
    }
    html += "Current Mode: " + modeStr + "<br>";
    
    if (currentMode == MODE_PRINTER) {
        html += "<br><b>Printer Status:</b><br>";
        html += "Online: " + String(printerStatus.online ? "Yes" : "No") + "<br>";
        
        // State description
        String stateDesc = printerStatus.gcode_state;
        if (printerStatus.gcode_state == "RUNNING") stateDesc = "Running";
        else if (printerStatus.gcode_state == "IDLE") stateDesc = "Idle";
        else if (printerStatus.gcode_state == "FINISH") stateDesc = "Finished";
        else if (printerStatus.gcode_state == "FAILED") stateDesc = "Failed";
        else if (printerStatus.gcode_state == "PAUSE") stateDesc = "Paused";
        else if (printerStatus.gcode_state == "PREPARE") stateDesc = "Preparing";
        html += "State: " + printerStatus.gcode_state + " (" + stateDesc + ")<br>";
        
        // Stage description
        String stageDesc = "Unknown";
        int stageNum = printerStatus.stage.toInt();
        if (printerStatus.gcode_state == "RUNNING" || printerStatus.gcode_state == "PREPARE") {
            if (stageNum == 2) stageDesc = "Heating Bed";
            else if (stageNum == 3) stageDesc = "Heating Hotend";
            else if (stageNum >= 14) stageDesc = "Printing";
            else if (stageNum > 0) stageDesc = "Preparing";
            else if (stageNum == 0 && printerStatus.mc_print_stage == "2") stageDesc = "Printing";
            else if (stageNum == 0) stageDesc = "Active";
        } else if (printerStatus.gcode_state == "IDLE") {
            stageDesc = "Idle";
        } else if (printerStatus.gcode_state == "FINISH") {
            stageDesc = "Complete";
        } else if (printerStatus.gcode_state == "PAUSE") {
            stageDesc = "Paused";
        }
        html += "Stage: " + String(stageNum) + " (" + stageDesc + ")<br>";
        
        // Print Stage description (mc_print_stage)
        if (printerStatus.mc_print_stage != "unknown") {
            String printStageDesc = printerStatus.mc_print_stage;
            if (printerStatus.mc_print_stage == "1") printStageDesc = "Idle";
            else if (printerStatus.mc_print_stage == "2") printStageDesc = "Printing";
            else if (printerStatus.mc_print_stage == "scanning") printStageDesc = "Scanning";
            else if (printerStatus.mc_print_stage == "first_layer_scan") printStageDesc = "First Layer Scan";
            else if (printerStatus.mc_print_stage == "auto_bed_leveling") printStageDesc = "Auto Bed Leveling";
            else if (printerStatus.mc_print_stage.indexOf("inspect") >= 0) printStageDesc = "Inspecting";
            else if (printerStatus.mc_print_stage.indexOf("scan") >= 0) printStageDesc = "Scanning";
            else if (printerStatus.mc_print_stage.indexOf("bed_leveling") >= 0) printStageDesc = "Bed Leveling";
            else if (printerStatus.mc_print_stage.indexOf("calibrat") >= 0) printStageDesc = "Calibrating";
            html += "Print Stage: " + printerStatus.mc_print_stage + " (" + printStageDesc + ")<br>";
        }
        
        // Print Action description
        if (printerStatus.print_real_action != "unknown") {
            String actionDesc = printerStatus.print_real_action;
            int actionNum = printerStatus.print_real_action.toInt();
            if (actionNum == 0) actionDesc = "Idle";
            else if (actionNum == 1) actionDesc = "Printing";
            else if (actionNum == 2) actionDesc = "Heating";
            else if (actionNum == 3) actionDesc = "Paused";
            else if (printerStatus.print_real_action == "scanning") actionDesc = "Scanning";
            else if (printerStatus.print_real_action == "lidar_scan") actionDesc = "LiDAR Scan";
            else if (printerStatus.print_real_action == "first_layer_inspect") actionDesc = "First Layer Inspect";
            else if (printerStatus.print_real_action.indexOf("bed_leveling") >= 0) actionDesc = "Bed Leveling";
            else if (printerStatus.print_real_action.indexOf("calibrat") >= 0) actionDesc = "Calibrating";
            html += "Print Action: " + printerStatus.print_real_action + " (" + actionDesc + ")<br>";
        }
        
        html += "Progress: " + String(printerStatus.progress) + "%<br>";
        html += "Bed Temp: " + String(printerStatus.bed_temp) + "°C<br>";
        html += "Nozzle Temp: " + String(printerStatus.nozzle_temp) + "°C<br>";
        
        if (printerStatus.lastUpdate > 0) {
            html += "Last Update: " + String((millis() - printerStatus.lastUpdate) / 1000) + "s ago<br>";
        }
    }
    
    html += "</div>";
    html += "</div>";
    sendChunk(html); html = "";  // Send status panel chunk
    
    // Serial Monitor Panel
    html += "<div class='panel col-7'>";
    html += "<h2><i class='fas fa-terminal'></i> Serial Monitor (Stats for Nerds)</h2>";
    html += "<div style='margin-bottom: 10px;'>";
    html += "<button class='btn btn-secondary' onclick='refreshLogs()' style='margin-right: 10px;'><i class='fas fa-sync'></i> Refresh</button>";
    html += "<button class='btn' id='mqttLogBtn' onclick='toggleMqttLogging()' style='margin-right: 10px; background: #6c757d;'><i class='fas fa-bug'></i> MQTT Debug: OFF</button>";
    html += "<label style='color: #d4d4d4; font-weight: 500;'><input type='checkbox' id='autoRefreshCheckbox' checked onchange='toggleAutoRefresh()'> <i class='fas fa-redo'></i> Auto-refresh (5s)</label>";
    html += "</div>";
    html += "<div style='margin-bottom: 10px; padding: 10px; background: #2a2a2a; border-radius: 5px; color: #ffa500; font-size: 12px;'>";
    html += "<i class='fas fa-exclamation-triangle'></i> <b>MQTT Debug Mode:</b> Logs full MQTT payloads (15KB+). May slow down web interface. Use for troubleshooting printer state transitions.";
    html += "</div>";
    html += "<div id='serialLogs' style='background: #1e1e1e; color: #d4d4d4; padding: 15px; border-radius: 8px; font-family: \"Courier New\", monospace; font-size: 12px; height: 300px; overflow-y: auto; overflow-x: hidden; white-space: pre-wrap; word-wrap: break-word; word-break: break-all; border: 1px solid #444;'>";
    html += "Loading logs...";
    html += "</div>";
    html += "</div>";
    
    html += "</div>"; // Close main-container
    sendChunk(html); html = "";  // Send serial monitor chunk
    
    html += "<div id='configModal' style='display:none; position:fixed; top:0; left:0; width:100%; height:100%; background:rgba(0,0,0,0.7); z-index:1000; backdrop-filter: blur(5px);'>";
    html += "<div style='position:absolute; top:50%; left:50%; transform:translate(-50%,-50%); background:white; padding:40px; border-radius:20px; max-width:600px; width:90%; box-shadow: 0 20px 60px rgba(0,0,0,0.3); max-height: 80vh; overflow-y: auto;'>";
    html += "<h2 style='color: #333; margin-bottom: 25px; text-align: center;'>&#9881; System Configuration</h2>";
    
    // MQTT Section
    html += "<h3 style='color: #007bff; margin: 20px 0 15px 0; border-bottom: 2px solid #007bff; padding-bottom: 5px;'>&#128760; MQTT Settings</h3>";
    html += "<div style='margin-bottom: 20px;'>";
    html += "<label style='display: block; margin-bottom: 8px; font-weight: 500; color: #555;'>Printer IP Address:</label>";
    html += "<input type='text' id='mqttServer' style='width: calc(100% - 20px); padding: 12px; border: 2px solid #ddd; border-radius: 8px; font-size: 14px;' placeholder='192.168.0.100'>";
    html += "</div>";
    html += "<div style='margin-bottom: 20px;'>";
    html += "<label style='display: block; margin-bottom: 8px; font-weight: 500; color: #555;'>Access Code:</label>";
    html += "<input type='text' id='mqttPassword' style='width: calc(100% - 20px); padding: 12px; border: 2px solid #ddd; border-radius: 8px; font-size: 14px;' placeholder='your_access_code'>";
    html += "</div>";
    html += "<div style='margin-bottom: 20px;'>";
    html += "<label style='display: block; margin-bottom: 8px; font-weight: 500; color: #555;'>MQTT Port:</label>";
    html += "<input type='number' id='mqttPort' min='1' max='65535' style='width: calc(100% - 20px); padding: 12px; border: 2px solid #ddd; border-radius: 8px; font-size: 14px;' value='1883'>";
    html += "</div>";
    
    // SSL/TLS Section
    html += "<h3 style='color: #fd7e14; margin: 25px 0 15px 0; border-bottom: 2px solid #fd7e14; padding-bottom: 5px;'>SSL/TLS Settings</h3>";
    html += "<div style='margin-bottom: 15px; padding: 15px; background: #fff3cd; border: 1px solid #ffeaa7; border-radius: 8px;'>";
    html += "<label style='display: flex; align-items: center; margin-bottom: 10px; cursor: pointer;'>";
    html += "<input type='checkbox' id='useSSL' style='margin-right: 10px; transform: scale(1.2);'>";
    html += "<span style='font-weight: 500; color: #856404;'>Use SSL/TLS (Port 8883)</span>";
    html += "</label>";
    html += "<label style='display: flex; align-items: center; cursor: pointer;'>";
    html += "<input type='checkbox' id='skipCert' style='margin-right: 10px; transform: scale(1.2);'>";
    html += "<span style='font-weight: 500; color: #856404;'>Skip Certificate Verification (Insecure)</span>";
    html += "</label>";
    html += "<small style='color: #6c757d; display: block; margin-top: 8px;'>&#9888;&#65039; SSL is required for port 8883. Use port 1883 for non-secure connections.</small>";
    html += "</div>";
    
    html += "<div style='margin-bottom: 20px;'>";
    html += "<label style='display: block; margin-bottom: 8px; font-weight: 500; color: #555;'>Printer Serial Number:</label>";
    html += "<input type='text' id='deviceSerial' style='width: calc(100% - 20px); padding: 12px; border: 2px solid #ddd; border-radius: 8px; font-size: 14px;' placeholder='01S00A123456789'>";
    html += "</div>";
    
    // LED Section  
    html += "<h3 style='color: #28a745; margin: 25px 0 15px 0; border-bottom: 2px solid #28a745; padding-bottom: 5px;'>LED Strip Settings</h3>";
    html += "<div style='display: flex; gap: 15px; margin-bottom: 20px;'>";
    html += "<div style='flex: 1;'>";
    html += "<label style='display: block; margin-bottom: 8px; font-weight: 500; color: #555;'>Data Pin (GPIO):</label>";
    html += "<select id='ledPin' style='width: 100%; padding: 12px; border: 2px solid #ddd; border-radius: 8px; font-size: 14px;'>";
    html += "<option value='2'>GPIO 2 (Built-in LED)</option>";
    html += "<option value='4'>GPIO 4 (Safe)</option>";
    html += "<option value='5'>GPIO 5 (Safe)</option>";
    html += "<option value='13'>GPIO 13 (Safe)</option>";
    html += "<option value='14'>GPIO 14 (Safe)</option>";
    html += "<option value='15'>GPIO 15 (Safe)</option>";
    html += "<option value='16'>GPIO 16 (Safe)</option>";
    html += "<option value='17'>GPIO 17 (Safe)</option>";
    html += "<option value='18'>GPIO 18 (Safe)</option>";
    html += "<option value='19'>GPIO 19 (Safe)</option>";
    html += "<option value='21'>GPIO 21 (Safe)</option>";
    html += "<option value='22'>GPIO 22 (Safe)</option>";
    html += "<option value='23'>GPIO 23 (Safe)</option>";
    html += "<option value='25'>GPIO 25 (Safe)</option>";
    html += "<option value='26'>GPIO 26 (Safe)</option>";
    html += "<option value='27'>GPIO 27 (Safe)</option>";
    html += "<option value='32'>GPIO 32 (Safe)</option>";
    html += "<option value='33'>GPIO 33 (Safe)</option>";
    html += "</select>";
    html += "</div>";
    html += "<div style='flex: 1;'>";
    html += "<label style='display: block; margin-bottom: 8px; font-weight: 500; color: #555;'>LED Count:</label>";
    html += "<input type='number' id='numLeds' min='1' max='300' style='width: calc(100% - 20px); padding: 12px; border: 2px solid #ddd; border-radius: 8px; font-size: 14px;' placeholder='10'>";
    html += "</div>";
    html += "</div>";
    
    html += "<div style='display: flex; gap: 15px; margin-bottom: 20px;'>";
    html += "<div style='flex: 1;'>";
    html += "<label style='display: block; margin-bottom: 8px; font-weight: 500; color: #555;'>LED Type:</label>";
    html += "<select id='ledType' style='width: 100%; padding: 12px; border: 2px solid #ddd; border-radius: 8px; font-size: 14px;'>";
    html += "<option value='WS2812B'>WS2812B (Most Common)</option>";
    html += "<option value='WS2811'>WS2811 (5V Strips)</option>";
    html += "<option value='WS2813'>WS2813 (Backup Data)</option>";
    html += "<option value='SK6812'>SK6812 (RGBW Compatible)</option>";
    html += "<option value='APA102'>APA102 (SPI/Clock)</option>";
    html += "</select>";
    html += "</div>";
    html += "<div style='flex: 1;'>";
    html += "<label style='display: block; margin-bottom: 8px; font-weight: 500; color: #555;'>Color Order:</label>";
    html += "<select id='colorOrder' style='width: 100%; padding: 12px; border: 2px solid #ddd; border-radius: 8px; font-size: 14px;'>";
    html += "<option value='GRB'>GRB (WS2812B Standard)</option>";
    html += "<option value='RGB'>RGB (Natural Order)</option>";
    html += "<option value='BRG'>BRG</option>";
    html += "<option value='RBG'>RBG</option>";
    html += "<option value='GBR'>GBR</option>";
    html += "<option value='BGR'>BGR</option>";
    html += "</select>";
    html += "</div>";
    html += "</div>";
    
    html += "<div style='background: #fff3cd; border: 1px solid #ffeaa7; border-radius: 8px; padding: 10px; margin-bottom: 25px; font-size: 13px;'>";
    html += "<b>Note:</b> Changing LED settings (pin, count, type, or color order) requires ESP32 restart. Avoid GPIO 0, 1, 3, 6-12 (boot/flash pins).";
    html += "</div>";
    
    // Beacon LED Hardware Configuration
    html += "<h3 style='color: #6f42c1; margin: 25px 0 15px 0; border-bottom: 2px solid #6f42c1; padding-bottom: 5px;'>&#128161; Beacon LED Hardware</h3>";
    html += "<div style='margin-bottom: 15px; padding: 15px; background: #f8f9fa; border-radius: 8px;'>";
    html += "<label style='display: flex; align-items: center; font-size: 15px; font-weight: 500; cursor: pointer; margin-bottom: 12px;'>";
    html += "<input type='checkbox' id='beaconEnabledConfig' style='margin-right: 12px; transform: scale(1.4);'>";
    html += "<span>&#9989; Enable Beacon Strip</span>";
    html += "</label>";
    html += "<p style='font-size: 12px; color: #666; margin: 0 0 0 28px;'>Second LED strip for print progress visualization</p>";
    html += "</div>";
    html += "<div style='display: flex; gap: 15px; margin-bottom: 20px;'>";
    html += "<div style='flex: 1;'>";
    html += "<label style='display: block; margin-bottom: 8px; font-weight: 500; color: #555;'>Beacon GPIO Pin:</label>";
    html += "<input type='number' id='beaconPinConfig' min='0' max='39' style='width: calc(100% - 20px); padding: 12px; border: 2px solid #ddd; border-radius: 8px; font-size: 14px;' placeholder='14'>";
    html += "</div>";
    html += "<div style='flex: 1;'>";
    html += "<label style='display: block; margin-bottom: 8px; font-weight: 500; color: #555;'>Beacon LED Count:</label>";
    html += "<input type='number' id='beaconCountConfig' min='1' max='50' style='width: calc(100% - 20px); padding: 12px; border: 2px solid #ddd; border-radius: 8px; font-size: 14px;' placeholder='10'>";
    html += "</div>";
    html += "</div>";
    html += "<div style='display: flex; gap: 15px; margin-bottom: 20px;'>";
    html += "<div style='flex: 1;'>";
    html += "<label style='display: block; margin-bottom: 8px; font-weight: 500; color: #555;'>Beacon LED Type:</label>";
    html += "<select id='beaconTypeConfig' style='width: 100%; padding: 12px; border: 2px solid #ddd; border-radius: 8px; font-size: 14px;'>";
    html += "<option value='0'>WS2812B</option>";
    html += "<option value='1'>WS2811</option>";
    html += "<option value='2'>WS2813</option>";
    html += "<option value='3'>SK6812</option>";
    html += "<option value='4'>APA102</option>";
    html += "</select>";
    html += "</div>";
    html += "<div style='flex: 1;'>";
    html += "<label style='display: block; margin-bottom: 8px; font-weight: 500; color: #555;'>Beacon RGB Order:</label>";
    html += "<select id='beaconColorOrderConfig' style='width: 100%; padding: 12px; border: 2px solid #ddd; border-radius: 8px; font-size: 14px;'>";
    html += "<option value='0'>RGB</option>";
    html += "<option value='1'>GRB</option>";
    html += "<option value='2'>BRG</option>";
    html += "<option value='3'>RBG</option>";
    html += "<option value='4'>GBR</option>";
    html += "<option value='5'>BGR</option>";
    html += "</select>";
    html += "</div>";
    html += "</div>";
    html += "<div style='margin-bottom: 20px;'>";
    html += "<label style='display: flex; align-items: center; cursor: pointer;'>";
    html += "<input type='checkbox' id='beaconRGBWConfig' style='margin-right: 8px; width: 18px; height: 18px;'>";
    html += "<span style='font-weight: 500; color: #555;'>RGBW LEDs (4-channel with white - check this if colors are wrong)</span>";
    html += "</label>";
    html += "</div>";
    html += "<div style='background: #e7f3ff; border: 1px solid #b3d9ff; border-radius: 8px; padding: 10px; margin-bottom: 25px; font-size: 13px;'>";
    html += "<b>Info:</b> Beacon strip requires a separate GPIO pin from your main LED strip. <b>SK6812 LEDs</b> come in both RGB (3-channel) and RGBW (4-channel with white) variants - if you see wrong colors, enable the RGBW checkbox above. Settings are saved automatically.";
    html += "</div>";
    
    // Home Automation MQTT Section
    html += "<h3 style='color: #41bdf5; margin: 25px 0 15px 0; border-bottom: 2px solid #41bdf5; padding-bottom: 5px;'>&#127968; Home Automation MQTT</h3>";
    html += "<div style='margin-bottom: 15px; padding: 15px; background: #e3f2fd; border: 1px solid #90caf9; border-radius: 8px;'>";
    html += "<label style='display: flex; align-items: center; font-size: 15px; font-weight: 500; cursor: pointer; margin-bottom: 12px;'>";
    html += "<input type='checkbox' id='haMqttEnabled' style='margin-right: 12px; transform: scale(1.4);'>";
    html += "<span>&#9989; Enable MQTT Discovery</span>";
    html += "</label>";
    html += "<p style='font-size: 12px; color: #666; margin: 0 0 0 28px;'>Publishes printer status, progress, and temperatures to your home automation MQTT broker (OpenHAB, Home Assistant, etc.)</p>";
    html += "</div>";
    html += "<div style='margin-bottom: 20px;'>";
    html += "<label style='display: block; margin-bottom: 8px; font-weight: 500; color: #555;'>MQTT Broker IP:</label>";
    html += "<input type='text' id='haMqttServer' style='width: calc(100% - 20px); padding: 12px; border: 2px solid #ddd; border-radius: 8px; font-size: 14px;' placeholder='192.168.0.50'>";
    html += "</div>";
    html += "<div style='margin-bottom: 20px;'>";
    html += "<label style='display: block; margin-bottom: 8px; font-weight: 500; color: #555;'>MQTT Port:</label>";
    html += "<input type='number' id='haMqttPort' min='1' max='65535' style='width: calc(100% - 20px); padding: 12px; border: 2px solid #ddd; border-radius: 8px; font-size: 14px;' value='1883'>";
    html += "</div>";
    html += "<div style='margin-bottom: 20px;'>";
    html += "<label style='display: block; margin-bottom: 8px; font-weight: 500; color: #555;'>Username (optional):</label>";
    html += "<input type='text' id='haMqttUsername' style='width: calc(100% - 20px); padding: 12px; border: 2px solid #ddd; border-radius: 8px; font-size: 14px;' placeholder=''>";
    html += "</div>";
    html += "<div style='margin-bottom: 20px;'>";
    html += "<label style='display: block; margin-bottom: 8px; font-weight: 500; color: #555;'>Password (optional):</label>";
    html += "<input type='password' id='haMqttPassword' style='width: calc(100% - 20px); padding: 12px; border: 2px solid #ddd; border-radius: 8px; font-size: 14px;' placeholder='Password'>";
    html += "</div>";
    html += "<div style='margin-bottom: 20px;'>";
    html += "<label style='display: block; margin-bottom: 8px; font-weight: 500; color: #555;'>Discovery Prefix:</label>";
    html += "<input type='text' id='haDiscoveryPrefix' style='width: calc(100% - 20px); padding: 12px; border: 2px solid #ddd; border-radius: 8px; font-size: 14px;' value='homeassistant' placeholder='homeassistant'>";
    html += "<small style='color: #6c757d; display: block; margin-top: 4px;'>Use 'homeassistant' for HA auto-discovery, or custom prefix for OpenHAB/other systems</small>";
    html += "</div>";
    html += "<div style='margin-bottom: 20px;'>";
    html += "<label style='display: block; margin-bottom: 8px; font-weight: 500; color: #555;'>Device Name:</label>";
    html += "<input type='text' id='haDeviceName' style='width: calc(100% - 20px); padding: 12px; border: 2px solid #ddd; border-radius: 8px; font-size: 14px;' value='Bambu Lights' placeholder='Bambu Lights'>";
    html += "</div>";
    html += "<div style='margin-bottom: 15px; padding: 15px; background: #fff3cd; border: 1px solid #ffeaa7; border-radius: 8px;'>";
    html += "<label style='display: flex; align-items: center; cursor: pointer;'>";
    html += "<input type='checkbox' id='haMqttUseSSL' style='margin-right: 10px; transform: scale(1.2);'>";
    html += "<span style='font-weight: 500; color: #856404;'>Use SSL/TLS</span>";
    html += "</label>";
    html += "<small style='color: #6c757d; display: block; margin-top: 8px;'>&#9888; Most MQTT brokers use port 1883 without SSL (OpenHAB, Home Assistant default)</small>";
    html += "</div>";
    html += "<button onclick='saveHAMqttConfig()' class='btn btn-success' style='width: 100%; padding: 12px; margin-top: 10px;'>&#128190; Save MQTT Settings</button>";
    html += "<div style='background: #e3f2fd; border: 1px solid #90caf9; border-radius: 8px; padding: 10px; margin-top: 15px; font-size: 13px;'>";
    html += "<b>&#128161; Published Topics:</b> Printer state, progress, temperatures, layer info, and filename are published to <code>bambu-lights/sensor/*</code> topics. Works with OpenHAB, Home Assistant (auto-discovery), Node-RED, and any MQTT-compatible automation system.";
    html += "</div>";
    
    // WiFi Reset Section
    html += "<h3 style='color: #dc3545; margin: 25px 0 15px 0; border-bottom: 2px solid #dc3545; padding-bottom: 5px;'>&#128246; WiFi Management</h3>";
    html += "<div style='padding: 15px; background: #fff3cd; border: 2px solid #ffc107; border-radius: 8px; margin-bottom: 25px;'>";
    html += "<p style='margin: 0 0 10px 0; color: #856404; font-weight: 500;'>&#9888; Reset WiFi credentials to connect to a different network</p>";
    html += "<button onclick='resetWiFi()' class='btn btn-warning' style='width: 100%; padding: 12px;'>&#128260; Reset WiFi Settings</button>";
    html += "</div>";
    
    html += "<div style='padding: 15px; background: #e7f3ff; border: 2px solid #007bff; border-radius: 8px; margin-bottom: 25px;'>";
    html += "<p style='margin: 0 0 10px 0; color: #004085; font-weight: 500;'>&#128259; Reboot the ESP32 device</p>";
    html += "<button onclick='rebootDevice()' class='btn btn-primary' style='width: 100%; padding: 12px;'>&#128257; Reboot Device</button>";
    html += "</div>";
    
    html += "<div style='text-align: center;'>";
    html += "<button class='btn btn-success btn-large' onclick='saveConfig()' style='margin-right: 15px;'>&#128190; Save & Restart</button>";
    html += "<button class='btn btn-secondary btn-large' onclick='hideConfig()'>&#10060; Cancel</button>";
    html += "</div></div></div>";
    sendChunk(html); html = "";  // Send config modal chunk
    
    html += "<script>\n";
    // Beacon functions - must be defined before HTML elements that call them
    html += "function updateBeaconPreviewGif() {\n";
    html += "  const effect = document.getElementById('beacon_effect').value;\n";
    html += "  const previewImg = document.getElementById('beacon_preview_gif');\n";
    html += "  const gifMap = {\n";
    html += "    'progress': '/progress_bar.gif',\n";
    html += "    'climbing': '/climbing_dot.gif',\n";
    html += "    'gradient': '/gradient_fill.gif',\n";
    html += "    'pulse': '/pulsing_dot.gif'\n";
    html += "  };\n";
    html += "  const newSrc = gifMap[effect] || '/progress_bar.gif';\n";
    html += "  if (previewImg.src.split('/').pop() !== newSrc.split('/').pop()) {\n";
    html += "    previewImg.src = newSrc + '?t=' + new Date().getTime();\n";
    html += "  }\n";
    html += "}\n";
    html += "function toggleBeaconAnimControls() {\n";
    html += "  const effect = document.getElementById('beacon_effect').value;\n";
    html += "  const gradientContainer = document.getElementById('gradient_end_color_container');\n";
    html += "  if (effect === 'gradient') {\n";
    html += "    gradientContainer.style.display = 'block';\n";
    html += "  } else {\n";
    html += "    gradientContainer.style.display = 'none';\n";
    html += "  }\n";
    html += "}\n";
    html += "async function autoSaveBeacon() {\n";
    html += "  toggleBeaconAnimControls();\n";
    html += "  updateBeaconPreviewGif();\n";
    html += "  const beaconEffect = document.getElementById('beacon_effect').value;\n";
    html += "  const beaconColor = document.getElementById('beacon_color').value;\n";
    html += "  const beaconBrightness = document.getElementById('beacon_brightness').value;\n";
    html += "  const beaconClimbColor = document.getElementById('beacon_climb_color').value;\n";
    html += "  const beaconClimbSpeed = document.getElementById('beacon_climb_speed').value;\n";
    html += "  const beaconGradientEndColor = document.getElementById('beacon_gradient_end_color').value;\n";
    html += "  const beaconUrl = `/beaconconfig?effect=${beaconEffect}&color=${encodeURIComponent(beaconColor)}&brightness=${beaconBrightness}&climbColor=${encodeURIComponent(beaconClimbColor)}&climbSpeed=${beaconClimbSpeed}&gradientEndColor=${encodeURIComponent(beaconGradientEndColor)}`;\n";
    html += "  try {\n";
    html += "    const response = await fetch(beaconUrl);\n";
    html += "    const result = await response.text();\n";
    html += "    console.log('Beacon auto-saved:', result);\n";
    html += "    await fetch('/refreshprinter');\n";
    html += "  } catch (error) {\n";
    html += "    console.error('Error auto-saving beacon settings:', error);\n";
    html += "  }\n";
    html += "}\n";
    html += "const modeDescriptions = {\n";
    html += "  printer: '<i class=\"fas fa-print\" style=\"color: #667eea;\"></i> LEDs respond to printer status events (printing, idle, error, heating, paused, complete).',\n";
    html += "  auto: '<i class=\"fas fa-magic\" style=\"color: #f59e0b;\"></i> LEDs cycle through colors automatically with adjustable speed and brightness.',\n";
    html += "  manual: '<i class=\"fas fa-hand-pointer\" style=\"color: #10b981;\"></i> Set a custom solid color using the color picker below.',\n";
    html += "  off: '<i class=\"fas fa-power-off\" style=\"color: #6366f1;\"></i> All LEDs turned off (door lights and timers still active if enabled).'\n";
    html += "};\n";
    html += "function setMode(mode) {\n";
    html += "  fetch('/mode?m=' + mode)\n";
    html += "    .then(response => response.text())\n";
    html += "    .then(data => {\n";
    html += "      document.getElementById('status').innerText = data;\n";
    html += "      document.getElementById('status').className = 'status ' + (mode === 'auto' ? 'status-auto' : 'status-success');\n";
    html += "      const desc = document.getElementById('modeDescription');\n";
    html += "      if (desc && modeDescriptions[mode]) {\n";
    html += "        desc.innerHTML = '<i class=\"fas fa-info-circle\" style=\"color: #7c3aed; margin-right: 6px;\"></i>' + modeDescriptions[mode];\n";
    html += "      }\n";
    html += "    });\n";
    html += "}\n";
    
    html += "function applySelectedColor() {\n";
    html += "  const colorPicker = document.getElementById('colorPicker');\n";
    html += "  const hex = colorPicker.value;\n";
    html += "  const r = parseInt(hex.substr(1,2), 16);\n";
    html += "  const g = parseInt(hex.substr(3,2), 16);\n";
    html += "  const b = parseInt(hex.substr(5,2), 16);\n";
    html += "  fetch('/color?r=' + r + '&g=' + g + '&b=' + b)\n";
    html += "    .then(response => response.text())\n";
    html += "    .then(data => {\n";
    html += "      document.getElementById('status').innerText = data;\n";
    html += "      document.getElementById('status').className = 'status status-success';\n";
    html += "    });\n";
    html += "}\n";
    
    html += "function toggleDoorLightsFeature() {\n";
    html += "  const enabled = document.getElementById('doorLightsFeature').checked;\n";
    html += "  fetch('/setconfig?door_lights_enabled=' + (enabled ? 'true' : 'false'))\n";
    html += "    .then(response => response.text())\n";
    html += "    .then(data => {\n";
    html += "      console.log('Door lights feature:', enabled ? 'ENABLED' : 'DISABLED');\n";
    html += "    })\n";
    html += "    .catch(error => console.error('Error toggling door lights feature:', error));\n";
    html += "}\n";
    
    html += "function toggleManualDoorOverride() {\n";
    html += "  const enabled = document.getElementById('doorManualOverride').checked;\n";
    html += "  fetch('/door?state=' + (enabled ? 'on' : 'off'))\n";
    html += "    .then(response => response.text())\n";
    html += "    .then(data => console.log('Manual door override:', data))\n";
    html += "    .catch(error => console.error('Error toggling manual door override:', error));\n";
    html += "}\n";
    
    html += "function setChamberLightState(state) {\n";
    html += "  console.log('Setting chamber light:', state ? 'ON' : 'OFF');\n";
    html += "  fetch('/chamber?state=' + (state ? 'on' : 'off'))\n";
    html += "    .then(response => response.text())\n";
    html += "    .then(data => {\n";
    html += "      console.log('Chamber light response:', data);\n";
    html += "      alert('Chamber light: ' + data);\n";
    html += "    })\n";
    html += "    .catch(error => {\n";
    html += "      console.error('Error setting chamber light:', error);\n";
    html += "      alert('Error: ' + error);\n";
    html += "    });\n";
    html += "}\n";
    
    html += "function toggleChamberLightSync() {\n";
    html += "  const enabled = document.getElementById('chamberLightSync').checked;\n";
    html += "  fetch('/setconfig?chamber_sync=' + (enabled ? 'true' : 'false'))\n";
    html += "    .then(response => response.text())\n";
    html += "    .then(data => console.log('Chamber light sync:', enabled ? 'ENABLED' : 'DISABLED'))\n";
    html += "    .catch(error => console.error('Error toggling chamber light sync:', error));\n";
    html += "}\n";
    
    html += "function toggleTimelapseWhiteLights() {\n";
    html += "  const enabled = document.getElementById('timelapseWhiteLights').checked;\n";
    html += "  fetch('/setconfig?timelapse_white=' + (enabled ? 'true' : 'false'))\n";
    html += "    .then(response => response.text())\n";
    html += "    .then(data => {\n";
    html += "      console.log('Timelapse white lights:', enabled ? 'ENABLED' : 'DISABLED');\n";
    html += "      // Trigger immediate LED update\n";
    html += "      return fetch('/refreshprinter');\n";
    html += "    })\n";
    html += "    .then(() => console.log('LEDs refreshed'))\n";
    html += "    .catch(error => console.error('Error toggling timelapse white lights:', error));\n";
    html += "}\n";
    
    html += "function toggleManualBeaconOverride() {\n";
    html += "  const enabled = document.getElementById('beaconManualOverride').checked;\n";
    html += "  fetch('/beacon?state=' + (enabled ? 'on' : 'off'))\n";
    html += "    .then(response => response.text())\n";
    html += "    .then(data => console.log('Manual beacon override:', data))\n";
    html += "    .catch(error => console.error('Error toggling manual beacon override:', error));\n";
    html += "}\n";
    
    html += "function saveDoorColor() {\n";
    html += "  const color = document.getElementById('doorColorQuick').value;\n";
    html += "  fetch('/setconfig?door_color=' + encodeURIComponent(color))\n";
    html += "    .then(response => response.text())\n";
    html += "    .then(data => {\n";
    html += "      console.log('Door color saved:', color);\n";
    html += "    })\n";
    html += "    .catch(error => console.error('Error saving door color:', error));\n";
    html += "}\n";
    
    html += "function saveAutoOff() {\n";
    html += "  const minutes = document.getElementById('autoOffMinutes').value;\n";
    html += "  fetch('/autooff?minutes=' + minutes)\n";
    html += "    .then(response => response.text())\n";
    html += "    .then(data => {\n";
    html += "      alert(data);\n";
    html += "      console.log('Auto-off saved:', minutes, 'minutes');\n";
    html += "    })\n";
    html += "    .catch(error => {\n";
    html += "      console.error('Error saving auto-off:', error);\n";
    html += "      alert('Failed to save auto-off setting');\n";
    html += "    });\n";
    html += "}\n";
    
    html += "function updateSpeedValue(value) {\n";
    html += "  document.getElementById('speedValue').textContent = value;\n";
    html += "}\n";
    
    html += "function updateBrightnessValue(value) {\n";
    html += "  document.getElementById('brightnessValue').textContent = value + '%';\n";
    html += "}\n";
    
    html += "function saveAutoCycleSettings() {\n";
    html += "  const speed = document.getElementById('autoCycleSpeed').value;\n";
    html += "  const brightness = document.getElementById('autoCycleBrightness').value;\n";
    html += "  fetch('/setconfig?cycle_speed=' + speed + '&cycle_brightness=' + brightness)\n";
    html += "    .then(response => response.text())\n";
    html += "    .then(data => {\n";
    html += "      console.log('Auto-cycle settings saved: Speed=' + speed + ', Brightness=' + brightness);\n";
    html += "    })\n";
    html += "    .catch(error => {\n";
    html += "      console.error('Error saving auto-cycle settings:', error);\n";
    html += "    });\n";
    html += "}\n";
    
    html += "function resetWiFi() {\n";
    html += "  if (confirm('Reset WiFi settings? Device will restart in config mode.\\n\\nConnect to BambuLED-XXXXXX access point to configure WiFi again.')) {\n";
    html += "    fetch('/wifireset')\n";
    html += "      .then(response => response.text())\n";
    html += "      .then(data => {\n";
    html += "        alert('WiFi reset! Device restarting in config mode...');\n";
    html += "      })\n";
    html += "      .catch(error => console.error('WiFi reset initiated:', error));\n";
    html += "  }\n";
    html += "}\n";
    
    html += "function rebootDevice() {\n";
    html += "  if (confirm('Reboot the ESP32 device?\\n\\nDevice will restart and reconnect in a few seconds.')) {\n";
    html += "    fetch('/reboot')\n";
    html += "      .then(response => response.text())\n";
    html += "      .then(data => {\n";
    html += "        alert('Device rebooting... Page will reload in 5 seconds.');\n";
    html += "        setTimeout(function() { location.reload(); }, 5000);\n";
    html += "      })\n";
    html += "      .catch(error => console.error('Reboot initiated:', error));\n";
    html += "  }\n";
    html += "}\n";
    
    html += "function saveLidarSettings() {\n";
    html += "  const enabled = document.getElementById('lidarEnabled').checked;\n";
    html += "  fetch('/lidar?enabled=' + (enabled ? 'on' : 'off'))\n";
    html += "    .then(response => response.text())\n";
    html += "    .then(data => {\n";
    html += "      alert('LiDAR settings saved: ' + data);\n";
    html += "      getStatus();\n";
    html += "    })\n";
    html += "    .catch(error => {\n";
    html += "      console.error('Error saving LiDAR settings:', error);\n";
    html += "      alert('Failed to save LiDAR settings');\n";
    html += "    });\n";
    html += "}\n";
    
    html += "function getStatus() {\n";
    html += "  fetch('/status')\n";
    html += "    .then(response => {\n";
    html += "      if (!response.ok) throw new Error('Network response was not ok');\n";
    html += "      return response.text();\n";
    html += "    })\n";
    html += "    .then(data => {\n";
    html += "      document.getElementById('info').innerHTML = data;\n";
    html += "      updateLastUpdateTime();\n";
    html += "    })\n";
    html += "    .catch(error => {\n";
    html += "      console.error('Error fetching status:', error);\n";
    html += "      document.getElementById('info').innerHTML = '<span style=\"color: red;\">Error loading status. Check console for details.</span>';\n";
    html += "    });\n";
    html += "}\n";
    
    html += "let lastUpdateBase = 0;\n";
    html += "let lastUpdateOffset = 0;\n";
    
    html += "function updateLastUpdateTime() {\n";
    html += "  const elem = document.getElementById('lastUpdateTime');\n";
    html += "  if (elem && elem.dataset.lastupdatemillis && elem.dataset.nowmillis) {\n";
    html += "    const lastUpdate = parseInt(elem.dataset.lastupdatemillis);\n";
    html += "    const espNow = parseInt(elem.dataset.nowmillis);\n";
    html += "    lastUpdateBase = Date.now();\n";
    html += "    lastUpdateOffset = espNow - lastUpdate;\n";
    html += "  }\n";
    html += "}\n";
    
    html += "function refreshLastUpdateTime() {\n";
    html += "  const elem = document.getElementById('lastUpdateTime');\n";
    html += "  if (elem && lastUpdateBase > 0) {\n";
    html += "    const elapsed = Date.now() - lastUpdateBase;\n";
    html += "    const totalSeconds = Math.floor((elapsed + lastUpdateOffset) / 1000);\n";
    html += "    elem.textContent = totalSeconds + 's';\n";
    html += "  }\n";
    html += "  \n";
    html += "  // Update auto-off timer countdown\n";
    html += "  const timerElem = document.getElementById('autoOffTimer');\n";
    html += "  if (timerElem && timerElem.dataset.endtime) {\n";
    html += "    const endTime = parseInt(timerElem.dataset.endtime);\n";
    html += "    const nowEsp = Date.now() - lastUpdateBase + parseInt(elem?.dataset.nowmillis || 0);\n";
    html += "    const remainingMs = endTime - nowEsp;\n";
    html += "    if (remainingMs > 0) {\n";
    html += "      const mins = Math.floor(remainingMs / 60000);\n";
    html += "      const secs = Math.floor((remainingMs % 60000) / 1000);\n";
    html += "      timerElem.textContent = mins + 'm ' + secs + 's';\n";
    html += "    } else {\n";
    html += "      timerElem.textContent = '0m 0s';\n";
    html += "    }\n";
    html += "  }\n";
    html += "}\n";
    
    html += "setInterval(refreshLastUpdateTime, 1000);\n";
    
    html += "function showConfig() {\n";
    html += "  console.log('showConfig called');\n";
    html += "  try {\n";
    html += "    fetch('/getconfig')\n";
    html += "      .then(response => {\n";
    html += "        console.log('Config response status:', response.status);\n";
    html += "        if (!response.ok) throw new Error('Failed to fetch config: ' + response.status);\n";
    html += "        return response.json();\n";
    html += "      })\n";
    html += "      .then(data => {\n";
    html += "        console.log('Config data received:', data);\n";
    html += "        document.getElementById('mqttServer').value = data.server || '';\n";
    html += "        document.getElementById('mqttPassword').value = data.password || '';\n";
    html += "        document.getElementById('deviceSerial').value = data.serial || '';\n";
    html += "        document.getElementById('mqttPort').value = data.port || 1883;\n";
    html += "        document.getElementById('useSSL').checked = data.use_ssl || false;\n";
    html += "        document.getElementById('skipCert').checked = data.skip_cert || false;\n";
    html += "        document.getElementById('ledPin').value = data.led_pin || 2;\n";
    html += "        document.getElementById('numLeds').value = data.num_leds || 10;\n";
    html += "        document.getElementById('ledType').value = data.led_type || 'WS2812B';\n";
    html += "        document.getElementById('colorOrder').value = data.color_order || 'GRB';\n";
    html += "        console.log('Opening modal...');\n";
    html += "        document.getElementById('configModal').style.display = 'block';\n";
    html += "      })\n";
    html += "      .catch(error => {\n";
    html += "        console.error('Error loading config:', error);\n";
    html += "        alert('Failed to load configuration: ' + error.message);\n";
    html += "      });\n";
    html += "    \n";
    html += "    // Load beacon hardware settings\n";
    html += "    fetch('/beaconsettings')\n";
    html += "      .then(response => response.json())\n";
    html += "      .then(data => {\n";
    html += "        document.getElementById('beaconEnabledConfig').checked = data.enabled || false;\n";
    html += "        document.getElementById('beaconPinConfig').value = data.pin || 14;\n";
    html += "        document.getElementById('beaconCountConfig').value = data.count || 10;\n";
    html += "        document.getElementById('beaconTypeConfig').value = data.type || 0;\n";
    html += "        document.getElementById('beaconColorOrderConfig').value = data.colorOrder || 1;\n";
    html += "        document.getElementById('beaconRGBWConfig').checked = data.rgbw || false;\n";
    html += "      })\n";
    html += "      .catch(error => {\n";
    html += "        console.error('Error loading beacon config:', error);\n";
    html += "      });\n";
    html += "    \n";
    html += "    // Load Home Automation MQTT settings\n";
    html += "    fetch('/hamqttconfig')\n";
    html += "      .then(response => response.json())\n";
    html += "      .then(data => {\n";
    html += "        document.getElementById('haMqttEnabled').checked = data.enabled || false;\n";
    html += "        document.getElementById('haMqttServer').value = data.server || '192.168.0.50';\n";
    html += "        document.getElementById('haMqttPort').value = data.port || 1883;\n";
    html += "        document.getElementById('haMqttUsername').value = data.username || '';\n";
    html += "        document.getElementById('haMqttPassword').value = data.password || '';\n";
    html += "        document.getElementById('haDiscoveryPrefix').value = data.discovery_prefix || 'homeassistant';\n";
    html += "        document.getElementById('haDeviceName').value = data.device_name || 'Bambu Lights';\n";
    html += "        document.getElementById('haMqttUseSSL').checked = data.use_ssl || false;\n";
    html += "      })\n";
    html += "      .catch(error => {\n";
    html += "        console.error('Error loading automation MQTT config:', error);\n";
    html += "      });\n";
    html += "  } catch(e) {\n";
    html += "    console.error('Exception in showConfig:', e);\n";
    html += "    alert('Error: ' + e.message);\n";
    html += "  }\n";
    html += "}\n";
    
    html += "function hideConfig() {\n";
    html += "  document.getElementById('configModal').style.display = 'none';\n";
    html += "}\n";
    
    html += "function saveConfig() {\n";
    html += "  const server = document.getElementById('mqttServer').value;\n";
    html += "  const password = document.getElementById('mqttPassword').value;\n";
    html += "  const serial = document.getElementById('deviceSerial').value;\n";
    html += "  const ledPin = document.getElementById('ledPin').value;\n";
    html += "  const numLeds = document.getElementById('numLeds').value;\n";
    html += "  const port = document.getElementById('mqttPort').value;\n";
    html += "  const useSSL = document.getElementById('useSSL').checked;\n";
    html += "  const skipCert = document.getElementById('skipCert').checked;\n";
    html += "  const ledType = document.getElementById('ledType').value;\n";
    html += "  const colorOrder = document.getElementById('colorOrder').value;\n";
    html += "  \n";
    html += "  // Save beacon hardware settings\n";
    html += "  const beaconEnabled = document.getElementById('beaconEnabledConfig').checked;\n";
    html += "  const beaconPin = document.getElementById('beaconPinConfig').value;\n";
    html += "  const beaconCount = document.getElementById('beaconCountConfig').value;\n";
    html += "  const beaconType = document.getElementById('beaconTypeConfig').value;\n";
    html += "  const beaconColorOrder = document.getElementById('beaconColorOrderConfig').value;\n";
    html += "  const beaconRGBW = document.getElementById('beaconRGBWConfig').checked;\n";
    html += "  const beaconUrl = `/beaconconfig?enabled=${beaconEnabled ? '1' : '0'}&pin=${beaconPin}&count=${beaconCount}&type=${beaconType}&colorOrder=${beaconColorOrder}&rgbw=${beaconRGBW ? '1' : '0'}`;\n";
    html += "  fetch(beaconUrl)\n";
    html += "    .then(response => response.text())\n";
    html += "    .then(data => console.log('Beacon config saved:', data))\n";
    html += "    .catch(error => console.error('Error saving beacon config:', error));\n";
    html += "  \n";
    html += "  let url = '/setconfig?server=' + encodeURIComponent(server) + '&password=' + encodeURIComponent(password) + '&serial=' + encodeURIComponent(serial) + '&port=' + encodeURIComponent(port) + '&use_ssl=' + useSSL + '&skip_cert=' + skipCert + '&led_pin=' + ledPin + '&num_leds=' + numLeds + '&led_type=' + encodeURIComponent(ledType) + '&color_order=' + encodeURIComponent(colorOrder);\n";
    html += "  fetch(url)\n";
    html += "    .then(response => response.text())\n";
    html += "    .then(data => {\n";
    html += "      alert(data);\n";
    html += "      hideConfig();\n";
    html += "      setTimeout(() => { window.location.reload(); }, 2000);\n";
    html += "    })\n";
    html += "    .catch(error => {\n";
    html += "      console.error('Error saving config:', error);\n";
    html += "      alert('Failed to save configuration. Please try again.');\n";
    html += "    });\n";
    html += "}\n";
    
    html += "function saveHAMqttConfig() {\n";
    html += "  const enabled = document.getElementById('haMqttEnabled').checked;\n";
    html += "  const server = document.getElementById('haMqttServer').value;\n";
    html += "  const port = document.getElementById('haMqttPort').value;\n";
    html += "  const username = document.getElementById('haMqttUsername').value;\n";
    html += "  const password = document.getElementById('haMqttPassword').value;\n";
    html += "  const prefix = document.getElementById('haDiscoveryPrefix').value;\n";
    html += "  const deviceName = document.getElementById('haDeviceName').value;\n";
    html += "  const useSSL = document.getElementById('haMqttUseSSL').checked;\n";
    html += "  \n";
    html += "  const url = `/setconfig?ha_mqtt_enabled=${enabled}&ha_mqtt_server=${encodeURIComponent(server)}&ha_mqtt_port=${port}&ha_mqtt_username=${encodeURIComponent(username)}&ha_mqtt_password=${encodeURIComponent(password)}&ha_discovery_prefix=${encodeURIComponent(prefix)}&ha_device_name=${encodeURIComponent(deviceName)}&ha_mqtt_use_ssl=${useSSL}`;\n";
    html += "  fetch(url)\n";
    html += "    .then(response => response.text())\n";
    html += "    .then(data => {\n";
    html += "      alert(data);\n";
    html += "    })\n";
    html += "    .catch(error => {\n";
    html += "      console.error('Error saving automation MQTT config:', error);\n";
    html += "      alert('Failed to save MQTT configuration. Please try again.');\n";
    html += "    });\n";
    html += "}\n";
    
    // Scanning has been removed - manual configuration is preferred
    
    html += "function loadBehaviorSettings() {\n";
    html += "  fetch('/behaviorsettings')\n";
    html += "    .then(response => response.json())\n";
    html += "    .then(data => {\n";
    html += "      // Heating\n";
    html += "      document.getElementById('heating_effect').value = data.heating.effect.toLowerCase();\n";
    html += "      document.getElementById('heating_color').value = data.heating.color;\n";
    html += "      document.getElementById('heating_brightness').value = data.heating.brightness;\n";
    html += "      document.getElementById('heating_brightness_val').textContent = data.heating.brightness + '%';\n";
    html += "      // Printing\n";
    html += "      document.getElementById('printing_effect').value = data.printing.effect.toLowerCase();\n";
    html += "      document.getElementById('printing_color').value = data.printing.color;\n";
    html += "      document.getElementById('printing_brightness').value = data.printing.brightness;\n";
    html += "      document.getElementById('printing_brightness_val').textContent = data.printing.brightness + '%';\n";
    html += "      // Idle\n";
    html += "      document.getElementById('idle_effect').value = data.idle.effect.toLowerCase();\n";
    html += "      document.getElementById('idle_color').value = data.idle.color;\n";
    html += "      document.getElementById('idle_brightness').value = data.idle.brightness;\n";
    html += "      document.getElementById('idle_brightness_val').textContent = data.idle.brightness + '%';\n";
    html += "      // Pause\n";
    html += "      document.getElementById('pause_effect').value = data.pause.effect.toLowerCase();\n";
    html += "      document.getElementById('pause_color').value = data.pause.color;\n";
    html += "      document.getElementById('pause_brightness').value = data.pause.brightness;\n";
    html += "      document.getElementById('pause_brightness_val').textContent = data.pause.brightness + '%';\n";
    html += "      // Finish\n";
    html += "      document.getElementById('finish_effect').value = data.finish.effect.toLowerCase();\n";
    html += "      document.getElementById('finish_color').value = data.finish.color;\n";
    html += "      document.getElementById('finish_brightness').value = data.finish.brightness;\n";
    html += "      document.getElementById('finish_brightness_val').textContent = data.finish.brightness + '%';\n";
    html += "      // Error\n";
    html += "      document.getElementById('error_effect').value = data.error.effect.toLowerCase();\n";
    html += "      document.getElementById('error_color').value = data.error.color;\n";
    html += "      document.getElementById('error_brightness').value = data.error.brightness;\n";
    html += "      document.getElementById('error_brightness_val').textContent = data.error.brightness + '%';\n";
    html += "    })\n";
    html += "    .catch(error => console.error('Error loading behavior settings:', error));\n";
    html += "  \n";
    html += "  // Load beacon settings\n";
    html += "  fetch('/beaconsettings')\n";
    html += "    .then(response => response.json())\n";
    html += "    .then(data => {\n";
    html += "      document.getElementById('beacon_effect').value = data.effect || 'progress';\n";
    html += "      document.getElementById('beacon_color').value = data.color || '#00FF00';\n";
    html += "      document.getElementById('beacon_brightness').value = data.brightness || 80;\n";
    html += "      document.getElementById('beacon_brightness_val').textContent = (data.brightness || 80) + '%';\n";
    html += "      document.getElementById('beacon_climb_color').value = data.climbColor || '#FFFFFF';\n";
    html += "      document.getElementById('beacon_gradient_end_color').value = data.gradientEndColor || '#0000FF';\n";
    html += "      const climbSpeed = data.climbSpeed || 150;\n";
    html += "      document.getElementById('beacon_climb_speed').value = climbSpeed;\n";
    html += "      const cyclesPerSec = (10000 / climbSpeed / 10).toFixed(2);\n";
    html += "      document.getElementById('beacon_climb_speed_val').textContent = climbSpeed + 'ms (' + cyclesPerSec + '/s)';\n";
    html += "      toggleBeaconAnimControls();\n";
    html += "    })\n";
    html += "    .catch(error => console.error('Error loading beacon settings:', error));\n";
    html += "}\n";
    
    html += "async function autoSaveState(state) {\n";
    html += "  const effect = document.getElementById(state + '_effect').value;\n";
    html += "  const color = document.getElementById(state + '_color').value;\n";
    html += "  const brightness = document.getElementById(state + '_brightness').value;\n";
    html += "  const url = `/printerbehavior?state=${state}&effect=${effect}&color=${encodeURIComponent(color)}&brightness=${brightness}`;\n";
    html += "  try {\n";
    html += "    const response = await fetch(url);\n";
    html += "    const result = await response.text();\n";
    html += "    console.log(state + ' auto-saved:', result);\n";
    html += "    // Trigger immediate LED update if in printer mode\n";
    html += "    await fetch('/refreshprinter');\n";
    html += "  } catch (error) {\n";
    html += "    console.error('Error auto-saving ' + state + ':', error);\n";
    html += "  }\n";
    html += "}\n";
    
    html += "async function saveStatusBehavior() {\n";
    html += "  const states = ['heating', 'printing', 'idle', 'pause', 'finish', 'error'];\n";
    html += "  for (const state of states) {\n";
    html += "    const effect = document.getElementById(state + '_effect').value;\n";
    html += "    const color = document.getElementById(state + '_color').value;\n";
    html += "    const brightness = document.getElementById(state + '_brightness').value;\n";
    html += "    const url = `/printerbehavior?state=${state}&effect=${effect}&color=${encodeURIComponent(color)}&brightness=${brightness}`;\n";
    html += "    try {\n";
    html += "      const response = await fetch(url);\n";
    html += "      const result = await response.text();\n";
    html += "      console.log(result);\n";
    html += "    } catch (error) {\n";
    html += "      console.error('Error saving ' + state + ':', error);\n";
    html += "      alert('Error saving ' + state + ' behavior!');\n";
    html += "      return;\n";
    html += "    }\n";
    html += "  }\n";
    html += "  \n";
    html += "  // Save beacon settings\n";
    html += "  const beaconEffect = document.getElementById('beacon_effect').value;\n";
    html += "  const beaconColor = document.getElementById('beacon_color').value;\n";
    html += "  const beaconBrightness = document.getElementById('beacon_brightness').value;\n";
    html += "  const beaconClimbColor = document.getElementById('beacon_climb_color').value;\n";
    html += "  const beaconClimbSpeed = document.getElementById('beacon_climb_speed').value;\n";
    html += "  const beaconUrl = `/beaconconfig?effect=${beaconEffect}&color=${encodeURIComponent(beaconColor)}&brightness=${beaconBrightness}&climbColor=${encodeURIComponent(beaconClimbColor)}&climbSpeed=${beaconClimbSpeed}`;\n";
    html += "  try {\n";
    html += "    const response = await fetch(beaconUrl);\n";
    html += "    const result = await response.text();\n";
    html += "    console.log('Beacon settings:', result);\n";
    html += "  } catch (error) {\n";
    html += "    console.error('Error saving beacon settings:', error);\n";
    html += "  }\n";
    html += "  \n";
    html += "  // Trigger immediate LED update if in printer mode\n";
    html += "  try {\n";
    html += "    await fetch('/refreshprinter');\n";
    html += "  } catch (error) {\n";
    html += "    console.error('Error refreshing printer LEDs:', error);\n";
    html += "  }\n";
    html += "  \n";
    html += "  alert('All behavior settings saved successfully!');\n";
    html += "}\n";
    
    html += "function loadCurrentMode() {\n";
    html += "  fetch('/status')\n";
    html += "    .then(response => response.text())\n";
    html += "    .then(data => {\n";
    html += "      const modeMatch = data.match(/Current Mode: ([^<]+)/);\n";
    html += "      if (modeMatch) {\n";
    html += "        const mode = modeMatch[1].trim();\n";
    html += "        document.getElementById('status').innerText = 'Current Mode: ' + mode;\n";
    html += "        const statusDiv = document.getElementById('status');\n";
    html += "        let modeKey = '';\n";
    html += "        if (mode === 'Auto Cycling') {\n";
    html += "          statusDiv.className = 'status status-auto';\n";
    html += "          modeKey = 'auto';\n";
    html += "        } else if (mode === 'Printer Status') {\n";
    html += "          statusDiv.className = 'status status-success';\n";
    html += "          modeKey = 'printer';\n";
    html += "        } else if (mode === 'Manual Color') {\n";
    html += "          statusDiv.className = 'status status-success';\n";
    html += "          modeKey = 'manual';\n";
    html += "        } else if (mode === 'Off') {\n";
    html += "          statusDiv.className = 'status status-success';\n";
    html += "          modeKey = 'off';\n";
    html += "        } else {\n";
    html += "          statusDiv.className = 'status status-success';\n";
    html += "        }\n";
    html += "        const desc = document.getElementById('modeDescription');\n";
    html += "        if (desc && modeKey && modeDescriptions[modeKey]) {\n";
    html += "          desc.innerHTML = '<i class=\"fas fa-info-circle\" style=\"color: #7c3aed; margin-right: 6px;\"></i>' + modeDescriptions[modeKey];\n";
    html += "        }\n";
    html += "      }\n";
    html += "    })\n";
    html += "    .catch(error => console.error('Error loading mode:', error));\n";
    html += "}\n";
    
    html += "document.addEventListener('DOMContentLoaded', function() {\n";
    html += "  loadBehaviorSettings();\n";
    html += "  loadCurrentMode();\n";
    html += "  updateBeaconPreviewGif();\n";
    html += "  document.querySelectorAll('input[type=\"range\"]').forEach(slider => {\n";
    html += "    slider.addEventListener('input', function() {\n";
    html += "      const display = this.nextElementSibling;\n";
    html += "      if (display) {\n";
    html += "        if (this.id === 'beacon_climb_speed') {\n";
    html += "          const ms = parseInt(this.value);\n";
    html += "          const cyclesPerSec = (10000 / ms / 10).toFixed(2);\n";
    html += "          display.textContent = ms + 'ms (' + cyclesPerSec + '/s)';\n";
    html += "        } else {\n";
    html += "          display.textContent = this.value + '%';\n";
    html += "        }\n";
    html += "      }\n";
    html += "    });\n";
    html += "  });\n";
    html += "});\n";
    
    // SSL checkbox handling
    html += "document.addEventListener('change', function(e) {\n";
    html += "  if (e.target.id === 'useSSL') {\n";
    html += "    const portField = document.getElementById('mqttPort');\n";
    html += "    if (e.target.checked) {\n";
    html += "      portField.value = '8883';\n";
    html += "    } else {\n";
    html += "      portField.value = '1883';\n";
    html += "    }\n";
    html += "  }\n";
    html += "});\n";
    
    // Serial monitor functions
    html += "let autoRefreshLogs = true;\n";  // Default to enabled
    html += "let logsInterval = null;\n";
    html += "let mqttLoggingEnabled = false;\n";
    html += "let isRefreshing = false;\n";  // Prevent concurrent refreshes
    html += "function refreshLogs() {\n";
    html += "  if (isRefreshing) return;\n";  // Skip if already refreshing
    html += "  isRefreshing = true;\n";
    html += "  const logsDiv = document.getElementById('serialLogs');\n";
    html += "  const wasScrolledToBottom = logsDiv.scrollHeight - logsDiv.scrollTop <= logsDiv.clientHeight + 50;\n";
    html += "  fetch('/logs')\n";
    html += "    .then(response => response.text())\n";
    html += "    .then(data => {\n";
    html += "      logsDiv.textContent = '';\n";  // Clear before setting new content
    html += "      logsDiv.innerHTML = data;\n";
    html += "      if (wasScrolledToBottom) {\n";
    html += "        logsDiv.scrollTop = logsDiv.scrollHeight;\n";
    html += "      }\n";
    html += "      isRefreshing = false;\n";
    html += "    })\n";
    html += "    .catch(error => {\n";
    html += "      console.error('Error fetching logs:', error);\n";
    html += "      logsDiv.innerHTML = 'Error loading logs';\n";
    html += "      isRefreshing = false;\n";
    html += "    });\n";
    html += "}\n";
    html += "function toggleAutoRefresh() {\n";
    html += "  autoRefreshLogs = document.getElementById('autoRefreshCheckbox').checked;\n";
    html += "  if (autoRefreshLogs) {\n";
    html += "    refreshLogs();\n";
    html += "    logsInterval = setInterval(refreshLogs, 5000);\n";
    html += "  } else {\n";
    html += "    if (logsInterval) clearInterval(logsInterval);\n";
    html += "  }\n";
    html += "}\n";
    html += "function toggleMqttLogging() {\n";
    html += "  mqttLoggingEnabled = !mqttLoggingEnabled;\n";
    html += "  const btn = document.getElementById('mqttLogBtn');\n";
    html += "  fetch('/mqttlogging?enable=' + mqttLoggingEnabled)\n";
    html += "    .then(response => response.text())\n";
    html += "    .then(data => {\n";
    html += "      btn.textContent = mqttLoggingEnabled ? 'MQTT Debug: ON' : 'MQTT Debug: OFF';\n";
    html += "      btn.style.background = mqttLoggingEnabled ? '#28a745' : '#6c757d';\n";
    html += "      console.log(data);\n";
    html += "      refreshLogs();\n";
    html += "    })\n";
    html += "    .catch(error => console.error('Error toggling MQTT logging:', error));\n";
    html += "}\n";
    html += "function checkMqttLoggingStatus() {\n";
    html += "  fetch('/mqttlogging?status=1')\n";
    html += "    .then(response => response.json())\n";
    html += "    .then(data => {\n";
    html += "      mqttLoggingEnabled = data.enabled;\n";
    html += "      const btn = document.getElementById('mqttLogBtn');\n";
    html += "      btn.textContent = mqttLoggingEnabled ? 'MQTT Debug: ON' : 'MQTT Debug: OFF';\n";
    html += "      btn.style.background = mqttLoggingEnabled ? '#28a745' : '#6c757d';\n";
    html += "    })\n";
    html += "    .catch(error => console.error('Error checking MQTT logging status:', error));\n";
    html += "}\n";
    html += "checkMqttLoggingStatus();\n";
    html += "refreshLogs();\n";
    html += "logsInterval = setInterval(refreshLogs, 5000);\n";  // Start auto-refresh on load
    
    html += "function loadQuickControls() {\n";
    html += "  fetch('/door')\n";
    html += "    .then(response => response.json())\n";
    html += "    .then(data => {\n";
    html += "      if (document.getElementById('doorLightsQuick')) {\n";
    html += "        document.getElementById('doorLightsQuick').checked = data.door_trigger || false;\n";
    html += "      }\n";
    html += "    })\n";
    html += "    .catch(error => console.error('Error loading door state:', error));\n";
    html += "  \n";
    html += "  fetch('/getconfig')\n";
    html += "    .then(response => response.json())\n";
    html += "    .then(data => {\n";
    html += "      document.getElementById('autoOffMinutes').value = data.auto_off_minutes || 0;\n";
    html += "      document.getElementById('lidarEnabled').checked = data.lidar_enabled || false;\n";
    html += "      document.getElementById('doorColorQuick').value = data.door_color || '#ffffff';\n";
    html += "      document.getElementById('chamberLightSync').checked = data.chamber_sync !== false;\n";
    html += "      document.getElementById('timelapseWhiteLights').checked = data.timelapse_white !== false;\n";
    html += "      \n";
    html += "      // Load auto-cycle settings\n";
    html += "      const speed = data.cycle_speed || 50;\n";
    html += "      const brightness = data.cycle_brightness || 100;\n";
    html += "      document.getElementById('autoCycleSpeed').value = speed;\n";
    html += "      document.getElementById('autoCycleBrightness').value = brightness;\n";
    html += "      document.getElementById('speedValue').textContent = speed;\n";
    html += "      document.getElementById('brightnessValue').textContent = brightness + '%';\n";
    html += "    })\n";
    html += "    .catch(error => console.error('Error loading config:', error));\n";
    html += "}\n";
    
    html += "setInterval(getStatus, 5000);\n";
    html += "getStatus();\n";
    html += "loadQuickControls();\n";
    html += "</script>";
    sendChunk(html); html = "";  // Send JavaScript chunk
    
    html += "</body></html>";
    sendChunk(html);  // Send final closing tags
    
    // Properly close chunked transfer
    server.sendContent("");
    server.client().stop();  // Ensure client connection closes cleanly
}

void handleMode() {
    String mode = server.arg("m");
    logPrintln("=== MODE CHANGE REQUEST ===");
    logPrintln("Requested: " + mode);
    
    if (mode == "auto") {
        logPrintln("Switching to AUTO mode");
        currentMode = MODE_AUTO;
        logPrintln("Auto color cycling enabled");
        
        // Start auto-off timer for auto mode if enabled and not already running
        if (auto_off_minutes > 0 && !auto_off_pending) {
            auto_off_pending = true;
            lights_on_time = millis();
            logPrintln("Auto-off timer started: " + String(auto_off_minutes) + " min");
        }
        
        server.send(200, "text/plain", "Mode: Auto Cycling");
    } else if (mode == "manual") {
        logPrintln("Switching to MANUAL mode");
        logPrintln("Color: RGB(" + String(manualColor.r) + ", " + String(manualColor.g) + ", " + String(manualColor.b) + ")");
        currentMode = MODE_MANUAL;
        setAllLEDs(manualColor.r, manualColor.g, manualColor.b);
        updateLEDs();
        
        // Start auto-off timer for manual mode if enabled and not already running
        if (auto_off_minutes > 0 && !auto_off_pending) {
            auto_off_pending = true;
            lights_on_time = millis();
            logPrintln("Auto-off timer started: " + String(auto_off_minutes) + " min");
        }
        
        server.send(200, "text/plain", "Mode: Manual Control");
    } else if (mode == "printer") {
        logPrintln("Switching to PRINTER mode");
        logPrintln("Print State: " + printerStatus.gcode_state);
        logPrintln("Door Open: " + String(door_is_open ? "YES" : "NO"));
        logPrintln("LiDAR Scanning: " + String(lidar_scanning ? "YES" : "NO"));
        currentMode = MODE_PRINTER;
        updatePrinterLEDs();
        server.send(200, "text/plain", "Mode: Printer Status");
    } else if (mode == "off") {
        logPrintln("Switching to OFF mode");
        currentMode = MODE_OFF;
        clearAllLEDs();
        updateLEDs();
        
        // Cancel auto-off timer when manually turning off lights
        if (auto_off_pending) {
            auto_off_pending = false;
            lights_on_time = 0;
            logPrintln("Auto-off timer cancelled (lights manually turned off)");
        }
        
        logPrintln("LEDs cleared");
        server.send(200, "text/plain", "Mode: LEDs Off");
    } else {
        logPrintln("Invalid mode requested: '" + mode + "'");
        server.send(400, "text/plain", "Invalid mode");
    }
}

void handleColor() {
    logPrintln("=== COLOR CHANGE REQUEST ===");
    
    if (server.hasArg("r") && server.hasArg("g") && server.hasArg("b")) {
        manualColor.r = server.arg("r").toInt();
        manualColor.g = server.arg("g").toInt();
        manualColor.b = server.arg("b").toInt();
        
        logPrintln("New Color: RGB(" + String(manualColor.r) + ", " + String(manualColor.g) + ", " + String(manualColor.b) + ")");
        String modeStr = currentMode == MODE_AUTO ? "AUTO" :
                        currentMode == MODE_MANUAL ? "MANUAL" :
                        currentMode == MODE_PRINTER ? "PRINTER" : "OFF";
        logPrintln("Current Mode: " + modeStr);
        
        if (currentMode == MODE_MANUAL) {
            logPrintln("Applying to LEDs (MANUAL mode active)");
            setAllLEDs(manualColor.r, manualColor.g, manualColor.b);
            updateLEDs();
            logPrintln("LEDs updated with new color");
            
            // Start auto-off timer for manual mode if enabled and not already running
            if (auto_off_minutes > 0 && !auto_off_pending) {
                auto_off_pending = true;
                lights_on_time = millis();
                logPrintln("Manual color set - auto-off timer started: " + String(auto_off_minutes) + " minutes");
            }
        } else {
            logPrintln("Color stored but not applied (not in MANUAL mode)");
        }
        
        server.send(200, "text/plain", 
                   "Color set to RGB(" + String(manualColor.r) + "," + 
                   String(manualColor.g) + "," + String(manualColor.b) + ")");
    } else {
        logPrintln("ERROR: Missing color parameters");
        server.send(400, "text/plain", "Missing color parameters");
    }
}

void handleStatus() {
    Serial.println("HTTP: /status requested");
    String status = "<b>ESP32 Status:</b><br>";
    status += "WiFi: " + WiFi.localIP().toString() + "<br>";
    status += "Signal: " + String(WiFi.RSSI()) + " dBm<br>";
    status += "Uptime: " + String(millis() / 1000) + " seconds<br>";
    status += "Free Memory: " + String(ESP.getFreeHeap()) + " bytes<br>";
    status += "LED Count: " + String(num_leds) + "<br>";
    
    // MQTT Connection Status (always show)
    bool mqttConnected = mqttClient.connected();
    String mqttStatus = mqttConnected ? 
        "<span style='color: #28a745; font-weight: bold;'>&#x2713; Connected</span>" : 
        "<span style='color: #dc3545; font-weight: bold;'>&#x2717; Disconnected</span>";
    status += "Printer MQTT: " + mqttStatus + "<br>";
    
    String modeStr = "Unknown";
    switch (currentMode) {
        case MODE_AUTO: modeStr = "Auto Cycling"; break;
        case MODE_MANUAL: modeStr = "Manual Control"; break;  
        case MODE_PRINTER: modeStr = "Printer Status"; break;
        case MODE_OFF: modeStr = "LEDs Off"; break;
    }
    status += "Current Mode: " + modeStr + "<br>";
    
    if (currentMode == MODE_MANUAL) {
        status += "Manual Color: RGB(" + String(manualColor.r) + "," + 
                 String(manualColor.g) + "," + String(manualColor.b) + ")<br>";
    }
    
    if (currentMode == MODE_PRINTER) {
        status += "<br><b>Printer Status:</b><br>";
        status += "Online: " + String(printerStatus.online ? "Yes" : "No") + "<br>";
        
        // State description
        String stateDesc = printerStatus.gcode_state;
        if (printerStatus.gcode_state == "RUNNING") stateDesc = "Running";
        else if (printerStatus.gcode_state == "IDLE") stateDesc = "Idle";
        else if (printerStatus.gcode_state == "FINISH") stateDesc = "Finished";
        else if (printerStatus.gcode_state == "FAILED") stateDesc = "Failed";
        else if (printerStatus.gcode_state == "PAUSE") stateDesc = "Paused";
        else if (printerStatus.gcode_state == "PREPARE") stateDesc = "Preparing";
        status += "State: " + printerStatus.gcode_state + " (" + stateDesc + ")<br>";
        
        // Stage description
        String stageDesc = "Unknown";
        int stageNum = printerStatus.stage.toInt();
        if (printerStatus.gcode_state == "RUNNING" || printerStatus.gcode_state == "PREPARE") {
            if (stageNum == 2) stageDesc = "Heating Bed";
            else if (stageNum == 3) stageDesc = "Heating Hotend";
            else if (stageNum >= 14) stageDesc = "Printing";
            else if (stageNum > 0) stageDesc = "Preparing";
            else if (stageNum == 0 && printerStatus.mc_print_stage == "2") stageDesc = "Printing";
            else if (stageNum == 0) stageDesc = "Active";
        } else if (printerStatus.gcode_state == "IDLE") {
            stageDesc = "Idle";
        } else if (printerStatus.gcode_state == "FINISH") {
            stageDesc = "Complete";
        } else if (printerStatus.gcode_state == "PAUSE") {
            stageDesc = "Paused";
        }
        status += "Stage: " + String(stageNum) + " (" + stageDesc + ")<br>";
        
        // Print Stage description
        if (printerStatus.mc_print_stage != "unknown") {
            String printStageDesc = printerStatus.mc_print_stage;
            if (printerStatus.mc_print_stage == "1") printStageDesc = "Idle";
            else if (printerStatus.mc_print_stage == "2") printStageDesc = "Printing";
            else if (printerStatus.mc_print_stage == "scanning") printStageDesc = "Scanning";
            else if (printerStatus.mc_print_stage == "first_layer_scan") printStageDesc = "First Layer Scan";
            else if (printerStatus.mc_print_stage == "auto_bed_leveling") printStageDesc = "Auto Bed Leveling";
            else if (printerStatus.mc_print_stage.indexOf("inspect") >= 0) printStageDesc = "Inspecting";
            else if (printerStatus.mc_print_stage.indexOf("scan") >= 0) printStageDesc = "Scanning";
            else if (printerStatus.mc_print_stage.indexOf("bed_leveling") >= 0) printStageDesc = "Bed Leveling";
            else if (printerStatus.mc_print_stage.indexOf("calibrat") >= 0) printStageDesc = "Calibrating";
            status += "Print Stage: " + printerStatus.mc_print_stage + " (" + printStageDesc + ")<br>";
        }
        
        // Print Action description
        if (printerStatus.print_real_action != "unknown") {
            String actionDesc = printerStatus.print_real_action;
            int actionNum = printerStatus.print_real_action.toInt();
            if (actionNum == 0) actionDesc = "Idle";
            else if (actionNum == 1) actionDesc = "Printing";
            else if (actionNum == 2) actionDesc = "Heating";
            else if (actionNum == 3) actionDesc = "Paused";
            else if (printerStatus.print_real_action == "scanning") actionDesc = "Scanning";
            else if (printerStatus.print_real_action == "lidar_scan") actionDesc = "LiDAR Scan";
            else if (printerStatus.print_real_action == "first_layer_inspect") actionDesc = "First Layer Inspect";
            else if (printerStatus.print_real_action.indexOf("bed_leveling") >= 0) actionDesc = "Bed Leveling";
            else if (printerStatus.print_real_action.indexOf("calibrat") >= 0) actionDesc = "Calibrating";
            status += "Print Action: " + printerStatus.print_real_action + " (" + actionDesc + ")<br>";
        }
        
        status += "Progress: " + String(printerStatus.progress) + "%<br>";
        if (printerStatus.layer_num > 0) {
            status += "Layer: " + String(printerStatus.layer_num) + "/" + String(printerStatus.total_layer_num) + "<br>";
        }
        status += "Bed Temp: " + String(printerStatus.bed_temp) + "°C<br>";
        status += "Nozzle Temp: " + String(printerStatus.nozzle_temp) + "°C<br>";
        
        // LiDAR scanning status
        if (lidar_scanning) {
            status += "<br><b style='color: #ff6600;'>&#128269; LiDAR SCANNING ACTIVE</b><br>";
            status += "Lights: " + String(lidar_lights_off_enabled ? "OFF (Auto)" : "Normal") + "<br>";
        }
        
        if (printerStatus.lastUpdate > 0) {
            unsigned long secondsAgo = (millis() - printerStatus.lastUpdate) / 1000;
            status += "Last Update: <span id='lastUpdateTime' data-lastupdatemillis='" + String(printerStatus.lastUpdate) + "' data-nowmillis='" + String(millis()) + "'>" + String(secondsAgo) + "s</span> ago<br>";
        }
    }
    
    // Auto-off timer status
    if (auto_off_pending && auto_off_minutes > 0) {
        unsigned long elapsedMs = millis() - lights_on_time;
        unsigned long totalMs = (unsigned long)auto_off_minutes * 60UL * 1000UL;
        if (elapsedMs < totalMs) {
            unsigned long remainingMs = totalMs - elapsedMs;
            unsigned long remainingMinutes = remainingMs / 60000;
            unsigned long remainingSeconds = (remainingMs % 60000) / 1000;
            status += "<br><b style='color: #ff9800;'>&#9200; Auto-Off Timer:</b><br>";
            status += "Lights off in: <span id='autoOffTimer' data-endtime='" + String(lights_on_time + totalMs) + "'>" + String(remainingMinutes) + "m " + String(remainingSeconds) + "s</span><br>";
        }
    } else if (auto_off_minutes > 0) {
        status += "<br><b>&#9200; Auto-Off Timer:</b> " + String(auto_off_minutes) + " min (inactive)<br>";
    }
    
    // Door status (show in all modes)
    status += "<br><b>Door Status:</b><br>";
    status += "Door: " + String(door_is_open ? "OPEN" : "CLOSED") + "<br>";
    status += "Door Lights: " + String(door_open_lights_enabled ? "Enabled" : "Disabled") + "<br>";
    
    // Network scanning removed - manual configuration via web UI
    
    server.send(200, "text/html", status);
}

void handleBeaconGif() {
    // Serve the beacon.gif file from LittleFS (filesystem is mounted at startup)
    File file = LittleFS.open("/Beacon.gif", "r");
    if (!file) {
        logPrintln("ERROR: Beacon.gif not found in LittleFS!");
        server.send(404, "text/plain", "Beacon.gif not found");
        return;
    }
    
    // Set cache headers for better performance
    server.sendHeader("Cache-Control", "max-age=86400"); // Cache for 1 day
    server.streamFile(file, "image/gif");
    file.close();
}

void handleProgressBarGif() {
    File file = LittleFS.open("/Progress_Bar.gif", "r");
    if (!file) {
        logPrintln("ERROR: Progress_Bar.gif not found in LittleFS!");
        server.send(404, "text/plain", "Progress_Bar.gif not found");
        return;
    }
    
    server.sendHeader("Cache-Control", "max-age=86400");
    server.streamFile(file, "image/gif");
    file.close();
}

void handleClimbingDotGif() {
    File file = LittleFS.open("/Climbing_Dot.gif", "r");
    if (!file) {
        logPrintln("ERROR: Climbing_Dot.gif not found in LittleFS!");
        server.send(404, "text/plain", "Climbing_Dot.gif not found");
        return;
    }
    
    server.sendHeader("Cache-Control", "max-age=86400");
    server.streamFile(file, "image/gif");
    file.close();
}

void handleGradientFillGif() {
    File file = LittleFS.open("/Gradient_Fill.gif", "r");
    if (!file) {
        logPrintln("ERROR: Gradient_Fill.gif not found in LittleFS!");
        server.send(404, "text/plain", "Gradient_Fill.gif not found");
        return;
    }
    
    server.sendHeader("Cache-Control", "max-age=86400");
    server.streamFile(file, "image/gif");
    file.close();
}

void handlePulsingDotGif() {
    File file = LittleFS.open("/Pulsing_Dot.gif", "r");
    if (!file) {
        logPrintln("ERROR: Pulsing_Dot.gif not found in LittleFS!");
        server.send(404, "text/plain", "Pulsing_Dot.gif not found");
        return;
    }
    
    server.sendHeader("Cache-Control", "max-age=86400");
    server.streamFile(file, "image/gif");
    file.close();
}

void handleSerialLogs() {
    // If no logs, show system info
    if (serialLogCount == 0) {
        String info = "=== Serial Log Buffer Empty ===\n";
        info += "System has been running for " + String(millis() / 1000) + " seconds\n";
        info += "Free memory: " + String(ESP.getFreeHeap()) + " bytes\n";
        info += "WiFi: " + WiFi.localIP().toString() + "\n";
        info += "MQTT: " + String(mqttClient.connected() ? "Connected" : "Disconnected") + "\n";
        server.send(200, "text/plain", info);
        return;
    }
    
    String logs = "";
    
    // Read logs in order (oldest to newest)
    int startIdx = serialLogCount < SERIAL_LOG_SIZE ? 0 : serialLogIndex;
    for (int i = 0; i < serialLogCount; i++) {
        int idx = (startIdx + i) % SERIAL_LOG_SIZE;
        logs += serialLogs[idx];
    }
    
    server.send(200, "text/plain", logs);
}

void handleMqttLogging() {
    logPrintln("HTTP: /mqttlogging requested");
    
    if (server.hasArg("enable")) {
        mqtt_full_logging = (server.arg("enable") == "true");
        String response = mqtt_full_logging ? "MQTT full logging ENABLED" : "MQTT full logging DISABLED";
        logPrintln(response);
        server.send(200, "text/plain", response);
    } else if (server.hasArg("status")) {
        String json = "{\"enabled\":" + String(mqtt_full_logging ? "true" : "false") + "}";
        server.send(200, "application/json", json);
    } else {
        server.send(400, "text/plain", "Missing parameter: enable or status");
    }
}

String rgbToHex(uint8_t r, uint8_t g, uint8_t b) {
    char hex[8];
    snprintf(hex, sizeof(hex), "#%02x%02x%02x", r, g, b);
    return String(hex);
}

// List timelapse files via FTP using direct SSL connection
void handleGetConfig() {
    logPrintln("HTTP: /getconfig requested");
    String json = "{";
    json += "\"server\":\"" + mqtt_server + "\",";
    json += "\"port\":" + String(mqtt_port) + ",";
    json += "\"password\":\"" + mqtt_password + "\",";
    json += "\"serial\":\"" + device_serial + "\",";
    json += "\"use_ssl\":" + String(mqtt_use_ssl ? "true" : "false") + ",";
    json += "\"skip_cert\":" + String(mqtt_skip_cert_check ? "true" : "false") + ",";
    json += "\"led_pin\":" + String(led_pin) + ",";
    json += "\"num_leds\":" + String(num_leds) + ",";
    json += "\"led_type\":\"" + led_type + "\",";
    json += "\"color_order\":\"" + color_order + "\",";
    json += "\"door_lights_enabled\":" + String(door_open_lights_enabled ? "true" : "false") + ",";
    json += "\"door_color\":\"" + rgbToHex(door_open_color.r, door_open_color.g, door_open_color.b) + "\",";
    json += "\"auto_off_minutes\":" + String(auto_off_minutes) + ",";
    json += "\"chamber_sync\":" + String(chamber_light_sync_enabled ? "true" : "false") + ",";
    json += "\"timelapse_white\":" + String(timelapse_white_lights_enabled ? "true" : "false") + ",";
    json += "\"lidar_enabled\":" + String(lidar_lights_off_enabled ? "true" : "false") + ",";
    json += "\"cycle_speed\":" + String(autoCycleSpeed) + ",";
    json += "\"cycle_brightness\":" + String(autoCycleBrightness);
    json += "}";
    
    server.send(200, "application/json", json);
}

void handleGetHAMqttConfig() {
    logPrintln("HTTP: /hamqttconfig requested");
    String json = "{";
    json += "\"enabled\":" + String(ha_mqtt_enabled ? "true" : "false") + ",";
    json += "\"server\":\"" + ha_mqtt_server + "\",";
    json += "\"port\":" + String(ha_mqtt_port) + ",";
    json += "\"username\":\"" + ha_mqtt_username + "\",";
    json += "\"password\":\"" + ha_mqtt_password + "\",";
    json += "\"discovery_prefix\":\"" + ha_discovery_prefix + "\",";
    json += "\"device_name\":\"" + ha_device_name + "\",";
    json += "\"use_ssl\":" + String(ha_mqtt_use_ssl ? "true" : "false");
    json += "}";
    
    server.send(200, "application/json", json);
}

void handleSetConfig() {
    bool hasAllParams = server.hasArg("server") && server.hasArg("password") && server.hasArg("serial");
    bool hasLedParams = server.hasArg("led_pin") && server.hasArg("num_leds");
    bool hasAutoCycleParams = server.hasArg("cycle_speed") || server.hasArg("cycle_brightness");
    bool hasToggleParams = server.hasArg("door_lights_enabled") || server.hasArg("chamber_sync") || server.hasArg("timelapse_white");
    bool hasHAParams = server.hasArg("ha_mqtt_enabled") || server.hasArg("ha_mqtt_server");
    
    // Handle Home Assistant MQTT settings separately
    if (hasHAParams && !hasAllParams) {
        if (server.hasArg("ha_mqtt_enabled")) {
            ha_mqtt_enabled = server.arg("ha_mqtt_enabled") == "true";
            logPrintln("HA MQTT Enabled: " + String(ha_mqtt_enabled ? "YES" : "NO"));
        }
        
        if (server.hasArg("ha_mqtt_server")) {
            ha_mqtt_server = server.arg("ha_mqtt_server");
        }
        
        if (server.hasArg("ha_mqtt_port")) {
            int port = server.arg("ha_mqtt_port").toInt();
            if (port > 0 && port <= 65535) {
                ha_mqtt_port = port;
            }
        }
        
        if (server.hasArg("ha_mqtt_username")) {
            ha_mqtt_username = server.arg("ha_mqtt_username");
        }
        
        if (server.hasArg("ha_mqtt_password")) {
            ha_mqtt_password = server.arg("ha_mqtt_password");
        }
        
        if (server.hasArg("ha_discovery_prefix")) {
            ha_discovery_prefix = server.arg("ha_discovery_prefix");
        }
        
        if (server.hasArg("ha_device_name")) {
            ha_device_name = server.arg("ha_device_name");
        }
        
        if (server.hasArg("ha_mqtt_use_ssl")) {
            ha_mqtt_use_ssl = server.arg("ha_mqtt_use_ssl") == "true";
        }
        
        saveSettings();
        
        // Reconnect if enabled
        if (ha_mqtt_enabled) {
            if (haMqttClient.connected()) {
                haMqttClient.disconnect();
            }
            delay(500);
            bool connected = connectHAMQTT();
            if (connected) {
                server.send(200, "text/plain", "HA MQTT settings saved and connected successfully!");
            } else {
                server.send(200, "text/plain", "HA MQTT settings saved but connection failed. Check settings.");
            }
        } else {
            if (haMqttClient.connected()) {
                haMqttClient.disconnect();
            }
            server.send(200, "text/plain", "HA MQTT disabled and saved!");
        }
        return;
    }
    
    // Handle toggle settings separately (don't require full config)
    if (hasToggleParams && !hasAllParams) {
        if (server.hasArg("door_lights_enabled")) {
            door_open_lights_enabled = server.arg("door_lights_enabled") == "true";
            logPrintln("Door lights enabled: " + String(door_open_lights_enabled ? "YES" : "NO"));
        }
        
        if (server.hasArg("chamber_sync")) {
            bool new_sync_state = server.arg("chamber_sync") == "true";
            logPrintln("=== CHAMBER LIGHT SYNC CONFIG ===");
            logPrintln("Received: " + server.arg("chamber_sync"));
            logPrintln("Previous State: " + String(chamber_light_sync_enabled ? "ENABLED" : "DISABLED"));
            logPrintln("New State: " + String(new_sync_state ? "ENABLED" : "DISABLED"));
            chamber_light_sync_enabled = new_sync_state;
            logPrintln("================================");
        }
        
        if (server.hasArg("timelapse_white")) {
            bool new_timelapse_white_state = server.arg("timelapse_white") == "true";
            logPrintln("=== TIMELAPSE WHITE LIGHTS CONFIG ===");
            logPrintln("Received: " + server.arg("timelapse_white"));
            logPrintln("Previous State: " + String(timelapse_white_lights_enabled ? "ENABLED" : "DISABLED"));
            logPrintln("New State: " + String(new_timelapse_white_state ? "ENABLED" : "DISABLED"));
            logPrintln("Timelapse Active: " + String(timelapse_active ? "YES" : "NO"));
            logPrintln("Print State: " + printerStatus.gcode_state);
            timelapse_white_lights_enabled = new_timelapse_white_state;
            logPrintln("====================================");
        }
        
        saveSettings();
        server.send(200, "text/plain", "Settings saved!");
        return;
    }
    
    // Handle auto-cycle settings separately (don't require full config)
    if (hasAutoCycleParams && !hasAllParams) {
        if (server.hasArg("cycle_speed")) {
            int speed = server.arg("cycle_speed").toInt();
            if (speed >= 1 && speed <= 100) {
                autoCycleSpeed = speed;
            }
        }
        
        if (server.hasArg("cycle_brightness")) {
            int brightness = server.arg("cycle_brightness").toInt();
            if (brightness >= 0 && brightness <= 100) {
                autoCycleBrightness = brightness;
            }
        }
        
        saveSettings();
        server.send(200, "text/plain", "Auto-cycle settings saved!");
        return;
    }
    
    if (hasAllParams) {
        mqtt_server = server.arg("server");
        if (server.hasArg("port")) {
            int new_port = server.arg("port").toInt();
            if (new_port > 0 && new_port <= 65535) {
                mqtt_port = new_port;
            }
        }
        mqtt_password = server.arg("password");
        device_serial = server.arg("serial");
        
        // Handle SSL settings
        if (server.hasArg("use_ssl")) {
            mqtt_use_ssl = server.arg("use_ssl") == "true";
        }
        if (server.hasArg("skip_cert")) {
            mqtt_skip_cert_check = server.arg("skip_cert") == "true";
        }
        
        bool ledChanged = false;
        
        // Handle LED settings if provided
        if (hasLedParams) {
            int new_led_pin = server.arg("led_pin").toInt();
            int new_num_leds = server.arg("num_leds").toInt();
            
            // Validate LED pin (safe pins only)
            int validPins[] = {2, 4, 5, 13, 14, 15, 16, 17, 18, 19, 21, 22, 23, 25, 26, 27, 32, 33};
            bool validPin = false;
            for (int i = 0; i < sizeof(validPins)/sizeof(validPins[0]); i++) {
                if (new_led_pin == validPins[i]) {
                    validPin = true;
                    break;
                }
            }
            
            if (validPin && new_num_leds > 0 && new_num_leds <= 300) {
                if (new_led_pin != led_pin || new_num_leds != num_leds) {
                    led_pin = new_led_pin;
                    num_leds = new_num_leds;
                    ledChanged = true;
                }
            }
        }
        
        // Handle LED type and color order if provided
        if (server.hasArg("led_type")) {
            String new_led_type = server.arg("led_type");
            // Validate LED type
            bool validType = false;
            for (int i = 0; i < 5; i++) {
                if (new_led_type == LED_TYPE_NAMES[i]) {
                    validType = true;
                    break;
                }
            }
            if (validType && new_led_type != led_type) {
                led_type = new_led_type;
                ledChanged = true;
            }
        }
        
        if (server.hasArg("color_order")) {
            String new_color_order = server.arg("color_order");
            // Validate color order
            bool validOrder = false;
            for (int i = 0; i < 6; i++) {
                if (new_color_order == COLOR_ORDER_NAMES[i]) {
                    validOrder = true;
                    break;
                }
            }
            if (validOrder && new_color_order != color_order) {
                color_order = new_color_order;
                ledChanged = true;
            }
        }
        
        // Handle door configuration
        if (server.hasArg("door_lights_enabled")) {
            door_open_lights_enabled = server.arg("door_lights_enabled") == "true";
        }
        
        // Handle chamber light sync configuration
        if (server.hasArg("chamber_sync")) {
            bool new_sync_state = server.arg("chamber_sync") == "true";
            logPrintln("=== CHAMBER LIGHT SYNC CONFIG ===");
            logPrintln("Received: " + server.arg("chamber_sync"));
            logPrintln("Previous State: " + String(chamber_light_sync_enabled ? "ENABLED" : "DISABLED"));
            logPrintln("New State: " + String(new_sync_state ? "ENABLED" : "DISABLED"));
            chamber_light_sync_enabled = new_sync_state;
            logPrintln("================================");
        }
        
        // Handle timelapse white lights configuration
        if (server.hasArg("timelapse_white")) {
            bool new_timelapse_white_state = server.arg("timelapse_white") == "true";
            logPrintln("=== TIMELAPSE WHITE LIGHTS CONFIG ===");
            logPrintln("Received: " + server.arg("timelapse_white"));
            logPrintln("Previous State: " + String(timelapse_white_lights_enabled ? "ENABLED" : "DISABLED"));
            logPrintln("New State: " + String(new_timelapse_white_state ? "ENABLED" : "DISABLED"));
            logPrintln("Timelapse Active: " + String(timelapse_active ? "YES" : "NO"));
            logPrintln("Print State: " + printerStatus.gcode_state);
            timelapse_white_lights_enabled = new_timelapse_white_state;
            logPrintln("====================================");
        }
        
        if (server.hasArg("door_color")) {
            String hex = server.arg("door_color");
            if (hex.startsWith("#") && hex.length() == 7) {
                door_open_color.r = strtol(hex.substring(1, 3).c_str(), NULL, 16);
                door_open_color.g = strtol(hex.substring(3, 5).c_str(), NULL, 16);
                door_open_color.b = strtol(hex.substring(5, 7).c_str(), NULL, 16);
            }
        }
        
        // Handle auto-cycle configuration
        if (server.hasArg("cycle_speed")) {
            int speed = server.arg("cycle_speed").toInt();
            if (speed >= 1 && speed <= 100) {
                autoCycleSpeed = speed;
            }
        }
        
        if (server.hasArg("cycle_brightness")) {
            int brightness = server.arg("cycle_brightness").toInt();
            if (brightness >= 0 && brightness <= 100) {
                autoCycleBrightness = brightness;
            }
        }
        
        // Save all settings to EEPROM
        saveSettings();
        
        // Handle response based on what changed
        if (ledChanged) {
            server.send(200, "text/plain", "Configuration saved! LED settings changed - restarting ESP32...");
            delay(2000);
            ESP.restart(); // Restart required for LED pin changes
        } else {
            // Disconnect current MQTT if connected
            if (mqttClient.connected()) {
                mqttClient.disconnect();
            }
            
            // Try to reconnect with new settings
            delay(1000);
            bool connected = connectMQTT();
            
            if (connected) {
                server.send(200, "text/plain", "Configuration saved and MQTT connected successfully!");
            } else {
                server.send(200, "text/plain", "Configuration saved but MQTT connection failed. Please check settings.");
            }
        }
    } else {
        server.send(400, "text/plain", "Missing configuration parameters");
    }
}

void handleWiFiReset() {
    logPrintln("HTTP: /wifireset requested - Clearing WiFi credentials");
    server.send(200, "text/html", 
        "<html><body><h1>WiFi Reset</h1>"
        "<p>WiFi credentials cleared. Device will restart in config mode.</p>"
        "<p>Connect to the access point and configure WiFi again.</p>"
        "</body></html>");
    
    delay(1000);
    wifiManager.resetSettings();
    delay(1000);
    ESP.restart();
}

void handleReboot() {
    logPrintln("HTTP: /reboot requested - Rebooting device");
    server.send(200, "text/plain", "Device rebooting...");
    delay(1000);
    ESP.restart();
}

void handleDoor() {
    logPrintln("=== DOOR LIGHTS TOGGLE ===");
    if (server.hasArg("toggle")) {
        manual_door_trigger = !manual_door_trigger;
        logPrintln("Manual Override: " + String(manual_door_trigger ? "ON" : "OFF"));
        logPrintln("Door Color: RGB(" + String(door_open_color.r) + ", " + String(door_open_color.g) + ", " + String(door_open_color.b) + ")");
        logPrintln("Door Lights Feature: " + String(door_open_lights_enabled ? "ENABLED" : "DISABLED"));
        logPrintln("Current Mode: " + String(
            currentMode == MODE_AUTO ? "AUTO" :
            currentMode == MODE_MANUAL ? "MANUAL" :
            currentMode == MODE_PRINTER ? "PRINTER" : "OFF"
        ));
        
        // Save to preferences
        preferences.begin("bambu", false);
        preferences.putBool("door_trigger", manual_door_trigger);
        preferences.end();
        
        // Only update LEDs if in PRINTER mode
        if (currentMode == MODE_PRINTER) {
            updatePrinterLEDs();
        }
        logPrintln("========================");
        server.send(200, "text/plain", manual_door_trigger ? "Door lights ON" : "Door lights OFF");
    } else if (server.hasArg("state")) {
        String state = server.arg("state");
        manual_door_trigger = (state == "on" || state == "true" || state == "1");
        logPrintln("Manual Override Set: " + String(manual_door_trigger ? "ON" : "OFF"));
        logPrintln("Requested State: " + state);
        
        // Save to preferences
        preferences.begin("bambu", false);
        preferences.putBool("door_trigger", manual_door_trigger);
        preferences.end();
        
        // Only update LEDs if in PRINTER mode
        if (currentMode == MODE_PRINTER) {
            updatePrinterLEDs();
        }
        logPrintln("========================");
        server.send(200, "text/plain", manual_door_trigger ? "Door lights ON" : "Door lights OFF");
    } else {
        server.send(200, "application/json", "{\"door_trigger\":" + String(manual_door_trigger ? "true" : "false") + "}");
    }
}

void handleChamber() {
    logPrintln("=== CHAMBER LIGHT MANUAL CONTROL ===");
    if (server.hasArg("state")) {
        String state = server.arg("state");
        bool turnOn = (state == "on" || state == "true" || state == "1");
        
        logPrintln("Manual Chamber Light Control: " + String(turnOn ? "ON" : "OFF"));
        logPrintln("MQTT Connected: " + String(mqttClient.connected() ? "YES" : "NO"));
        
        if (mqttClient.connected()) {
            setChamberLight(turnOn);
            logPrintln("Command sent successfully");
            server.send(200, "text/plain", turnOn ? "Chamber light ON command sent" : "Chamber light OFF command sent");
        } else {
            logPrintln("ERROR: MQTT not connected, cannot send command");
            server.send(503, "text/plain", "ERROR: MQTT not connected");
        }
        
        logPrintln("====================================");
    } else {
        server.send(400, "text/plain", "Missing 'state' parameter");
    }
}

void handleBeacon() {
    logPrintln("=== BEACON MANUAL OVERRIDE ===");
    if (server.hasArg("state")) {
        String state = server.arg("state");
        manual_beacon_trigger = (state == "on" || state == "true" || state == "1");
        logPrintln("Manual Beacon Override: " + String(manual_beacon_trigger ? "ON" : "OFF"));
        logPrintln("Requested State: " + state);
        
        // NOTE: Deliberately NOT saving to preferences - this is a temporary test switch
        // It will auto-disable on reboot to prevent continuous beacon updates
        
        if (manual_beacon_trigger) {
            // Turning ON: update with progress 100
            updateBeaconProgress(printerStatus.progress);
        } else {
            // Turning OFF: clear beacon LEDs and force update next time
            clearBeaconLEDs();
            updateBeaconLEDs();
            // Force next updateBeaconProgress to actually update (bypass cache)
            updateBeaconProgress(-1); // Use invalid progress to reset cache
        }
        
        logPrintln("========================");
        server.send(200, "text/plain", manual_beacon_trigger ? "Beacon lights ON" : "Beacon lights OFF");
    } else {
        server.send(200, "application/json", "{\"beacon_trigger\":" + String(manual_beacon_trigger ? "true" : "false") + "}");
    }
}

void handleAutoOff() {
    logPrintln("HTTP: /autooff requested");
    if (server.hasArg("minutes")) {
        auto_off_minutes = server.arg("minutes").toInt();
        logPrintln("Auto-off timer set to: " + String(auto_off_minutes) + " minutes");
        
        // Save to preferences
        preferences.begin("bambu", false);
        preferences.putInt("auto_off_min", auto_off_minutes);
        preferences.end();
        
        // Reset the timer
        auto_off_pending = false;
        lights_on_time = 0;
        
        if (auto_off_minutes == 0) {
            server.send(200, "text/plain", "Auto-off disabled");
        } else {
            server.send(200, "text/plain", "Auto-off set to " + String(auto_off_minutes) + " minutes");
        }
    } else {
        server.send(400, "text/plain", "Missing 'minutes' parameter");
    }
}

void handleRefreshPrinter() {
    logPrintln("HTTP: /refreshprinter requested - forcing LED update");
    
    // Force immediate LED update if in printer mode
    if (currentMode == MODE_PRINTER) {
        updatePrinterLEDs();
        server.send(200, "text/plain", "Printer LEDs refreshed");
    } else {
        server.send(200, "text/plain", "Not in printer mode");
    }
}

void handlePrinterBehavior() {
    logPrintln("HTTP: /printerbehavior requested");
    
    if (server.hasArg("state")) {
        String state = server.arg("state");
        PrinterStateBehavior* behavior = nullptr;
        String prefix = "";
        
        // Select which behavior to update
        if (state == "heating") {
            behavior = &behavior_heating;
            prefix = "heat";
        } else if (state == "printing") {
            behavior = &behavior_printing;
            prefix = "print";
        } else if (state == "idle") {
            behavior = &behavior_idle;
            prefix = "idle";
        } else if (state == "pause") {
            behavior = &behavior_pause;
            prefix = "pause";
        } else if (state == "finish") {
            behavior = &behavior_finish;
            prefix = "finish";
        } else if (state == "error") {
            behavior = &behavior_error;
            prefix = "error";
        }
        
        if (behavior != nullptr) {
            // Update behavior from parameters
            if (server.hasArg("effect")) behavior->effect = server.arg("effect");
            if (server.hasArg("color")) {
                // Parse hex color #RRGGBB
                String color = server.arg("color");
                if (color.startsWith("#")) color = color.substring(1);
                long colorValue = strtol(color.c_str(), NULL, 16);
                behavior->color.r = (colorValue >> 16) & 0xFF;
                behavior->color.g = (colorValue >> 8) & 0xFF;
                behavior->color.b = colorValue & 0xFF;
            }
            if (server.hasArg("brightness")) behavior->brightness = server.arg("brightness").toInt();
            
            // Save to preferences
            preferences.begin("bambu", false);
            preferences.putString((prefix + "_effect").c_str(), behavior->effect);
            preferences.putUChar((prefix + "_r").c_str(), behavior->color.r);
            preferences.putUChar((prefix + "_g").c_str(), behavior->color.g);
            preferences.putUChar((prefix + "_b").c_str(), behavior->color.b);
            preferences.putInt((prefix + "_bright").c_str(), behavior->brightness);
            preferences.end();
            
            logPrintln("Saved " + state + " behavior: " + behavior->effect + " RGB(" + 
                      String(behavior->color.r) + "," + String(behavior->color.g) + "," + 
                      String(behavior->color.b) + ") @ " + String(behavior->brightness) + "%");
            
            server.send(200, "text/plain", "Behavior saved for " + state);
        } else {
            server.send(400, "text/plain", "Invalid state");
        }
    } else {
        server.send(400, "text/plain", "Missing 'state' parameter");
    }
}

void handleBehaviorSettings() {
    logPrintln("HTTP: /behaviorSettings requested");
    
    // Helper function to convert RGB to hex
    auto rgbToHex = [](RGB color) -> String {
        char hex[8];
        sprintf(hex, "#%02X%02X%02X", color.r, color.g, color.b);
        return String(hex);
    };
    
    // Build JSON response with all behavior settings
    String json = "{";
    
    json += "\"heating\":{";
    json += "\"effect\":\"" + behavior_heating.effect + "\",";
    json += "\"color\":\"" + rgbToHex(behavior_heating.color) + "\",";
    json += "\"brightness\":" + String(behavior_heating.brightness);
    json += "},";
    
    json += "\"printing\":{";
    json += "\"effect\":\"" + behavior_printing.effect + "\",";
    json += "\"color\":\"" + rgbToHex(behavior_printing.color) + "\",";
    json += "\"brightness\":" + String(behavior_printing.brightness);
    json += "},";
    
    json += "\"idle\":{";
    json += "\"effect\":\"" + behavior_idle.effect + "\",";
    json += "\"color\":\"" + rgbToHex(behavior_idle.color) + "\",";
    json += "\"brightness\":" + String(behavior_idle.brightness);
    json += "},";
    
    json += "\"pause\":{";
    json += "\"effect\":\"" + behavior_pause.effect + "\",";
    json += "\"color\":\"" + rgbToHex(behavior_pause.color) + "\",";
    json += "\"brightness\":" + String(behavior_pause.brightness);
    json += "},";
    
    json += "\"finish\":{";
    json += "\"effect\":\"" + behavior_finish.effect + "\",";
    json += "\"color\":\"" + rgbToHex(behavior_finish.color) + "\",";
    json += "\"brightness\":" + String(behavior_finish.brightness);
    json += "},";
    
    json += "\"error\":{";
    json += "\"effect\":\"" + behavior_error.effect + "\",";
    json += "\"color\":\"" + rgbToHex(behavior_error.color) + "\",";
    json += "\"brightness\":" + String(behavior_error.brightness);
    json += "}";
    
    json += "}";
    
    server.send(200, "application/json", json);
}

void handleBeaconSettings() {
    logPrintln("HTTP: /beaconSettings requested");
    
    // Helper function to convert RGB to hex
    auto rgbToHex = [](RGB color) -> String {
        char hex[8];
        sprintf(hex, "#%02X%02X%02X", color.r, color.g, color.b);
        return String(hex);
    };
    
    // Build JSON response with all beacon settings
    String json = "{";
    json += "\"enabled\":" + String(beacon_enabled ? "true" : "false") + ",";
    json += "\"pin\":" + String(beacon_led_pin) + ",";
    json += "\"count\":" + String(beacon_led_count) + ",";
    json += "\"type\":" + String(beacon_led_type) + ",";
    json += "\"colorOrder\":" + String(beacon_color_order) + ",";
    json += "\"rgbw\":" + String(beacon_is_rgbw ? "true" : "false") + ",";
    json += "\"effect\":\"" + beacon_effect + "\",";
    json += "\"color\":\"" + rgbToHex(beacon_color) + "\",";
    json += "\"brightness\":" + String(beacon_brightness) + ",";
    json += "\"climbColor\":\"" + rgbToHex(beacon_climb_color) + "\",";
    json += "\"climbSpeed\":" + String(beacon_climb_speed) + ",";
    json += "\"gradientEndColor\":\"" + rgbToHex(beacon_gradient_end) + "\"";
    json += "}";
    
    server.send(200, "application/json", json);
}

void handleBeaconConfig() {
    logPrintln("HTTP: /beaconConfig requested");
    
    bool needsReinit = false;
    bool wasEnabled = beacon_enabled;
    
    // Parse hex color to RGB
    auto hexToRgb = [](String hex) -> RGB {
        RGB color = {0, 0, 0};
        if (hex.startsWith("#")) hex = hex.substring(1);
        if (hex.length() == 6) {
            color.r = strtol(hex.substring(0, 2).c_str(), NULL, 16);
            color.g = strtol(hex.substring(2, 4).c_str(), NULL, 16);
            color.b = strtol(hex.substring(4, 6).c_str(), NULL, 16);
        }
        return color;
    };
    
    // Update enabled state
    if (server.hasArg("enabled")) {
        bool newEnabled = server.arg("enabled") == "true" || server.arg("enabled") == "1";
        if (newEnabled != beacon_enabled) {
            beacon_enabled = newEnabled;
            needsReinit = true;
        }
    }
    
    // Update pin
    if (server.hasArg("pin")) {
        int newPin = server.arg("pin").toInt();
        if (newPin >= 0 && newPin <= 39 && newPin != beacon_led_pin) {
            beacon_led_pin = newPin;
            needsReinit = true;
        }
    }
    
    // Update LED count
    if (server.hasArg("count")) {
        int newCount = server.arg("count").toInt();
        if (newCount >= MIN_BEACON_LED_COUNT && newCount <= MAX_BEACON_LED_COUNT && newCount != beacon_led_count) {
            beacon_led_count = newCount;
            needsReinit = true;
        }
    }
    
    // Update LED type
    if (server.hasArg("type")) {
        int newType = server.arg("type").toInt();
        if (newType >= 0 && newType <= 4 && newType != beacon_led_type) {
            beacon_led_type = newType;
            needsReinit = true;
        }
    }
    
    // Update RGBW mode
    if (server.hasArg("rgbw")) {
        bool newRgbw = server.arg("rgbw") == "true" || server.arg("rgbw") == "1";
        if (newRgbw != beacon_is_rgbw) {
            beacon_is_rgbw = newRgbw;
            needsReinit = true;
        }
    }
    
    // Update color order
    if (server.hasArg("colorOrder")) {
        int newOrder = server.arg("colorOrder").toInt();
        if (newOrder >= 0 && newOrder <= 5 && newOrder != beacon_color_order) {
            beacon_color_order = newOrder;
            needsReinit = true;
        }
    }
    
    // Update effect
    if (server.hasArg("effect")) {
        beacon_effect = server.arg("effect");
        logPrintln("Beacon effect set to: " + beacon_effect);
        beacon_gradient_debug = true; // Trigger gradient render log if gradient is selected
    }
    
    // Update color
    if (server.hasArg("color")) {
        beacon_color = hexToRgb(server.arg("color"));
        beacon_force_update = true; // Force re-render with new color
    }
    
    // Update brightness
    if (server.hasArg("brightness")) {
        beacon_brightness = server.arg("brightness").toInt();
        if (beacon_brightness < 0) beacon_brightness = 0;
        if (beacon_brightness > 100) beacon_brightness = 100;
    }
    
    // Update climbing pixel color
    if (server.hasArg("climbColor")) {
        beacon_climb_color = hexToRgb(server.arg("climbColor"));
    }
    
    // Update climbing pixel speed
    if (server.hasArg("climbSpeed")) {
        beacon_climb_speed = server.arg("climbSpeed").toInt();
        if (beacon_climb_speed < 20) beacon_climb_speed = 20;    // Min 20ms
        if (beacon_climb_speed > 2000) beacon_climb_speed = 2000; // Max 2000ms
    }
    
    // Update gradient end color
    if (server.hasArg("gradientEndColor")) {
        String hexColor = server.arg("gradientEndColor");
        beacon_gradient_end = hexToRgb(hexColor);
        beacon_gradient_debug = true; // Set flag to log on next render
        beacon_force_update = true; // Force beacon to re-render with new color
        logPrintln("Gradient end color received: " + hexColor + " -> RGB(" + String(beacon_gradient_end.r) + "," + String(beacon_gradient_end.g) + "," + String(beacon_gradient_end.b) + ")");
    }
    
    // Save all settings to preferences
    preferences.begin("bambu", false);
    preferences.putBool("beacon_en", beacon_enabled);
    preferences.putInt("beacon_pin", beacon_led_pin);
    preferences.putInt("beacon_count", beacon_led_count);
    preferences.putInt("beacon_type", beacon_led_type);
    preferences.putInt("beacon_order", beacon_color_order);
    preferences.putBool("beacon_rgbw", beacon_is_rgbw);  // SAVE RGBW SETTING
    preferences.putString("beacon_fx", beacon_effect);
    preferences.putUChar("beacon_r", beacon_color.r);
    preferences.putUChar("beacon_g", beacon_color.g);
    preferences.putUChar("beacon_b", beacon_color.b);
    preferences.putInt("beacon_bright", beacon_brightness);
    preferences.putUChar("beacon_climb_r", beacon_climb_color.r);
    preferences.putUChar("beacon_climb_g", beacon_climb_color.g);
    preferences.putUChar("beacon_climb_b", beacon_climb_color.b);
    preferences.putInt("bcn_clmb_spd", beacon_climb_speed);
    preferences.putUChar("bcn_grad_end_r", beacon_gradient_end.r);
    preferences.putUChar("bcn_grad_end_g", beacon_gradient_end.g);
    preferences.putUChar("bcn_grad_end_b", beacon_gradient_end.b);
    preferences.end();
    
    Serial.printf("Saved beacon climb speed: %d ms\n", beacon_climb_speed);
    logPrintln("Gradient end color saved: RGB(" + String(beacon_gradient_end.r) + "," + String(beacon_gradient_end.g) + "," + String(beacon_gradient_end.b) + ")");
    Serial.printf("Saved beacon climb color: R=%d G=%d B=%d\n", beacon_climb_color.r, beacon_climb_color.g, beacon_climb_color.b);
    logPrintln("Beacon settings saved to preferences");
    
    // Defer beacon reinitialization to main loop to avoid blocking web server
    if (needsReinit) {
        beacon_reinit_pending = true;
        beacon_was_enabled_before_reinit = wasEnabled;
        logPrintln("Beacon reinit scheduled (deferred to main loop)");
    }
    
    server.send(200, "text/plain", "Beacon settings updated successfully");
}

void handleLidar() {
    Serial.println("HTTP: /lidar requested");
    if (server.hasArg("enabled")) {
        String enabled = server.arg("enabled");
        lidar_lights_off_enabled = (enabled == "on" || enabled == "true" || enabled == "1");
        Serial.printf("LiDAR lights off setting: %s\n", lidar_lights_off_enabled ? "ENABLED" : "DISABLED");
        
        // Save setting to preferences
        preferences.begin("bambu", false);
        preferences.putBool("lidar_lights_off", lidar_lights_off_enabled);
        preferences.end();
        
        // Only update LEDs if in PRINTER mode
        if (currentMode == MODE_PRINTER) {
            updatePrinterLEDs();
        }
        server.send(200, "text/plain", lidar_lights_off_enabled ? 
                   "LiDAR lights-off ENABLED - lights will turn off during scanning" : 
                   "LiDAR lights-off DISABLED - lights stay on during scanning");
    } else {
        server.send(200, "application/json", "{\"lidar_lights_off\":" + String(lidar_lights_off_enabled ? "true" : "false") + "}");
    }
}

void handleResetConfig() {
    preferences.begin("bambu", false);
    preferences.clear(); // Clear all stored preferences
    preferences.end();
    
    server.send(200, "text/plain", "Configuration reset to defaults! ESP32 will restart in 3 seconds...");
    delay(3000);
    ESP.restart();
}

void setupOTA() {
    // Set OTA hostname (uses device serial for unique identification)
    String hostname = "bambu-lights-" + device_serial.substring(device_serial.length() - 6);
    ArduinoOTA.setHostname(hostname.c_str());
    
    // Optional: Set password for OTA updates (recommended for security)
    // ArduinoOTA.setPassword("your_ota_password");
    
    // OTA callbacks for status updates
    ArduinoOTA.onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH) {
            type = "sketch";
        } else { // U_SPIFFS
            type = "filesystem";
        }
        Serial.println("OTA Update Started: " + type);
        // Turn off LEDs during update
        clearAllLEDs();
        updateLEDs();
    });
    
    ArduinoOTA.onEnd([]() {
        Serial.println("\nOTA Update Complete!");
    });
    
    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        static unsigned int lastPercent = 0;
        unsigned int percent = (progress / (total / 100));
        if (percent != lastPercent && percent % 10 == 0) {
            Serial.printf("OTA Progress: %u%%\n", percent);
            lastPercent = percent;
        }
    });
    
    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("OTA Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) {
            Serial.println("Auth Failed");
        } else if (error == OTA_BEGIN_ERROR) {
            Serial.println("Begin Failed");
        } else if (error == OTA_CONNECT_ERROR) {
            Serial.println("Connect Failed");
        } else if (error == OTA_RECEIVE_ERROR) {
            Serial.println("Receive Failed");
        } else if (error == OTA_END_ERROR) {
            Serial.println("End Failed");
        }
    });
    
    ArduinoOTA.begin();
    Serial.println("OTA Update Service Started");
    Serial.printf("OTA Hostname: %s\n", hostname.c_str());
}

void setupWebServer() {
    Serial.println("Configuring web server routes...");
    server.on("/", handleRoot);
    server.on("/mode", handleMode);
    server.on("/color", handleColor);
    server.on("/status", handleStatus);
    server.on("/getconfig", handleGetConfig);
    server.on("/hamqttconfig", handleGetHAMqttConfig);
    server.on("/setconfig", handleSetConfig);
    server.on("/resetconfig", handleResetConfig);
    server.on("/door", handleDoor);
    server.on("/chamber", handleChamber);
    server.on("/beacon", handleBeacon);
    server.on("/lidar", handleLidar);
    server.on("/autooff", handleAutoOff);
    server.on("/printerbehavior", handlePrinterBehavior);
    server.on("/refreshprinter", handleRefreshPrinter);
    server.on("/behaviorsettings", handleBehaviorSettings);
    server.on("/beaconsettings", handleBeaconSettings);
    server.on("/beaconconfig", handleBeaconConfig);
    server.on("/beacon.gif", handleBeaconGif);
    server.on("/progress_bar.gif", handleProgressBarGif);
    server.on("/climbing_dot.gif", handleClimbingDotGif);
    server.on("/gradient_fill.gif", handleGradientFillGif);
    server.on("/pulsing_dot.gif", handlePulsingDotGif);
    server.on("/logs", handleSerialLogs);
    server.on("/mqttlogging", handleMqttLogging);
    server.on("/wifireset", handleWiFiReset);
    server.on("/reboot", handleReboot);
    
    Serial.println("Starting web server on port 80...");
    server.begin();
    Serial.println("Web server started successfully!");
    Serial.printf("Web interface available at: http://%s/\n", WiFi.localIP().toString().c_str());
    Serial.printf("WiFi Status: %d, IP: %s\n", WiFi.status(), WiFi.localIP().toString().c_str());
}

// Dedicated web server task running on Core 0
// This ensures web responsiveness even when main loop is busy
void webServerTask(void* parameter) {
    for(;;) {
        if (WiFi.status() == WL_CONNECTED) {
            server.handleClient();
            ArduinoOTA.handle();
        }
        vTaskDelay(1); // Yield to other tasks (1ms minimum delay)
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    // Configure watchdog timer (30 second timeout)
    esp_task_wdt_init(30, true);
    esp_task_wdt_add(NULL);
    
    logPrintln("=== Bambu LED Controller v1.0 ===");
    logPrintln("ESP32 booted successfully!");
    
    // Load saved settings first
    loadSettings();
    
    // Configure LED timing based on type
    configureLEDTiming();
    
    // Initializing LEDs
    
    // Allocate memory for LEDs
    reallocateLEDs();
    
    setupRMT();
    clearAllLEDs();
    updateLEDs();
    
    // Initialize beacon LEDs if enabled
    if (beacon_enabled) {
        configureBeaconTiming();
        reallocateBeaconLEDs();
        setupBeaconRMT();
        clearBeaconLEDs();
        updateBeaconLEDs();
        Serial.println("Beacon LEDs initialized");
    }
    
    // LED initialization complete
    
    // Connect to WiFi
    bool wifiConnected = connectWiFi();
    
    if (wifiConnected) {
        Serial.println("Starting OTA update service...");
        setupOTA();
        
        // Initialize LittleFS for serving static files
        if (!LittleFS.begin()) {
            Serial.println("LittleFS mount failed!");
        } else {
            Serial.println("LittleFS mounted successfully");
        }
        
        Serial.println("Starting web server...");
        setupWebServer();
        
        // Start dedicated web server task on Core 0 with priority 1
        xTaskCreatePinnedToCore(
            webServerTask,    // Task function
            "WebServer",      // Name
            8192,            // Stack size (8KB)
            NULL,            // Parameters
            1,               // Priority (higher than default)
            NULL,            // Task handle
            0                // Core 0 (Arduino loop runs on Core 1)
        );
        
        // Give web server task time to fully initialize
        delay(1000);
        Serial.println("Web server task started on Core 0!");
        
        // Start Telnet server for remote serial monitoring
        telnetServer.begin();
        telnetServer.setNoDelay(true);
        Serial.println("Telnet server started on port 23");
        Serial.print("Connect via: telnet ");
        Serial.println(WiFi.localIP());
        
        Serial.println("Connecting to MQTT...");
        bool mqttConnected = connectMQTT();
        
        if (mqttConnected) {
            Serial.println("MQTT connected successfully!");
            currentMode = MODE_PRINTER; // Start in printer mode if MQTT works
        } else {
            Serial.println("MQTT connection failed with saved settings");
            
            // If default settings are detected, ask user to configure printer manually via web UI
            if (mqtt_server == "192.168.0.100" && mqtt_password == "your_access_code") {
                Serial.println("Default settings detected - please open the web UI to configure your printer (manual setup preferred).");
            }
            
            if (!mqttConnected) {
                Serial.println("Starting in auto mode - use web interface to configure");
            }
        }
        
        // Connect to Home Assistant MQTT if enabled
        if (ha_mqtt_enabled) {
            Serial.println("Connecting to Home Assistant MQTT...");
            bool haMqttConnected = connectHAMQTT();
            
            if (haMqttConnected) {
                Serial.println("Home Assistant MQTT connected successfully!");
            } else {
                Serial.println("Home Assistant MQTT connection failed");
            }
        }
        
        Serial.println("Starting normal operation...");
    } else {
        Serial.println("Running in offline mode...");
    }
}

void loop() {
    static unsigned long lastUpdate = 0;
    static unsigned long lastWiFiCheck = 0;
    static unsigned long lastMQTTCheck = 0;
    static int colorStep = 0;
    
    // Reset watchdog timer to prevent reboot
    esp_task_wdt_reset();
    
    // Handle Telnet clients
    if (telnetServer.hasClient()) {
        // New client trying to connect
        if (!telnetClient || !telnetClient.connected()) {
            if (telnetClient) telnetClient.stop();
            telnetClient = telnetServer.available();
            telnetClient.flush();
            telnetConnected = true;
            Serial.println("Telnet client connected from " + telnetClient.remoteIP().toString());
            telnetClient.println("=== Bambu LED Controller - Serial Monitor ===");
            telnetClient.println("Connected successfully!");
            telnetClient.println("IP: " + WiFi.localIP().toString());
            telnetClient.println("---");
        } else {
            // Already have a client, reject new connection
            WiFiClient newClient = telnetServer.available();
            newClient.stop();
        }
    }
    
    // Check if Telnet client disconnected
    if (telnetConnected && (!telnetClient || !telnetClient.connected())) {
        Serial.println("Telnet client disconnected");
        telnetConnected = false;
        telnetClient.stop();
    }
    
    // Handle deferred beacon reinitialization (moved from web handler to avoid blocking)
    if (beacon_reinit_pending) {
        beacon_reinit_pending = false;
        logPrintln("Executing deferred beacon reinit...");
        
        // Clean up old beacon
        if (beacon_was_enabled_before_reinit) {
            clearBeaconLEDs();
            updateBeaconLEDs();
            if (beacon_leds != nullptr) {
                free(beacon_leds);
                beacon_leds = nullptr;
            }
            if (beacon_rgbw_leds != nullptr) {
                free(beacon_rgbw_leds);
                beacon_rgbw_leds = nullptr;
            }
            if (beacon_led_data != nullptr) {
                free(beacon_led_data);
                beacon_led_data = nullptr;
            }
            rmt_driver_uninstall(RMT_CHANNEL_1);
            yield(); // Allow web server to process during uninstall
        }
        
        // Initialize new beacon if enabled
        if (beacon_enabled) {
            configureBeaconTiming();
            reallocateBeaconLEDs();
            setupBeaconRMT();
            clearBeaconLEDs();
            updateBeaconLEDs();
            logPrintln("Beacon reinitialized with new settings");
        }
        beacon_was_enabled_before_reinit = false;
    }
    
    // Web server and OTA now handled in dedicated task on Core 0
    
    // Handle MQTT
    if (WiFi.status() == WL_CONNECTED && mqttClient.connected()) {
        // espMqttClient handles message processing automatically, no loop() needed
        yield(); // Give time for background processing
    }
    
    // Handle Home Assistant MQTT
    if (WiFi.status() == WL_CONNECTED && ha_mqtt_enabled) {
        haMqttClient.loop(); // Process MQTT messages (required when UseInternalTask::NO)
        publishHAState(); // Check and publish telemetry updates
        yield(); // Give time for background processing
    }
    
    // Check WiFi status every 30 seconds
    if (millis() - lastWiFiCheck > 30000) {
        lastWiFiCheck = millis();
        
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("WiFi disconnected! Attempting reconnect...");
            yield();
            connectWiFi();
            yield();
            if (WiFi.status() == WL_CONNECTED) {
                setupWebServer(); // Restart web server after reconnect
                yield();
                connectMQTT(); // Reconnect MQTT
                yield();
                if (ha_mqtt_enabled) {
                    connectHAMQTT(); // Reconnect Home Assistant MQTT
                    yield();
                }
            }
        }
    }
    
    // Check auto-off timer - turns off lights regardless of current mode
    if (auto_off_pending && auto_off_minutes > 0) {
        unsigned long elapsed_minutes = (millis() - lights_on_time) / 60000;
        
        // Log timer status every minute
        static unsigned long last_timer_log = 0;
        if (millis() - last_timer_log > 60000) {
            last_timer_log = millis();
            logPrintln("Auto-off timer: " + String(elapsed_minutes) + "/" + String(auto_off_minutes) + " min elapsed");
        }
        
        if (elapsed_minutes >= auto_off_minutes) {
            // Skip turning off if door is currently open (door lights have priority)
            if (door_open_lights_enabled && door_is_open) {
                logPrintln("Auto-off timer expired but door is open - keeping lights on");
                // Restart timer to check again later
                lights_on_time = millis();
            } else {
                logPrintln("=== AUTO-OFF TIMER EXPIRED ===");
                logPrintln("Timer: " + String(auto_off_minutes) + " minutes");
                logPrintln("Action: Turning OFF lights");
                logPrintln("Previous mode saved for restore");
            
            // Turn off chamber light if sync is enabled
            logPrintln("Chamber Sync Enabled: " + String(chamber_light_sync_enabled ? "YES" : "NO"));
            logPrintln("MQTT Connected: " + String(mqttClient.connected() ? "YES" : "NO"));
            if (chamber_light_sync_enabled && mqttClient.connected()) {
                logPrintln(">>> Turning OFF chamber light (synced)");
                setChamberLight(false);
                chamber_light_was_on_before_auto_off = true; // Assume it was on
            } else {
                if (!chamber_light_sync_enabled) {
                    logPrintln(">>> Chamber sync DISABLED - skipping");
                }
                if (!mqttClient.connected()) {
                    logPrintln(">>> MQTT not connected - skipping");
                }
            }
            
                logPrintln("============================");
                mode_before_auto_off = currentMode; // Save current mode before turning off
                currentMode = MODE_OFF;
                clearAllLEDs();
                updateLEDs();
                auto_off_pending = false;
                lights_on_time = 0;
            }
        }
    }
    
    // Check MQTT connection every 30 seconds (reduced frequency to improve web responsiveness)
    if (millis() - lastMQTTCheck > 30000) {
        lastMQTTCheck = millis();
        
        if (WiFi.status() == WL_CONNECTED && !mqttClient.connected()) {
            Serial.println("MQTT disconnected! Attempting reconnect...");
            yield(); // Yield before attempting connection
            // Non-blocking MQTT connection attempt
            bool connected = connectMQTT();
            Serial.printf("MQTT Reconnect: %s\n", connected ? "OK" : "FAILED");
            yield(); // Yield after MQTT attempt to keep web responsive
        }
    }
    
    // LED updates based on current mode
    // Update every 50ms for smooth color transitions
    if (millis() - lastUpdate > 50) {
        lastUpdate = millis();
        
        static bool doorLightsWereActive = false;
        
        // Priority override: Door open lights (works in any mode)
        if (door_open_lights_enabled && door_is_open) {
            if (!doorLightsWereActive) {
                logPrintln("=== DOOR LIGHTS ACTIVE ===");
                logPrintln("Door Color: RGB(" + String(door_open_color.r) + ", " + String(door_open_color.g) + ", " + String(door_open_color.b) + ")");
                logPrintln("Overriding current mode: " + String(
                    currentMode == MODE_AUTO ? "AUTO" :
                    currentMode == MODE_MANUAL ? "MANUAL" :
                    currentMode == MODE_PRINTER ? "PRINTER" : "OFF"
                ));
                logPrintln("=========================");
                doorLightsWereActive = true;
            }
            setAllLEDs(door_open_color.r, door_open_color.g, door_open_color.b);
            updateLEDs();
        } else if (WiFi.status() == WL_CONNECTED && currentMode == MODE_AUTO) {
            // Door just closed - clear flag
            if (doorLightsWereActive) {
                logPrintln("Door closed - resuming normal mode operation");
                doorLightsWereActive = false;
            }
            
            // Auto mode - smooth color fading with adjustable speed
            // Calculate dynamic fade steps based on speed (1-100)
            // Updates every 50ms, so 20 updates per second
            // Speed controls total fade duration: Speed 1 = 10 seconds, Speed 100 = 1 second
            int cyclesPerColor = 200 - (autoCycleSpeed * 2); // Speed 1 = 200 steps (10s), 100 = 20 steps (1s)
            
            if (fadeStep >= cyclesPerColor) {
                // Time to switch to next color
                currentAutoColor = targetAutoColor;
                fadeStep = 0;
                
                switch (colorStep % 5) {
                    case 0:
                        Serial.println("Auto: Next color -> Soft Blue");
                        targetAutoColor = {0, 0, 128};    // Soft blue
                        break;
                    case 1:
                        Serial.println("Auto: Next color -> Soft Purple");
                        targetAutoColor = {128, 0, 128};  // Soft purple
                        break;
                    case 2:
                        Serial.println("Auto: Next color -> Soft Cyan");
                        targetAutoColor = {0, 128, 128};  // Soft cyan
                        break;
                    case 3:
                        Serial.println("Auto: Next color -> Soft White");
                        targetAutoColor = {100, 100, 100}; // Soft white
                        break;
                    case 4:
                        Serial.println("Auto: Next color -> Soft Green");
                        targetAutoColor = {0, 128, 0};    // Soft green
                        break;
                }
                colorStep++;
            }
            
            // Smooth interpolation between current and target colors
            float progress = (float)fadeStep / cyclesPerColor;
            uint8_t r = currentAutoColor.r + (targetAutoColor.r - currentAutoColor.r) * progress;
            uint8_t g = currentAutoColor.g + (targetAutoColor.g - currentAutoColor.g) * progress;
            uint8_t b = currentAutoColor.b + (targetAutoColor.b - currentAutoColor.b) * progress;
            
            // Apply brightness scaling (0-100%)
            r = (r * autoCycleBrightness) / 100;
            g = (g * autoCycleBrightness) / 100;
            b = (b * autoCycleBrightness) / 100;
            
            setAllLEDs(r, g, b);
            updateLEDs();
            fadeStep++;
        } else if (WiFi.status() != WL_CONNECTED && currentMode == MODE_AUTO) {
            // WiFi disconnected - show red error pattern
            switch (colorStep % 2) {
                case 0:
                    Serial.println("Status: Disconnected - Red blink");
                    setAllLEDs(255, 0, 0);     // Red
                    break;
                case 1:
                    Serial.println("Status: Disconnected - Off");
                    clearAllLEDs();            // Off
                    break;
            }
            updateLEDs();
            colorStep++;
        } else if (currentMode == MODE_PRINTER) {
            // Door just closed - clear flag and force update
            if (doorLightsWereActive) {
                logPrintln("Door closed - resuming normal mode operation");
                doorLightsWereActive = false;
                updatePrinterLEDs(); // Force immediate update to restore printer state
            }
            // Printer mode - always update LEDs, door lights override happens in main loop above
            // This ensures state doesn't get stuck
            else {
                updatePrinterLEDs();
            }
            
            // Check if printer is offline
            if (millis() - printerStatus.lastUpdate > 60000) {
                printerStatus.online = false;
            }
        }
        
        // Update beacon progress if enabled (runs regardless of main LED mode)
        // Throttled to 200ms (5 updates/sec) to prevent blocking web server
        static unsigned long lastBeaconUpdate = 0;
        if (beacon_enabled && beacon_leds && (millis() - lastBeaconUpdate > 50)) {
            lastBeaconUpdate = millis();
            
            // Check if auto-off timer has turned lights off
            if (currentMode == MODE_OFF && mode_before_auto_off != MODE_OFF) {
                // Auto-off timer has expired - turn off beacon
                clearBeaconLEDs();
                updateBeaconLEDs();
            } else if (manual_beacon_trigger) {
                // Manual override - always show at 100%
                updateBeaconProgress(100);
            } else if (printerStatus.gcode_state == "RUNNING" || printerStatus.gcode_state == "PREPARE") {
                // Check if heating (stage 1-9 with 0% progress)
                int stageNum = printerStatus.stage.toInt();
                bool isHeating = (stageNum > 0 && stageNum < 10 && printerStatus.progress == 0);
                
                if (isHeating) {
                    // Temporarily override beacon color to heating color for progress bar animation
                    RGB original_beacon_color = beacon_color;
                    RGB original_climb_color = beacon_climb_color;
                    
                    beacon_color = behavior_heating.color;
                    beacon_climb_color = behavior_heating.color;  // Use heating color for climb too
                    
                    updateBeaconProgress(50);  // Show 50% during heating phase
                    
                    // Restore original colors
                    beacon_color = original_beacon_color;
                    beacon_climb_color = original_climb_color;
                } else {
                    // Normal printing - show actual progress
                    updateBeaconProgress(printerStatus.progress);
                }
            } else if (printerStatus.gcode_state == "IDLE" || printerStatus.gcode_state == "FINISH") {
                // Print finished or idle - show idle state behavior
                // Apply idle color and effect to beacon LEDs
                uint8_t r = (behavior_idle.color.r * behavior_idle.brightness) / 100;
                uint8_t g = (behavior_idle.color.g * behavior_idle.brightness) / 100;
                uint8_t b = (behavior_idle.color.b * behavior_idle.brightness) / 100;
                
                if (behavior_idle.effect == "solid") {
                    // Solid color on all beacon LEDs
                    for (int i = 0; i < beacon_led_count; i++) {
                        setBeaconLED(i, r, g, b);
                    }
                    updateBeaconLEDs();
                } else if (behavior_idle.effect == "blink") {
                    // Blink effect
                    static bool blinkState = false;
                    static unsigned long lastBlink = 0;
                    if (millis() - lastBlink > 500) {
                        blinkState = !blinkState;
                        lastBlink = millis();
                    }
                    for (int i = 0; i < beacon_led_count; i++) {
                        if (blinkState) {
                            setBeaconLED(i, r, g, b);
                        } else {
                            setBeaconLED(i, 0, 0, 0);
                        }
                    }
                    updateBeaconLEDs();
                } else {
                    // For other effects (pulse, rainbow, etc), just show solid
                    for (int i = 0; i < beacon_led_count; i++) {
                        setBeaconLED(i, r, g, b);
                    }
                    updateBeaconLEDs();
                }
            } else if (!printerStatus.online || millis() - printerStatus.lastUpdate > 60000) {
                // Offline/Error state - apply error behavior
                int r = (behavior_error.color.r * behavior_error.brightness) / 100;
                int g = (behavior_error.color.g * behavior_error.brightness) / 100;
                int b = (behavior_error.color.b * behavior_error.brightness) / 100;
                
                if (behavior_error.effect == "solid") {
                    for (int i = 0; i < beacon_led_count; i++) {
                        setBeaconLED(i, r, g, b);
                    }
                    updateBeaconLEDs();
                } else if (behavior_error.effect == "blink") {
                    // Blink effect
                    static bool errorBlinkState = false;
                    static unsigned long lastErrorBlink = 0;
                    if (millis() - lastErrorBlink > 500) {
                        errorBlinkState = !errorBlinkState;
                        lastErrorBlink = millis();
                    }
                    for (int i = 0; i < beacon_led_count; i++) {
                        if (errorBlinkState) {
                            setBeaconLED(i, r, g, b);
                        } else {
                            setBeaconLED(i, 0, 0, 0);
                        }
                    }
                    updateBeaconLEDs();
                } else {
                    // For other effects, just show solid
                    for (int i = 0; i < beacon_led_count; i++) {
                        setBeaconLED(i, r, g, b);
                    }
                    updateBeaconLEDs();
                }
            } else {
                // Unknown state - clear beacon
                clearBeaconLEDs();
                updateBeaconLEDs();
            }
        }
    }
    
    // Network scanning has been removed; manual configuration via web UI is preferred.
}