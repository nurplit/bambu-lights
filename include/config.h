#ifndef CONFIG_H
#define CONFIG_H

// Hardware Configuration
#define LED_PIN 2
#define DEFAULT_LED_COUNT 36
#define MAX_LED_COUNT 256
#define MIN_LED_COUNT 1

// Beacon LED Configuration
#define BEACON_LED_PIN 14
#define DEFAULT_BEACON_LED_COUNT 10
#define MAX_BEACON_LED_COUNT 50
#define MIN_BEACON_LED_COUNT 1

// Network Configuration
#define WIFI_CONNECT_TIMEOUT_MS 20000
#define MQTT_RECONNECT_INTERVAL_MS 5000
#define MQTT_KEEPALIVE_INTERVAL_S 60

// Memory Management
#define JSON_BUFFER_SIZE 4096
#define MQTT_MESSAGE_BUFFER_SIZE 16384
#define WS_MESSAGE_BUFFER_SIZE 2048
#define CONFIG_STRING_MAX_LENGTH 64

// Task Configuration
#define LED_TASK_STACK_SIZE 4096
#define WIFI_TASK_STACK_SIZE 4096
#define MQTT_TASK_STACK_SIZE 8192
#define WS_TASK_STACK_SIZE 4096

// Security Configuration
#define AP_PASSWORD_LENGTH 32
#define DEVICE_ID_LENGTH 16

// EEPROM Configuration
#define EEPROM_SIZE 4096
#define EEPROM_COMMIT_INTERVAL_MS 300000  // 5 minutes instead of 1 minute
#define CONFIG_MAGIC_BYTES 0x42414D42      // "BAMB"

// Timing Configuration
#define LED_UPDATE_INTERVAL_MS 33          // ~30 FPS
#define HEARTBEAT_INTERVAL_MS 10000
#define WATCHDOG_TIMEOUT_MS 30000

// Error Handling
#define MAX_RECONNECT_ATTEMPTS 10
#define RECONNECT_BACKOFF_MAX_MS 60000

// Debug Configuration - declarations only, implementations in main.cpp
#ifdef DEBUG
void debug_print(const char* str);
void debug_println(const char* str);
void debug_printf(const char* fmt, ...);
#define DEBUG_PRINT(x) debug_print(x)
#define DEBUG_PRINTLN(x) debug_println(x)
#define DEBUG_PRINTF(fmt, ...) debug_printf(fmt, ##__VA_ARGS__)
#else
#define DEBUG_PRINT(x)
#define DEBUG_PRINTLN(x)
#define DEBUG_PRINTF(fmt, ...)
#endif

// Validation Macros
#define VALIDATE_BUFFER_SIZE(size, max_size) \
    do { \
        if ((size) > (max_size)) { \
            DEBUG_PRINTF("Buffer overflow prevented: %zu > %zu\n", (size_t)(size), (size_t)(max_size)); \
            return false; \
        } \
    } while(0)

#define SAFE_STRING_COPY(dest, src, max_len) \
    do { \
        strncpy((dest), (src), (max_len) - 1); \
        (dest)[(max_len) - 1] = '\0'; \
    } while(0)

#endif // CONFIG_H