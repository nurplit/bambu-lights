# Security Notice

✅ **GOOD NEWS: WiFi credentials are NO LONGER hardcoded!**

This project now uses WiFiManager for automatic WiFi setup via captive portal. No WiFi credentials are stored in the code!

## ⚠️ Still Check These:

### src/main.cpp
The MQTT/Printer default values should be placeholders:

```cpp
// Line ~27-29: MQTT/Printer Default Values
String mqtt_password = "your_access_code";    // ✅ Generic placeholder (OK)
String device_serial = "01S00A123456789";     // ✅ Generic placeholder (OK)  
String mqtt_server = "192.168.0.100";         // ⚠️ Check this isn't YOUR real printer IP
```

## Recommended: Verify Placeholders

The code should have generic placeholders like this:

```cpp
// WiFi - Handled by WiFiManager (no hardcoded credentials!)
WiFiManager wifiManager;                      // ✅ GOOD

// MQTT defaults (configured via web interface)
String mqtt_server = "192.168.1.100";         // Generic local IP
String mqtt_password = "your_access_code";    // Placeholder
String device_serial = "01S00A301234567";     // Generic serial
```

## After Securing Credentials:

1. Review all code files for any other sensitive information
2. Check commit history - if you've already committed credentials, you'll need to:
   - Either start with a fresh repository (recommended)
   - Or use git tools to remove sensitive data from history

## Safe to Commit:

✅ Hardware schematics  
✅ PIN configurations  
✅ LED settings  
✅ Code logic and algorithms  
✅ Web interface HTML/CSS/JS  

## Never Commit:

❌ WiFi passwords  
❌ MQTT access codes  
❌ API keys  
❌ Device serial numbers (can be used to identify/access your printer)  
❌ IP addresses of your local network devices  

---

**Ready to commit?** Delete this file after you've secured your credentials!
