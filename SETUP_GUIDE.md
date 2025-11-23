# Getting Your Bambu Lab Printer Credentials

To connect this controller to your Bambu Lab printer, you'll need three pieces of information:

## 1. Printer IP Address

**Method 1: From Printer Screen**
- On the printer touchscreen: Settings → Network → IP Address

**Method 2: From Router**
- Check your router's DHCP client list
- Look for a device named similar to your printer model (e.g., "BambuLab-X1C")

**Method 3: From Bambu Studio**
- Open Bambu Studio
- Look at the printer connection info
- The IP address is shown in the printer selection dialog

## 2. Access Code

The access code is a password that allows MQTT access to your printer.

**How to Get It:**
1. Download **Bambu Handy** app on your phone (iOS/Android)
2. Connect to your printer in the app
3. Go to: **Settings** → **General** → **Access Code**
4. You'll see a code like: `12345678`
5. Copy this code - you'll use it as the MQTT password

**Note:** The access code is the same for all MQTT clients connecting to your printer. Keep it secure!

## 3. Device Serial Number

Your printer's unique serial number.

**Method 1: From Printer**
- On the printer touchscreen: Settings → About → Serial Number
- Format: `01S00A301423` (example)

**Method 2: From Bambu Studio**
- Open Bambu Studio
- Look at printer properties
- Serial number is listed in the device information

**Method 3: Physical Label**
- Check the label on the back or bottom of your printer
- Look for "S/N:" followed by the serial number

## 4. Configure in the Code

Edit `src/main.cpp` and update these lines:

```cpp
// Line ~27: Your printer's IP address
String mqtt_server = "192.168.0.100";  // ⚠️ CHANGE THIS

// Line ~28: Access code from Bambu Handy app
String mqtt_password = "12345678";     // ⚠️ CHANGE THIS

// Line ~29: Your printer's serial number
String device_serial = "01S00A301423"; // ⚠️ CHANGE THIS
```

## 5. Alternative: Configure via Web Interface

After first boot, you can also configure these settings through the web interface:
1. Upload the firmware with just WiFi credentials set
2. Access the web interface
3. Go to **Config** tab
4. Enter MQTT settings and save

## MQTT Topics

The controller subscribes to:
```
device/{YOUR_SERIAL_NUMBER}/report
```

This is the standard Bambu Lab MQTT topic that contains:
- Print progress
- Printer state (IDLE, RUNNING, PAUSE, FINISH, FAILED)
- Temperatures (bed, nozzle, chamber)
- Chamber light status
- Door status (via home_flag)
- Stage information (printing, heating, etc.)

## Security Notes

⚠️ **Important:**
- Your access code grants MQTT access to your printer
- Don't share your access code publicly
- Don't commit code with real credentials to public repositories
- Use placeholder values in public code

## Troubleshooting

**Can't connect to MQTT?**
- Verify printer IP is correct (ping it from command line)
- Verify access code is entered correctly (no spaces)
- Verify serial number format is correct
- Check that printer is powered on and connected to network
- Try port 8883 (SSL) or 1883 (non-SSL) depending on your printer model

**Need more help?**
- Check the [GitHub Issues](../../issues)
- Review serial monitor output for connection errors
- Enable debug logging in the web interface
