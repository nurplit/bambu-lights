# 🎨 Bambu Lights - Advanced LED Controller

An ESP32-based smart LED controller that brings your Bambu Lab 3D printer to life with dynamic status lighting. Features dual LED strip support, progress visualization beacon, smart chamber light control, and comprehensive printer state tracking via MQTT.

**Inspired by:** [BambuLights by judge2005](https://github.com/judge2005/BambuLights)

[![PlatformIO](https://img.shields.io/badge/PlatformIO-ESP32-blue.svg)](https://platformio.org/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

---

## 📺 Demo

### Beacon LED Effects
<p align="center">
  <img src="Media/Progress_Bar.gif" width="200" alt="Progress Bar Effect"/>
  <img src="Media/Climbing_Dot.gif" width="200" alt="Climbing Dot Effect"/>
  <img src="Media/Beacon.gif" width="200" alt="Beacon Animation"/>
</p>

*Main LED strip demo coming soon!*

## ✨ Features

### � LED Control
- **Dual LED Strip Support** - Main status strip + optional progress beacon strip
- **Multiple Modes** - Auto-cycle, Manual color, Printer status, or Off
- **RGBW Support** - Optimized white channel for SK6812 RGBW LEDs
- **Custom Effects** - Solid, Blink, Breathe, Pulse, Progress Bar, Rainbow
- **Per-State Customization** - Different colors and effects for each printer state

### 📊 Printer Status Tracking
- **Real-time MQTT Integration** - Secure SSL/TLS connection to Bambu Lab printer
- **Comprehensive State Detection** - Idle, Heating, Printing, Paused, Complete, Error, Failed
- **Progress Visualization** - Progress bar effect shows print completion
- **Layer Tracking** - Displays current layer and total layers
- **Temperature Monitoring** - Bed and nozzle temperature display

### 🎯 Beacon LED Strip (Optional Second Strip)
- **Progress Bar Animation** - Fills from bottom to top with climbing pixel
- **Climbing Dot Effect** - Single LED climbs the strip
- **Gradient Fill** - Smooth color transition as print progresses
- **Pulsing Effect** - Breathing animation at current progress level
- **Heating Color Override** - Beacon uses heating color during warmup phase

### 🚪 Smart Automation
- **Door Detection** - Auto-responds to printer door open/close
- **Door Protection** - Prevents LED flicker during jogging with door open
- **Chamber Light Sync** - Controls printer chamber light based on printing color
- **LiDAR Pause** - Disables LEDs during LiDAR scanning to prevent interference
- **Auto-Off Timer** - Lights turn off X minutes after print completes (20-120 min)
- **Timelapse Detection** - Maintains proper lighting during timelapse recording

### 🌐 Web Interface
- **Modern Dark UI** - Beautiful responsive web interface with live updates
- **WiFiManager Integration** - Captive portal for easy first-time WiFi setup
- **OTA Updates** - Wireless firmware updates via mDNS hostname
- **Configuration Management** - All settings configurable via web UI
- **Serial Monitor** - Built-in web-based logging for debugging
- **Web Logs** - Access logs remotely at `http://your-device-ip/logs`
- **Telnet Support** - Remote serial monitoring via Telnet (port 23)
- **MQTT Debug Mode** - Toggle full payload logging for troubleshooting
- **Status Dashboard** - Real-time printer and system status display

## 🚀 Quick Start

### Hardware Requirements

#### Required Components
- **ESP32 Development Board** (ESP32-DevKitC, ESP32-WROOM-32, or compatible)
- **Addressable LED Strip** (WS2811, WS2812B, WS2813, SK6812, SK6812 RGBW, APA102, or APA106)
- **Power Supply** (5V DC, rated for your LED count - typically 60mA per LED at full white)
- **Jumper Wires** (3-4 wires depending on setup)

#### Optional Components
- **Second LED Strip** (Beacon progress visualization strip - same types as above)
  - **Recommended:** 10 pixels for optimal beacon effects
- **Logic Level Shifter** (if experiencing signal issues - 3.3V to 5V)
- **Capacitor** (1000µF, 6.3V or higher - smooths power fluctuations)

#### 3D Printed Enclosure
STL and Fusion 360 files included for ESP32 enclosure that mounts to the [BLV AMS Riser for X1C](https://makerworld.com/en/models/19535-blv-ams-ams-2-riser-for-x1c-p1s-p2s-v4#profileId-19420). Files located in `Enclosure/` folder.

### Wiring Diagram

#### Single LED Strip (Main Status)
```
ESP32 GPIO 2  ──→  LED Strip Data In
ESP32 GND     ──┐
                ├──→  LED Strip GND
Power Supply - ─┘
Power Supply + ────→  LED Strip VCC (5V)
```

#### Dual LED Strips (Main + Beacon)
```
ESP32 GPIO 2   ──→  Main LED Strip Data In
ESP32 GPIO 14  ──→  Beacon LED Strip Data In
ESP32 GND      ──┐
                 ├──→  Both LED Strips GND
Power Supply -  ─┘
Power Supply +  ────→  Both LED Strips VCC (5V)
```

> ⚠️ **Important:** Never power LED strips directly from ESP32 pins! Always use an external 5V power supply rated for your LED count. Connect all grounds together (ESP32 GND + LED GND + Power Supply GND).

### Installation

#### 1. Install PlatformIO
- Install [VS Code](https://code.visualstudio.com/)
- Install [PlatformIO IDE](https://platformio.org/platformio-ide) extension in VS Code

#### 2. Clone Repository
```bash
git clone https://github.com/YOUR_USERNAME/bambu-lights.git
cd bambu-lights
```

#### 3. Upload Filesystem (GIF Animations)
```bash
# Upload beacon animation GIFs to ESP32 filesystem
pio run --target uploadfs
```

#### 4. Flash Firmware
```bash
# Build and upload firmware
pio run --target upload

# Monitor serial output (optional)
pio device monitor
```

#### 5. First Boot Setup
1. ESP32 creates WiFi Access Point: `Bambu-Lights-XXXXXX`
2. Connect to this AP with your phone/computer
3. Captive portal opens automatically (or visit `192.168.4.1`)
4. Enter your WiFi SSID and password
5. Device connects and shows IP address in serial monitor

#### 6. Configure Printer
1. Open web browser to `http://bambu-lights-XXXXXX.local/` or the IP address
2. Click **Config** button
3. Enter your printer settings:
   - **Printer IP:** Your Bambu Lab printer's local IP address
   - **Access Code:** From Bambu Handy app (Settings → Lan Mode → Access Code)
   - **Serial Number:** From printer (Settings → Device → Serial Number)
4. Configure LED settings:
   - **Main LED Pin:** GPIO pin number (default: 2)
   - **LED Count:** Number of LEDs in your strip
   - **LED Type:** WS2811, WS2812B, WS2813, SK6812, SK6812 RGBW, APA102, or APA106
   - **Color Order:** GRB, RGB, etc. (depends on your LED type)
5. (Optional) Configure Beacon LED strip with same settings
6. Click **Save & Restart**

> 💡 **Tip:** Use the serial monitor to see connection status and debug any issues

## 🎛️ Web Interface

Access at `http://bambu-lights-XXXXXX.local/` or the IP shown in serial monitor.

### Mode Control
- **Printer Status Mode** - LEDs respond to printer state (Idle, Heating, Printing, Paused, Complete, Error)
- **Auto-Cycle Mode** - LEDs cycle through colors automatically
- **Manual Mode** - Set a custom solid color
- **Off Mode** - All LEDs off (timers and door lights still active)

### Printer Status LED Behavior
Customize colors, brightness, and effects for each printer state:
- **Idle** - Printer ready (default: blue)
- **Heating** - Bed/nozzle warming up (default: red)
- **Printing** - Active print (default: green)
- **Paused** - Print paused (default: yellow)
- **Complete** - Print finished (default: white)
- **Error** - Printer error (default: red blink)

Each state supports multiple effects:
- Solid, Blink, Breathe, Pulse, Progress Bar, Rainbow

### Beacon Settings (Optional Second Strip)
- **Enable/Disable** - Toggle beacon LED strip
- **Effect Selection** - Progress Bar, Climbing Dot, Gradient Fill, Pulsing
- **Color Customization** - Start color, end color (gradient), climb color
- **Brightness Control** - Independent brightness setting
- **GPIO Pin** - Configurable pin assignment

### Quick Controls
- **Door Lights** - Auto white lights when door opens (configurable color)
- **Chamber Light Sync** - Controls printer chamber light:
  - Auto-on during timelapse recording
  - Smart control: ON for white printing colors, OFF for colored filaments
- **LiDAR Pause** - Turns off LEDs during LiDAR scanning
- **Auto-Off Timer** - Lights turn off X minutes after print completes
  - Timer starts when print finishes
  - Lights stay on during active printing
  - Timer restarts if door opened

### System Status
Real-time display of:
- ESP32 WiFi signal, uptime, free memory
- MQTT connection status
- Current LED mode and settings
- Printer state (online/offline, temperatures, progress, layer count)
- Door status
- Timer status (active/inactive, time remaining)

### Serial Monitor & Remote Logging
Built-in logging accessible three ways:

1. **Web Interface** - Serial monitor panel on main page with auto-refresh
2. **Direct URL** - Access logs at `http://your-device-ip/logs` via any browser
3. **Telnet** - Connect via Telnet for real-time streaming logs:
   ```bash
   telnet your-device-ip 23
   ```

Logged events include:
- Connection events
- MQTT messages
- State changes
- Error messages
- Toggle MQTT debug mode for full payload logging (15KB+ per message)

## � Advanced Features

### Smart Chamber Light Control
The system intelligently manages your printer's chamber light:
- **Timelapse Mode** - Auto-on during timelapse recording
- **Color-Aware Control** - Prevents chamber light from washing out custom LED colors:
  - **White filament (255,255,255)** → Chamber light ON
  - **Colored filament** → Chamber light OFF
- **Dynamic Updates** - Responds to printing color changes mid-print

### Door Open Protection
Sophisticated door detection with multiple benefits:
- **Bidirectional Detection** - Accurately tracks door open AND close states
- **Flicker Prevention** - LEDs don't change during jogging operations with door open
- **Automatic Restoration** - LEDs restore to printer state when door closes
- **Timer Integration** - Auto-off timer restarts when door opened

### Progress Beacon Animations
Four unique visualization effects for the optional beacon strip:

1. **Progress Bar** - Green bar fills from bottom to top with white climbing pixel
2. **Climbing Dot** - Single LED climbs up the strip
3. **Gradient Fill** - Smooth color transition (customizable start/end colors)
4. **Pulsing** - Breathing animation at current progress level

All effects show real-time print progress and automatically adapt to heating/printing states.

### RGBW LED Optimization
Special support for SK6812 RGBW LEDs:
- Dedicated white channel for true white output
- Automatic RGBW pixel allocation
- Optimized memory usage for large LED counts

---

## 📊 Technical Details

### System Requirements
- **ESP32**: ESP32-DevKitC or compatible (240MHz dual-core, 320KB RAM, 4MB Flash minimum)
- **Memory**: ~120KB free RAM during operation
- **Power**: 5V/2A minimum for small setups, calculate 60mA per LED at full white for larger installations
- **Network**: 2.4GHz WiFi (ESP32 doesn't support 5GHz)

### Performance Specifications
- **LED Update Rate**: 30 FPS
- **MQTT Latency**: <500ms typical response time
- **Web Interface Load Time**: 2-5 seconds
- **Memory Management**: Chunked HTTP transfer prevents fragmentation
- **Uptime**: Stable operation for weeks with proper power supply

### Supported LED Types
- **WS2811** (RGB) - 5V LED strips, common in older/budget setups
- **WS2812B** (RGB) - Most common, affordable, integrated controller
- **WS2813** (RGB) - Backup data line for reliability
- **SK6812** (RGB or RGBW) - Better color accuracy, true white with RGBW
- **APA102** (RGB) - SPI-based, faster refresh rates
- **APA106** (RGB) - Alternative to WS2812B

### Color Order Options
Configure in web interface to match your specific LED strip:
- GRB (most common for WS2812B)
- RGB
- BGR
- BRG
- RBG
- GBR

---

## 🐛 Troubleshooting

### LEDs Not Responding
**Symptoms:** LEDs stay off or show wrong colors

**Solutions:**
1. Check wiring:
   - ESP32 GND → LED Strip GND → Power Supply GND (all connected)
   - ESP32 GPIO pin → LED Strip Data In
   - Power Supply 5V → LED Strip VCC
2. Verify power supply capacity (60mA × LED count)
3. Try different color order setting (web config → LED Settings)
4. Use logic level shifter if LED strip requires 5V data signal
5. Add 470Ω resistor between ESP32 GPIO and LED data line
6. Add 1000µF capacitor between LED VCC and GND

### WiFi Connection Issues
**Symptoms:** Can't connect to WiFi, AP mode doesn't appear

**Solutions:**
1. Ensure WiFi network is 2.4GHz (ESP32 doesn't support 5GHz)
2. Check WiFi credentials in serial monitor
3. Factory reset: Hold GPIO 0 button during boot for 5 seconds
4. Look for AP: `Bambu-Lights-XXXXXX` (last 6 chars of MAC address)
5. Strong WiFi signal required - ESP32 antenna is small
6. Restart router if connection fails repeatedly

### MQTT Connection Failures
**Symptoms:** "MQTT disconnected" in serial monitor, printer status not updating

**Solutions:**
1. Verify printer IP address is correct (check router DHCP)
2. Confirm access code from Bambu Handy app (Settings → LAN Mode)
3. Ensure printer serial number is accurate (Settings → Device)
4. Check printer is on same network as ESP32
5. Enable LAN Mode on printer if not already enabled
6. Printer must be powered on and connected to WiFi
7. Check firewall isn't blocking port 8883 (MQTT SSL)

### Web Interface Problems
**Symptoms:** Page won't load, hangs, or shows errors

**Solutions:**
1. Try IP address instead of mDNS hostname
2. Find IP in serial monitor: `IP: 192.168.x.x`
3. Clear browser cache (Ctrl+Shift+Del)
4. Try different browser (Chrome/Firefox recommended)
5. Disable browser extensions that might interfere
6. Check ESP32 free memory in status panel (should be >100KB)
7. Reboot ESP32 if memory is low

### Memory Issues
**Symptoms:** ESP32 reboots, crashes, or web interface slow

**Solutions:**
1. Check free memory in web interface status panel
2. Reduce LED count if using very large strips (>300 LEDs)
3. Disable MQTT debug mode (uses significant memory)
4. Reboot ESP32 to clear fragmentation
5. Verify firmware was compiled with correct partition scheme

### Wrong Colors or Glitches
**Symptoms:** Colors are wrong, flickering, or random patterns

**Solutions:**
1. Try different color order in web config (GRB, RGB, etc.)
2. Add 470Ω resistor between ESP32 and LED data line
3. Shorten data wire if longer than 1 meter
4. Use shielded wire for data line in electrically noisy environments
5. Check power supply voltage (should be 5.0V ±0.5V)
6. Verify power and ground are connected to BOTH ends of long LED strips

### OTA Update Failures
**Symptoms:** Firmware upload fails or times out

**Solutions:**
1. Ensure ESP32 and computer are on same network
2. Use mDNS hostname: `bambu-lights-XXXXXX.local`
3. Disable firewall temporarily during upload
4. Try USB upload instead: `pio run --target upload`
5. Increase OTA timeout in platformio.ini if on slow network

### Serial Monitor Output
Normal startup sequence (viewable via Serial, Telnet, or `/logs`):
```
=== Bambu Lights Controller ===
Chip: ESP32-D0WDQ6 (revision 1)
Free Memory: 296504 bytes
WiFi connecting...
WiFi connected! IP: 192.168.0.163
mDNS started: bambu-lights-123456.local
Telnet server started on port 23
Starting MQTT connection...
MQTT connected successfully!
```

**Access Logs:**
- **USB Serial:** Connect via USB, 115200 baud
- **Telnet:** `telnet 192.168.0.163 23` (use your device IP)
- **Web Browser:** `http://192.168.0.163/logs` (use your device IP)

Error indicators:
- `WiFi connection failed` - Check SSID/password
- `MQTT connection failed` - Check printer IP/access code
- `Low memory warning` - Reduce LED count or features
- `LED initialization failed` - Check GPIO pin and LED type

---

## 🏠 Home Automation Integration

### OpenHAB / Home Assistant / Node-RED

The ESP32 can be integrated with home automation systems for advanced control scenarios like:
- Turn on printer lights when room scene activates
- Sync printer lights with other smart lighting
- Create automations based on print status
- Remote control from automation dashboards

**Current Implementation:**
The ESP32 currently subscribes only to Bambu Lab printer MQTT topics. For home automation control, you have two options:

**Option 1: HTTP API (Available Now)**
Use HTTP POST requests to control the device via the web interface:
```bash
# OpenHAB HTTP Binding Example
# Turn lights to specific color (mode=1, manual mode)
curl -X POST http://192.168.0.163/save \
  -d "currentMode=1&currentColor=%23FF0000&brightness=100"

# Turn off lights (mode=3)
curl -X POST http://192.168.0.163/save \
  -d "currentMode=3"

# Switch to printer mode (mode=0)
curl -X POST http://192.168.0.163/save \
  -d "currentMode=0"
```

**Option 2: MQTT Commands (Feature Request)**
Future enhancement could add command subscription topics like:
```
bambu-lights/command/mode          # 0=printer, 1=manual, 2=auto-cycle, 3=off
bambu-lights/command/color         # Hex color code (e.g., FF0000)
bambu-lights/command/brightness    # 0-100
bambu-lights/command/effect        # solid, blink, breathe, pulse, rainbow
```

**OpenHAB Example Configuration (HTTP):**
```
Thing http:url:bambuLights "Bambu Lights" [
    baseURL="http://192.168.0.163",
    refresh=30
] {
    Channels:
        Type switch : power "Power" [
            onValue="POST:/save:currentMode=0",
            offValue="POST:/save:currentMode=3"
        ]
        Type color : color "Color" [
            commandExtension="POST:/save?currentMode=1&currentColor=%s"
        ]
}
```

**Home Assistant Example (HTTP):**
```yaml
rest_command:
  bambu_lights_on:
    url: "http://192.168.0.163/save"
    method: POST
    payload: "currentMode=0"
  
  bambu_lights_off:
    url: "http://192.168.0.163/save"
    method: POST
    payload: "currentMode=3"
  
  bambu_lights_color:
    url: "http://192.168.0.163/save"
    method: POST
    payload: "currentMode=1&currentColor={{ color }}&brightness={{ brightness }}"

automation:
  - alias: "Sync Printer Lights with Room Scene"
    trigger:
      - platform: state
        entity_id: scene.office_evening
    action:
      - service: rest_command.bambu_lights_on
```

**Node-RED Example:**
Use HTTP Request node to POST to `/save` endpoint with appropriate parameters.

> 💡 **Tip:** The HTTP API uses the same parameters as the web interface form. Inspect the web page network traffic to see all available parameters.

> 🔧 **Want MQTT control?** This would be a great community contribution! Add subscription to `bambu-lights/command/#` topics in the `connectToMQTT()` function.

---

## ❓ FAQ

### Q: How many LEDs can I control?
**A:** Tested stable with 300 LEDs on single strip. Hardware limit is ~1000 LEDs, but practical limit depends on memory available. RGBW uses more memory than RGB.

### Q: Can I control multiple printers?
**A:** Currently supports one printer per ESP32. For multiple printers, use multiple ESP32 boards or wait for future multi-printer support.

### Q: Do I need the beacon LED strip?
**A:** No, beacon strip is optional. Main LED strip alone provides full printer status visualization. Beacon adds progress visualization.

### Q: What's the difference between SK6812 RGB and RGBW?
**A:** RGBW has dedicated white LED for true white color. RGB mixes red+green+blue for white, which looks slightly blue/pink. RGBW recommended for better color accuracy.

### Q: Can I use longer wires between ESP32 and LEDs?
**A:** Keep data wire under 1 meter for best results. For longer distances, use differential signaling (SPI LEDs) or repeater/buffer chips.

### Q: Does this work with Bambu Lab P1P, P1S, X1C, A1?
**A:** Yes! Works with all Bambu Lab printers that support LAN Mode and MQTT. Requires access code from Bambu Handy app.

### Q: How do I find my printer's IP address?
**A:** Check your router's DHCP client list or use Bambu Handy app (Settings → Network). Most routers show connected devices.

### Q: What happens if WiFi disconnects?
**A:** ESP32 automatically reconnects. LEDs continue last pattern until reconnected. No settings are lost (stored in flash).

### Q: Can I add custom LED effects?
**A:** Yes! Code is open-source. Add your effect to `handleAnimations()` function in `src/main.cpp`. Contributions welcome!

### Q: How much power does this use?
**A:** ESP32: ~500mA. LEDs: ~60mA each at full white. 36 LEDs at full white = ~2.7A total. Typical printing use is 30-50% brightness = ~1-1.5A.

### Q: Can I 3D print an enclosure?
**A:** Recommended! Keep ESP32 cool and protect from shorts. Mount near printer but away from heated bed. Plenty of ESP32 enclosure designs on Thingiverse/Printables.

---

## 🤝 Contributing

Contributions are welcome! This project thrives on community input.

### How to Contribute

**Report Bugs:**
1. Check existing [GitHub Issues](../../issues) first
2. Create new issue with clear title
3. Include:
   - Hardware setup (ESP32 model, LED type/count)
   - Firmware version
   - Serial monitor output
   - Steps to reproduce
   - Expected vs actual behavior

**Suggest Features:**
1. Check existing [GitHub Discussions](../../discussions)
2. Describe use case and benefits
3. Include mockups/diagrams if applicable

**Submit Code:**
1. Fork the repository
2. Create feature branch: `git checkout -b feature-amazing-effect`
3. Follow existing code style (4 spaces, descriptive variables)
4. Test on real hardware with various LED counts
5. Update README.md if adding features
6. Commit with clear messages: `git commit -m "Add rainbow spiral effect"`
7. Push to your fork: `git push origin feature-amazing-effect`
8. Open Pull Request with detailed description

### Development Setup
```bash
# Clone your fork
git clone https://github.com/YOUR_USERNAME/bambu-lights.git
cd bambu-lights

# Create branch
git checkout -b my-new-feature

# Build and test
pio run --target upload
pio device monitor

# Make changes, test thoroughly

# Commit
git add .
git commit -m "Descriptive message"
git push origin my-new-feature
```

### Code Standards
- **Formatting:** 4 spaces, no tabs
- **Naming:** `camelCase` for variables, `UPPER_CASE` for constants
- **Comments:** Explain WHY, not WHAT
- **Memory:** Avoid String class, use char arrays
- **Testing:** Test with 10, 50, 100+ LED configurations

---

## 📜 License

This project is licensed under the **MIT License**.

```
MIT License

Copyright (c) 2024 Bambu Lights Contributors

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

See [LICENSE](LICENSE) file for full details.

---

## 🙏 Acknowledgments

This project stands on the shoulders of giants:

### Core Inspiration
- **[BambuLights](https://github.com/judge2005/BambuLights)** by **judge2005** - Original concept, MQTT integration patterns, and beacon LED visualization. This project wouldn't exist without this pioneering work!

### Additional Inspiration
- **[BLLED Controller](https://github.com/DutchDevelop/BLLEDController)** - LED control patterns and status visualization ideas

### Key Libraries
- **[Adafruit_NeoPixel](https://github.com/adafruit/Adafruit_NeoPixel)** - Robust LED strip control
- **[PubSubClient](https://github.com/knolleary/pubsubclient)** - Reliable MQTT communication
- **[WiFiManager](https://github.com/tzapu/WiFiManager)** - Seamless WiFi configuration
- **[ArduinoJson](https://github.com/bblanchon/ArduinoJson)** - Efficient JSON parsing
- **[AsyncTCP](https://github.com/me-no-dev/AsyncTCP)** + **[ESPAsyncWebServer](https://github.com/me-no-dev/ESPAsyncWebServer)** - Fast web interface

### Community
- ESP32 Arduino framework maintainers
- PlatformIO team for excellent tooling
- Bambu Lab for creating amazing printers with open MQTT API
- All contributors and users who report bugs and suggest features

### Special Thanks
- 3D printing community for inspiration and support
- Everyone who shares their builds and improvements

---

## 📞 Support

### Get Help
- **Issues**: [GitHub Issues](../../issues) - Bug reports and technical problems
- **Discussions**: [GitHub Discussions](../../discussions) - Questions, ideas, and general chat
- **Documentation**: This README and code comments

### Quick Links
- [Installation Guide](#-installation) - Step-by-step setup
- [Troubleshooting](#-troubleshooting) - Common problems and solutions  
- [FAQ](#-faq) - Frequently asked questions
- [Contributing](#-contributing) - How to help improve this project

### Response Time
This is a community project maintained by volunteers. Please be patient - we'll respond as soon as we can!

---

**⭐ If this project helped you, please star the repository and share it with other Bambu Lab users!**

**Made with ❤️ for the 3D printing community**
