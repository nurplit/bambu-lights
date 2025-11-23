# Changelog - Bambu Lights Controller

## Recent Updates (November 16, 2025)

### Features Added

#### 1. Gradient End Color Feature
- Added gradient end color configuration for gradient effect
- New settings in LED behavior configuration:
  - Gradient start color (existing)
  - Gradient end color (new)
- Force update mechanism to immediately apply gradient changes
- Affects: Heating, First Layer, and any other states using gradient effect

#### 2. SK6812 RGBW White Channel Optimization
- Implemented proper RGBW (4-channel) LED support for SK6812 strips
- Added automatic white channel detection when `led_type = "SK6812"`
- Smart white channel optimization:
  - When RGB values are equal (e.g., 255,255,255 for white), the firmware automatically uses only the dedicated white channel
  - Provides more efficient power usage and purer white color
- Separate RGBW data structures and arrays:
  - `RGBW` struct with r, g, b, w fields
  - `RGBW* rgbw_leds` array for 4-channel LEDs
  - `RGB* leds` array for 3-channel LEDs
- Updated functions:
  - `setLED()` - Routes to correct array based on `is_rgbw` flag
  - `rgbToRMT()` - Handles 32-bit RGBW data vs 24-bit RGB data
  - `updateLEDs()` - Checks `is_rgbw` to use correct data
  - `applyColorOrder()` - Supports GRBW byte order for SK6812
  - `reallocateLEDs()` - Allocates RGBW array when needed

#### 3. Beacon LED Hardware Settings
- Added RGBW checkbox for beacon LED strip configuration
- Separate configuration from main LED strip
- Label: "RGBW LEDs (4-channel with white - check this if colors are wrong)"
- Beacon uses its own `beacon_is_rgbw` flag

#### 4. Dynamic Beacon GIF Animations
- Enlarged beacon GIF preview from 280px to 480x480px for better visibility
- Implemented dynamic GIF switching based on selected beacon effect:
  - Progress Bar → `Progress_Bar.gif` (shows green bar + white climbing dot)
  - Climbing Dot → `Climbing_Dot.gif` (white dot with trail)
  - Gradient Fill → `Gradient_Fill.gif` (green-to-red gradient)
  - Pulsing Dot → `Pulsing_Dot.gif` (blue breathing dot)
- Created Python script (`generate_beacon_gifs.py`) to generate all 4 beacon effect animations
- Total GIF size: ~51KB (optimized for ESP32 filesystem)
- JavaScript function order fix: Moved `autoSaveBeacon()` definition before HTML that calls it

#### 5. Mode-Based LED Update Control
- **CRITICAL FIX**: MQTT updates and web handlers now respect current mode
- LED updates only apply printer state when in **PRINTER mode**
- Prevents printer state from interfering with:
  - **MANUAL mode**: User-selected colors stay stable
  - **AUTO mode**: Color cycling continues smoothly
  - **OFF mode**: LEDs stay off
- Updated functions:
  - `onMqttMessage()` - Only calls `updatePrinterLEDs()` if in PRINTER mode
  - `handleDoorTrigger()` - Only updates LEDs if in PRINTER mode
  - `handleLidarLightsOff()` - Only updates LEDs if in PRINTER mode
  - Main loop - Only calls `updatePrinterLEDs()` if in PRINTER mode

#### 6. Door Open Protection During Jogging
- When door is open in PRINTER mode, LED updates are paused
- Prevents jogging commands from causing LED color flickering
- LEDs maintain their current color while door is open
- Normal printer state updates resume when door closes
- Logic: Skip `updatePrinterLEDs()` when `door_is_open == true`

### Code Structure Changes

#### Memory Management
- Proper allocation/deallocation for both RGB and RGBW arrays
- `reallocateLEDs()` now handles both LED types
- Cleanup in `reallocateLEDs()`:
  ```cpp
  if (leds) delete[] leds;
  if (rgbw_leds) delete[] rgbw_leds;
  if (led_data) delete[] led_data;
  ```

#### Data Structures
```cpp
struct RGBW {
    uint8_t r, g, b, w;
};

RGB* leds = nullptr;           // 3-channel LEDs
RGBW* rgbw_leds = nullptr;     // 4-channel LEDs
bool is_rgbw = false;          // Main LED type flag
bool beacon_is_rgbw = false;   // Beacon LED type flag
```

#### LED Type Detection
```cpp
// Automatic RGBW detection based on LED type
is_rgbw = (led_type == "SK6812");
```

### Bug Fixes

1. **JavaScript Function Order**: Fixed `autoSaveBeacon()` not defined error by moving function definition before HTML
2. **MQTT Override**: Fixed printer state overriding Manual/Auto modes
3. **Jogging Flicker**: Fixed LED color changing during print head jogging with door open
4. **RGBW Allocation**: Fixed missing RGBW array allocation causing white channel not to work
5. **Progress Bar GIF**: Updated to show both colors (green progress bar + white climbing dot)

### File Structure

#### New Files
- `generate_beacon_gifs.py` - Python script to generate beacon effect animations
- `data/Progress_Bar.gif` (5.2 KB)
- `data/Climbing_Dot.gif` (21.77 KB)
- `data/Gradient_Fill.gif` (6.76 KB)
- `data/Pulsing_Dot.gif` (17.8 KB)

#### Modified Files
- `src/main.cpp` - Core firmware with all updates (~4810 lines)
- `platformio.ini` - Using default.csv partition (1.5MB filesystem)

### Configuration

#### LED Strip Settings
- **LED Type**: Dropdown includes "SK6812 (RGBW Compatible)" option
- Automatic RGBW mode when SK6812 is selected
- Supports both 3-channel (WS2812B, WS2811, WS2813) and 4-channel (SK6812) LEDs

#### Beacon Settings
- Independent RGBW checkbox for beacon strip
- Beacon LED type selector (separate from main LEDs)
- GIF preview updates dynamically with effect selection

### Technical Details

#### RGBW Implementation
- **Color Order**: GRBW (Green, Red, Blue, White) for SK6812
- **Bit Depth**: 32 bits per LED (RGBW) vs 24 bits (RGB)
- **RMT Driver**: Properly configured for both 3 and 4 channel LEDs
- **White Optimization**: Automatic when r==g==b
  ```cpp
  if (w == 0 && r > 0 && r == g && g == b) {
      rgbw_leds[i].w = r;
      rgbw_leds[i].r = rgbw_leds[i].g = rgbw_leds[i].b = 0;
  }
  ```

#### Mode Priority System
1. **Door Open Lights** (highest priority) - Works in all modes
2. **Manual Mode** - User control, ignores printer state
3. **Auto Mode** - Color cycling, ignores printer state
4. **Printer Mode** - Follows printer state (pauses during door open)
5. **Off Mode** - All LEDs off (except door lights)

### Flash Usage
- Current: 96.8% (1,269,225 / 1,310,720 bytes)
- Filesystem: ~1.5MB (default.csv partition)
- GIF Assets: ~51KB total

### Testing Notes
- Tested with SK6812 RGBW LED strips
- Verified white channel usage with scope/multimeter
- Confirmed mode isolation (MANUAL/AUTO don't respond to printer)
- Validated door open protection during jogging
- All 4 beacon GIFs switching correctly

### Python Dependencies (for GIF generation)
```
Pillow >= 10.0.0
numpy >= 1.24.0
```

### Known Limitations
- GIF animations are static files (not generated in real-time)
- Beacon strip configuration separate from main LED configuration
- White channel optimization only for equal RGB values (gray/white colors)

---

## Upgrade Notes

If upgrading from previous version:
1. Select "SK6812 (RGBW Compatible)" in LED Type if using RGBW strips
2. Upload all 4 GIF files to `/data/` directory via LittleFS
3. Configure beacon RGBW checkbox if beacon uses 4-channel LEDs
4. Test door open behavior in PRINTER mode to ensure no flickering during jogging

---

## Future Enhancements (Suggestions)
- Real-time beacon effect rendering (eliminate need for GIF files)
- Configurable white channel threshold (currently r==g==b)
- Per-state RGBW/RGB mode override
- Additional beacon effect animations
