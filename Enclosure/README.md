# 🔧 ESP32 Enclosure for Bambu Lights

## Overview

Custom 3D printed enclosure designed to mount on the [BLV AMS Riser for X1C/P1S/P2S](https://makerworld.com/en/models/19535-blv-ams-ams-2-riser-for-x1c-p1s-p2s-v4#profileId-19420).

## Files Included

### STL Files (Print-Ready)
- `ESP32_Enclosure_Main.stl` - Main enclosure body
- `ESP32_Enclosure_Lid.stl` - Lid with ventilation
- `Beacon_Mount_10LED.stl` - Mount for 10-pixel beacon LED strip

### Source Files (Editable)
- `Bambu_Lights_Enclosure.f3d` - Fusion 360 project file
  - Fully parametric design
  - Easy to modify for different ESP32 boards or mounting solutions

## Print Settings

**Recommended:**
- **Material:** PETG or ABS (for heat resistance near printer)
- **Layer Height:** 0.2mm
- **Infill:** 20%
- **Supports:** Required for lid ventilation slots
- **Orientation:** Print main body upside down

**Notes:**
- No supports needed for main body if printed upside down
- Lid requires supports for ventilation grilles
- Test fit before final assembly

## Assembly

1. Insert ESP32 board into main enclosure
2. Route LED strip wires through cable management slots
3. Mount beacon LED strip (10 pixels recommended) to beacon mount
4. Secure lid with M3 screws (optional - can snap fit)
5. Mount assembled unit to AMS riser using provided mounting holes

## Mounting to AMS Riser

The enclosure is designed to bolt directly to the BLV AMS Riser:
- **Hardware:** 2x M3x8mm screws
- **Position:** Side mount for easy access to USB port
- **Cable routing:** Integrated channels for clean wire management

## Customization

### Editing in Fusion 360
1. Open `Bambu_Lights_Enclosure.f3d` in Fusion 360
2. Modify parameters in the "User Parameters" section:
   - `board_width` - ESP32 board width
   - `board_length` - ESP32 board length
   - `wall_thickness` - Enclosure wall thickness
   - `ventilation_slot_width` - Adjust airflow
3. Update and export new STL files

### Common Modifications
- **Different ESP32 board:** Adjust board dimensions in parameters
- **Larger beacon:** Modify `Beacon_Mount_10LED.stl` for more LEDs
- **Alternative mounting:** Edit mounting hole positions

## Tips

- **Airflow:** Keep ventilation slots clear for ESP32 cooling
- **Wire management:** Use provided cable channels to prevent interference with printer
- **Beacon placement:** Position for clear visibility from front of printer
- **USB access:** Enclosure designed for easy access to USB port for updates

## License

These files are released under the same MIT License as the main project. Feel free to modify and share!

## Credits

Designed to complement the [BLV AMS Riser](https://makerworld.com/en/models/19535) by BLV.
