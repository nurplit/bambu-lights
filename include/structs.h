#ifndef STRUCTS_H
#define STRUCTS_H

#include <Arduino.h>

// RGB color structure
struct RGB {
    int r;
    int g;
    int b;
};

// Printer status information
struct PrinterStatus {
    String stage = "unknown";
    int progress = 0;
    String gcode_state = "IDLE";  // Default to IDLE state on boot
    float bed_temp = 0.0;
    float nozzle_temp = 0.0;
    bool online = false;
    unsigned long lastUpdate = 0;
    String mc_print_stage = "unknown";
    String print_real_action = "unknown";
    int layer_num = 0;
    int total_layer_num = 0;
    String gcode_file = "";        // Current/last print file name
    String subtask_name = "";      // Current subtask
    int print_error = 0;           // Print error code (0 = no error)
};

// Printer LED behavior configuration
struct PrinterStateBehavior {
    String effect;      // "solid", "breathe", "pulse", "progress", "blink"
    RGB color;
    int brightness;     // 0-100
};

#endif // STRUCTS_H
