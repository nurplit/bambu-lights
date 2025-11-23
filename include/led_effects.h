#ifndef LED_EFFECTS_H
#define LED_EFFECTS_H

#include <Arduino.h>
#include "structs.h"

// Function declaration - implementation is in main.cpp
void applyEffect(PrinterStateBehavior& behavior, int progress = -1);

#endif // LED_EFFECTS_H
