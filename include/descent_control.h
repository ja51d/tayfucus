#ifndef DESCENT_CONTROL_H
#define DESCENT_CONTROL_H

#include <Arduino.h>

// --- State Machine Definitions ---
// Exposing this allows main.cpp and your telemetry module to read the current flight phase.
enum DescentState {
    FREEFALL_DESCENT, // Translates to Telemetry Status: 4
    HOVERING,         // Translates to Telemetry Status: 4
    FINAL_DESCENT,    // Translates to Telemetry Status: 4
    LANDED            // Translates to Telemetry Status: 5
};

// --- Global Variable Access (Externs) ---
// 'extern' tells the compiler these variables exist in the .cpp file, 
// allowing main.cpp to read them without creating duplicate copies in memory.
extern DescentState currentState;

// Exposing the gains allows you to potentially update them via 
// ground station commands later without reflashing the board.
extern float Kp;
extern float Ki;
extern float Kd;

// --- Core Function Prototype ---
// Takes the current altitude (m) and velocity (m/s), 
// returns the computed throttle percentage (0.0 to 100.0).
float calculate_throttle(float current_alt, float current_vel);

#endif