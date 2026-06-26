#include <Arduino.h>
#include "descent_control.h"

// --- Mission Physics Constants --- (Expecting a systemic aerodynamic efficiency of 57.4%)
const float BASE_THROTTLE = 60.0f;        // % throttle to hover a 1.526 kg mass
const float TARGET_DESCENT_RATE = -9.0f;  // Downward velocity (m/s)
const float HOVER_ALTITUDE = 200.0f;      // Meters AGL (Bonus Mission 1)
const uint32_t HOVER_DURATION = 10000;    // 10 seconds in milliseconds

// --- PID Gains (Requires tuning) ---
float Kp = 2.5f; 
float Ki = 0.1f;
float Kd = 0.8f;

// --- Importing Enum from descent_control.h ---

DescentState currentState = FREEFALL_DESCENT;
float integral_error = 0.0f;
float previous_error = 0.0f;
uint32_t last_time = 0;
uint32_t hoverStartTime = 0;

float calculate_throttle(float current_alt, float current_vel) {
    uint32_t current_time = millis();
    
    // 1. Calculate Delta Time (dt) in seconds
    if (last_time == 0) { last_time = current_time; return BASE_THROTTLE; }
    float dt = (current_time - last_time) / 1000.0f; 
    if (dt > 0.1f) dt = 0.1f;
    if (dt <= 0.0f) return BASE_THROTTLE; 

    float target_vel = 0.0f;

    // 2. State Machine Logic
    switch (currentState) {
        case FREEFALL_DESCENT:
            target_vel = TARGET_DESCENT_RATE;
            if (current_alt <= HOVER_ALTITUDE) {
                currentState = HOVERING;
                hoverStartTime = current_time;
            }
            break;

        case HOVERING:
            target_vel = 0.0f; // Stop vertically
            if ((current_time - hoverStartTime) >= HOVER_DURATION) {
                currentState = FINAL_DESCENT;
            }
            break;

        case FINAL_DESCENT:
            target_vel = TARGET_DESCENT_RATE;
            
            // CRITICAL FIX #3: Debounce timer to prevent false landings
            static uint32_t landing_timer = 0;
            if (current_alt <= 5.0f) { 
                if (landing_timer == 0) landing_timer = current_time;
                
                if ((current_time - landing_timer) >= 3000) { // Must read <= 5m for 3 full seconds
                    currentState = LANDED;
                }
            } else {
                landing_timer = 0; // Reset timer if a pressure spike pushes reading above 5m
            }
            break;

        case LANDED:
            return 0.0f; // Kill motors
    }

    // 3. PID Math
    float error = target_vel - current_vel;

    float P_out = Kp * error;
    
    integral_error += error * dt;
    // ANTI-WINDUP: Prevent the integral from accumulating endlessly
    if (integral_error > 15.0f) integral_error = 15.0f;
    if (integral_error < -15.0f) integral_error = -15.0f;
    float I_out = Ki * integral_error;
    
    float derivative = (error - previous_error) / dt;
    float D_out = Kd * derivative;

    // 4. Final Throttle Calculation
    float final_throttle = BASE_THROTTLE + P_out + I_out + D_out;

    // Save memory for next loop
    previous_error = error;
    last_time = current_time;

    // 5. Hardware Clamping (Ensure we don't send impossible commands to the ESC)
    if (final_throttle > 85.0f) final_throttle = 85.0f; // Leave headroom for gyro balancing
    
    if (currentState != LANDED) {
        // Enforce the 15% floor ONLY if we are above 2.0 meters AGL
        if (current_alt > 2.0f && final_throttle < 15.0f) {
            final_throttle = 15.0f;
        } else if (final_throttle < 0.0f) {
            // Below 2.0m, allow the PID to spool down to 0%, but prevent negative numbers
            final_throttle = 0.0f;
        }
    } else {
        // Hard kill when LANDED state is fully latched
        final_throttle = 0.0f;
    }

    return final_throttle;
}