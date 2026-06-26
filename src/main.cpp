#include <Arduino.h>
#include <math.h> // Required for fabsf()
#include "sensor_tested.h"
#include "descent_control.h"
#include "msp_link.h"

void setup() {
    sensor_setup();
    msp_setup(); 
}

void loop() {
    sensor_loop();
    
    // BUG 9 FIX: 50Hz Rate Gate for MSP [cite: 956, 957, 958]
    static uint32_t last_msp_send = 0;
    if (millis() - last_msp_send >= 20) { 
        last_msp_send = millis();
        
        SensorData flight_data = get_latest_sensor_data();
        
        // Safer float comparison for the -999.0f sentinel 
        if (fabsf(flight_data.altitude - (-999.0f)) > 0.5f) {
            float required_throttle = calculate_throttle(flight_data.altitude, flight_data.velocity);
            send_msp_rc(required_throttle);
        }
    }
}