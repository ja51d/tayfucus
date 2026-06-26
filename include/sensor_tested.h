#ifndef SENSOR_TESTED_H
#define SENSOR_TESTED_H

// The data package that will be securely passed to the PID controller
struct SensorData {
    float altitude;
    float velocity;
};

// Function prototypes
void sensor_setup();
void sensor_loop();
SensorData get_latest_sensor_data();

#endif