#ifndef MSP_LINK_H
#define MSP_LINK_H

#include <Arduino.h>

// Initialize the UART hardware serial port
void msp_setup();

// Takes the computed throttle percentage (0.0 to 100.0) 
// and transmits the MSP_SET_RAW_RC packet
void send_msp_rc(float throttle_percent);

#endif