#include "msp_link.h"

// Hardware Serial2 on Teensy 4.1 (Pin 7 = RX2, Pin 8 = TX2)
#define MSP_SERIAL Serial2
#define MSP_BAUD 115200

void msp_setup() {
    MSP_SERIAL.begin(MSP_BAUD);
}

void send_msp_rc(float throttle_percent) {
    // 1. Map throttle percentage (0-100) to RC microsecond range (1000-2000)
    uint16_t throttle_rc = 1000 + (uint16_t)(throttle_percent * 10.0f);
    
    // Safety clamp (Hardware limits)
    if (throttle_rc > 2000) throttle_rc = 2000;
    if (throttle_rc < 1000) throttle_rc = 1000;

    // 2. Set the 8 Standard RC Channels (AETR1234 mapping)
    // Betaflight AETR1234 Mapping
    uint16_t roll     = 1500;
    uint16_t pitch    = 1500;
    uint16_t throttle = throttle_rc; // Ch3
    uint16_t yaw      = 1500;        // Ch4
    
    // BUG 2 FIX: Gating the ARM switch. 
    // WARNING: aux1 = 2000 makes the FC instantly LIVE. Propellers MUST be off.
    //Previously: uint16_t aux1     = 2000;        // ARM [cite: 925]
    // CRITICAL FIX #4: Dynamically disarm the FC when throttle reaches absolute zero
    // (This automatically triggers when currentState == LANDED)
    uint16_t aux1 = (throttle_percent <= 0.0f) ? 1000 : 2000;
    uint16_t aux2     = 1000;
    uint16_t aux3     = 1000;
    uint16_t aux4     = 1000;

    // 3. Construct the Payload Array (Little-Endian formatting)
    uint8_t payload[16];
    // BUG 1 FIX: Correct AETR packing order [cite: 918, 919, 920, 921, 922]
    payload[0]  = roll & 0xFF;        payload[1]  = (roll >> 8) & 0xFF;
    payload[2]  = pitch & 0xFF;       payload[3]  = (pitch >> 8) & 0xFF;
    payload[4]  = throttle & 0xFF;    payload[5]  = (throttle >> 8) & 0xFF; // Ch3 = Throttle
    payload[6]  = yaw & 0xFF;         payload[7]  = (yaw >> 8) & 0xFF;      // Ch4 = Yaw
    payload[8]  = aux1 & 0xFF;        payload[9]  = (aux1 >> 8) & 0xFF;
    payload[10] = aux2 & 0xFF;        payload[11] = (aux2 >> 8) & 0xFF;
    payload[12] = aux3 & 0xFF;        payload[13] = (aux3 >> 8) & 0xFF;
    payload[14] = aux4 & 0xFF;        payload[15] = (aux4 >> 8) & 0xFF;

    // 4. Define MSP Packet Headers
    uint8_t size = 16;
    uint8_t command = 200; // Command 200 is MSP_SET_RAW_RC
    uint8_t checksum = size ^ command;

    // 5. Send Header
    MSP_SERIAL.write('$');
    MSP_SERIAL.write('M');
    MSP_SERIAL.write('<');
    MSP_SERIAL.write(size);
    MSP_SERIAL.write(command);

    // 6. Send Payload & Calculate XOR Checksum
    for (int i = 0; i < 16; i++) {
        MSP_SERIAL.write(payload[i]);
        checksum ^= payload[i];
    }

    // 7. Send Checksum
    MSP_SERIAL.write(checksum);
}