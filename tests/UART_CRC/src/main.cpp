
#include <Arduino.h>
#include <string.h>

/*
All of the following are little endian

    Sending frame structure:
    - 8 bits sof (0xAE)
    - 8 bits sequence number (wraps around at 0xFF, sequence gaps are permitted, and repeated sequence numbers indicate stalled telemetry)
    - 16 bits BMS Current (signed) -- mA
    - 16 bits BMS output voltage (unsigned) -- mV
    - 16 bits Total Battery Voltage (unsigned) -- mV
    - 16 bits Temp (signed) -- centi celsius
    - 8 bits ERROR (say bms turned off or smth, bit field, reflects current state)
    - 8 bits CRC8
        - 0x07 -- x^8 + x^2 + x + 1
        - does not cover sof
        - Init value: 0x00
        - RefIn: false
        - RefOut: false
        - XorOut: 0x00
        - Coverage: all bytes except SOF
        
    ERROR format
        bit 0  : BMS off
        bit 1  : Overcurrent
        bit 2  : Undervoltage
        bit 3  : Overtemperature
        bit 4  : Comm fault
        bit 5  : Sensor fault
        bit 6–7: Reserved
    When error is set, call interrupt to make master immediately push a telemetry request

    Recieved frame structure:
    - 8 bits sof 
    - 8 bits command
    - 8 bits notted command
    - 8 bits crc
    SOF is AE
    
    Command structure:
        bit 7   = 1 -> command
        bit 6   = safety-critical (STOP_SYSTEM, STOP_THRUSTERS)
        bit 5-0 = command ID

    Commands: (All have minimum distance of 3)
    1. STOP_ELECTRONICS: No ACK, just shut down -- does not require crc match 
        11 000000
    2. STOP_THRUSTERS: ACK -- does not require crc match
        11 000111
    3. START_THRUSTERS: ACK -- requires crc match
        10 111000
    4. TELEMETRY: response frame -- requires crc match
        10 111111
    5. RESERVED (Safety non critical)
        10 010101
    6. RESERVED (Safety critical)
        11 101010

    ACK frame:
        Single byte: 0xA5 (4 bit ack with inverse appended on)
        Meaning: Command received and accepted
        Does NOT imply action completed 
        Make sure to query on master side
    NACK frame:
        Single byte: 0x5A
        Meaning: Command recieved BUT NOT accepted due to various reasons
            - CMD / ~CMD mismatch
            - Reserved command ID
            - START_THRUSTERS while fault is latched 
            - i.e, if the command is perfect, but the logic doesnt work or the command doesnt exist, send NACK. If malformed, let master time out
    ACK/NACK is sent immediately after command validation. No response is guaranteed for STOP_ELECTRONICS
*/

String receivedMessage = "";

void setup() {
  Serial.begin(115200);
  
  Serial.println("ESP32 is ready. Please enter a message:");
}

void loop() {
  while (Serial.available()) {
    char incomingChar = Serial.read();
    
    if (incomingChar == '\n') { 
      Serial.print("You sent: ");
      Serial.println(receivedMessage);
      
      receivedMessage = "";
    } else {
      receivedMessage += incomingChar;
    }
  }
}
