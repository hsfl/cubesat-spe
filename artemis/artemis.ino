// The following libraries are used:
// SparkFun ADIN1110 Arduino Library
#include "SparkFun_SinglePairEthernet.h"
// Adafruit Unified Sensor
#include <Adafruit_Sensor.h>

// The Single-Pair Ethernet object.
SinglePairEthernet adin1110;

// The objects used to hold IMU sensor readings.
sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp;

// The time since the last sensor reading request.
unsigned long last_request;
// The single-byte payload indicating a sensor reading request from the OBC.
byte request_byte = 0x0A;

const int MAX_MSG_SIZE = 200;

// The MAC addresses for the boards.
byte artemisMAC[6] = {0x00, 0xE0, 0x22, 0xFE, 0xDA, 0xC9};
byte esp32MAC[6] = {0x00, 0xE0, 0x22, 0xFE, 0xDA, 0xCA};

// Whether a reply from the sensor board has been received.
bool recvFlag = false;

// The incoming packet from the sensor board.
byte recvPacket[MAX_MSG_SIZE];

// The callback function that runs when a packet is received.
static void rxCallback(byte * data, int dataLen, byte * senderMac)
{
  // Copy the packet's data to the local buffer for analysis.
  memcpy(recvPacket, data, dataLen);
  // Indicate that a packet has been received.
  recvFlag = true;
}

void setup() {
  // Start the serial connection for debugging.
  Serial.begin(115200);
  while(!Serial);
  
  Serial.println("CubeSat SPE Demo - OBC simulator");
  
  // Set up Ethernet object.
  if (!adin1110.begin(artemisMAC)) 
  {
    Serial.print("Failed to connect to ADIN1110 MACPHY. Make sure board is connected and pins are defined for target.");
    while(1); //If we can't connect just stop here      
  }
  Serial.println("Connected to ADIN1110 MACPHY");

  // Set the callback function, to be called when a packet is received.
  adin1110.setRxCallback(rxCallback);

  // Wait for Ethernet connection to be established.
  Serial.println("Device Configured, waiting for connection...");
  while (!adin1110.getLinkStatus());
}

void loop() {
  // If 5 seconds have passed since the last data request,
  unsigned long now = millis();
  if(now-last_request >= 5000)
  {
    // and there is an active link to the sensor board,
    if (adin1110.getLinkStatus())
    {
      Serial.println("Requesting data from sensor board...");
      // Send a request packet consisting of a single byte in its payload.
      adin1110.sendData(&request_byte, 1, esp32MAC);
      // Update the last data request counter.
      last_request = now;
    }
    else
    {
      Serial.println("Link down. Waiting to re-establish link before requesting sensor data.");
    }
  }

  // If a reply packet from the sensor board have been received,
  if(recvFlag)
  {
    // Fill the local sensor variables with the packet's payload data.
    memcpy(&accel,  recvPacket,                                 sizeof(accel));
    memcpy(&gyro,   recvPacket + sizeof(accel),                 sizeof(gyro));
    memcpy(&temp,   recvPacket + sizeof(accel) + sizeof(gyro),  sizeof(temp));

    // Print the results.
    Serial.print("\t\tTemperature ");
    Serial.print(temp.temperature);
    Serial.println(" deg C");
  
    Serial.print("\t\tAccel X: ");
    Serial.print(accel.acceleration.x);
    Serial.print(" \tY: ");
    Serial.print(accel.acceleration.y);
    Serial.print(" \tZ: ");
    Serial.print(accel.acceleration.z);
    Serial.println(" m/s^2 ");
  
    Serial.print("\t\tGyro X: ");
    Serial.print(gyro.gyro.x);
    Serial.print(" \tY: ");
    Serial.print(gyro.gyro.y);
    Serial.print(" \tZ: ");
    Serial.print(gyro.gyro.z);
    Serial.println(" radians/s ");
    Serial.println();

    // Reset the received packet flag to print the next set of readings.
    recvFlag = false;
    // Clear the contents of the received packet.
    memset(recvPacket, 0, sizeof(recvPacket));
  }
}
