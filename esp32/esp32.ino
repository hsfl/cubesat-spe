// The following libraries are used:
// SparkFun ADIN1110 Arduino Library
#include "SparkFun_SinglePairEthernet.h"
// Adafruit LSM6DS
// Adafruit BusIO
// Adafruit Unified Sensor
#include <Adafruit_LSM6DSOX.h>
// Adafruit LIS3MDL
#include <Adafruit_LIS3MDL.h>

// The Single-Pair Ethernet object.
SinglePairEthernet  adin1110;
// The IMU object.
Adafruit_LSM6DSOX   imu;
// The magnetometer object.
Adafruit_LIS3MDL    magnetometer;

// The objects used to hold IMU sensor readings.
sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp;

// The object used to hold magnetometer readings.
sensors_event_t magnet; 

// The single-byte payload indicating a sensor reading request from the OBC.
byte request_byte = 0x0A;

const int MAX_MSG_SIZE = 200;

// The MAC addresses for the boards.
byte artemisMAC[6] = {0x00, 0xE0, 0x22, 0xFE, 0xDA, 0xC9};
byte esp32MAC[6] = {0x00, 0xE0, 0x22, 0xFE, 0xDA, 0xCA};

// Whether the board has had sensor readings requested of it.
bool requestFlag = false;
// The outgoing packet to the OBC board.
byte xmitPacket[MAX_MSG_SIZE];

// The callback function that runs when a packet is received.
// The ESP32 has a callback watchdog timer that will reset the ESP32
// if the function takes too long.
static void rxCallback(byte * data, int dataLen, byte * senderMac)
{
  // Check the first byte of the packet for a sensor readings request.
  if(data[0] == request_byte)
  {
    // Set the request flag if this byte has been received.
    requestFlag = true;
  }
}

void setup() {
  // Start the serial connection for debugging.
  Serial.begin(115200);
  while(!Serial);
  
  Serial.println("CubeSat SPE Demo - Sensor Board simulator");
  
  // Set up Ethernet object.
  if (!adin1110.begin(esp32MAC)) 
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

  Wire.begin();

  // Wait for IMU to be initialized.
  Serial.println("Waiting for IMU to initialize...");
  while (!imu.begin_I2C());

  // Wait for IMU to be initialized.
  Serial.println("Waiting for magnetometer to initialize...");
  while (!magnetometer.begin_I2C());
}

void loop() {
  // If sensor readings are requested,
  if(requestFlag)
  {
    // and there is an active link to the OBC board,
    if(adin1110.getLinkStatus())
    {
      Serial.println("Got sensor data request from OBC board. Replying...");
      // poll the IMU and magnetometer for their sensor readings.
      imu.getEvent(&accel, &gyro, &temp);
      magnetometer.getEvent(&magnet);
      // Clear the outgoing packet.
      memset(xmitPacket, 0, sizeof(xmitPacket));
      // Copy the sensor readings into the packet's payload.
      memcpy(xmitPacket,                                                &accel,   sizeof(accel));
      memcpy(xmitPacket + sizeof(accel),                                &gyro,    sizeof(gyro));
      memcpy(xmitPacket + sizeof(accel) + sizeof(gyro),                 &temp,    sizeof(temp));
      memcpy(xmitPacket + sizeof(accel) + sizeof(gyro) + sizeof(temp),  &magnet,  sizeof(magnet));
      // Send the packet to the OBC board.
      adin1110.sendData((byte *)xmitPacket, sizeof(xmitPacket), artemisMAC);
      // Reset the sensor data request flag to await another request.
      requestFlag = false;
    }
    else
    {
      Serial.println("Link down. Waiting to re-establish link before handling sensor data request.");
    }
  }
}
