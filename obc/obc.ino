// Choose which sensors are connected to the OBC board.
//#define SPARKFUN                                // SparkFun sensors.
#define ADAFRUIT                              // Adafruit sensors.

// The following libraries are used:
#include "SparkFun_SinglePairEthernet.h"        // SparkFun ADIN1110 Arduino Library
#include "sfe_spe_advanced.h"
#ifdef ADAFRUIT
#include <Adafruit_Sensor.h>                    // Adafruit Unified Sensor
#endif

SinglePairEthernet  adin1110;                   // The Single-Pair Ethernet object.
#ifdef ADAFRUIT
sensors_event_t     accel;                      // The objects used to hold IMU sensor readings.
sensors_event_t     gyro;
sensors_event_t     temp;
sensors_event_t     magnet;                     // The object used to hold magnetometer readings.
#elif defined(SPARKFUN)
float               accel[3];                   // An array of IMU readings. Stores X, Y, Z values in that order.
float               gyro[3];        
float               temp;
double              magnet[3];                  // An array of magnetometer readings. Stores X, Y, Z values in that order.
#endif

const int MAX_MSG_SIZE = 200;                   // The maximum size, in bytes, of a packet payload.
const byte request_byte = 0x0A;                 // The first byte of a payload indicating a sensor reading request from the OBC.

unsigned long last_request;                     // The time since the last sensor reading request.
bool recvFlag;                                  // Whether a reply from the sensor board has been received.

byte xmitPacket[MAX_MSG_SIZE];                  // The payload indicating a sensor reading request from to the sensor board.
byte recvPacket[MAX_MSG_SIZE];                  // The incoming payload from the sensor board.

byte obcMAC[6] = {0x00, 0xE0, 0x22, 0xFE, 0xDA, 0xC9};  // The MAC addresses for the boards.
byte sensorMAC[6] = {0x00, 0xE0, 0x22, 0xFE, 0xDA, 0xCA};

// The callback function that runs when a packet is received.
static void rxCallback(byte * data, int dataLen, byte * senderMac)
{
  // Copy the packet's payload data to the local buffer for printing.
  memcpy((byte *)recvPacket, data, dataLen);
  // Indicate that a packet has been received.
  recvFlag = true;
}

void setup() {
  // Start the serial connection.
  Serial.begin(115200);
  while(!Serial);
  
  Serial.println("CubeSat SPE Demo - OBC simulator");
  
  // Set up Ethernet object.
  if (!adin1110.begin(obcMAC)) 
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

  // Clear the transmit packet and set the first byte to 0x0A, the "request byte".
  memset(&xmitPacket, 0, sizeof(xmitPacket));
  memset(&xmitPacket, request_byte, 1);

  // Reset the receive flag for first loop.
  recvFlag = false;
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
      // Send a request packet. The minimum size of the payload must be 46 bytes.
      adin1110.sendData(xmitPacket, MAX_MSG_SIZE, sensorMAC);
      // Update the last data request counter.
      last_request = now;
    }
    else
    {
      Serial.println("Link down. Waiting to re-establish link before requesting sensor data.");
    }
  }
  
  // If a reply packet from the sensor board has been received,
  if(recvFlag)
  {
    // Fill the local sensor variables with the packet's payload data.
    memcpy(&accel,    recvPacket,                                               sizeof(accel));
    memcpy(&gyro,     recvPacket + sizeof(accel),                               sizeof(gyro));
    memcpy(&temp,     recvPacket + sizeof(accel) + sizeof(gyro),                sizeof(temp));
    memcpy(&magnet,   recvPacket + sizeof(accel) + sizeof(gyro) + sizeof(temp), sizeof(magnet));

    // Print the results.
    Serial.println("Got reply:");
    #ifdef ADAFRUIT
    Serial.print("Temperature ");
    Serial.print(temp.temperature);
    Serial.println(" deg C");
  
    Serial.print("Accel \tX: ");
    Serial.print(accel.acceleration.x);
    Serial.print(" \tY: ");
    Serial.print(accel.acceleration.y);
    Serial.print(" \tZ: ");
    Serial.print(accel.acceleration.z);
    Serial.println(" m/s^2 ");
  
    Serial.print("Gyro \tX: ");
    Serial.print(gyro.gyro.x);
    Serial.print(" \tY: ");
    Serial.print(gyro.gyro.y);
    Serial.print(" \tZ: ");
    Serial.print(gyro.gyro.z);
    Serial.println(" radians/s ");
    
    Serial.print("Magnet \tX: "); 
    Serial.print(magnet.magnetic.x);
    Serial.print(" \tY: "); 
    Serial.print(magnet.magnetic.y); 
    Serial.print(" \tZ: "); 
    Serial.print(magnet.magnetic.z); 
    Serial.println(" uTesla ");
    Serial.println();
    #elif defined(SPARKFUN) 
    Serial.print("Temperature ");
    Serial.print(temp);
    Serial.println(" deg C");
    
    Serial.print("Accel \tX: ");
    Serial.print(accel[0]);
    Serial.print(" \tY: ");
    Serial.print(accel[1]);
    Serial.print(" \tZ: ");
    Serial.print(accel[2]);
    Serial.println(" m/s^2 ");
  
    Serial.print("Gyro \tX: ");
    Serial.print(gyro[0]);
    Serial.print(" \tY: ");
    Serial.print(gyro[1]);
    Serial.print(" \tZ: ");
    Serial.print(gyro[2]);
    Serial.println(" radians/s ");
    
    Serial.print("Magnet \tX: "); 
    Serial.print(magnet[0]);
    Serial.print(" \tY: "); 
    Serial.print(magnet[1]); 
    Serial.print(" \tZ: "); 
    Serial.print(magnet[2]); 
    Serial.println(" uTesla ");
    Serial.println();
    #endif

    // Reset the received packet flag to print the next set of readings.
    recvFlag = false;
    // Clear the contents of the received packet.
    memset(recvPacket, 0, sizeof(recvPacket));
  }
}
