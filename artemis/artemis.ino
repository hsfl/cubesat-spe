#include "SparkFun_SinglePairEthernet.h"
// Adafruit Unified Sensor

#define REQUEST 0x0A;

SinglePairEthernet adin1110;

unsigned long last_report;

const int MAX_MSG_SIZE = 200;

byte artemisMAC[6] = {0x00, 0xE0, 0x22, 0xFE, 0xDA, 0xC9};
byte esp32MAC[6] = {0x00, 0xE0, 0x22, 0xFE, 0xDA, 0xCA};

bool xmitFlag = false;
bool recvFlag = false;

byte xmitPacket[MAX_MSG_SIZE];
byte recvPacket[MAX_MSG_SIZE];

sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp;

static void rxCallback(byte * data, int dataLen, byte * senderMac)
{
  memcpy(recvPacket, data, dataLen);
  recvFlag = true;
}

void setup() {
  memset(recvBuffer, 0, sizeof(recvBuffer));
  Serial.begin(115200);
  while(!Serial);
  
  Serial.println("CubeSat SPE Demo - OBC simulator");
  /* Start up adin1110 */
  if (!adin1110.begin(artemisMAC)) 
  {
    Serial.print("Failed to connect to ADIN1110 MACPHY. Make sure board is connected and pins are defined for target.");
    while(1); //If we can't connect just stop here      
  }
  Serial.println("Connected to ADIN1110 MACPHY");

  /* Set up callback, to control what we do when data is recieved */
  adin1110.setRxCallback(rxCallback);

  /* Wait for link to be established */
  Serial.println("Device Configured, waiting for connection...");
  while (adin1110.getLinkStatus() != true);
}

void loop() {
  unsigned long now = millis();
  
  if(now-last_report >= 5000)
  {
    if (adin1110.getLinkStatus())
    {
      Serial.println("Requesting data from sensor board...");
      memset(xmitPacket, 0, sizeof(xmitPacket));
      memcpy(xmitPacket, REQUEST, 1);
      adin1110.sendData(xmitPacket, 1);
      last_report = now;
    }
    else
    {
      Serial.println("Waiting for link to resume sending");
    }
  }

  if(recvFlag)
  {
    memcpy(accel, recvPacket, sizeof(accel));
    memcpy(gyro, recvPacket + sizeof(accel), sizeof(gyro));
    memcpy(temp, recvPacket + sizeof(accel) + sizeof(gyro), sizeof(temp));
    
    Serial.print("\t\tTemperature ");
    Serial.print(temp.temperature);
    Serial.println(" deg C");
  
    /* Display the results (acceleration is measured in m/s^2) */
    Serial.print("\t\tAccel X: ");
    Serial.print(accel.acceleration.x);
    Serial.print(" \tY: ");
    Serial.print(accel.acceleration.y);
    Serial.print(" \tZ: ");
    Serial.print(accel.acceleration.z);
    Serial.println(" m/s^2 ");
  
    /* Display the results (rotation is measured in rad/s) */
    Serial.print("\t\tGyro X: ");
    Serial.print(gyro.gyro.x);
    Serial.print(" \tY: ");
    Serial.print(gyro.gyro.y);
    Serial.print(" \tZ: ");
    Serial.print(gyro.gyro.z);
    Serial.println(" radians/s ");
    Serial.println();

    recvFlag = false;
  }
}
