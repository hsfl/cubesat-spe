#include "SparkFun_SinglePairEthernet.h"

SinglePairEthernet adin1110;

const int MAX_MSG_SIZE = 200;

byte artemisMAC[6] = {0x00, 0xE0, 0x22, 0xFE, 0xDA, 0xC9};
byte esp32MAC[6] = {0x00, 0xE0, 0x22, 0xFE, 0xDA, 0xCA};

char recvBuffer[MAX_MSG_SIZE];

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
  /* If we are still connected, send a message */
  if(adin1110.getLinkStatus())
  {      
      Serial.print("Sending:\t");
      Serial.println(outputString[msg]); //This is ok since we know they are all null terminated strings
      
      adin1110.sendData((byte *)outputString[msg], sizeof(outputString[msg]), destinationMAC);
      
      msg++;
      if(msg >= NUM_MSGS)
      {
        msg = 0;
      }
  }
  else
  {
      Serial.println("Waiting for link to resume sending");
  }
}
