#include "Entropy.h"
#include <RFM12B_arssi.h>
#include <EEPROM.h>

#define EEPROM_NODE_ID_LOCATION    0

//radio module pins
#define RFM_CS_PIN  10 // RFM12B Chip Select Pin
#define RFM_IRQ_PIN 2  // RFM12B IRQ Pin
#define FREQUENCY RF12_868MHZ
//jeenodes run on 3.3 V, this value is in mV
#define BOARD_VOLTAGE 3300

#define nodeA 1
#define nodeB 2

byte pairID = 0;

RFM12B radio;
Entropy e;

uint16_t counter;

typedef struct {
  int           nodeId; //store this nodeId
  uint16_t      seqNum;    // current sequence number
} Payload;

Payload theData;

#define bufferLen 1000
int8_t buffer[bufferLen];

unsigned long time1;
unsigned long time2;

byte nodeID = 0;

void setup() {
  Serial.begin(9600);
  randomSeed(analogRead(0));
  nodeID = EEPROM.read(EEPROM_NODE_ID_LOCATION);

  //init rssi measurement - pin and iddle voltage
  radio.SetRSSI( 0, 450);

  if(nodeID == nodeA || nodeID == nodeB){
    radio.Initialize(nodeID, FREQUENCY, 200);
  } else {
    radio.Initialize(0, FREQUENCY, 200);
  }
  if(nodeID == nodeA){
    pairID = nodeB;
  } else {
    pairID = nodeA;
  }
}

void blink(byte PIN, int DELAY_MS){
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,LOW);
  delay(DELAY_MS);
  digitalWrite(PIN,HIGH);
}

void printRSSI(int8_t rssi, uint16_t counter){
  Serial.print(rssi);
  Serial.print(",");
  Serial.print(counter);
  Serial.print(";");
}


//function for initiating node
void sendRSSI(){
  theData.nodeId = nodeID;
  theData.seqNum = counter;
  radio.Send(pairID, (const void*)(&theData), sizeof(theData), true);

  for(int i = 0; i < 10; i++){
    if (radio.ReceiveComplete()){
      if (radio.CRCPass()){
        int8_t rssi = radio.ReadARSSI(BOARD_VOLTAGE);

        if (*radio.DataLen != sizeof(Payload)){
          continue;
        }
        theData = *(Payload*)radio.Data; //assume radio.DATA actually contains our struct and not something else

        printRSSI(rssi, theData.seqNum);
        buffer[theData.seqNum] = rssi;
        counter++;
        return;
      }
    }
    delay(10);
  }
}

//function for responder node
void receiveRSSI(){
  if (radio.ReceiveComplete()){
    if (radio.CRCPass()){
      int8_t rssi = radio.ReadARSSI(BOARD_VOLTAGE);
      if (*radio.DataLen != sizeof(Payload)){
        return;
      }
      theData = *(Payload*)radio.Data; //assume radio.DATA actually contains our struct and not something else

      printRSSI(rssi, theData.seqNum);
      buffer[theData.seqNum] = rssi;
      //send same data back
      theData.nodeId = nodeID;
      radio.Send(pairID, (const void*)(&theData), sizeof(theData), true);
    }
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println("Program ready");

}
