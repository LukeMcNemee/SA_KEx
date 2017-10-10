#include "Arduino_EntropyAssesment/Entropy.h"
#include <RFM12B_arssi.h>
#include <EEPROM.h>
#include "Statistic.h"

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
Statistic myStats;

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
  myStats.clear(); //explicitly start clean
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
  Serial.println("Setup completed");
}

void blink(byte PIN, int DELAY_MS){
  pinMode(PIN, OUTPUT);
  digitalWrite(PIN,LOW);
  delay(DELAY_MS);
  digitalWrite(PIN,HIGH);
}

void printRSSI(int8_t rssi, uint16_t counter){
  myStats.add(rssi);
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

char stringToChar(String input){
  char cur_byte_out = 0;
  for(byte j = 0; j<8; j++){
    if(input[j] == '1'){
      cur_byte_out |= 1;
    } else {
      cur_byte_out |= 0;
    }
    if(j != 7){
      cur_byte_out <<= 1;
    }
  }
  return cur_byte_out;
}

String processRSSI(){
  Serial.println(myStats.average());
  double avg = myStats.average();
  double sd = myStats.pop_stdev();
  double top = avg + sd * 0.5;
  double bottom = avg - sd * 0.5;
  String result = "";

  for(size_t i = 0; i < bufferLen; i++){
    if(buffer[i] >= top){
      //1
      result += 1;
    } else if(buffer[i] <= bottom){
      //0
      result += 0;
    }
  }
  Serial.println("Bits ready");
  Serial.println(result.length());
  return result;
}

void processBits(String input, char* output){
  size_t len = (input.length() / 8) * 8;
  char bits[len+1] = "";
  size_t pos = 0;
  for (size_t i = 0; i < len/8; i++){
    char res_byte = stringToChar(input.substring(i*8, (i+1)*8));
    output[i] = res_byte;
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  if(counter < bufferLen){
    if(nodeID == nodeA ){
      sendRSSI();
      //wait 1sec
      delay(1000);
    } else {
      receiveRSSI();
    }
  } else {
    Serial.print("Data ready");
    time1 = millis();
    String SBits = processRSSI();
    char bits [(SBits.length()/8) +1] = "";
    processBits(SBits, bits);
    double entropy = e.mostCommonValueEstimate(bits, 8);
    time2 = millis();
    Serial.print("Entropy : ");
    Serial.println(entropy);
    Serial.print("Time : ");
    Serial.println(time2-time1);
    Serial.flush();
    for(;;){
      delay(1000);
    }
  }
  blink(9,10);


}
