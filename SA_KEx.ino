#include <Entropy.h>
#include <RFM12B_arssi.h>
#include <EEPROM.h>
#include "Statistic.h"
#include "sha256.h"

#define MAX(a,b) (((a)>(b))?(a):(b))

#define NODE_ID_LOCATION    0
#define GROUP_ID_LOCATION   1
#define PARENT_ID_LOCATION  2


byte nodeID      = 0;
byte groupID     = 0;
byte parentID    = 0;

//radio module pins
#define RFM_CS_PIN  10 // RFM12B Chip Select Pin
#define RFM_IRQ_PIN 2  // RFM12B IRQ Pin
#define FREQUENCY RF12_868MHZ
//jeenodes run on 3.3 V, this value is in mV
#define BOARD_VOLTAGE 3300
#define LIGHT_PIN A4
#define LED_PIN 9

boolean root = false;

RFM12B radio;
Entropy e;
Statistic myStats;

//uint16_t counter;

typedef struct {
  int           destId; //store this nodeId
  int           srcId;
  float         data;
  uint16_t      seqNum;    // current sequence number
} Payload;

Payload theData;

#define bufferLen 200

//to parent
uint8_t uplink[bufferLen] = {0};
uint8_t uplinkCounter = 0;
float uplinkEntropy = 0;

#define CHILD_MAX 4
uint8_t childs[4][bufferLen] = {0};
uint8_t childIDs[4] = {0};
uint8_t childCounter[4] = {0};
float childEntropy[4] = {0};

void setup() {
  Serial.begin(9600);
  randomSeed(analogRead(0));
  myStats.clear(); //explicitly start clean
  Sha256.init();
  //init ID
  nodeID = EEPROM.read(NODE_ID_LOCATION);
  groupID = EEPROM.read(GROUP_ID_LOCATION);
  parentID = EEPROM.read(PARENT_ID_LOCATION);

  root = nodeID == parentID;

  //init rssi measurement - pin and iddle voltage
  radio.SetRSSI( 0, 450);
  pinMode(LIGHT_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);

  float light = analogRead(LIGHT_PIN);
  analogWrite(LED_PIN, map(light, 0, 1023, 0, 255));

  Serial.println("Setup completed");
}

void blink(int DELAY_MS){
  digitalWrite(LED_PIN,LOW);
  delay(DELAY_MS);
  digitalWrite(LED_PIN,HIGH);
}

void printHash(uint8_t* hash) {
  int i;
  for (i=0; i<32; i++) {
    Serial.print("0123456789abcdef"[hash[i]>>4]);
    Serial.print("0123456789abcdef"[hash[i]&0xf]);
  }
  Serial.println();
}


//function for initiating node
void reportRSSI(float data){
  if(root){
    return;
  }
  theData.srcId = nodeID;
  theData.destId = parentID;
  theData.seqNum = uplinkCounter;
  theData.data = data;

  radio.Send(parentID, (const void*)(&theData), sizeof(theData), true);
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

      //check recipient
      if(theData.destId != nodeID){
        return;
      }
      //from uplink
      if(theData.srcId == parentID){
        //save RSSI
        uplink[theData.seqNum] = rssi;
        uplinkCounter++;
      } else {
        //respond back
        int pair = theData.srcId;

        theData.srcId = nodeID;
        theData.destId = pair;
        radio.Send(pair, (const void*)(&theData), sizeof(theData), true);
        //save RSSI
        /*
        int childIndex = 0;
        for(int i = 0; i < CHILD_MAX; i++){
          if(childIDs[i] == pair){
            childIndex = i;
            break;
          }
          if(childIDs[i] == 0){
            childIDs[i] = pair;
            childIndex = i;
          }
          childs[childIndex][theData.seqNum] = rssi;
          childCounter[childIndex] = theData.seqNum;
          */
          //report to uplink
          reportRSSI(theData.data);
        //}
      }
    }
  }
}

//quant.
//String processRSSI(char* output, uint8_t* buffer, int len){
String processRSSI(uint8_t* buffer, int len){
  myStats.clear();
  for(int i = 0; i < len; i++){
    myStats.add(buffer[i]);
  }
  double avg = myStats.average();
  double sd = myStats.pop_stdev();

  double top = avg + sd * 0.5;
  double bottom = avg - sd * 0.5;
  String result = "";

  for(size_t i = 0; i < len; i++){
    if(buffer[i] >= top){
      //1
      result += 1;
    } else if(buffer[i] <= bottom){
      //0
      result += 0;
    }
  }
  /*
  size_t l = (result.length() / 8) * 8;

  char bits[l+1] = "";
  size_t pos = 0;
  for (size_t i = 0; i < l/8; i++){
    char res_byte = stringToChar(result.substring(i*8, (i+1)*8));
    bits[i] = res_byte;
  }
  Serial.println();
  output = bits;
  */
  return result;
}

//convert 8 bit string (1&0) to single char
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

//compress whole string (1&0) to chars
void processBits(String input, char* output){
  size_t len = (input.length() / 8) * 8;
  Serial.print("Length ");
  Serial.println(input.length());
  char bits[len+1] = "";
  size_t pos = 0;
  for (size_t i = 0; i < len/8; i++){
    char res_byte = stringToChar(input.substring(i*8, (i+1)*8));
    output[i] = res_byte;
    Serial.print("byte:");
    Serial.print(res_byte);
  }
  Serial.println();
}

#define TRESHOLD 100

void checkLight(){
  float light = analogRead(LIGHT_PIN);
  if(light > TRESHOLD){
    //send rssi
    reportRSSI(light);
  }
}

uint16_t loopCounter = 0;

void loop() {
  loopCounter++;
  delay(1);

  if(loopCounter % 500 == 0){
    blink(100);
    Serial.println(loopCounter);
    loopCounter = 0;
    checkLight();
  }
  receiveRSSI();

  //check counter values
  if(uplinkCounter >= bufferLen -10){
    String res = processRSSI(uplink, uplinkCounter);
    char* bits;
    size_t len = (res.length() / 8) * 8;
    processBits(res, bits);

    float a = e.mostCommonValueEstimate(bits, len);
    float b = e.collisionEstimate(bits, len);
    uplinkEntropy += MAX(a,b);
    Sha256.print(bits);
    Serial.print("Entropy: ");
    Serial.println(uplinkEntropy);
    printHash(Sha256.result());
    Serial.println("=============================");

  }
}
