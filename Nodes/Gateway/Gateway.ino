#include <stdio.h>     
#include <stdlib.h>     
#include <iostream>
#include <RadioLib.h>
#include <Wire.h>
#include "RTClib.h"

RTC_DS3231 rtc;

DateTime defaultTime = DateTime(2021,07,18,21,34,20);

RFM95 radio = new Module(10, 8, 9, 7);

const int deviceID = 255;
int numGateways = 7;
int regionArr[3][4]={{0,   0 ,  1 ,  2},
                   {255, 3,   4,   0},
                   {0,   5,   6,   0}};

auto shortestPathTable = new int [numGateways][3];
auto unvisitedNodes = new int [numGateways];

const int rowMax = (sizeof(regionArr)/sizeof(regionArr[0]))-1;
const int colMax = (sizeof(regionArr[0])/sizeof(int))-1;

//channel                        13    14    15    16    17
float channelFrequencies[4] = {866.1,866.4,866.7,867.0,868.0};

// flag to indicate that a packet was received
volatile bool receivedFlag = false;

// disable interrupt when it's not needed
volatile bool enableInterrupt = true;

// digital pin 2 is the hall pin
int hall_pin = 2;
// set number of hall trips for RPM reading (higher improves accuracy)
float hall_thresh = 10.0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(hall_pin, INPUT);
  pinMode(1, OUTPUT);

  Wire.begin();
  initRTC(defaultTime);
  //rtc.adjust(defaultTime);

  loraInit();
}

void sampleWindSpeed(float sampleTime){
  Serial.println("Sampling...");
  bool triggered = false;
  float hall_count = 0.0;
  float start = millis();

  while((millis()-start)<(sampleTime*1000)){
    if (digitalRead(hall_pin)==0){
      if(triggered == false){
        triggered = true;
        hall_count++;
        Serial.print(hall_count);Serial.print(",");
      }
    }
    if(digitalRead(hall_pin)==1){
      triggered = false;
    }   
  }
    float rps = hall_count/(sampleTime);
    float metersPerSecond = rps*0.28274;
    Serial.println();
    Serial.print("RPS: ");Serial.print(rps);
    Serial.println();
    Serial.print("m/s: ");Serial.print(metersPerSecond);
    Serial.println();Serial.println();
    
  }

void loop() {
  float sampleTime = 10.0;
  sampleWindSpeed(sampleTime);
  /*if(digitalRead(hall_pin)==0){
    digitalWrite(1, HIGH);
  }
  else{
    digitalWrite(1, LOW);
  }*/
}

//LoRa Functions
/**********************************************************************/
void loraInit(){
    Serial.print(F("[RFM95] Initializing ... "));
    int state = radio.begin();
    //channel 17
    radio.setFrequency(channelFrequencies[4]);
    radio.setSpreadingFactor(7);
    radio.setBandwidth(250);
    radio.setOutputPower(13);

    radio.setDio0Action(setFlag);
    
    if (state == ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    while (true);
  } 
}

void setFlag(void) {
  // check if the interrupt is enabled
  if(!enableInterrupt) {
    return;
  }
  // we got a packet, set the flag
  receivedFlag = true;
}

void transmitData(int channelNum, byte *data){
  radio.setFrequency(channelFrequencies[channelNum-13]);
  int state = radio.transmit(data);

  if (state == ERR_NONE) {
      // the packet was successfully transmitted
      Serial.println(F(" success!"));
      // print measured data rate
      /*Serial.print(F("[RFM95] Datarate:\t"));*/
      Serial.print(radio.getDataRate());
      Serial.print(F(" bps"));
      Serial.println();
 
    } else if (state == ERR_PACKET_TOO_LONG) {
      // the supplied packet was longer than 256 bytes
      Serial.println(F("too long!"));
  
    } else if (state == ERR_TX_TIMEOUT) {
      // timeout occurred while transmitting packet
      Serial.println(F("timeout!"));
  
    } else {
      // some other error occurred
      Serial.print(F("failed, code "));
      Serial.println(state);
    }
}

byte * checkReceive(){
  if(receivedFlag) {
    //testStarted=1;
    // disable the interrupt service routine while
    // processing the data
    
    enableInterrupt = false;

    // reset flag
    receivedFlag = false;

    // you can read received data as an Arduino String
    byte data;
    int state = radio.readData(data);
    
    // you can also read received data as byte array
    /*
      byte byteArr[8];
      int state = radio.readData(byteArr, 8);
    */

    if (state == ERR_NONE) {
      // packet was successfully received
      //Serial.println(F("[RFM95] Received packet!"));

      // print data of the packet
      //Serial.print(F("[RFM95] Data:\t\t"));
      Serial.println();
      Serial.print(radio.getSNR());Serial.print(", ");Serial.print(radio.getRSSI());//Serial.print(",");Serial.print(str);

      // print RSSI (Received Signal Strength Indicator)
      //Serial.print(F("[RFM95] RSSI:\t\t"));
      //Serial.print(radio.getRSSI());
      //Serial.println(F(" dBm"));

      // print SNR (Signal-to-Noise Ratio)
      //Serial.print(F("[RFM95] SNR:\t\t"));
      //Serial.print(radio.getSNR());
      //Serial.println(F(" dB"));
  /*
      // print frequency error
      Serial.print(F("[RFM95] Frequency error:\t"));
      Serial.print(radio.getFrequencyError(true));
      Serial.println(F(" Hz"));

      Serial.print(F("[RFM95] Packet length:\t"));
      Serial.print(radio.getPacketLength());
      Serial.println(F(""));
      */
    } else if (state == ERR_CRC_MISMATCH) {
      // packet was received, but is malformed
      Serial.println(F("[RFM95] CRC error!"));

    } else {
      // some other error occurred
      Serial.print(F("[RFM95] Failed, code "));
      Serial.println(state);

    }
    
    //Serial.println();
    //Serial.println(F("[RFM95] Starting to listen ... "));
  
    radio.startReceive();
    

    // we're ready to receive more packets,
    // enable interrupt service routine
    enableInterrupt = true;
    
  }
  return data;  
}


/**********************************************************************/

//dijkstra functions
/**********************************************************************/
int *findNodeLocation(int deviceID){
  int location[1];
  for(int i = 0; i<rowMax; i++){
    for(int j = 0; j<colMax; j++){
      if(regionArr[i][j]==deviceID){
        location[0] = i; //row
        location[1] = j; //col
      }
    }  
  }
  return location;
}

void updateShortestPathTable(int neighbourNodeID, int anchorNodeID, int vectorWeight){
    //if ID is negative...i.e. not reachable
    if (neighbourNodeID!=0){
      neighbourNodeID = abs(neighbourNodeID);
      shortestPathTable[neighbourNodeID - 1][0] = -neighbourNodeID;
      unvisitedNodes[neighbourNodeID - 1] = -neighbourNodeID;
      vectorWeight = 255;
    }
    
    if(neighbourNodeID==255){
      neighbourNodeID = numGateways;
    }
    if(anchorNodeID==255){
      anchorNodeID = numGateways;
    }
    int weightSum = shortestPathTable[anchorNodeID-1][1] + vectorWeight;
    if(weightSum<shortestPathTable[neighbourNodeID-1][1]){
      shortestPathTable[neighbourNodeID-1][1]=weightSum;
      shortestPathTable[neighbourNodeID-1][2]=anchorNodeID;
    }
}

int findNextSmallestFromTable(){
    int currentMinDist = 255;
    int currentMinID = 0;
    for(int i = 0; i<numGateways; i++){
      if(isInArray(shortestPathTable[i][0])){
        currentMinDist = shortestPathTable[i][1];
        currentMinID = i;
      }
    }
    return currentMinID+1;
}

bool isInArray(int targetID){
  for(int i = 0; i<numGateways; i++){
    if (unvisitedNodes[i]==targetID){
      return true;
    }
  }
  return false;
}

void resetArraysAndTables(){
  for (int i = 0; i < rowMax; i++){
    for (int j = 0; j < colMax; j++){
        shortestPathTable[i][j] = 0;
    }
  }
  for (int i = 0; i < numGateways; i++){
    unvisitedNodes[i] = 0;
  }
}

void dijkstraTable(int chosenID){
  resetArraysAndTables();
  //populates shortest path table
  for(int i = 0;i<numGateways;i++){
    shortestPathTable[i][0] = i+1;
    if (i==numGateways-1){
      shortestPathTable[i][0] = 255;
    }
    if (shortestPathTable[i][0] == chosenNodeID){
      shortestPathTable[i][1] = 0;
    }
    else{
      shortestPathTable[i][1] = 255;
    }
  }
  //populates unvisited array
  for(int i = 0; i<numGateways; i++){
    unvisitedNodes[i] = shortestPathTable[i][0];
  }

  //start process
  //goes through every node in table
  for(int i = 0; i<numGateways; i++){
    int currSelectedID = findNextSmallestFromTable();
    int selectedIDTableDist = shortestPathTable[currSelectedID-1][1];

    if(currSelectedID==numGateways){
      currSelectedID = 255;
    }
    int loc[1] = findNodeLocation(currSelectedID);
    int row = loc[0]; 
    int col = loc[1];

    //check current selected ID neighbours
    //1 up
    if(row-1>=0){
      neighbourNodeID = regionArray[row-1][col];
      updateShortestPathTable(neighbourNodeID, currSelectedID,2);
    }
    //2 up-right
    if(row-1>=0 && col+1<=colMax){
      neighbourNodeID = regionArray[row-1][col+1];
      updateShortestPathTable(neighbourNodeID, currSelectedID,3);
    }
    //3 right
    if(col+1<=colMax){
      neighbourNodeID = regionArray[row][col+1];
      updateShortestPathTable(neighbourNodeID, currSelectedID,2);
    }
    //4 down-right
    if(row+1<=rowMax and col+1<=colMax){
      neighbourNodeID = regionArray[row+1][col+1];
      updateShortestPathTable(neighbourNodeID, currSelectedID,3);
    }
    //5 down
    if(row+1<=rowMax){
      neighbourNodeID = regionArray[row+1][col];
      updateShortestPathTable(neighbourNodeID, currSelectedID,2);
    }
    //6 down-left
    if(row+1<=rowMax and col-1>=0){
      neighbourNodeID = regionArray[row+1][col-1];
      updateShortestPathTable(neighbourNodeID, currSelectedID,3);
    }
    //7 left
    if(col-1>=0){
      neighbourNodeID = regionArray[row][col-1];
      updateShortestPathTable(neighbourNodeID, currSelectedID,2);
    }
    //8 up-left
    if(row-1>=0 and col-1>=0){
      neighbourNodeID = regionArray[row-1][col-1];
      updateShortestPathTable(neighbourNodeID, currSelectedID,3);
    }
    if(currSelectedID==255){
      currSelectedID = numGateways;
      unvisitedNodes[currSelectedID-1] = 0;
    }
}

int calcNextTxNode(int destNodeID){
  currNode = destNodeID;
  int path[255];
  int count = 0;
  path[count] = destNodeID;
  while(currNode!=deviceID){
    count = count+1;
    path[count] = shortestPathTable[currNode][3];
    currNode = path[count];
  }
  int nextNodeID = path[count-1];
  return nextNodeID;
}
  

/**********************************************************************/

//command handler
/**********************************************************************/
void decodeInstructions(char *data){
  byte byteData[255] = (byte*)data;

  if(byteData[0]==0x01){// 0x01 -> request cluster data
    int requestedClusterID = byteData[1];
    requestDataFromCluster(requestedClusterID);
  }
  if(byteData[0]==0x02){//0x02 -> change thresholds
    int tempThreshold = byteData[1];
    int humidThreshold = byteData[2];
  }
  if(byteData[0]==0x03){//0x03 -> change system sampling Interval
    int sysSamplingInterval = byteData[1];
  }
  if(byteData[0]==0x04){//0x04 -> change sensor node sampling Interval
    int sensorSamplingInterval = byteData[1];
  }
}

void requestDataFromCluster(int clusterID){
  byte nextClusterIDPath = (byte)calcPath();
  byte* data = {nextClusterIDPath,(byte)deviceID, (byte)clusterID, 0x00, 0x01};
  transmitData(data);
}


/**********************************************************************/
