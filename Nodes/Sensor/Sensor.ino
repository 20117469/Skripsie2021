#include <stdio.h>     
#include <stdlib.h>   
#include <string.h>  
#include <iostream>
#include <RadioLib.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <Wire.h>
#include "RTClib.h"

RTC_DS3231 rtc;


// Set DHT pin:
#define DHTPIN 2
// Set DHT type, uncomment whatever type you're using!
#define DHTTYPE DHT22
// Initialize DHT sensor for normal 16mhz Arduino:
DHT dht = DHT(DHTPIN, DHTTYPE);

//Include the library

#include <MQUnifiedsensor.h>
/************************Hardware Related Macros************************************/
#define         Board                   ("Arduino NANO")
#define         Pin                     (A0)  //Analog input 3 of your arduino
/***********************Software Related Macros************************************/
#define         Type                    ("MQ-2") //MQ2
#define         Voltage_Resolution      (5)
#define         ADC_Bit_Resolution      (10) // For arduino UNO/MEGA/NANO
#define         RatioMQ2CleanAir        (9.83) //RS / R0 = 9.83 ppm 

MQUnifiedsensor MQ2(Board, Voltage_Resolution, ADC_Bit_Resolution, Pin, Type);

RFM95 radio = new Module(10, 8, 9, 7);

const int deviceID = 1;

//channel                        13    14    15    16    17
float channelFrequencies[4] = {866.1,866.4,866.7,867.0,868.0};

// flag to indicate that a packet was received
volatile bool receivedFlag = false;

// disable interrupt when it's not needed
volatile bool enableInterrupt = true;

DateTime defaultTime = DateTime(2021,07,18,21,34,20);

void setup() {
  pinMode(3, OUTPUT);
  digitalWrite(3,1);

  dht.begin();
  Wire.begin();

  initRTC(defaultTime);
  //rtc.adjust(defaultTime);
  
  MQ2.setRegressionMethod(1); //_PPM =  a*ratio^b
  MQ2.setA(36974); MQ2.setB(-3.109); // Configurate the ecuation values to get LPG concentration
  /*
    Exponential regression:
    Gas    | a      | b
    H2     | 987.99 | -2.162
    LPG    | 574.25 | -2.222
    CO     | 36974  | -3.109
    Alcohol| 3616.1 | -2.675
    Propane| 658.71 | -2.168
  */
  MQ2.init();
  
  Serial.begin(9600);
  //while (!Serial) {
  //  ; // wait for serial port to connect. Needed for native USB port only
  //}
  loraInit();
}

void loop() {
  // put your main code here, to run repeatedly:

}

//MQ2 calibration
/**********************************************************************/

void calibrateMQ2(){
  //Serial.print("Calibrating please wait.");
  float calcR0 = 0;
  for(int i = 1; i<=10; i ++)
  {
    MQ2.update(); // Update data, the arduino will be read the voltage on the analog pin
    calcR0 += MQ2.calibrate(RatioMQ2CleanAir);
    Serial.print(".");
  }
  MQ2.setR0(calcR0/10);
  Serial.println("  done!.");
  
  if(isinf(calcR0)) {Serial.println("Warning: Conection issue founded, R0 is infite (Open circuit detected) please check your wiring and supply"); while(1);}
  if(calcR0 == 0){Serial.println("Warning: Conection issue founded, R0 is zero (Analog pin with short circuit to ground) please check your wiring and supply"); while(1);}

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

//command handler
/**********************************************************************/
void decodeInstructions(char *data){
  byte byteData[255] = (byte*)data;
  
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

/**********************************************************************/


// Sample Functions
/**********************************************************************/
float sampleHumid(){
  return dht.readHumidity();
}

float sampleTemp(){
  return dht.readTemperature();
}

float sampleSmoke(){
  MQ2.update();
  return MQ2.readSensor();
}

/**********************************************************************/

// RTC Init
/**********************************************************************/
void initRTC(DateTime defaultTime){ 
  //start RTC software connection
  if (! rtc.begin()) {
    Serial.println("No RTC Module Found...");
    while (1);
  }
  //checks if RTC lost power
  if (rtc.lostPower()) {
    Serial.println("RTC power lost, setting time to 2021/01/01 00:00:00");
    rtc.adjust(defaultTime);
  }
}
/**********************************************************************/

// EEPROM Functions
/**********************************************************************/
void writeByteToAddr(int memoryModuleAddress, int targetAddr, byte data ){
    int rdata = data;
    Wire.beginTransmission(memoryModuleAddress);
    Wire.write((int)(targetAddr >> 8)); // MSB
    Wire.write((int)(targetAddr & 0xFF)); // LSB
    Wire.write(rdata);
    Wire.endTransmission();
}

void writeArrayToAddr( int memoryModuleAddress, int startAddr, byte* data, byte length) {
    Wire.beginTransmission(memoryModuleAddress);
    Wire.write((int)(startAddr >> 8)); // MSB
    Wire.write((int)(startAddr & 0xFF)); // LSB
    byte c;
    for ( c = 0; c < length; c++)
        Wire.write(data[c]);
    Wire.endTransmission();
}

byte readByteFromAddr( int memoryModuleAddress, int targetAddr ) {
    byte rdata = 0xFF;
    Wire.beginTransmission(memoryModuleAddress);
    Wire.write((int)(targetAddr >> 8)); // MSB
    Wire.write((int)(targetAddr & 0xFF)); // LSB
    Wire.endTransmission();
    Wire.requestFrom(memoryModuleAddress,1);
    if (Wire.available()) rdata = Wire.read();
    return rdata;
}

byte* readArrayFromAddr( int memoryModuleAddress, int startAddr, int length ) {
    byte readBuf [13];
    Wire.beginTransmission(memoryModuleAddress);
    Wire.write((int)(startAddr >> 8)); // MSB
    Wire.write((int)(startAddr & 0xFF)); // LSB
    Wire.endTransmission();
    Wire.requestFrom(memoryModuleAddress,length);
    int c = 0;
    for ( c = 0; c < length; c++ )
        if (Wire.available()) readBuf[c] = Wire.read();
    return readBuf;
}
/**********************************************************************/
