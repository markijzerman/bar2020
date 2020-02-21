//________________________________________________________________SensorData:

#define kThreshold  30 //Treshold "change in movement"
#define ReadSpeed 20 //Scan every .. ms
#define kMaxSensors 26 //this value is only used to set array size in main code. for the sensor settings, see kRawLengthDefault kRawLengthMax in Trill.h they should equal (2 * kMaxSensors).
float  newDataMultiplier = 0.85;
float  oldDataMultiplier = (1 - newDataMultiplier);

//For Setup functions Prescaler, Noise Threshold etc. : see Trill.h (line 96 - 104)

typedef struct _sd
{
  //int rawData; //@rawData
  int newData;
  int oldData;
  int delta;
  bool dir;
  int change = 0;  //0 = no movement, 1 = movement towards sensor, 2 = movement away from sensor.
  
} SensorData;

SensorData sensList[kMaxSensors];

int index;

//_________________________________________________________________OSC:

#include <OSCMessage.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <SPI.h>    
#include <OSCBundle.h>

EthernetUDP Udp;

IPAddress ip(192, 168, 1, 101); //the Arduino's IP first 3 numbers have to match destination IP

IPAddress outIp(192, 168, 1, 85); //destination IP

//
//IPAddress ip(169, 254, 14, 6); //the Arduino's IP first 3 numbers have to match destination IP
//
//IPAddress outIp(169, 254, 14, 82); //destination IP

const unsigned int outPort = 9999; //Port

byte mac[] = {0x90, 0xA2, 0xDA, 0x0F, 0xEB, 0xC7}; //mac adress

//_________________________________________________________________TRILL:
 
#include <Wire.h>
#include "Trill.h"

#define I2C_ADDRESS 24

Trill slider(I2C_ADDRESS);

char gSerialBuffer[9];
int gSerialBufferIndex = 0;

int gCommandValues[4];
int gCommandValueIndex = 0;
long gLastMillis = 0;

//________________________________________________________________SETUP:

void setup() {

  Ethernet.begin(mac,ip);
  Udp.begin(8888);
  //Serial.begin(115200);


  
  if(!slider.begin())
    Serial.println("failed to initialise slider");
    
  slider.setMode(TRILL_MODE_DIFF);
  slider.requestRawData(52);

    if(slider.rawDataAvailable() > 0) {
     index = 0;
      while(slider.rawDataAvailable() > 0) {
        int data = slider.rawDataRead();
        sensList[index].oldData = data;
        Serial.print(sensList[index].oldData);
        Serial.print(" ");
        index++;
      }
       Serial.print("");
    } 
    //hier proberen setupfuncties neer te zetten.
    index = 0;   
}

//________________________________________________________________LOOP:

void loop() {
  
  //static unsigned long lastms = 0;
  
  if(Serial.available() > 0) {
    byte input = Serial.read();
    if(input == '\n') {
      if(gSerialBufferIndex > 0 && gCommandValueIndex < 4) {
        gSerialBuffer[gSerialBufferIndex] = '\0';
        gCommandValues[gCommandValueIndex++] = atoi(gSerialBuffer);
      }
      
      if(gCommandValueIndex > 0) {
        // TODO: send I2C message to chip
        Wire.beginTransmission(I2C_ADDRESS);
        Wire.write(0); // Always begin at byte 0 for commands
        for(int i = 0; i < gCommandValueIndex; i++)
          Wire.write(gCommandValues[i]);
        Wire.endTransmission();
        gLastMillis = millis();  // Give 20ms for the chip to catch up
      }  
      gCommandValueIndex = 0;
      gSerialBufferIndex = 0;
    }
    else if(input == ' ' || input == '\t' || input == ',') {
      if(gSerialBufferIndex > 0 && gCommandValueIndex < 4) {
        gSerialBuffer[gSerialBufferIndex] = '\0';
        gCommandValues[gCommandValueIndex++] = atoi(gSerialBuffer);
      }
      gSerialBufferIndex = 0;
    }
    else {  
      if(gSerialBufferIndex < 8)
        gSerialBuffer[gSerialBufferIndex++] = input;
    }
  }

  // Scan every (readSpeed)ms
  if((millis() - gLastMillis) > ReadSpeed) {  
    gLastMillis += ReadSpeed;
    slider.requestRawData(52);

    if(slider.rawDataAvailable() > 0) {
      index = 0;
      while(slider.rawDataAvailable() > 0) {
        int value = slider.rawDataRead();  
//        sensList[index].rawData = value; //@rawData
         
        sensList[index].newData = (newDataMultiplier * value) + (oldDataMultiplier * sensList[index].oldData); //filter
         if((sensList[index].newData - sensList[index].oldData) > 0)
      {
      sensList[index].delta = sensList[index].newData - sensList[index].oldData;
      sensList[index].dir = true; 
      }
    else
      {
      sensList[index].delta = sensList[index].oldData - sensList[index].newData;
      sensList[index].dir = false;
       
      } 
      
        sensList[index].oldData = sensList[index].newData;


              
  if(sensList[index].delta > kThreshold)    
    {
      //Serial.print(" +"); //PRINT:CHANGE 
       
      if(sensList[index].dir){
      //Serial.print(" towards");
      sensList[index].change = 1;
       // moving towards sensor    
      }
      
      else {    
      //Serial.print(" away");
      sensList[index].change = 2;
      //moving away from sensor
     }
       
      }
 
   else
     {
     //Serial.print(" -"); // no change
      sensList[index].change = 0; 
     }
   //Serial.print sensList[index].newData
   //Serial.print(" ");   
   index++; 

  
    }
    
// Serial.println("");

       OSCBundle bndl;

//       bndl.add("/rawData").add((int)sensList[0].rawData).add((int)sensList[1].rawData).add((int)sensList[2].rawData).add((int)sensList[3].rawData).add((int)sensList[4].rawData).add((int)sensList[5].rawData).add((int)sensList[6].rawData).add((int)sensList[7].rawData).add((int)sensList[8].rawData).add((int)sensList[9].rawData).add((int)sensList[10].rawData).add((int)sensList[11].rawData).add((int)sensList[12].rawData).add((int)sensList[13].rawData).add((int)sensList[14].rawData).add((int)sensList[15].rawData).add((int)sensList[16].rawData).add((int)sensList[17].rawData).add((int)sensList[18].rawData).add((int)sensList[19].rawData).add((int)sensList[20].rawData).add((int)sensList[21].rawData).add((int)sensList[22].rawData).add((int)sensList[23].rawData).add((int)sensList[24].rawData).add((int)sensList[25].rawData);//@rawData
//    
//       Udp.beginPacket(outIp, outPort);//@rawData
//       bndl.send(Udp); // send the bytes to the SLIP stream//@rawData
//       Udp.endPacket(); // mark the end of the OSC Packet//@rawData
//       bndl.empty(); // free space occupied by message //@rawData
       
       bndl.add("/Data").add((int)sensList[0].newData).add((int)sensList[1].newData).add((int)sensList[2].newData).add((int)sensList[3].newData).add((int)sensList[4].newData).add((int)sensList[5].newData).add((int)sensList[6].newData).add((int)sensList[7].newData).add((int)sensList[8].newData).add((int)sensList[9].newData).add((int)sensList[10].newData).add((int)sensList[11].newData).add((int)sensList[12].newData).add((int)sensList[13].newData).add((int)sensList[14].newData).add((int)sensList[15].newData).add((int)sensList[16].newData).add((int)sensList[17].newData).add((int)sensList[18].dir).add((int)sensList[19].newData).add((int)sensList[20].newData).add((int)sensList[21].newData).add((int)sensList[22].newData).add((int)sensList[23].newData).add((int)sensList[24].newData).add((int)sensList[25].newData);
    
       Udp.beginPacket(outIp, outPort);
       bndl.send(Udp); // send the bytes to the SLIP stream
       Udp.endPacket(); // mark the end of the OSC Packet
       bndl.empty(); // free space occupied by message   
      
       bndl.add("/Change").add(sensList[0].change).add(sensList[1].change).add(sensList[2].change).add(sensList[3].change).add(sensList[4].change).add(sensList[5].change).add(sensList[6].change).add(sensList[7].change).add(sensList[8].change).add(sensList[9].change).add(sensList[10].change).add(sensList[11].change).add(sensList[12].change).add(sensList[13].change).add(sensList[14].change).add(sensList[15].change).add(sensList[16].change).add(sensList[17].change).add(sensList[18].change).add(sensList[19].change).add(sensList[20].change).add(sensList[21].change).add(sensList[22].change).add(sensList[23].change).add(sensList[24].change).add(sensList[25].change);
     
       Udp.beginPacket(outIp, outPort);
       bndl.send(Udp); // send the bytes to the SLIP stream
       Udp.endPacket(); // mark the end of the OSC Packet
       bndl.empty(); // free space occupied by message 
   }  
     
//Serial.println(gLastMillis);
         
  }

}
