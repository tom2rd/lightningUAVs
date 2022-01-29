/*-----------------------------------------
 *  Lightning UAV Environment data logger 
 *  M5Atom GPS + ENV III 
 *  SD card 
 *  Produced by Tetsuya Tominaga
 *  
 *  Filename GPSDATE&TIME(DDMMYY_HHMMSS.csv) 
 *  
 *  Log Format
 *  Gdate,Ghour:Gmint:Gsecd,GLat,GLon,GAlt,tmp,hum,prs
 *  290122  24:16:28  35.72541  139.50615 59.7  26.71 15.24 100192
 *  DDMMYY,JST,Latitude,Longitude,Altitude(m),C,%,Pa
 *  
 *  Sampling Rate: Around 250 msec
 *  
 *  LED = RED : Error 
 *  LED = Yellow blink : Write SD
 *  
 *  Power Comsumption Around 150mA / 5V
 *  
 *  Bluetooth pairing 
 */

#include <Arduino.h>
#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <M5Atom.h>
#include "Adafruit_Sensor.h"
#include <Adafruit_BMP280.h>
#include "UNIT_ENV.h"
#include <SPI.h>
#include "FS.h"
#include <SD.h>
#include <BluetoothSerial.h>

BluetoothSerial SerialBT;

// const char filename[] = "/GPSdata.txt";
String f_name="";
File txtFile;
uint64_t chipid;
char chipname[256];

/*------------------------- 
 * for GPS
 --------------------------*/
TinyGPSPlus gps;
double GLat = 0.0;
double GLon = 0.0;
double GAlt = 0.0;
uint32_t Gdate = 0;
uint8_t Ghour = 0;
uint8_t Gmint = 0;
uint8_t Gsecd = 0;

void GPS_get(){
  
  while (Serial1.available() > 0) {
    char c = Serial1.read();
    // Serial.print(c);
    gps.encode(c);
  }

  if (gps.location.isValid()) {
    M5.dis.drawpix(0, 0x00ff00);  //GREEN
    GLat=gps.location.lat();
    GLon=gps.location.lng();
    GAlt=gps.altitude.meters();
    Gdate=gps.date.value();
    Ghour=gps.time.hour()+9; // JST 9hour plus
    Gmint=gps.time.minute();
    Gsecd=gps.time.second();  
  } else {
    Serial.println("GPS not valid");
    SerialBT.println("GPS not valid");
    M5.dis.drawpix(0, 0xff0000);  //RED
  }  
}



void GPS_fnameset(){ 
  do{ 
   M5.dis.drawpix(0, 0xff0000);  //RED
   GPS_get();
   Serial.printf("time %d, loc %d\n",gps.time.isValid(),gps.location.isValid());
       f_name="";
       /* Serial.printf("DATE=%ld,TIME=%2d:%2d:%2d\r\n",Gdate,Ghour,Gmint,Gsecd);  
          Serial.print(Gdate);
          Serial.print(Ghour);
          Serial.print(Gmint);
          Serial.print(Gsecd);*/
          f_name="/"+String(Gdate, DEC)+"_"+String(Ghour, DEC)+String(Gmint, DEC)+String(Gsecd, DEC)+".csv";    
  }while(gps.time.isValid()==false || gps.location.isValid()==false || Gdate==0);
  //Serial.print("Filename=");
  //Serial.println(f_name);
  SerialBT.print("Filename=");
  SerialBT.println(f_name);
    
  M5.dis.drawpix(0, 0x00ff00);  //GREEN
}

//------ for ENVIII
SHT3X sht30;
QMP6988 qmp6988;

float tmp = 0.0;
float hum = 0.0;
float prs = 0.0;

void ENV_get(){
  prs = qmp6988.calcPressure();
  if(sht30.get()==0){ //Obtain the data of shT30.  
    tmp = sht30.cTemp;  //Store the temperature obtained from shT30.  
    hum = sht30.humidity; //Store the humidity obtained from the SHT30.  
  }else{
    tmp=0,hum=0;
  }
}
/*------------
 * LED 
 -----------*/
void LED_blink(uint32_t dd){
  delay(50);
  M5.dis.drawpix(0, 0x0000f0);  //BLUE
  delay(dd);
  M5.dis.drawpix(0, 0xff0000);  //RED
  delay(dd);
  M5.dis.drawpix(0, 0x00ff00);  //GREEN
  delay(dd);
  M5.dis.drawpix(0, 0xfff000);  //YELLOW
  delay(dd);
  M5.dis.drawpix(0, 0x000000);  //------
}

/*-------------------------------
 * SD write log
 -------------------------------*/
bool writeLog(String file_name) {          //Write GPSdata to SDcard.  
  //Serial.println(file_name);
  txtFile = SD.open(file_name, FILE_APPEND);
  if(txtFile){
    //Serial.println(txtFile);
    M5.dis.drawpix(0, 0xfff000);  //YELLOW
    txtFile.print(Gdate);
    txtFile.print(",");
    txtFile.print(Ghour);
    txtFile.print(":");
    txtFile.print(Gmint);
    txtFile.print(":");
    txtFile.print(Gsecd);
    txtFile.print(",");
    txtFile.printf("%.5f",GLat);
    txtFile.print(",");
    txtFile.printf("%.5f",GLon);
    txtFile.print(",");
    txtFile.printf("%.3f",GAlt);
    txtFile.print(",");
    txtFile.print(tmp);
    txtFile.print(",");
    txtFile.print(hum);
    txtFile.print(",");    
    txtFile.println(prs);
    txtFile.close();
  }else{
    return false;
    M5.dis.drawpix(0, 0xff0000);  //RED
  }
  return true;
}

bool writeLogfirst(String file_name) {          //Write GPSdata to SDcard.  
  M5.dis.drawpix(0, 0xff0000);  //RED
  txtFile = SD.open(file_name, FILE_WRITE);
  SerialBT.print("Write :");
  SerialBT.println(txtFile);
  if(txtFile){
    M5.dis.drawpix(0, 0xfff000);  //YELLOW
    txtFile.print("Gdate");
    txtFile.print(",");
    txtFile.print("Ghour");
    txtFile.print(":");
    txtFile.print("Gmint");
    txtFile.print(":");
    txtFile.print("Gsecd");
    txtFile.print(",");
    txtFile.printf("GLat");
    txtFile.print(",");
    txtFile.printf("GLon");
    txtFile.print(",");
    txtFile.printf("GAlt");
    txtFile.print(",");
    txtFile.print("tmp");
    txtFile.print(",");
    txtFile.print("hum");
    txtFile.print(",");    
    txtFile.println("prs");
    txtFile.close();
  }else{
    M5.dis.drawpix(0, 0xff0000);  //RED
    return false;
  }
  return true;
}

/*-------------------
 * Setup Atom
 --------------------*/
void setup() {
  M5.begin(true,false,true);               //Init M5Atom. 
  LED_blink(1000);             //LED RED  
  Serial.begin(115200);                    // RX  TX
  chipid = ESP.getEfuseMac();
  sprintf( chipname, "SerialBT_%04X", (uint16_t)(chipid >> 32));
  Serial.printf("Bluetooth: %s\n", chipname);   
  SerialBT.begin(chipname);
  Serial1.begin(9600, SERIAL_8N1, 22,-1);  //For GPS serial
  Wire.begin(26,32);                       //Init pin 26,32. For ENV III
  qmp6988.init();
  GPS_fnameset();
  delay(100);
  //SD card start
  SPI.begin(23,33,19,-1);
  delay(100);
    if(!SD.begin(-1, SPI, 40000000)){
      SerialBT.println("initialization failed!");
    } 
  delay(100);  
  sdcard_type_t Type = SD.cardType();

  SerialBT.printf("SDCard Type = %d \r\n",Type);
  SerialBT.printf("SDCard Size = %d \r\n" , (int)(SD.cardSize()/1024/1024));
  SerialBT.print("Filename=");SerialBT.println(f_name);
  SerialBT.print("Firstline=");SerialBT.println(writeLogfirst(f_name));
  
}

void loop() {

  GPS_get();
  ENV_get();
  SerialBT.printf("LAT=%.5f,LNG=%.5f,Alt=%.2f \r\n",GLat,GLon,GAlt);
  SerialBT.printf("DATE=%ld,TIME=%2d:%2d:%2d\r\n",Gdate,Ghour,Gmint,Gsecd);  
  SerialBT.printf("Temp: %2.1f  ,Humi: %2.0f%%  ,Pressure:%2.0fPa\r\n", tmp, hum, prs);
  writeLog(f_name);
  //delay(100);
}
