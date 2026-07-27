#include "config/DataDictionary.h"
//#include "include/WiFiFunctions.h"
//#include "include/HTTPClientFunctions.h"
#include "include/TimerFunctions.h" //works
//#include "include/GPSFunctions.h"
#include "include/MathFunctions.h"
//#include "include/ServoFunctions.h"
#include "include/OBD2Functions.h"
//#include "include/SignalProcessingFunctions.h"

//RTCZero rtc;
//SFE_UBLOX_GNSS myGNSS;
OBD2Config OBD2S1Commands;
//IPAddress localIP;
//SimpleMovingAverage SMA1(SIZE_16);
//PIDControls heightControl(25.0, 0.5, 0.04, 5.0, 20.0, 0.1);

//unsigned long lastGPSUpdate = 0, lastLEDBlink = 0, lastSpeedQuery = 0, lastSerialSend = 0;
unsigned long lastLEDBlink = 0, lastSpeedQuery = 0, lastSerialSend = 0;
//byte tHr = 0, tMin = 0, tSec = 0;
//GPSData gpsData;
float vehSpd = 0;
int LED_on = 1;

void setup()
{
  pinMode(STATUS_INDICATOR, OUTPUT);
  digitalWrite(STATUS_INDICATOR,1);

  //Debug setup
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Serial started.");
  //Wifi setup
  //if (initializeWifi() == WiFiReturnStatus::OK) Serial.println("Wifi connected.");
  //else Serial.println("Wifi not connected.");
  
  //RTC setup
  //initializeRTC(rtc);
  //if (setRTCDateTime(rtc) == TimerReturnStatus::OK) Serial.println("RTC set!");
  //else Serial.println("RTC not set up.");
  //getRTCTime(rtc, tHr, tMin, tSec);

  //GPS Shield setup
  //if (initializeGPS(myGNSS) == GPSReturnStatus::OK) Serial.println("GPS module started.");
  //else Serial.println("GPS module failed");
  
  if (initializeOBD2(OBD2S1Commands,OBD2_TX_GLOBAL,OBD2_RX_ECM_1) != CANReturnStatus::OK) Serial.print("CAN module init failed.");
  else Serial.println("CAN module starts");
}

void loop()
{ 
  /*
  if (isTimeout(500,lastGPSUpdate)){
    GPSReturnStatus gpsStatus = getGPSData(myGNSS, gpsData);
    if (gpsStatus != GPSReturnStatus::DATA_STALE) {
      GPSSignalStrength gpsSignal = evaluateSignal(gpsData);
    }
    lastGPSUpdate = millis();
  }
  */
  if (isTimeout(100,lastSpeedQuery)){
    fetchSpeed(OBD2S1Commands,vehSpd);
    lastSpeedQuery = millis();
  }
  
  if (isTimeout(2000,lastLEDBlink)){
    LED_on ^= 1;
    digitalWrite(STATUS_INDICATOR,LED_on);
    lastLEDBlink = millis();
  }

  if (isTimeout(100,lastSerialSend)){
    Serial.println(vehSpd);
    lastSerialSend = millis();
  }
}