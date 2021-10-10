/*
    Video: https://www.youtube.com/watch?v=oCMOYS71NIU
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleNotify.cpp
    Ported to Arduino ESP32 by Evandro Copercini

   Create a BLE server that, once we receive a connection, will send periodic notifications.
   The service advertises itself as: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
   Has a characteristic of: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E - used for receiving data with "WRITE" 
   Has a characteristic of: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E - used to send data with  "NOTIFY"

   The design of creating the BLE server is:
   1. Create a BLE Server
   2. Create a BLE Service
   3. Create a BLE Characteristic on the Service
   4. Create a BLE Descriptor on the characteristic
   5. Start the service.
   6. Start advertising.

   In this example rxValue is the data received (only accessible inside that function).
   And txValue is the data to be sent, in this example just a byte incremented every second. 
*/
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <EEPROM.h>
#include <Wire.h>
#include <RTCLib.h>
#define FLASH_MEMORY_SIZE 256
#define CMD_TURN_ON     'O'
#define CMD_TURN_OFF    'F'
#define CMD_GET_STATUS  'G'
#define CMD_SCH_ON      'S'
#define CMD_SCH_OFF     'P'
#define CMD_SCH_DIS     'D'
#define CMD_SCH_READ    'R'
#define CMD_ERROR       'E'
#define CHANEL_0    26
#define CHANEL_1    25
#define CHANEL_2    32
#define CHANEL_3    33
BLEServer *pServer = NULL;
BLECharacteristic * pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
RTC_DS1307 rtc;
DateTime now;
uint8_t txValue = 0;
uint8_t idx=0;
uint8_t* pointer;
uint8_t txstring[18];
char rxstring[18];

struct Schedule{
    uint8_t HH;
    uint8_t mm;
    uint8_t ss;
    uint8_t dd;
    uint8_t MM;
    uint8_t yy;
    uint8_t state;
};
struct DataSave{
    uint8_t ChanelState[4];
    Schedule Schedules[12];
    uint8_t CRC;
};
DataSave dataSave;
// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

uint8_t STR_TO_INT(char* str){
  if((*str>'9')||(*(str+1)>'9')||(*str<'0')||(*(str+1)<'0')) return 0xFF;
  return (*str-48)*10 + *(str+1)-48;
}
void INT_TO_STR(uint8_t* str,uint8_t num){
  *str=(num/10)+48;
  *(str+1)=(num%10)+48;
}
void saveData(){
  uint8_t size = sizeof(dataSave);
  pointer = &dataSave.ChanelState[0];
  dataSave.CRC=0;
  for(uint8_t i=0;i<size-1;i++){
    dataSave.CRC=dataSave.CRC^*(pointer+i);
  }
  EEPROM.put(0,dataSave);EEPROM.put(size,dataSave);
  EEPROM.commit();
}
void readData(){
  uint8_t size = sizeof(dataSave);
  uint8_t CRC=0;
  uint8_t i=0;
  pointer = &dataSave.ChanelState[0];
  EEPROM.get(0,dataSave);
  Serial.print("Read data 0:");
  for(i=0;i<size-1;i++){
    CRC=CRC^*(pointer+i);
    Serial.write(*(pointer+i));
  }
  if(CRC==dataSave.CRC) return;
  Serial.print("Data 0 error. Read data 1");
  EEPROM.get(size,dataSave);
  for(i=0;i<size-1;i++){
    CRC=CRC^*(pointer+i);
  }
  Serial.println();
  if(CRC==dataSave.CRC) return;
  Serial.print("Data 1 error. Save data init");
  dataSave.ChanelState[0]='F';
  dataSave.ChanelState[1]='F';
  dataSave.ChanelState[2]='F';
  dataSave.ChanelState[3]='F';
  for(i=0;i<12;i++){
    dataSave.Schedules[i].state='D';
    dataSave.Schedules[i].HH=0;
    dataSave.Schedules[i].mm=0;
    dataSave.Schedules[i].ss=0;
    dataSave.Schedules[i].dd=0;
    dataSave.Schedules[i].MM=0;
    dataSave.Schedules[i].yy=0;
  }
  saveData();
}
boolean setDateTime(){
  DateTime datetime;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint8_t day;
  uint8_t month;
  uint8_t year;
  hour = STR_TO_INT(&rxstring[4]);
  if(hour>24) return false;
  minute = STR_TO_INT(&rxstring[6]);
  if(minute>60) return false;
  second = STR_TO_INT(&rxstring[8]);
  if(second>60) return false;
  day = STR_TO_INT(&rxstring[11]);
  if(day>31) return false;
  month = STR_TO_INT(&rxstring[13]);
  if(month>12) return false;
  year = STR_TO_INT(&rxstring[15]);
  if(year>99) return false;
  datetime = DateTime(year,month,day,hour,minute,second);
  rtc.adjust(datetime);
  return true;
}
void setOutput(){
  if(dataSave.ChanelState[0]=='O'){
    digitalWrite(CHANEL_0,HIGH);
  }
  else{
    digitalWrite(CHANEL_0,LOW);
  }
  if(dataSave.ChanelState[1]=='O'){
    digitalWrite(CHANEL_1,HIGH);
  }
  else{
    digitalWrite(CHANEL_1,LOW);
  }
  if(dataSave.ChanelState[2]=='O'){
    digitalWrite(CHANEL_2,HIGH);
  }
  else{
    digitalWrite(CHANEL_2,LOW);
  }
  if(dataSave.ChanelState[3]=='O'){
    digitalWrite(CHANEL_3,HIGH);
  }
  else{
    digitalWrite(CHANEL_3,LOW);
  }
}
boolean setSchedule(char CMD, char ID){
  uint8_t hour;
  uint8_t minute;
  uint8_t second;
  uint8_t day;
  uint8_t month;
  uint8_t year;
  hour = STR_TO_INT(&rxstring[4]);
  if(hour>24) return false;
  minute = STR_TO_INT(&rxstring[6]);
  if(minute>60) return false;
  second = STR_TO_INT(&rxstring[8]);
  if(second>60) return false;
  day = STR_TO_INT(&rxstring[11]);
  if(day>31) return false;
  month = STR_TO_INT(&rxstring[13]);
  if(month>12) return false;
  year = STR_TO_INT(&rxstring[15]);
  if(year>99) return false;
  dataSave.Schedules[ID-48].state = CMD;
  dataSave.Schedules[ID-48].HH = hour;
  dataSave.Schedules[ID-48].mm = minute;
  dataSave.Schedules[ID-48].ss = second;
  dataSave.Schedules[ID-48].dd = day;
  dataSave.Schedules[ID-48].MM = month;
  dataSave.Schedules[ID-48].yy = year;
  return true;
}

void sendResponse(char CMD, char ID){
  txstring[1]=CMD;
  txstring[2]=ID;
  txstring[0]='A';
  txstring[17]='B';
  txstring[3]='-';
  txstring[10]='-';
  switch(CMD){
    case CMD_TURN_ON:
    case CMD_TURN_OFF:
    case CMD_GET_STATUS:
    case CMD_ERROR:
      now = rtc.now();
      INT_TO_STR(&txstring[4],now.hour());
      INT_TO_STR(&txstring[6],now.minute());
      INT_TO_STR(&txstring[8],now.second());
      INT_TO_STR(&txstring[11],now.day());
      INT_TO_STR(&txstring[13],now.month());
      INT_TO_STR(&txstring[15],now.year()%100);
    break;
    case CMD_SCH_DIS:
    case CMD_SCH_OFF:
    case CMD_SCH_ON:
      INT_TO_STR(&txstring[4],dataSave.Schedules[ID-48].HH);
      INT_TO_STR(&txstring[6],dataSave.Schedules[ID-48].mm);
      INT_TO_STR(&txstring[8],dataSave.Schedules[ID-48].ss);
      INT_TO_STR(&txstring[11],dataSave.Schedules[ID-48].dd);
      INT_TO_STR(&txstring[13],dataSave.Schedules[ID-48].MM);
      INT_TO_STR(&txstring[15],dataSave.Schedules[ID-48].yy);
    default:
    break;
  }
  Serial.print("SEND:");Serial.println((char*)txstring);
  if (deviceConnected) {
    pTxCharacteristic->setValue(txstring, 18);
    pTxCharacteristic->notify();
		delay(100); // bluetooth stack will go into congestion, if too many packets are sent
	}
}
void checkSchedule(){
  now = rtc.now();
  uint8_t yy = now.year()%100;
  // uint8_t x[2];
  // INT_TO_STR(&x[0],now.minute());
  // Serial.write(x[0]);Serial.write(x[1]);
  for(idx=0;idx<12;idx++){
    // Serial.printf("value %02u:%02u-%02u:%02u:%02u=%02u:%02u-%02u:%02u:%02u",now.hour(),now.minute(),
    // now.day(),now.month(),yy,dataSave.Schedules[idx].HH,dataSave.Schedules[idx].mm,dataSave.Schedules[idx].dd,
    // dataSave.Schedules[idx].MM,dataSave.Schedules[idx].yy);
    // Serial.println();
    if((yy==dataSave.Schedules[idx].yy)
        &&(now.month()==dataSave.Schedules[idx].MM)
        &&(now.day()==dataSave.Schedules[idx].dd)
        &&(now.hour()==dataSave.Schedules[idx].HH)
        &&(now.minute()==dataSave.Schedules[idx].mm)
        &&(now.second()==dataSave.Schedules[idx].ss)){
      if(idx<3){
        if(dataSave.Schedules[idx].state=='S'){
          dataSave.ChanelState[0]='O';
          if(deviceConnected) sendResponse('O','0');
        }
        else{
          dataSave.ChanelState[0]='F';
          if(deviceConnected) sendResponse('F','0');
        }
      }
      else if(idx<6){
        if(dataSave.Schedules[idx].state=='S'){
          dataSave.ChanelState[1]='O';
          if(deviceConnected) sendResponse('O','1');
        }
        else{
          dataSave.ChanelState[1]='F';
          if(deviceConnected) sendResponse('F','1');
        }
      }
      else if(idx<9){
        if(dataSave.Schedules[idx].state=='S'){
          dataSave.ChanelState[2]='O';
          if(deviceConnected) sendResponse('O','2');
        }
        else{
          dataSave.ChanelState[2]='F';
          if(deviceConnected) sendResponse('F','2');
        }
      }
      else{
        if(dataSave.Schedules[idx].state=='S'){
          dataSave.ChanelState[3]='O';
          if(deviceConnected) sendResponse('O','3');
        }
        else{
          dataSave.ChanelState[3]='F';
          if(deviceConnected) sendResponse('F','3');
        }
      }
      saveData();
      setOutput();
    }
  }
}
void handle_cmd(){
  char CMD = 'E';
  char ID = '0';
  if((rxstring[3]!='-')&&(rxstring[10]!='-')){
    Serial.write(rxstring[3]);Serial.write(rxstring[10]);
    Serial.println("E7-1");
    sendResponse(CMD_ERROR,'7'); return;
  }
  if(STR_TO_INT(&rxstring[15])<21){
    Serial.println("E4-1");
    sendResponse(CMD_ERROR,'4'); return;
  }
  if((rxstring[2]<0x30)||(rxstring[2]>0x3B)){
    Serial.println("E6-1");
    sendResponse(CMD_ERROR,'6'); return;
  }
  if((rxstring[1]==CMD_TURN_ON)||(rxstring[1]==CMD_TURN_OFF)
    ||(rxstring[1]==CMD_GET_STATUS)||(rxstring[1]==CMD_SCH_READ)){
    if(!setDateTime()) {
      Serial.println("E4-1");
      sendResponse(CMD_ERROR,'4'); return;
    }
  }
  switch(rxstring[1]){
    case CMD_TURN_ON:
    case CMD_TURN_OFF:
      if(rxstring[2]<'4'){
        dataSave.ChanelState[rxstring[2]-48]=rxstring[1];
        CMD=rxstring[1];ID=rxstring[2];
        saveData();
        setOutput();
      }
      else {
        Serial.println("E7-2");
        CMD='E';ID='7';
      }
    break;
    case CMD_GET_STATUS:
      if(rxstring[2]<'4'){
        CMD=dataSave.ChanelState[rxstring[2]-48];ID=rxstring[2];
      }
      else {
        Serial.println("E7-3");
        CMD='E';ID='7';
      }
    break;
    case CMD_SCH_DIS:
    case CMD_SCH_OFF:
    case CMD_SCH_ON:
      CMD=rxstring[1];ID=rxstring[2];
      if(!setSchedule(CMD,ID)){
        Serial.println("E4-2");
        CMD='E';ID='4';
      }
      else{
        saveData();
      }
    break;
    case CMD_SCH_READ:
      ID=rxstring[2];
      CMD=dataSave.Schedules[rxstring[2]-48].state;
    break;
    default:
      Serial.println("E5-1");
      CMD='E';ID='5';
    break;
  }
  sendResponse(CMD,ID);
}

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("Connected...");
    };
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("Disconnected...");
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();
      if (rxValue.length() > 0) {
        Serial.print("Received Value: ");
        for (int i = 0; i < rxValue.length(); i++){
          Serial.print(rxValue[i]);
        }
        Serial.println();
      }
      else return;
      if(rxValue.length() != 18) return;
      if((rxValue[0]!='A')||(rxValue[17]!='B')) return;
      memcpy(&rxstring[0],&rxValue[0],18);
      handle_cmd();
    }
};
void setup() {
  Serial.begin(115200);
  pinMode(CHANEL_0, OUTPUT);
  pinMode(CHANEL_1, OUTPUT);
  pinMode(CHANEL_2, OUTPUT);
  pinMode(CHANEL_3, OUTPUT);
  EEPROM.begin(FLASH_MEMORY_SIZE);
  readData();
  setOutput();
 // Wire.begin(21,22);
  if (!rtc.begin())
  {
    Serial.print("Couldn't find RTC");
  }
  delay(100);
  if (!rtc.isrunning())
  {
    Serial.print("RTC is NOT running!");
    Serial.println();
  }
  
  // Create the BLE Device
  BLEDevice::init("SMART DEVICES");
  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);
  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(
										CHARACTERISTIC_UUID_TX,
										BLECharacteristic::PROPERTY_NOTIFY
									);              
  pTxCharacteristic->addDescriptor(new BLE2902());
  BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(
											 CHARACTERISTIC_UUID_RX,
											BLECharacteristic::PROPERTY_WRITE
										);
  pRxCharacteristic->setCallbacks(new MyCallbacks());
  // Start the service
  pService->start();
  // Start advertising
  pServer->getAdvertising()->addServiceUUID(SERVICE_UUID);
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
  txValue=0;
  BLEAddress thisAddress = BLEDevice::getAddress();
	Serial.printf("BLE address: %s\n", thisAddress.toString().c_str());
}
void loop() {
    txValue++;delay(10);
    if(txValue>=100){
      txValue = 0;
      checkSchedule();
    }
    // if (deviceConnected) {
    //     pTxCharacteristic->setValue(&txValue, 1);
    //     pTxCharacteristic->notify();
    //     txValue++;
    //     if(txValue >= 0x39) txValue = 0x30;
		// delay(1000); // bluetooth stack will go into congestion, if too many packets are sent
	  // }
    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
		// do stuff here on connecting
        oldDeviceConnected = deviceConnected;
    }
}