// .\ArduinoSketchUploader.exe --file=SorTeS_GW07.hex --port=COM4 --model=Leonardo
// TODO: ctrl F for 'TODO'

#include <Arduino_FreeRTOS.h>
#include <EEPROM.h> //to bel able to writ in EEPROM using EEPROM.write(addr, val); our EEPROM is 1024 bits big
#include <LoRa.h>
#include <avr/sleep.h>
#include <avr/power.h>

#define SCK     15
#define MISO    14
#define MOSI    16
#define SS      8
#define RST     4
#define DI0     7
#define BAND    869300000
#define PABOOST true 

int sleepEEPROMLocation=0;
int nextEntryAdressLocation=8;
int firstdbEntry=nextEntryAdressLocation+8;

// define tasks
//void TaskBlink( void *pvParameters );
//void TaskAnalogRead( void *pvParameters );
void TaskPrintdB( void *pvParameters );
TaskHandle_t TaskHandle_PrintdB;
void TaskListenForBeacon( void *pvParameters );
TaskHandle_t TaskHandle_ListenForBeacon;
void TaskPrintLastEntry(void *pvParameters);
TaskHandle_t TaskHandle_PrintLastEntry;
void TaskDeepSleep(void *pvParameters);
TaskHandle_t TaskHandle_DeepSleep;

//nextEntryAdress
byte nextEntryAdress;
bool listeningToCommands;
byte command =0;


// the setup function runs once when you press reset or power the board
void setup() {
  //clear database
  //EEPROM.write(nextEntryAdressLocation,byte(firstdbEntry));
  
  Serial.begin(9600);
  while (!Serial);
  
  listeningToCommands=true;
  
  //DB SETUP
  EEPROM.write(sleepEEPROMLocation,byte(10)); //10 sec
  nextEntryAdress=EEPROM.read(nextEntryAdressLocation);


  //INTERUPTS SETUP
  interrupts();

  
  //LORA setup
  Serial.println("LoRa Receiver");
  LoRa.setPins(SS,RST,DI0);
  if (!LoRa.begin(BAND,PABOOST )) {
    Serial.println("Starting LoRa failed!");
  }
  //LoRa.onReceive(onReceive);
  //LoRa.receive();
  
  //temp init
  getTemperatureInternal();
  
  //Tasks
  xTaskCreate(
    TaskDeepSleep
    ,  "TaskDeepSleep"   // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  &TaskHandle_DeepSleep );
  

  
}

void onReceive (int packetSize){
  Serial.println("OnReceive");
  xTaskCreate(
    TaskListenForBeacon
    ,  "TaskListenForBeacon"   // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  &TaskHandle_ListenForBeacon );
}


void loop() {
  //empty, we use Tasks
}
/*
  
  if (listeningToCommands){
    //Serial.println("passed listeningToCommands");
     if (Serial.available() > 0) {
        // read the incoming byte:
        command=Serial.parseInt();
        switch (command) {
          case 1:
            xTaskCreate(
                TaskprintLastEntry
                ,  "TaskprintLastEntry"   // A name just for humans
                ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
                ,  NULL
                ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
                ,  NULL );
            break;
          case 2:
            xTaskCreate(
                TaskPrintdB
                ,  "TaskPrintdB"   // A name just for humans
                ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
                ,  NULL
                ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
                ,  &TaskHandle_PrintdB );
            
            break;
          case 3:           
            deepSleep();
            break;
          default:
            break;
        }
      }
    }
}*/

 //source: https://www.gammon.com.au/power
void TaskDeepSleep(void *pvParameters){
  (void) pvParameters;
  Serial.println("DeepSleepMode");
  Serial.println("iets");
  delay(500);
  
  noInterrupts();//turn off interrupts 
  
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); 
  

  Serial.end(); // disable serial
  
  LoRa.end(); //diable LoRa
  
  for (byte i = 0; i <= A5; i++){
    pinMode (i, OUTPUT);    // changed as per below
    digitalWrite (i, LOW);  //     ditto
  }
  // disable ADC
  ADCSRA = 0; 

  // turn off brown-out enable in software
  // by changing C:\Users\ruben\Documents\Arduino\hardware\BSFrance-avr-master\avr
        //"boards.txt"
  //from lora32u4II.bootloader.extended_fuses=0xcb
  // to  lora32u4II.bootloader.extended_fuses=0xF7


  //low clock speed using :https://www.engbedded.com/fusecalc
  //from lora32u4II.bootloader.low_fuses=0xE3
  // to  lora32u4II.bootloader.low_fuses=0xff

  
  power_all_disable();//disables a lot of moduals
  
  
  sleep_enable();
  sleep_cpu();
}

void TaskPrintdB(void *pvParameters)
{
  (void) pvParameters;
  
  //printdb
  /*if (!Serial){
    Serial.begin(9600);
    while(!Serial){};
  }
  */
  nextEntryAdress=EEPROM.read(nextEntryAdressLocation);
  Serial.println("Printing DB");

  if (nextEntryAdress<=firstdbEntry){
    Serial.println("No entries");
    return;
  }
  
  /*
  Serial.print("start at adress ");
  Serial.print(firstdbEntry);
  Serial.print(" - ");
  Serial.print("stop at adress ");
  Serial.println(nextEntryAdress-8);
  */
  
  int entry = 1;
  for (byte i=byte(firstdbEntry); i<nextEntryAdress; i+=8){
    Serial.print("entry ");
    Serial.print(entry);
    Serial.print(": ");
    Serial.println(i);
    Serial.print("address: ");
    Serial.print(EEPROM.read(i),DEC);
    Serial.println("°c");
    Serial.println();
    entry++;
  }
  vTaskDelete(TaskHandle_PrintdB); //deletes task after done
}

void TaskListenForBeacon(void *pvParameters)
{
  (void) pvParameters;
  
  String gateway="";
  while(gateway!="GW07"){
    gateway="";
    for (int i=0;i<4;i++) {            // can't use readString() in callback, so
      gateway += (char)LoRa.read();      // add bytes one by one
    }
  }
  Serial.print("Gateway: " );
  Serial.println(gateway);
  
  int8_t sec;                 // payload of packet
  char charr="";
  charr=(char)LoRa.read();
  
  sec = charr - '0';      

  Serial.println("sleep sec: " + String(sec));
  Serial.println("RSSI: " + String(LoRa.packetRssi()));
  //Serial.println("Snr: " + String(LoRa.packetSnr()));

  listeningToCommands=false;

  int8_t temp =getTemperatureInternal(); 
  
  LoRa.beginPacket(); 
  LoRa.write(temp);                     
  LoRa.endPacket();  
  LoRa.receive(); //back to receive mode

  Serial.print("Temp: ");
  Serial.println(temp);
  writeTempSleepEEPROM(temp,sec);
  Serial.println("-------------");
  //TODO ^ possibly remove it since it could use a lot of power
  
  vTaskDelete(TaskHandle_ListenForBeacon); //deletes task after done

}

void TaskprintLastEntry(void *pvParameters)
{
  (void) pvParameters;
  
  Serial.println("printLastEntry: ");
  nextEntryAdress=EEPROM.read(nextEntryAdressLocation);
  byte adressLastEntry=nextEntryAdress-8;
  if (adressLastEntry<firstdbEntry){
    Serial.println("No entries");
    return;
  }
  //Serial.print("at adress: ");
  //Serial.println(adressLastEntry);
  Serial.print("last Recorded temp: ");
  Serial.println(EEPROM.read(adressLastEntry),DEC);

  vTaskDelete(TaskHandle_PrintdB); //deletes task after done

}


void writeTempSleepEEPROM(int8_t temp,int8_t sleepSec){
  Serial.println("---------------------------");
  Serial.print("Inputs: ");
  Serial.print(temp);
  Serial.print(" ");
  Serial.println(sleepSec);
  Serial.println("---------------------------");


  EEPROM.write(sleepEEPROMLocation,sleepSec);
  nextEntryAdress=EEPROM.read(nextEntryAdressLocation);
  EEPROM.write(nextEntryAdress,temp);
  
  Serial.print("writen temp at adress: ");
  Serial.println(nextEntryAdress,DEC);

  Serial.print("writen temp : ");
  Serial.println(temp,BIN);
  
  nextEntryAdress+=8; //move nextEntry 
  EEPROM.write(nextEntryAdressLocation,nextEntryAdress);
  
}

//Deprecated
void X_onReceive (int packetSize){
  if (packetSize == 0) return;          // if there's no packet, return

  listeningToCommands=false;

  String gateway="";
  for (int i=0;i<4;i++) {            // can't use readString() in callback, so
    gateway += (char)LoRa.read();      // add bytes one by one
  }
  Serial.print("Gateway: " );
  Serial.println(gateway);
  
  if (gateway!="GW07"){
    //Serial.println("wrong Gateway");
    return;
  }
  
  int8_t sec;                 // payload of packet
  char charr="";
  charr=(char)LoRa.read();
  
  sec = charr - '0';      

  Serial.println("sleep sec: " + String(sec));
  Serial.println("RSSI: " + String(LoRa.packetRssi()));
  //Serial.println("Snr: " + String(LoRa.packetSnr()));

  // send ack with temp
  LoRa.beginPacket(); 
  int8_t temp =getTemperatureInternal(); 
  LoRa.write(temp);                     
  LoRa.endPacket();
  Serial.print("Temp: ");
  Serial.println(temp);
  writeTempSleepEEPROM(temp,sec);
  Serial.println("-------------");
  LoRa.receive();
}


//Deprecated
void printLastEntry(){
  Serial.println("printLastEntry: ");
  nextEntryAdress=EEPROM.read(nextEntryAdressLocation);
  byte adressLastEntry=nextEntryAdress-8;
  if (adressLastEntry<firstdbEntry){
    Serial.println("No entries");
    return;
  }
  //Serial.print("at adress: ");
  //Serial.println(adressLastEntry);
  Serial.print("last Recorded temp: ");
  Serial.println(EEPROM.read(adressLastEntry),DEC);
  
}

//Deprecated
void printdb(){
  nextEntryAdress=EEPROM.read(nextEntryAdressLocation);
  Serial.println("Printing DB");

  if (nextEntryAdress<=firstdbEntry){
    Serial.println("No entries");
    return;
  }
  
  /*
  Serial.print("start at adress ");
  Serial.print(firstdbEntry);
  Serial.print(" - ");
  Serial.print("stop at adress ");
  Serial.println(nextEntryAdress-8);
  */
  
  int entry = 1;
  for (byte i=byte(firstdbEntry); i<nextEntryAdress; i+=8){
    Serial.print("entry ");
    Serial.print(entry);
    Serial.print(": ");
    Serial.println(i);
    Serial.print(EEPROM.read(i),DEC);
    Serial.println("°c");
    Serial.println();
  }

}


//function modified from
//https://www.avrfreaks.net/forum/teensy-atmega32u4-chip-temperature-sensor-0
int8_t getTemperatureInternal() {

  //The temperature sensor and its internal driver are enabled when ADMUX value selects the temperature sensor as ADC input. The propagation delay of this driver is approximately 2μS. 
  //Therefore two successive conversions are required. The correct temperature measurement will be the second one.  
  //source http://ww1.microchip.com/downloads/en/devicedoc/atmel-7766-8-bit-avr-atmega16u4-32u4_datasheet.pdf
  //ADMUX value selects the temperature sensor as ADC input, 100111
  //Wrong: ADMUX = (1<<REFS1) | (1<<REFS0) | (1<<MUX2) | (1<<MUX1) | (1<<MUX0);
  //source see page 314 of http://ww1.microchip.com/downloads/en/devicedoc/atmel-7766-8-bit-avr-atmega16u4-32u4_datasheet.pdf
  ADMUX = (1<<REFS1) | (1<<MUX2) | (1<<MUX1) | (1<<MUX0);
  ADCSRB |= (1<<MUX5);

  //delay(2) onacceptable TODO
 
  // start the conversion
  ADCSRA |= bit(ADSC);

  // ADSC is cleared when the conversion finishes
  while (ADCSRA & bit(ADSC));

  uint8_t low  = ADCL;
  uint8_t high = ADCH;

  //discard first reading
  ADCSRA |= bit(ADSC);
  while (ADCSRA & bit(ADSC));
  low  = ADCL;
  high = ADCH;
  int a = (high << 8) | low;

  //return temperature in C
  return a - (272+3); // +3 is calibration
}
