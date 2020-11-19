// .\ArduinoSketchUploader.exe --file= SorTeS_GW07.hex --port=COM74 --model=Leonardo
// TODO: ctrl F for 'TODO'

#include <Arduino_FreeRTOS.h>
#include <EEPROM.h> //to bel able to writ in EEPROM using EEPROM.write(addr, val); our EEPROM is 1024 bits big
#include <LoRa.h>

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

//nextEntryAdress
byte nextEntryAdress;
bool listeningToCommands;

// the setup function runs once when you press reset or power the board
void setup() {
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
    while (1);
  }
  LoRa.onReceive(onReceive);
  LoRa.receive();
  
  // Now set up two tasks to run independently.
  /*
  xTaskCreate(
    TaskBlink
    ,  "Blink"   // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );

  xTaskCreate(
    TaskAnalogRead
    ,  "AnalogRead"
    ,  128  // Stack size
    ,  NULL
    ,  1  // Priority
    ,  NULL );

  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
  */
}


void loop() {
  //empty, we use Tasks
  
  if (listeningToCommands){
    int command=0;
     if (Serial.available() > 0) {
        // read the incoming byte:
        command=Serial.parseInt();
        switch (command) {
          case 1:
            printLastEntry();
            break;
          case 2:
            printdb();
            break;
          case 3:
            //deepSleep();
            break;
          default:
            break;
        }
      }
    }
}



void onReceive (int packetSize){
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

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/
void TaskBlink(void *pvParameters)  // This is a task.
{
  (void) pvParameters;


  // initialize digital LED_BUILTIN on pin 13 as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  for (;;) // A Task shall never return or exit.
  {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    vTaskDelay( 1000 / portTICK_PERIOD_MS ); // wait for one second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    vTaskDelay( 1000 / portTICK_PERIOD_MS ); // wait for one second
  }
}

void TaskAnalogRead(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  
  for (;;)
  {
    // read the input on analog pin 0:
    int sensorValue = analogRead(A0);
    // print out the value you read:
    Serial.println(sensorValue);
    vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
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
