// JC borewell device
// GSM code
// Node_ID = 2023;
//const char gprsUser[] = "";
//const char gprsPass[] = "";
//const char simPIN[] = "";
#define SERVER "api.thingspeak.com";
#define RESOURCE "https://api.thingspeak.com/update?";
#define API_KEY "PI7HWMKGZH0RS83G";
#define PORT 80;
#define APN "cmnet";
#define SerialMon Serial
#define SerialAT Serial1
#define TINY_GSM_MODEM_SIM800
#define WDT_TIMEOUT 32400000 // 9 hours
#define TINY_GSM_RX_BUFFER 1024  // Set RX buffer to 1Kb

#define ENCODERPINA 4 
#define ENCODERPINB 16
#define TENSION_SWITCH 17
#define WINDUP_SWITCH 5
#define MODEM_RST 2
#define TX 18
#define RX 19
#define CS 15
#define CLK 14
#define MISO 12
#define MOSI 13
#define MOTFWD 27 
#define MOTREV 26
#define SDA 25
#define SCL 33

#include <SPI.h>
#include <Wire.h>
#include <SoftwareWire.h>  
#include <RtcDS1307.h>
#include <esp_task_wdt.h>
#include <TinyGsmClient.h>
#ifdef DUMP_AT_COMMANDS
#include <StreamDebugger.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
StreamDebugger debugger(SerialAT, SerialMon);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

SoftwareWire myWire(SDA, SCL);
RtcDS1307<SoftwareWire> Rtc(myWire);
TinyGsmClient client(modem);


bool windup_reading = 0;
bool tension_reading = 0;
int last_encoded = 0; // Here updated value of encoder store.
long encoder_value = 0; // Raw encoder value
unsigned long int reset_time = millis();
float diameter = 10
float circumference = 3.14159 * diameter 
bool res = 0;
bool wrong_distance = 0;
long duration;
bool sent_status = 0;


void windUp()  // This function needs to be called only once in the void setup because it has to run only when the system is powered
{
  do {
    digitalWrite(MOTFWD, LOW);  
    digitalWrite(MOTREV, HIGH);
    windup_reading = digitalRead(WINDUP_SWITCH);
  } while(windup_reading == 1); 
  digitalWrite(MOTREV, LOW);
  distance = 0;
  encoder_value = 0;
}

void IRAM_ATTR updateEncoder(){
  int MSB = digitalRead(ENCODERPINA); //MSB = most significant bit
  int LSB = digitalRead(ENCODERPINB); //LSB = least significant bit

  int encoded = (MSB << 1) |LSB; //converting the 2 pin value to single number
  int sum  = (last_encoded << 2) | encoded; //adding it to the previous encoded value

  if(sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011) encoder_value --;
  if(sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000) encoder_value ++;

  last_encoded = encoded; //store this value for next time

}

void writeData(float distance, const RtcDateTime& dt){
  File dataFile;
  dataFile = SD.open("datalog.txt", FILE_WRITE);
  if (dataFile) {
  char datestring[26];
  snprintf_P(datestring,  
      countof(datestring),
      PSTR("%04u-%02u-%02uT%02u:%02u:%02u+00:00"),
      dt.Year(),
      dt.Month(),
      dt.Day(),
      dt.Hour(),
      dt.Minute(),
      dt.Second());
  dataFile.print(datestring);
  dataFile.print(" ");
  dataFile.print("distance : ");
  dataFile.println(distance);
  dataFile.close();
  // print to the serial port too:
  //Serial.println(dataString);
  }
//  else {
//    Serial.println("error opening datalog.txt");
//  }
  delay(2000);
  //reset();
}

void send_data(float Reading) {
  int csq = modem.getSignalQuality();
  SerialMon.println("Signal Quality - ");
  SerialMon.print(csq);
  SerialMon.println(" OK");
  SerialMon.print("Connecting to ");
  SerialMon.print(server);
  if (!client.connect(SERVER, PORT)) {
    SerialMon.println(" fail");
  } else {
    SerialMon.println(" OK");
    SerialMon.println("Performing HTTP POST request...");
    String httpRequestData = "GET https://api.thingspeak.com/update?api_key="+ API_KEY + "&field1=" + Reading + "";
    client.println(httpRequestData);
    delay(100);

    unsigned long timeout = millis();
    while (client.connected() && millis() - timeout < 10000L) {
      while (client.available()) {
        String line = client.readStringUntil('\n');
        if (line.substring(0, 14).equals("Status: 200 OK")) {
          sent_status = 1;
        }
        timeout = millis();
      }
    }
    SerialMon.println();
    delay(1000);
    client.stop();
    delay(2000);
  }
}

void setup() {
  Serial.begin(115200);
  SerialAT.begin(115200, SERIAL_8N1, RX, TX);
  
  pinMode(TENSION_SWITCH, INPUT_PULLUP);
  pinMode(WINDUP_SWITCH, INPUT_PULLUP);
  pinMode(MOTFWD, OUTPUT); 
  pinMode(MOTREV, OUTPUT);
  pinMode(MODEM_RST, OUTPUT);
  pinMode(ENCODERPINA, INPUT_PULLUP); 
  pinMode(ENCODERPINB, INPUT_PULLUP);
  delay(1000);
  Serial.flush();

  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL);
  delay(3000);
  
  SerialMon.println("Initializing modem...");
  digitalWrite(MODEM_RST, HIGH);
  modem.restart(); 
  if (GSM_PIN && modem.getSimStatus() != 3) { modem.simUnlock(GSM_PIN); } // Unlock your SIM card with a PIN if needed
  #endif
  
  attachInterrupt(ENCODERPINA, updateEncoder, CHANGE); 
  attachInterrupt(ENCODERPINB, updateEncoder, CHANGE);

  if (!SD.begin(CS)) {
    send_data(-100.69);
  }
  windUp();  // This will make sure that the bob is at the top before starting readings
}

void loop() {
    sent_status = 0;
    SerialMon.println("Starting the Collection of Data");
    tension_reading = digitalRead(TENSION_SWITCH);
    while (tension_reading == 0){
      digitalWrite(MOTFWD, LOW);  
      digitalWrite(MOTREV, HIGH); 
      delay(200);   
      tension_reading = digitalRead(TENSION_SWITCH);
    }
    digitalWrite(MOTREV, LOW); 
    distance = encoder_value * circumference;
    Serial.print("Distance = ");
    Serial.println((int)distance);

    while (tension_reading == 1){
      digitalWrite(MOTFWD, HIGH);  
      digitalWrite(MOTREV, LOW); 
      delay(200);   
      tension_reading = digitalRead(TENSION_SWITCH);
    }
    digitalWrite(MOTFWD, LOW); 
    distance = encoder_value * circumference;
    Serial.print("Distance = ");
    Serial.println((int)distance);
    delay(500);
    
    writeData(distance,RtcDateTime(__DATE__, __TIME__));
    
    modem.gprsConnect(apn);
    modem.waitForNetwork(600000L);
    res = modem.isGprsConnected();
    send_data(distance);
    while (sent_status == 0) {
      SerialMon.println("Sending Data Again");
      modem.restart();
      modem.gprsConnect(apn);
      modem.waitForNetwork(600000L);
      send_data(distance);
    }
    modem.gprsDisconnect();
    SerialMon.println("Data Sent Successfully");
    delay(15 * 60 * 1000);
    esp_task_wdt_reset();
    if (millis() - reset_time >= 8* 60* 60 * 1000) {
        reset_time = millis();
        windUp(); 
    }
  }
