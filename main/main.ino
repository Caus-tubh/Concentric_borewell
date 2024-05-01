// JC borewell device
// GSM code
// Node_ID = 2023;
#define SERVER = "api.thingspeak.com";
#define RESOURCE = "https://api.thingspeak.com/update?";
#define API_KEY = "PI7HWMKGZH0RS83G";
#define PORT = 80;
#define APN = "cmnet";
//const char gprsUser[] = "";
//const char gprsPass[] = "";
//const char simPIN[] = "";

bool res = 0;
bool wrong_distance = 0;
long duration;
int sent_status = 0;

#define WDT_TIMEOUT 32400000
#define WINDUP_PIN 13
#define MODEM_RST 14
#define MODEM_TX 26
#define MODEM_RX 27
#define SerialMon Serial
#define SerialAT Serial1
#define TINY_GSM_MODEM_SIM800
// Modem is SIM800
#define TINY_GSM_RX_BUFFER 1024  // Set RX buffer to 1Kb
#include <Wire.h>
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

TinyGsmClient client(modem);
#include <SPI.h>
#include <Wire.h>

int windup_reading = 0;
int stepPin = 2;
int dirPin = 0;
int forcePin = 34;
int reading = 0;

float distance = 0;  // Stores the distance of the bob from the ground surface in cm
int stepval = 50;    // We want 50 mm ticks
int nsteps = floor(stepval / dist_per_step);

unsigned long int reset_time = millis();
#include <esp_task_wdt.h>

void windUp()  // This function needs to be called only once in the void setup because it has to run only when the system is powered
{
  do {
    digitalWrite(dirPin, LOW);
    for (int i = 0; i < nsteps; i++)  // On full stepping, 200 steps = 1 revolution
    {
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(2500);
      digitalWrite(stepPin, LOW);
      delayMicroseconds(2500);
//      esp_task_wdt_reset();
    }
    windup_reading = (windup_reading + analogRead(WINDUP_PIN))/2;
  } while(windup_reading < 3000);
  distance = 0;
}

void setup() {
  // Declaring PinModes
  Serial.begin(115200);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(forcePin, INPUT);
  delay(1000);
  Serial.flush();

  esp_task_wdt_init(WDT_TIMEOUT, true); //enable panic so ESP32 restarts
  esp_task_wdt_add(NULL);

  // GSM setup code
  pinMode(MODEM_RST, OUTPUT);
  digitalWrite(MODEM_RST, HIGH);
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(3000);
  SerialMon.println("Initializing modem...");
  modem.restart(); 
  #if TINY_GSM_USE_GPRS
  // Unlock your SIM card with a PIN if needed
  if (GSM_PIN && modem.getSimStatus() != 3) { modem.simUnlock(GSM_PIN); }
  #endif
  windUp();  // This will make sure that the bob is at the top before starting readings
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

void loop() {
    sent_status = 0;
    SerialMon.println("Starting the Collection of Data");
    reading = analogRead(forcePin);
    if (reading >= 2500) {
      reading = 4095;
    } else {
      reading = 0;
    }
    stepval = 50;  // We want 50 mm ticks
    int nsteps = floor(stepval / dist_per_step);

    while (reading <= 10)  // This takes care of the case when the bob is on the water surface when the microcontroller wakes up
    {
      digitalWrite(dirPin, LOW);
      for (int i = 0; i < nsteps; i++)  // On full stepping, 200 steps = 1 revolution
      {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(2500);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(2500);
//        esp_task_wdt_reset();
      }
      delay(200);  // This delay allows for the force sensor value to stabilize after the motor rotates
      reading = analogRead(forcePin);
      if (reading >= 2500) {
        reading = 4095;
      } else {
        reading = 0;
      }
      distance -= stepval / 10;
      Serial.print("Distance = ");
      Serial.println((int)distance);
    }

    while (reading >= 10)  // This takes care of the case when the bob is hanging when the microcontroller wakes up
    {
      digitalWrite(dirPin, HIGH);
      for (int i = 0; i < nsteps; i++)  // On full stepping, 200 steps = 1 revolution
      {
        digitalWrite(stepPin, HIGH);
        delayMicroseconds(2500);
        digitalWrite(stepPin, LOW);
        delayMicroseconds(2500);
//        esp_task_wdt_reset();
      }
      delay(200);  // This delay allows for the force sensor value to stabilize after the motor rotates
      reading = analogRead(forcePin);
      if (reading >= 2500) {
        reading = 4095;
      } else {
        reading = 0;
      }
      distance += stepval / 10;
      Serial.print("Distance  = ");
      Serial.println((int)reading);
    }
    delay(500);
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
