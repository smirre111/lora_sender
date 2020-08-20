//https://www.alictronix.com/archives/860

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include "FS.h"
#include "SPIFFS.h"
#include "SSD1306.h"
#include <ArduinoJson.h>
 
//OLED pins to ESP32 GPIOs via this connecthin:
//OLED_SDA -- GPIO4
//OLED_SCL -- GPIO15
//OLED_RST -- GPIO16
 
SSD1306  display(0x3c, 4, 15);
 

StaticJsonDocument<200> doc;


// WIFI_LoRa_32 ports
// GPIO5  -- SX1278's SCK
// GPIO19 -- SX1278's MISO
// GPIO27 -- SX1278's MOSI
// GPIO18 -- SX1278's CS
// GPIO14 -- SX1278's RESET
// GPIO26 -- SX1278's IRQ(Interrupt Request)
 
#define SS      18
#define RST     14
#define DI0     26
#define BAND    433E6  //915E6 

struct Config
{
  char hostname[64];
  uint8_t address;
  uint8_t subnet;
};

const char *filename = "/config.txt"; // <- SD library uses 8.3 filenames
Config config;                        // <- global configuration object

const int loraCsPin = SS;     // LoRa radio chip select
const int resetPin = RST; // LoRa radio reset
const int irqPin = DI0;   // change for your board; must be a hardware interrupt pin
int counter = 0;
 
const byte broadcastAddressing = 0xFF;
const byte subnetAddressing = 0xFE;
String outgoing;          // outgoing message
byte msgCount = 0;        // count of outgoing messages
byte myAddress = 0x10;    // address of this device
byte mySubnet = 0x01;     // address of this device
byte destAddress = 0x11;  // destination to send to
byte destSubnet = 0x01;   // destination to send to
long lastSendTime = 0;    // last send time
int interval = 2000;      // interval between sends


void loadConfiguration(const char *filename, Config &config)
{
  // Open file for reading
  File file = SPIFFS.open(filename);

  // Allocate a temporary JsonDocument
  // Don't forget to change the capacity to match your requirements.
  // Use arduinojson.org/v6/assistant to compute the capacity.
  StaticJsonDocument<512> doc;

  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, file);
  if (error)
    Serial.println(F("Failed to read file, using default configuration"));

  // Copy values from the JsonDocument to the Config
  config.address = doc["address"] | 0x00;
  config.subnet = doc["subnet"] | 0x00;
  strlcpy(config.hostname,                 // <- destination
          doc["hostname"] | "example.com", // <- source
          sizeof(config.hostname));        // <- destination's capacity

  // Close the file (Curiously, File's destructor doesn't close the file)
  file.close();
}

void saveConfiguration(const char *filename, const Config &config)
{
  // Delete existing file, otherwise the configuration is appended to the file
  SPIFFS.remove(filename);

  // Open file for writing
  File file = SPIFFS.open(filename, FILE_WRITE);
  if (!file)
  {
    Serial.println(F("Failed to create file"));
    return;
  }

  // Allocate a temporary JsonDocument
  // Don't forget to change the capacity to match your requirements.
  // Use arduinojson.org/assistant to compute the capacity.
  StaticJsonDocument<256> doc;

  // Set the values in the document
  doc["hostname"] = config.hostname;
  doc["address"] = config.address;
  doc["subnet"] = config.subnet;

  // Serialize JSON to file
  if (serializeJson(doc, file) == 0)
  {
    Serial.println(F("Failed to write to file"));
  }

  // Close the file
  file.close();
}


const uint8_t CMD_UP = 1;
const uint8_t CMD_DOWN = 2;
const uint8_t CMD_ENABLE_WIFI = 3;
const uint8_t CMD_DISABLE_WIFI = 4;
const uint8_t CMD_OTA = 5;
const uint8_t CMD_STATUS = 6;

//B0 ... destination address
//B1 ... senderAddress address
//B2 ... message ID
//B3 ... payload length
//B4..N ... payload

//P0 ... command
//  01 ... up
//  02 ... down
//  03 ... enable wifi
//  04 ... disable wifi
//  05 ... ota
//  06 ... status

//P1 ... status field
//  01 ... battery voltage
//  02 ...


void sendMessage(byte payload)
{
  LoRa.beginPacket();            // start packet
  LoRa.write(destAddress);       // add destination address
  LoRa.write(destSubnet);      // add senderAddress address
  LoRa.write(myAddress);      // add senderAddress address
  LoRa.write(msgCount);          // add message ID
  //LoRa.write(outgoing.length()); // add payload length
  //LoRa.print(outgoing);          // add payload
  LoRa.write(1); // add payload length
  LoRa.write(payload);          // add payload
  LoRa.endPacket();              // finish packet and send it
  msgCount++;                    // increment message ID
}


void setup() {

  Serial.begin(115200);
  while (!Serial); //If just the the basic function, must connect to a computer

  if (!SPIFFS.begin())
  {
    Serial.println("Cannot access SPIFFS");
    return;
  }

  Serial.println(F("Loading configuration..."));
  loadConfiguration(filename, config);
  myAddress = config.address;
  mySubnet = config.subnet;
  Serial.print(F("Set address to..."));
  Serial.println(myAddress);


  pinMode(25,OUTPUT); //Send success, LED will bright 1 second
  pinMode(16,OUTPUT);
  digitalWrite(16, LOW);    // set GPIO16 low to reset OLED
  delay(50); 
  digitalWrite(16, HIGH);
   
  
  // Initialising the UI will init the display too.
  display.init();
  display.flipScreenVertically();
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(5,5,"LoRa Sender");
  display.display();
   
  SPI.begin(5,19,27,18);
  LoRa.setPins(SS,RST,DI0);
  Serial.println("LoRa Sender");
  if (!LoRa.begin(BAND)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }

  LoRa.setSpreadingFactor(7);
  LoRa.setCodingRate4(5);
  LoRa.setPreambleLength(8);
  LoRa.setSignalBandwidth(125e3);
  LoRa.setSyncWord(0x12);

  Serial.println("LoRa Initial OK!");
  display.drawString(5,20,"LoRa Initializing OK!");
  display.display();
  delay(2000);

}

byte *payload = new byte[128];

void loop() {
  
  Serial.print("Sender: packet: ");
  Serial.println(counter);
  display.clear();
  display.setFont(ArialMT_Plain_16);
  display.drawString(3, 5, "Sender: packet ");
  display.drawString(50, 30, String(counter));
  display.display();
  // send packet
#if 0
  LoRa.beginPacket();
  LoRa.print("Hello..");
  LoRa.print(counter);
  LoRa.endPacket();
#else
  
  payload[0] = counter;

  sendMessage(payload[0]);
#endif
  counter++;
  if (counter > CMD_STATUS) {
    counter = CMD_UP;
  }
  digitalWrite(25, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(1000);                       // wait for a second
  digitalWrite(25, LOW);    // turn the LED off by making the voltage LOW
  delay(1000);                       // wait for a second
  delay(5000);

}