#include <SPI.h>
#include <RH_RF95.h>
#include <RHMesh.h>

/************ Radio Setup ***************/
// First 3 here are boards w/radio BUILT-IN. Boards using FeatherWing follow.
#if defined (__AVR_ATmega32U4__)  // Feather 32u4 w/Radio
  #define RFM95_CS    8
  #define RFM95_INT   7
  #define RFM95_RST   4
  #define LED        13

#elif defined(ADAFRUIT_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0_EXPRESS) || defined(ARDUINO_SAMD_FEATHER_M0)  // Feather M0 w/Radio
  #define RFM95_CS    8
  #define RFM95_INT   3
  #define RFM95_RST   4
  #define LED        13

#elif defined(ARDUINO_ADAFRUIT_FEATHER_RP2040_RFM)  // Feather RP2040 w/Radio
  #define RFM95_CS   16
  #define RFM95_INT  21
  #define RFM95_RST  17
  #define LED        LED_BUILTIN

#elif defined (__AVR_ATmega328P__)  // Feather 328P w/wing
  #define RFM95_CS    4  //
  #define RFM95_INT   3  //
  #define RFM95_RST   2  // "A"
  #define LED        13

#elif defined(ESP8266)  // ESP8266 feather w/wing
  #define RFM95_CS    2  // "E"
  #define RFM95_INT  15  // "B"
  #define RFM95_RST  16  // "D"
  #define LED         0

#elif defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2) || defined(ARDUINO_NRF52840_FEATHER) || defined(ARDUINO_NRF52840_FEATHER_SENSE)
  #define RFM95_CS   10  // "B"
  #define RFM95_INT   9  // "A"
  #define RFM95_RST  11  // "C"
  #define LED        13

#elif defined(ESP32)  // ESP32 feather w/wing
  #define RFM95_CS   33  // "B"
  #define RFM95_INT  27  // "A"
  #define RFM95_RST  13  // same as LED
  #define LED        13

#elif defined(ARDUINO_NRF52832_FEATHER)  // nRF52832 feather w/wing
  #define RFM95_CS   11  // "B"
  #define RFM95_INT  31  // "C"
  #define RFM95_RST   7  // "A"
  #define LED        17
#endif

//Changeable params
#define RF95_FREQ 915.0
#define DEST_ADDRESS 1
#define TX_POWER 2
#define LISTEN_TIME 5000  //time we listen for a message
#define MY_ADDRESS   2

uint8_t data[] = "Node 2 speaking";   //Changeable data
char buf[RH_RF95_MAX_MESSAGE_LEN];  //allocate memory for message buffer on the heap
RH_RF95 rf95(RFM95_CS, RFM95_INT);
RHMesh manager(rf95, MY_ADDRESS);

char* getErrorString(uint8_t error) {
  switch(error) {
    case 1: return "invalid length";
    break;
    case 2: return "no route";
    break;
    case 3: return "timeout";
    break;
    case 4: return "no reply";
    break;
    case 5: return "unable to deliver";
    break;
  }
  return "unknown";
}

void setup() {
  Serial.begin(115200);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.println("Feather Addressed RFM95 RX Test!");
  Serial.println();

  // manual reset
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  if (!manager.init()) {
    Serial.println("RFM95 radio init failed");
    while (1);
  }
  Serial.println("RFM95 radio init OK!");
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
  }
  rf95.setTxPower(TX_POWER, false);  
  Serial.print("RFM95 radio @");  Serial.print((int)RF95_FREQ);  Serial.println(" MHz");
}

void loop() {
    // send an acknowledged message to the target node
    Serial.print("Sending message: ");
    Serial.println((char*)data);
    uint8_t error = manager.sendtoWait(data, sizeof(data), DEST_ADDRESS);
    if (error != RH_ROUTER_ERROR_NONE) {
      Serial.print("Error: ");
      Serial.println(getErrorString(error));
    } else {
      Serial.println("Message sent successfully.");
    }

    // listen for incoming messages. Wait a random amount of time before we transmit
    // again to the next node
    unsigned long nextTransmit = millis() + LISTEN_TIME;
    while (nextTransmit > millis()) {
      uint8_t len = sizeof(buf);
      uint8_t from;
      if (manager.recvfromAckTimeout((uint8_t *)buf, &len, LISTEN_TIME, &from)) {
        buf[len] = '\0'; // null terminate string
        Serial.print("Got a message from address: "); Serial.print(from);
        Serial.print(" [RSSI :");
        Serial.print(rf95.lastRssi());
        Serial.print("] : ");
        Serial.println(buf);
        // we received data from node 'from', but it may have actually come from an intermediate node
        RHRouter::RoutingTableEntry *route = manager.getRouteTo(from);
        if (route->next_hop != 0) {
          Serial.print("Last hop: ");
          Serial.println(route->next_hop);
        }
      }
    }
}