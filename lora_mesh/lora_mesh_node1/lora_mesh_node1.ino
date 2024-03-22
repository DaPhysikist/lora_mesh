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

// Change to 434.0 or other frequency, must match RX's freq!
#define RF95_FREQ 915.0
// Who am i? (client address)
#define MY_ADDRESS   1
#define DEST_ADDRESS 2
#define N_NODES 255

uint8_t routes[N_NODES]; // full routing table for mesh
int16_t rssi[N_NODES]; // signal strength info

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Class to manage message delivery and receipt, using the driver declared above
RHMesh manager(rf95, MY_ADDRESS);

void updateRoutingTable() {
  for(uint8_t n=1;n<=N_NODES;n++) {
    RHRouter::RoutingTableEntry *route = manager.getRouteTo(n);
    if (n == MY_ADDRESS) {
      routes[n-1] = 255; // self
    } else {
      routes[n-1] = route->next_hop;
      if (routes[n-1] == 0) {
        // if we have no route to the node, reset the received signal strength
        rssi[n-1] = 0;
      }
    }
  }
}

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
  rf95.setTxPower(2, false);  
  Serial.print("RFM95 radio @");  Serial.print((int)RF95_FREQ);  Serial.println(" MHz");

  for(uint8_t n=1;n<=N_NODES;n++) {
    routes[n-1] = 0;
    rssi[n-1] = 0;
  }
}

// Dont put this on the stack:
uint8_t data[] = "Node 1 speaking";
uint8_t response[] = "Node 1 responding";
// Dont put this on the stack:
char buf[RH_RF95_MAX_MESSAGE_LEN];

void loop() {
    updateRoutingTable();

    // listen for incoming messages. Wait a random amount of time before we transmit
    // again to the next node
    unsigned long nextTransmit = millis() + random(3000, 5000);
    while (nextTransmit > millis()) {
      int waitTime = nextTransmit - millis();
      uint8_t len = sizeof(buf);
      uint8_t from;
      if (manager.recvfromAckTimeout((uint8_t *)buf, &len, waitTime, &from)) {
        buf[len] = '\0'; // null terminate string
        Serial.print("Got a message from address: "); Serial.print(from);
        Serial.print(" [RSSI :");
        Serial.print(rf95.lastRssi());
        Serial.print("] : ");
        Serial.println(buf);

        uint8_t error = manager.sendtoWait(response, sizeof(response), from);
        if (error != RH_ROUTER_ERROR_NONE) {
          Serial.print("Error: ");
          Serial.println(getErrorString(error));
        }

        RHRouter::RoutingTableEntry *route = manager.getRouteTo(from);
        if (route->next_hop != 0) {
          rssi[route->next_hop-1] = rf95.lastRssi();
        }
      }
    }
}