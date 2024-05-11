#include <SPI.h>
#include <RH_RF95.h>
#include <RHMesh.h>
#include <array>

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
#define TEST_DELAY 1000 //delay start of next test cyle by 1 second for now

int beginTest;
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

String getValue(String data, char separator, int index) {
    int found = 0;
    int strIndex[] = {0, -1};
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            if (found == index) {
                strIndex[1] = (i == maxIndex && data.charAt(i) != separator) ? i + 1 : i;
            }
            found++;
            if (found <= index) {
                strIndex[0] = i + 1;
            }
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
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
    if (beginTest == 0){
      // send an acknowledged message to the target node
      String str = "Node 2 speaking";

      if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n'); // Read until newline character
        
        String txPower = getValue(input, ',', 0);
        String bandwidth = getValue(input, ',', 1);
        String spreading_factor = getValue(input, ',', 2);
        String coding_rate = getValue(input, ',', 3);
        String message = getValue(input, ',', 4);

        if (txPower != "noupdate"){
          int tp = txPower.toInt();
          if (tp != 0) {
              if (tp > 1 && tp < 21) {
                rf95.setTxPower(tp, false);
                String output = "Set transmission power to: " + String(tp);
                Serial.println(output);
              }
              else {
                Serial.println("Error: Invalid power value");
              } 
          } else {
              Serial.println("Error: Invalid tp input");
          }
        }

        if (bandwidth != "noupdate"){
          int bw = bandwidth.toInt();
            if (bw != 0) {
                rf95.setSignalBandwidth(bw);
                String output = "Set bandwidth to: " + String(bw);
                Serial.println(output);
            } else {
                Serial.println("Error: Invalid bandwidth input");
            }
        }

        if (spreading_factor != "noupdate"){
            int sf = spreading_factor.toInt();
            if (sf != 0) {
                if (sf > 5 && sf < 13) {
                  rf95.setSpreadingFactor(sf);
                  String output = "Set spreading factor to: " + String(sf);
                  Serial.println(output);
                }
                else {
                  Serial.println("Error: Invalid spreading factor");
                } 
            } else {
                Serial.println("Error: Invalid sf input");
            }
        }

        if (coding_rate != "noupdate"){
          int cr = coding_rate.toInt();
          if (cr != 0) {
              if (cr > 4 && cr < 9) {
                rf95.setCodingRate4(cr);
                String output = "Set coding rate to: " + String(cr);
                Serial.println(output);
              }
              else {
                Serial.println("Error: Invalid coding rate");
              } 
          } else {
              Serial.println("Error: Invalid cr input");
          }
        }

        if (message != "noupdate") {
            Serial.print("Message is: ");
            Serial.println(message);
            str = message;
        }
      }

      uint8_t data[str.length() + 1];
      memcpy(data, str.c_str(), str.length() + 1);
      data[str.length()] = 0;
      Serial.print("Sending message: ");
      Serial.println((char*)data);
      uint8_t error = manager.sendtoWait(data, sizeof(data), DEST_ADDRESS);
      if (error != RH_ROUTER_ERROR_NONE) {
        Serial.print("Error: ");
        Serial.println(getErrorString(error));
      } else {
        Serial.println("Message sent successfully.");
        if (str == "begin test"){
          beginTest = 1;
        }
      }

      // listen for incoming messages. Wait a random amount of time before we transmit
      // again to the next node
      unsigned long nextTransmit = millis() + LISTEN_TIME;
      while (nextTransmit > millis()) {
        char buf[RH_RF95_MAX_MESSAGE_LEN];
        uint8_t len = sizeof(buf);
        uint8_t from;
        if (manager.recvfromAckTimeout((uint8_t *)buf, &len, LISTEN_TIME, &from)) {
          buf[len] = '\0'; // null terminate string
          Serial.print("Got a message from address: "); Serial.print(from);
          Serial.print(" [RSSI :");
          Serial.print(rf95.lastRssi());
          Serial.print("] : ");
          Serial.print(" [SNR :");
          Serial.print(rf95.lastSNR());
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
    else if (beginTest == 1) {
      while (1){
        uint16_t packet_id = 0;
        int txPower;
        int spreadingFactor;
        int bandwidth;
        for (int i = 20; i >= 2; i--){
          rf95.setTxPower(20, false);
          txPower = i;
          for (int j = 12; j >= 6; j--){
            rf95.setSpreadingFactor(i);
            spreadingFactor = j;
            for (int k = 62.5; k<=500; k*=2){
              bandwidth = k * 1000;
              rf95.setSignalBandwidth(bandwidth);
              bandwidth = k;
              for (int l = 0; l < 10; l++){
                uint8_t packet_id_hi = (packet_id >> 8);
                uint8_t packet_id_lo = (packet_id & 0xFF);
                uint8_t data[] = {packet_id_hi, packet_id_lo, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A};
                uint8_t error = manager.sendtoWait(data, sizeof(data), DEST_ADDRESS);
                if (error != RH_ROUTER_ERROR_NONE) {
                  Serial.print("Error: ");
                  Serial.println(getErrorString(error));
                } else {
                  Serial.println("Message sent successfully.");
                }
                packet_id++;

                //Calculating on air time, based on datasheet formula on page 29
                float symbolTime = 1000.0 * pow(2, spreadingFactor) / bandwidth;  //symbol time in ms

                uint8_t preambleLength = 8;
                float preambleTime = (preambleLength + 4.25)*symbolTime; //preamble time in ms

                uint8_t de = 0;    //low data rate is off by default
                if (symbolTime > 16.0){       //threshold for low data rate to be on is 16 ms symbol time according to RadioHead
                  de = 1;
                }      
                uint8_t cr = 1;  //default coding rate is 4/5, which is 1 by the datasheet
                uint8_t ih = 0; //explicit instead of implicit header
                uint8_t crc = 1; //crc on
                int payloadLength = 12;  //twelve bytes in payload, including packet id  

                float payloadTime = (8 + max(ceil((8*payloadLength-4*spreadingFactor+28+16*crc-20*ih)/(4*(spreadingFactor-2*de)))*(cr+4), 0))*symbolTime;  //calculate payload time in ms based on datasheet formula

                int listenTime = 2*ceil(preambleTime+payloadTime);  //calculate on air time in ms, rounded up to nearest integer, then multiply by two to account for both ways
                unsigned long nextTransmit = millis() + listenTime;
                while (nextTransmit > millis()) {
                  uint8_t buf[12];
                  uint8_t len = sizeof(buf);
                  uint8_t from;
                  if (manager.recvfromAckTimeout(buf, &len, listenTime, &from)) {
                    uint16_t packet_id = (buf[0] << 8) | (buf[1]);
                    uint8_t correctCount;
                    for (uint8_t i = 2; i < len; i++){
                      if ((i-1) == buf[i]){
                        correctCount++;
                      }
                    }
                    Serial.print("Got a message from address: "); Serial.print(from);
                    Serial.print(" [Packet ID :");
                    Serial.print(packet_id);
                    Serial.print(" [RSSI :");
                    Serial.print(rf95.lastRssi());
                    Serial.print("] [SNR :");
                    Serial.print(rf95.lastSNR());
                    Serial.print("]  [# Correct Bytes Received : ");
                    Serial.println(correctCount);
                    Serial.print("]");
                  }
                }
              }
            }
          }
        }
        delay(TEST_DELAY);  
      }
    }
}