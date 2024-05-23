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
#define RF95_FREQ 915.0   //using 915 MHz for US
#define MY_ADDRESS   1    //node's address
#define TX_POWER 2    //transmision power for lab testing
#define LISTEN_TIME 5000  //time we listen for a message
#define TEST_DELAY 1000  //wait 1 second before beginning next cycle

RH_RF95 rf95(RFM95_CS, RFM95_INT);  //radio driver
RHMesh manager(rf95, MY_ADDRESS);   //mesh manager instance
int beginTest;

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
  Serial.begin(9600);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  Serial.println("Feather Addressed RFM95 RX Test!");
  Serial.println();

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
    if (Serial.available() > 0) {
      String input = Serial.readStringUntil('\n'); // Read until newline character
      String txPower = getValue(input, ',', 0);
      String bandwidth = getValue(input, ',', 1);
      String spreading_factor = getValue(input, ',', 2);
      String coding_rate = getValue(input, ',', 3);

      if (txPower != "noupdate"){
        int tp = txPower.toInt();
        if (tp != 0) {
            if (tp > 0 && tp < 21) {
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
    }

    char buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);
    uint8_t from;
    String responseStr = "Node 1 responding";
    if (manager.recvfromAckTimeout((uint8_t *)buf, &len, LISTEN_TIME, &from)) {  // listen for incoming messages
      Serial.print("Got a message from address: "); Serial.print(from);
      Serial.print(" [RSSI :");
      Serial.print(rf95.lastRssi());
      Serial.print("] : ");
      Serial.print(" [SNR :");
      Serial.print(rf95.lastSNR());
      Serial.print("] : ");
      Serial.println(buf);

      if (strcmp(buf, "begin test") == 0){
        beginTest = 1;
        Serial.println("Begin the test");
        responseStr = "begin test";
      }

      uint8_t response[responseStr.length() + 1];
      memcpy(response, responseStr.c_str(), responseStr.length() + 1);
      response[responseStr.length()] = 0;

      uint8_t error = manager.sendtoWait(response, sizeof(response), from);   //respond to messages
      if (error != RH_ROUTER_ERROR_NONE) {
        Serial.print("Error: ");
        Serial.println(getErrorString(error));
      }
    }
  }
  else if (beginTest == 1){
    uint32_t startTime = millis();
    String startMessage = "Begin Test! Begin time: " + String(startTime);
    Serial.println(startMessage);
    while (1){
      uint16_t packet_id = 0;
      int txPower;
      int spreadingFactor = 7;
      int bandwidth = 125000;
      for (int k = 62500; k<=500000; k*=2){
        bandwidth = k;
        rf95.setSignalBandwidth(bandwidth);
        for (int j = 9; j >= 7; j--){
          rf95.setSpreadingFactor(j);
          spreadingFactor = j;
          for (int i = 20; i >= 2; i--){
            rf95.setTxPower(i, false);
            txPower = i;
            for (int l = 0; l < 10; l++){
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
              int payloadLength = 28;  //twelve bytes in payload, including packet id  

              float payloadTime = (8 + max(ceil((8*payloadLength-4*spreadingFactor+28+16*crc-20*ih)/(4*(spreadingFactor-2*de)))*(cr+4), 0))*symbolTime;  //calculate payload time in ms based on datasheet formula

              int listenTime = 2*ceil(preambleTime+payloadTime);  //calculate on air time in ms, rounded up to nearest integer, then multiply by two to account for both ways
              uint8_t buf[28];
              uint8_t len = sizeof(buf);
              uint8_t from;
              if (manager.recvfromAckTimeout(buf, &len, listenTime, &from)) {  // listen for incoming messages
                uint16_t recvPacketId = (buf[0] << 8) | (buf[1]);
                uint32_t sent_time = ((buf[2] << 24) | (buf[3] << 16) | (buf[4] << 8) | (buf[5]));
                uint32_t recv_bw = ((buf[6] << 24) | (buf[7] << 16) | (buf[8] << 8) | (buf[9]));
                uint32_t recv_txp = ((buf[10] << 24) | (buf[11] << 16) | (buf[12] << 8) | (buf[13]));
                uint32_t recv_sf = ((buf[14] << 24) | (buf[15] << 16) | (buf[16] << 8) | (buf[17]));
                uint32_t recv_time = millis();
                uint8_t correctCount = 0;
                for (uint8_t i = 6; i < len; i++){
                  if ((i-17) == buf[i]){
                    correctCount++;
                  }
                }
                Serial.print("Got a message from address: "); Serial.print(from);
                Serial.print(" [Packet ID :");
                Serial.print(recvPacketId);
                Serial.print("] [Sent Time :");
                Serial.print(sent_time);
                Serial.print("] [Recv Time :");
                Serial.print(recv_time);
                Serial.print("] [Recv BW :");
                Serial.print(recv_bw);
                Serial.print("] [Recv TX Power :");
                Serial.print(recv_txp);
                Serial.print("] [Recv SF :");
                Serial.print(recv_sf);
                Serial.print("] [RSSI :");
                Serial.print(rf95.lastRssi());
                Serial.print("] [SNR :");
                Serial.print(rf95.lastSNR());
                Serial.print("]  [# Correct Bytes Received : ");
                Serial.print(correctCount);
                Serial.println("]");

                uint8_t packet_id_hi = (packet_id >> 8);
                uint8_t packet_id_lo = (packet_id & 0xFF);

                uint8_t bandwidth_bytes[4];
                bandwidth_bytes[0] = (bandwidth >> 24) & 0xFF;
                bandwidth_bytes[1] = (bandwidth >> 16) & 0xFF;
                bandwidth_bytes[2] = (bandwidth >> 8) & 0xFF;
                bandwidth_bytes[3] = bandwidth & 0xFF;

                uint8_t tp_bytes[4];
                tp_bytes[0] = (txPower >> 24) & 0xFF;
                tp_bytes[1] = (txPower >> 16) & 0xFF;
                tp_bytes[2] = (txPower >> 8) & 0xFF;
                tp_bytes[3] = txPower & 0xFF;

                uint8_t sf_bytes[4];
                sf_bytes[0] = (spreadingFactor >> 24) & 0xFF;
                sf_bytes[1] = (spreadingFactor >> 16) & 0xFF;
                sf_bytes[2] = (spreadingFactor >> 8) & 0xFF;
                sf_bytes[3] = spreadingFactor & 0xFF;

                uint32_t sentTime = millis();
                uint8_t sentTime_bytes[4];
                sentTime_bytes[0] = (sentTime >> 24) & 0xFF;
                sentTime_bytes[1] = (sentTime >> 16) & 0xFF;
                sentTime_bytes[2] = (sentTime >> 8) & 0xFF;
                sentTime_bytes[3] = sentTime & 0xFF;
                uint8_t response[] = {packet_id_hi, packet_id_lo, sentTime_bytes[0], sentTime_bytes[1], sentTime_bytes[2], sentTime_bytes[3], bandwidth_bytes[0], bandwidth_bytes[1], bandwidth_bytes[2], bandwidth_bytes[3], tp_bytes[0], tp_bytes[1], tp_bytes[2], tp_bytes[3], sf_bytes[0], sf_bytes[1], sf_bytes[2], sf_bytes[3], 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A};
                uint8_t error = manager.sendtoWait(response, sizeof(response), from);   //respond to messages
                if (error != RH_ROUTER_ERROR_NONE) {
                  Serial.print("Error: ");
                  Serial.print(getErrorString(error));
                  String message = " Packet ID: " + String(packet_id) + " Sent Time: " + String(sentTime) + " Bandwidth: " + String(bandwidth) + " TX Power: " + String(txPower) + " SF: " + String(spreadingFactor);
                  Serial.println(message);
                }
                else {
                  uint32_t ackTime = millis();
                  String message = "Message sent successfully. Packet ID: " + String(packet_id) + " Sent Time: " + String(sentTime) + " Ack Time: " + String(ackTime);
                  Serial.println(message);
                }
                packet_id++;
                delay(1);
              }
            }
          }
        }
      }
      delay(TEST_DELAY);  
    }
    beginTest = 0;
    uint32_t endTime = millis();
    String endMessage = "End Test! End time: " + String(endTime);
    Serial.println(endMessage);
  }
}