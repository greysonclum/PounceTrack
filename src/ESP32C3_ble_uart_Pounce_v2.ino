/*
    This code is a poorly organized combination of BLE UART and IR remote examples from ESP32. GSC 12/12/2025.

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
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

/*
 * Specify which protocol(s) should be used for decoding.
 * If no protocol is defined, all protocols (except Bang&Olufsen) are active.
 * This must be done before the #include <IRremote.hpp>
 */

#include <Arduino.h>
#define DECODE_LG
#define DECODE_NEC          // Includes Apple and Onkyo. To enable all protocols , just comment/disable this line.
//#define DECODE_BOSEWAVE
//#define DECODE_LEGO_PF
//#define DECODE_MAGIQUEST
//#define DECODE_WHYNTER
//#define DECODE_FAST
//#define DECODE_DISTANCE_WIDTH // Universal decoder for pulse distance width protocols
//#define DECODE_HASH         // special decoder for all protocols
//#define DECODE_BEO          // This protocol must always be enabled manually, i.e. it is NOT enabled if no protocol is defined. It prevents decoding of SONY!
//#define DEBUG               // Activate this for lots of lovely debug output from the decoders.
//#define RAW_BUFFER_LENGTH  750 // For air condition remotes it requires 750. Default is 200.

/*
 * This include defines the actual pin number for pins like IR_RECEIVE_PIN, IR_SEND_PIN for many different boards and architectures
 */
#include "PinDefinitionsAndMore.h"
#include <IRremote.hpp> // include the library

BLEServer *pServer = NULL;
BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;
bool oldDeviceConnected = false;
bool convert = false;
String rxString = "";
uint8_t txValue = 0;

// Pin definitions
int dirA = D0;
int dirB = D1;
int switchPin = 7;

//Variables
int speedA = 0;
int speedB = 0;
int speed = 250;
int switchState = HIGH;
bool shuffleEnable = false;
int state = 0;
int sample = 0;
int runTime = 10000; //how long shuffle mode will run before changing state in ms
int maxTimeOn = 600000; //run for 10 min max so that the motor does burn up or something bad
unsigned long previousTime = 0;
unsigned long manualStartTime = 0;

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"  // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    deviceConnected = true;
    Serial.println("Device connected");
  };

  void onDisconnect(BLEServer *pServer) {
    deviceConnected = false;
    Serial.println("Device disconnected");
  }
};

class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String rxValue = pCharacteristic->getValue();

    if (rxValue.length() > 0) {
      convert = true;
      Serial.println("*********");
      Serial.print("Received Value: ");
      for (int i = 0; i < rxValue.length(); i++) {
        Serial.print(rxValue[i]);
        rxString = rxString + rxValue[i];
      }

      Serial.println();
      Serial.println("*********");
    }
  }
};

void setup() {
  Serial.begin(115200);
  pinMode(dirA, OUTPUT);
  pinMode(dirB, OUTPUT);
  pinMode(switchPin, INPUT_PULLUP);
  //seems like I need to set IRpin as input but it seems to work so far without?

  // Create the BLE Device
  BLEDevice::init("Pounce Track v1.0");

  // Create the BLE Server
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX, BLECharacteristic::PROPERTY_NOTIFY);

  // Descriptor 2902 is not required when using NimBLE as it is automatically added based on the characteristic properties
  pTxCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX, BLECharacteristic::PROPERTY_WRITE);

  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");

  //IR Remote initialization
  // Just to know which program is running on my Arduino
  Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing library version " VERSION_IRREMOTE));

  // Start the receiver and if not 3. parameter specified, take LED_BUILTIN pin from the internal boards definition as default feedback LED
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

  Serial.print(F("Ready to receive IR signals of protocols: "));
  printActiveIRProtocols(&Serial);
  Serial.println(F("at pin " STR(IR_RECEIVE_PIN)));
}

void loop() {
  //Check what position the switch is in for remote or manual control.
  switchState = digitalRead(switchPin);

  if (switchState == LOW) {
    //manual mode
    shuffleEnable = true;

    //sample time when this switch is active
    if (sample == 0) {
      manualStartTime = millis();
      sample = 1;
    }

    //Let max amount of time pass. Once passed, turn off motors to keep stuff from burning up. Set and forget.
    if (millis()-manualStartTime > maxTimeOn) {
      speedA = 0;
      speedB = 0;

      //Put controller in a loop that never times out until reset
      while (true) {
        Serial.println("Timed out");
        delay(2000);
      }
    }
  }



  else {
    //restart time to sample for motor timeout since 2 pos switch is in remote mode now
    sample = 0;
    
    //remote control mode
    if (deviceConnected) {
      //Serial.print("Notifying Value: ");
      //Serial.println(txValue);
      pTxCharacteristic->setValue(&txValue, 1);
      pTxCharacteristic->notify();
      txValue++;
      delay(100);  // Notifying every .1 second

      if(convert) convertControlpad();
    }

    // disconnecting
    if (!deviceConnected && oldDeviceConnected) {
      delay(500);                   // give the bluetooth stack the chance to get things ready
      pServer->startAdvertising();  // restart advertising
      Serial.println("Started advertising again...");
      oldDeviceConnected = false;
    }
    // connecting
    if (deviceConnected && !oldDeviceConnected) {
      // do stuff here on connecting
      oldDeviceConnected = true;
    }

    //Always look for IR control but only run BLE control if device connected 
    IRcontrol();
  }

    //Shuffle mode 
    if(shuffleEnable) shuffleMode1();
  
    //Control motor based on inputs from BLE or IR remotes
    analogWrite(dirA, speedA);
    analogWrite(dirB, speedB);
  
}

// ************************* CONVERT CONTROLPAD CODE ************************
void convertControlpad() {
  convert = false;
  Serial.print("      ");
  if (rxString == "!B11:") {
    //MANUAL CONTROL
    Serial.println("********** Start Action 1");
    shuffleEnable = false;
    speedA = 0;
    speedB = 0;
  } 
  else if (rxString == "!B219") {
    //SHUFFLE MODE 1
    Serial.println("********** Start Action 2");
    shuffleEnable = true; 
  } 
  else if (rxString == "!B318") Serial.println("********** Start Action 3");
  else if (rxString == "!B417") Serial.println("********** Start Action 4");
  else if (rxString == "!B516") {
    Serial.println("********** Start Action UP");
    speed += 25;
    if (speed > 255) {
      speed = 255; //max speed
    }
    Serial.print("New Speed: ");
    Serial.print(speed);
  } 
  else if (rxString == "!B615") {
    Serial.println("********** Start Action DOWN");
    speed -= 25;
    if (speed < 150) {
      speed = 150; //min speed
    }
    Serial.print("New Speed: ");
    Serial.print(speed);
  }
  else if (rxString == "!B714") {
    Serial.println("********** Start Action LEFT");
    shuffleEnable = false;
    speedA = 0;
    speedB = speed;
  }
  else if (rxString == "!B813") {
    Serial.println("********** Start Action RIGHT");
    shuffleEnable = false;
    speedA = speed;
    speedB = 0;
  }
  else if (rxString == "!B10;") Serial.println("********** Stop Action 1");
  else if (rxString == "!B20:") Serial.println("********** Stop Action 2");
  else if (rxString == "!B309") Serial.println("********** Stop Action 3");
  else if (rxString == "!B408") Serial.println("********** Stop Action 4");
  else if (rxString == "!B507") Serial.println("********** Stop Action UP");
  else if (rxString == "!B606") Serial.println("********** Stop Action DOWN");
  else if (rxString == "!B705") Serial.println("********** Stop Action LEFT");
  else if (rxString == "!B804") Serial.println("********** Stop Action RIGHT");  
  rxString = "";
}

void shuffleMode1() {
  //check if desired amount of time has passed
  if (millis()-previousTime > runTime) {
    state = random(0,3); //4 different options. Full left, full right, stop. 
    previousTime = millis();
    runTime = random(0,10000);
    Serial.print("New runtime is: ");
    Serial.println(runTime);
    Serial.print("New state is: ");
    Serial.println(state);
  }

  if (state == 0) {
    speedA = 0;
    speedB = speed;
  }
  else if (state == 1) {
    speedA = speed;
    speedB = 0;
  }
  else if (state == 2) {
    speedA = 0;
    speedB = 0;
  }
}
 
 void IRcontrol() {
  /*
     * Check if received data is available and if yes, try to decode it.
     * Decoded result is in the IrReceiver.decodedIRData structure.
     *
     * E.g. command is in IrReceiver.decodedIRData.command
     * address is in command is in IrReceiver.decodedIRData.address
     * and up to 32 bit raw data in IrReceiver.decodedIRData.decodedRawData
     */
    if (IrReceiver.decode()) {

        /*
         * Print a summary of received data
         */
        if (IrReceiver.decodedIRData.protocol == UNKNOWN) {
            Serial.println(F("Received noise or an unknown (or not yet enabled) protocol"));
            // We have an unknown protocol here, print extended info
            IrReceiver.printIRResultRawFormatted(&Serial, true);
            IrReceiver.resume(); // Do it here, to preserve raw data for printing with printIRResultRawFormatted()
        } else {
            IrReceiver.resume(); // Early enable receiving of the next IR frame
            IrReceiver.printIRResultShort(&Serial);
            IrReceiver.printIRSendUsage(&Serial);
        }
        Serial.println();

        /*
         * Check recieved IR data from remote selected above (currently LG) or cheapo remote receiver comes with.
         */
        if (IrReceiver.decodedIRData.command == 0x8 || IrReceiver.decodedIRData.command == 0x7) {
            shuffleEnable = false;
            speedA = speed;
            speedB = 0;
            Serial.print(speed);
            Serial.println(" left");
        } else if (IrReceiver.decodedIRData.command == 0x5A || IrReceiver.decodedIRData.command == 0x6) {
            shuffleEnable = false;
            speedA = 0;
            speedB = speed;
            Serial.print(speed);
            Serial.println(" right");
        } else if (IrReceiver.decodedIRData.command == 0x18 || IrReceiver.decodedIRData.command == 0x40) {
            speed += 25; //set max speed below
            if (speed > 255) {
              speed = 255;
            }
            Serial.print("speed increased to ");
            Serial.println(speed);
        } else if (IrReceiver.decodedIRData.command == 0x52 || IrReceiver.decodedIRData.command == 0x41) {
            speed -= 25; //set min speed below
            if (speed < 100) {
              speed = 100;
            }
            Serial.print("speed decreased to ");
            Serial.println(speed);
            //button 0 turn on shuffle mode
        } else if (IrReceiver.decodedIRData.command == 0x10 || IrReceiver.decodedIRData.command == 0x19) {
            shuffleEnable = true;
            Serial.println("Shuffle mode enabled via IR remote");
        } 
        
        else {
            speedA = 0;
            speedB = 0;
            shuffleEnable = false;
        }
    }
    delay(50);
}
