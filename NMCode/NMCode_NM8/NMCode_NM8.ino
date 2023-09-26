
/*
   NMCode by this.is.NOISE inc. 

   https://github.com/thisisnoiseinc/NMCode

   Built upon:
    
    "BLE_MIDI Example by neilbags 
    https://github.com/neilbags/arduino-esp32-BLE-MIDI
    
    Based on BLE_notify example by Evandro Copercini."


    -----

    NM8 variant by Bruce Menace
    brucemenace.com

    This software changes/adds some functionality
    to turn NMVSE into a "live tool" for Dirtywave M8.
    The top 8 buttons for track used as track mutes
    and both the knob and the pot sends MIDI CC Messages.

    There's a very barebones "deadzone" implementation
    for the fader so it can be used "comfortably" with
    the DJFX Filter. The goal was to easily stay on the
    value 80 for DJFX filter where neither of the filters
    are engaged, when the fader is roughly in the middle
    (where the deadzone is).
*/


#include <BLEDevice.h>

#include <BLEUtils.h>

#include <BLEServer.h>

#include <BLE2902.h>

#include "esp_bt_main.h"

#include "esp_bt_device.h"

#define SERVICE_UUID        "03b80e5a-ede8-4b33-a751-6ce34ec4c700"

#define CHARACTERISTIC_UUID "7772e5db-3868-4112-a1a9-f2669d106bf3"


int potPin = 36; // Slider
int rotPin = 39; // Rotary Knob
bool rotMoving = true;
bool rotMoving2 = true;
int midiCState = 0; // General current state
int midiCState2 = 0; // General current state 2
int led_Blue = 14; // BLE LED
int led_Green = 4; // CHANNEL LED
const int button = 12;
int potCState = 0; // Slider current state
int rotCState = 0; // Rotary Knob current state
int outputValue = 0;
int ButtonNote = 0;
int Channel_SelectON = 0;
int Channel_SelectOFF = 0;
int Channel_SelectCC = 0;
int Buttonselect[button] = {  // Buttons put in order of reference board.
 16,
 17,
 18,
 21,
 19,
 25,
 22,
 23,
 27,
 26,
 35,
 34
  };
int buttonCstate[button] = {0}; // Button current state
int buttonPState[button] = {0}; // Button previous state
int OffNote[button] = {0};
int debounceDelay = 5;
int lastDebounceTime[button] = {0};
int i = 0;
const int numReadings = 15;
int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average1 = 0; // average current state
int lastaverage1 = 0; // average previous state
int average2 = 0; // average current state 2
int lastaverage2 = 0; // average previous state 2

  const int midPoint = 2047; // Midpoint of the 12-bit range (0-4095)
  const int sliderDeadZone = 1024;  // Example value, adjust as needed
  unsigned long lastBlinkTime = 0;


BLECharacteristic *pCharacteristic;

bool deviceConnected = false;

uint8_t midiPacket[] = {

   0x80,  // header

   0x80,  // timestamp, not implemented 

   0x00,  // status

   0x3c,  // 0x3c == 60 == middle c

   0x00   // velocity

};

class MyServerCallbacks: public BLEServerCallbacks {

    void onConnect(BLEServer* pServer) {

      deviceConnected = true;

    };



    void onDisconnect(BLEServer* pServer) {

      deviceConnected = false;

    }

};

bool initBluetooth()
{
  if (!btStart()) {
    Serial.println("Failed to initialize controller");
    return false;
  }
 
  if (esp_bluedroid_init() != ESP_OK) {
    Serial.println("Failed to initialize bluedroid");
    return false;
  }
 
  if (esp_bluedroid_enable() != ESP_OK) {
    Serial.println("Failed to enable bluedroid");
    return false;
  }
 
}

void setup() {

  Serial.begin(115200);

  initBluetooth();
  const uint8_t* point = esp_bt_dev_get_address();
 
  char str[6];
 
  sprintf(str, "NMSVE %02X %02X %02X", (int)point[3], (int)point[4], (int)point[5]);
  Serial.print(str);

  BLEDevice::init(str);

    

  // Create the BLE Server

  BLEServer *pServer = BLEDevice::createServer();

  pServer->setCallbacks(new MyServerCallbacks());



  // Create the BLE Service

  BLEService *pService = pServer->createService(BLEUUID(SERVICE_UUID));



  // Create a BLE Characteristic

  pCharacteristic = pService->createCharacteristic(

    BLEUUID(CHARACTERISTIC_UUID),

    BLECharacteristic::PROPERTY_READ   |

    BLECharacteristic::PROPERTY_WRITE  |

    BLECharacteristic::PROPERTY_NOTIFY |

    BLECharacteristic::PROPERTY_WRITE_NR

  );



  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml

  // Create a BLE Descriptor

  pCharacteristic->addDescriptor(new BLE2902());



  // Start the service

  pService->start();



  // Start advertising

  BLEAdvertising *pAdvertising = pServer->getAdvertising();

  pAdvertising->addServiceUUID(pService->getUUID());

  pAdvertising->start();

  // Initialize buttons + LED's

   for (int i = 0; i < button; i++){
  pinMode (Buttonselect[i], INPUT);
  pinMode (led_Blue, OUTPUT);
  pinMode (led_Green, OUTPUT);
  }

  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }

  while (Channel_SelectON == 0) {
  
  digitalWrite(led_Green, HIGH);
  
    for (int i = 0; i < button; i++) {
      buttonCstate[i] = digitalRead(Buttonselect[i]);
      if (buttonCstate[i] == HIGH) {
        Channel_SelectON = (i + 144);
        Channel_SelectOFF = (i + 128);
        Channel_SelectCC = (i + 176);
      }
    }
      }
      
        digitalWrite(led_Green, LOW);

}

void loop(){

 // Ensure device is connected to BLE

  if (deviceConnected == false) { 
  digitalWrite(led_Blue, HIGH);
  delay(1000);
  digitalWrite(led_Blue, LOW);
  delay(1000);
  }

  // Enter Default Mode

  else {

    digitalWrite(led_Blue, HIGH);
    BUTTONS();
    ROTARY();
    SLIDER();
  }
}
 


bool buttonToggled[button] = {false}; // Initialize the toggle state for each button to false

void BUTTONS() {
  for (int i = 0; i < button; i++) {
    buttonCstate[i] = digitalRead(Buttonselect[i]);
    ButtonNote = (12 + i);

    if ((millis() - lastDebounceTime[i]) > debounceDelay) {
      if (buttonPState[i] != buttonCstate[i]) {
        lastDebounceTime[i] = millis();

        if (buttonCstate[i] == HIGH) {
          // Button is pressed
          if (i >= 8 && i <= 11) {
            // Buttons 9 to 12: Send two consecutive notes
            midiPacket[2] = Channel_SelectON;
            midiPacket[4] = 100;

            // a super hacky part that mutes 2 channels at once with a keypress. kinda broken now.
            if (i == 8) {
              // Button 9 (two consecutive notes)
              midiPacket[3] = 12;
              pCharacteristic->setValue(midiPacket, 5);
              pCharacteristic->notify();

              midiPacket[3] = 13;
              pCharacteristic->setValue(midiPacket, 5);
              pCharacteristic->notify();
            } else if (i == 9) {
              // Button 10 (two consecutive notes)
              midiPacket[3] = 14;
              pCharacteristic->setValue(midiPacket, 5);
              pCharacteristic->notify();

              midiPacket[3] = 15;
              pCharacteristic->setValue(midiPacket, 5);
              pCharacteristic->notify();              
            } else if (i == 10) {
              // Button 11 (two consecutive notes)
              midiPacket[3] = 16;
              pCharacteristic->setValue(midiPacket, 5);
              pCharacteristic->notify();

              midiPacket[3] = 17;
              pCharacteristic->setValue(midiPacket, 5);
              pCharacteristic->notify();              
            } else if (i == 11) {
              // Button 12 (two consecutive notes)
              midiPacket[3] = 18;
              pCharacteristic->setValue(midiPacket, 5);
              pCharacteristic->notify();

              midiPacket[3] = 19;
              pCharacteristic->setValue(midiPacket, 5);
              pCharacteristic->notify();              
            }
            pCharacteristic->setValue(midiPacket, 5);
            pCharacteristic->notify();
          } else {
            // Buttons 1 to 8: Toggle behavior
            if (buttonToggled[i]) {
              // If the note is toggled on, send a Note Off message
              midiPacket[2] = Channel_SelectOFF;
              midiPacket[3] = ButtonNote;
              midiPacket[4] = 0;
              buttonToggled[i] = false; // Toggle off
            } else {
              // If the note is toggled off, send a Note On message
              midiPacket[2] = Channel_SelectON;
              midiPacket[3] = ButtonNote;
              midiPacket[4] = 100;
              buttonToggled[i] = true; // Toggle on
            }
            pCharacteristic->setValue(midiPacket, 5);
            pCharacteristic->notify();
          }
        } else {
          // Button is released
          if (!(i >= 8 && i <= 11)) {
            // For buttons 1 to 8, do nothing when released (toggle behavior)
          } else {
            // For buttons 9 to 12, send Note Off when released (momentary behavior)
            midiPacket[2] = Channel_SelectOFF;
            midiPacket[3] = ButtonNote;
            midiPacket[4] = 0;
            pCharacteristic->setValue(midiPacket, 5);
            pCharacteristic->notify();
          }
        }
        buttonPState[i] = buttonCstate[i];
      }
    }
  }
}







void potaverage1() {
  
  for (int p = 0; p < 15; p++) {
  rotCState = analogRead(rotPin);
  midiCState = map(rotCState, 0, 4095, 0, 127);
  
  // subtract the last reading:
  total = total - readings[readIndex];
  // read from the sensor:
  readings[readIndex] = midiCState;
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  // calculate the average:
  average1 = total / numReadings;
  delay(1);        // delay in between reads for stability
  }
}

void potaverage2() {

  for (int p = 0; p < 15; p++) {
    potCState = analogRead(potPin);

    // Calculate the midpoint of the deadzone
    int deadzoneMidpoint = midPoint - (sliderDeadZone / 2);

    // Check if the slider is within the deadzone
    if (potCState >= (midPoint - (sliderDeadZone / 2)) && potCState <= (midPoint + (sliderDeadZone / 2))) {
      potCState = 64; // Inside the deadzone, set to 64
    } else if (potCState < midPoint) {
      // Map the left side of the slider position to the desired output range
      potCState = map(potCState, 0, midPoint, 0, 63);
    } else {
      // Map the right side of the slider position to the desired output range
      potCState = map(potCState, midPoint, 4095, 65, 127);
      potCState = constrain(potCState, 65, 127); // Ensure it's within 64-127
    }

    // subtract the last reading:
    total = total - readings[readIndex];
    // read from the sensor:
    readings[readIndex] = potCState;
    // add the reading to the total:
    total = total + readings[readIndex];
    // advance to the next position in the array:
    readIndex = readIndex + 1;

    // if we're at the end of the array...
    if (readIndex >= numReadings) {
      // ...wrap around to the beginning:
      readIndex = 0;
    }

    // calculate the average:
    average2 = total / numReadings;
    delay(1); // delay in between reads for stability
  }
}



void ROTARY(){

 potaverage1();
 
 if (average1 != lastaverage1) {
    rotMoving = true;
  }

  else {
    rotMoving = false;
  }

  if (rotMoving == true) {
    
   midiPacket[2] = Channel_SelectCC;
   Serial.println(Channel_SelectCC);

   midiPacket[3] = 10; // was 0x01
   Serial.println(10);

   midiPacket[4] = average1;
   Serial.println(average1);

   pCharacteristic->setValue(midiPacket, 5);

   pCharacteristic->notify();

   lastaverage1 = average1;
  }
}

// Control Slider function for Default Mode
void SLIDER() {
  potaverage2();

  if (average2 != lastaverage2) {
    rotMoving2 = true;
  } else {
    rotMoving2 = false;
  }


  // green led blinks if the slider is outside of deadzone
  // and stays lit if inside the deadzone

    // Check if the slider is outside of the deadzone
  if (average2 != 64) {
    // Slider is outside the deadzone, implement a blinking pattern for the green LED
    unsigned long currentMillis = millis();
    
    // Define the blink frequency
    const unsigned long blinkInterval = 30;
    
    // Toggle the green LED based on the blink interval
    if (currentMillis - lastBlinkTime >= blinkInterval) {
      lastBlinkTime = currentMillis;
      
      // Toggle the LED state
      static bool ledState = LOW;
      ledState = (ledState == LOW) ? HIGH : LOW;
      digitalWrite(led_Green, ledState);
    }
  } else {
    // Slider is inside the deadzone, the green LED stays lit
    digitalWrite(led_Green, LOW);
  }

  if (rotMoving2 == true) {
    
   midiPacket[2] = Channel_SelectCC;
   Serial.println(Channel_SelectCC);

   midiPacket[3] = 11;
   Serial.println(11);

   midiPacket[4] = average2;
   Serial.println(average2);

   pCharacteristic->setValue(midiPacket, 5);

   pCharacteristic->notify();

   lastaverage2 = average2;
  }
}


