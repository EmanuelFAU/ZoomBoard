/* 
 * Sample program for ESP32 acting as a Bluetooth keyboard
 * 
 * Copyright (c) 2019 Manuel Bl
 * 
 * Licensed under MIT License
 * https://opensource.org/licenses/MIT
 */

//
// This program lets an ESP32 act as a keyboard connected via Bluetooth.
// When a button attached to the ESP32 is pressed, it will generate the key strokes for a message.
//
// For the setup, a momentary button should be connected to pin 2 and to ground.
// Pin 2 will be configured as an input with pull-up.
//
// In order to receive the message, add the ESP32 as a Bluetooth keyboard of your computer
// or mobile phone:
//
// 1. Go to your computers/phones settings
// 2. Ensure Bluetooth is turned on
// 3. Scan for Bluetooth devices
// 4. Connect to the device called "ESP32 Keyboard"
// 5. Open an empty document in a text editor
// 6. Press the button attached to the ESP32

// not sure if this applies:
//key 56    ALT_LEFT
//key 42    SHIFT_LEFT
//key 215   AT
//key 57    SPACE
//key 53    SLASH
//key 127   SYM
//key 100   ALT_LEFT

#define US_KEYBOARD 1

#include <Arduino.h>
#include "BLEDevice.h"
#include "BLEHIDDevice.h"
#include "HIDTypes.h"
#include "HIDKeyboardTypes.h"

#include <FastLED.h>
#include "FastLED_RGBW.h"


// LED Stripe
#define DATA_PIN 25
#define NUM_LEDS 4


// Change the below values if desired
#define BUTTON_PIN_A 13
#define BUTTON_PIN_V 14
#define BUTTON_PIN_S 27
#define BUTTON_PIN_H 26
#define MESSAGE "Hello from ZoomBoard\n"
#define DEVICE_NAME "ZoomBoard"

// FastLED with RGBW
CRGBW leds[NUM_LEDS];
CRGB *ledsRGB = (CRGB *) &leds[0];


// Forward declarations
void bluetoothTask(void*);
void typeText(const char* text);
void toggle_audio();
void toggle_video();
void toggle_screenshare();
void toggle_hand();
void wait_release();


bool isBleConnected = false;

bool audio = false;
bool video = false;
bool screenshare = false;
bool hand = false;


void setup() {
    Serial.begin(115200);

    // configure pin for button
    pinMode(BUTTON_PIN_A, INPUT_PULLUP);
    pinMode(BUTTON_PIN_V, INPUT_PULLUP);
    pinMode(BUTTON_PIN_S, INPUT_PULLUP);
    pinMode(BUTTON_PIN_H, INPUT_PULLUP);

  int brightness = 10; // halfe of maximum
  FastLED.addLeds<WS2812B, DATA_PIN, RGB>(ledsRGB, getRGBWsize(NUM_LEDS));
  FastLED.setBrightness(  brightness );

  leds[0] = CRGB::Red; // Audio
  leds[1] = CRGB::Red; // Video
  leds[2] = CRGB::Red; // Screenshare
  leds[3] = CRGB::Red; // Hand
  FastLED.show();

    // start Bluetooth task
    xTaskCreate(bluetoothTask, "bluetooth", 20000, NULL, 5, NULL);
}


void loop() {  
    if (isBleConnected && digitalRead(BUTTON_PIN_A) == LOW) {
        // button has been pressed: type message
        /*//const char text_msg[2] = {56, 47};
        Serial.println(MESSAGE);
        typeText(MESSAGE);*/
        toggle_audio();
        wait_release();
    } else if (isBleConnected && digitalRead(BUTTON_PIN_V) == LOW) {
        toggle_video();
        wait_release();
    } else if (isBleConnected && digitalRead(BUTTON_PIN_S) == LOW) {
        toggle_screenshare();
        wait_release();
    } else if (isBleConnected && digitalRead(BUTTON_PIN_H) == LOW) {
        toggle_hand();
        wait_release();
    }

    delay(100);
}


// Message (report) sent when a key is pressed or released
struct InputReport {
    uint8_t modifiers;	     // bitmask: CTRL = 1, SHIFT = 2, ALT = 4
    uint8_t reserved;        // must be 0
    uint8_t pressedKeys[6];  // up to six concurrenlty pressed keys
};

// Message (report) received when an LED's state changed
struct OutputReport {
    uint8_t leds;            // bitmask: num lock = 1, caps lock = 2, scroll lock = 4, compose = 8, kana = 16
};


// The report map describes the HID device (a keyboard in this case) and
// the messages (reports in HID terms) sent and received.
static const uint8_t REPORT_MAP[] = {
    USAGE_PAGE(1),      0x01,       // Generic Desktop Controls
    USAGE(1),           0x06,       // Keyboard
    COLLECTION(1),      0x01,       // Application
    REPORT_ID(1),       0x01,       //   Report ID (1)
    USAGE_PAGE(1),      0x07,       //   Keyboard/Keypad
    USAGE_MINIMUM(1),   0xE0,       //   Keyboard Left Control
    USAGE_MAXIMUM(1),   0xE7,       //   Keyboard Right Control
    LOGICAL_MINIMUM(1), 0x00,       //   Each bit is either 0 or 1
    LOGICAL_MAXIMUM(1), 0x01,
    REPORT_COUNT(1),    0x08,       //   8 bits for the modifier keys
    REPORT_SIZE(1),     0x01,       
    HIDINPUT(1),        0x02,       //   Data, Var, Abs
    REPORT_COUNT(1),    0x01,       //   1 byte (unused)
    REPORT_SIZE(1),     0x08,
    HIDINPUT(1),        0x01,       //   Const, Array, Abs
    REPORT_COUNT(1),    0x06,       //   6 bytes (for up to 6 concurrently pressed keys)
    REPORT_SIZE(1),     0x08,
    LOGICAL_MINIMUM(1), 0x00,
    LOGICAL_MAXIMUM(1), 0x65,       //   101 keys
    USAGE_MINIMUM(1),   0x00,
    USAGE_MAXIMUM(1),   0x65,
    HIDINPUT(1),        0x00,       //   Data, Array, Abs
    REPORT_COUNT(1),    0x05,       //   5 bits (Num lock, Caps lock, Scroll lock, Compose, Kana)
    REPORT_SIZE(1),     0x01,
    USAGE_PAGE(1),      0x08,       //   LEDs
    USAGE_MINIMUM(1),   0x01,       //   Num Lock
    USAGE_MAXIMUM(1),   0x05,       //   Kana
    LOGICAL_MINIMUM(1), 0x00,
    LOGICAL_MAXIMUM(1), 0x01,
    HIDOUTPUT(1),       0x02,       //   Data, Var, Abs
    REPORT_COUNT(1),    0x01,       //   3 bits (Padding)
    REPORT_SIZE(1),     0x03,
    HIDOUTPUT(1),       0x01,       //   Const, Array, Abs
    END_COLLECTION(0)               // End application collection
};


BLEHIDDevice* hid;
BLECharacteristic* input;
BLECharacteristic* output;

const InputReport NO_KEY_PRESSED = { };


/*
 * Callbacks related to BLE connection
 */
class BleKeyboardCallbacks : public BLEServerCallbacks {

    void onConnect(BLEServer* server) {
        isBleConnected = true;

        // Allow notifications for characteristics
        BLE2902* cccDesc = (BLE2902*)input->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
        cccDesc->setNotifications(true);

        Serial.println("Client has connected");
    }

    void onDisconnect(BLEServer* server) {
        isBleConnected = false;

        // Disallow notifications for characteristics
        BLE2902* cccDesc = (BLE2902*)input->getDescriptorByUUID(BLEUUID((uint16_t)0x2902));
        cccDesc->setNotifications(false);

        Serial.println("Client has disconnected");
    }
};


/*
 * Called when the client (computer, smart phone) wants to turn on or off
 * the LEDs in the keyboard.
 * 
 * bit 0 - NUM LOCK
 * bit 1 - CAPS LOCK
 * bit 2 - SCROLL LOCK
 */
 class OutputCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* characteristic) {
        OutputReport* report = (OutputReport*) characteristic->getData();
        Serial.print("LED state: ");
        Serial.print((int) report->leds);
        Serial.println();
    }
};


void bluetoothTask(void*) {

    // initialize the device
    BLEDevice::init(DEVICE_NAME);
    BLEServer* server = BLEDevice::createServer();
    server->setCallbacks(new BleKeyboardCallbacks());

    // create an HID device
    hid = new BLEHIDDevice(server);
    input = hid->inputReport(1); // report ID
    output = hid->outputReport(1); // report ID
    output->setCallbacks(new OutputCallbacks());

    // set manufacturer name
    hid->manufacturer()->setValue("Emanuel Sizmann");
    // set USB vendor and product ID
    hid->pnp(0x02, 0xe502, 0xa111, 0x0210);
    // information about HID device: device is not localized, device can be connected
    hid->hidInfo(0x00, 0x02);

    // Security: device requires bonding
    BLESecurity* security = new BLESecurity();
    security->setAuthenticationMode(ESP_LE_AUTH_BOND);

    // set report map
    hid->reportMap((uint8_t*)REPORT_MAP, sizeof(REPORT_MAP));
    hid->startServices();

    // set battery level to 100%
    hid->setBatteryLevel(100);

    // advertise the services
    BLEAdvertising* advertising = server->getAdvertising();
    advertising->setAppearance(HID_KEYBOARD);
    advertising->addServiceUUID(hid->hidService()->getUUID());
    advertising->addServiceUUID(hid->deviceInfo()->getUUID());
    advertising->addServiceUUID(hid->batteryService()->getUUID());
    advertising->start();

    Serial.println("BLE ready");
    delay(portMAX_DELAY);
};

void wait_release(){
  while(digitalRead(BUTTON_PIN_A) == LOW || digitalRead(BUTTON_PIN_V) == LOW || digitalRead(BUTTON_PIN_S) == LOW || digitalRead(BUTTON_PIN_H) == LOW){
    delay(5);
  }
}

void toggle_audio(){
  KEYMAP map = {0x04, 0x04};

  // create input report
        InputReport report = {
            .modifiers = map.modifier,
            .reserved = 0,
            .pressedKeys = {
                map.usage,
                0, 0, 0, 0, 0
            }
        };

        // send the input report
        input->setValue((uint8_t*)&report, sizeof(report));
        input->notify();

        delay(5);

        // release all keys between two characters; otherwise two identical
        // consecutive characters are treated as just one key press
        input->setValue((uint8_t*)&NO_KEY_PRESSED, sizeof(NO_KEY_PRESSED));
        input->notify();

        delay(5);

        Serial.println("toggeling Audio");

        if(audio == false){
          audio = true;
          // Green
          leds[0] = CRGB::Green; // Audio
          //leds[1] = CRGB::Red; // Video
          //leds[2] = CRGB::Red; // Screenshare
          //leds[3] = CRGB::Red; // Hand
          FastLED.show();

        } else {
          audio = false;
          // Red
          leds[0] = CRGB::Red; // Audio
          //leds[1] = CRGB::Red; // Video
          //leds[2] = CRGB::Red; // Screenshare
          //leds[3] = CRGB::Red; // Hand
          FastLED.show();

        }
}

void toggle_video(){
  KEYMAP map = {0x19, 0x04};

  // create input report
        InputReport report = {
            .modifiers = map.modifier,
            .reserved = 0,
            .pressedKeys = {
                map.usage,
                0, 0, 0, 0, 0
            }
        };

        // send the input report
        input->setValue((uint8_t*)&report, sizeof(report));
        input->notify();

        delay(5);

        // release all keys between two characters; otherwise two identical
        // consecutive characters are treated as just one key press
        input->setValue((uint8_t*)&NO_KEY_PRESSED, sizeof(NO_KEY_PRESSED));
        input->notify();

        delay(5);

        Serial.println("toggeling Video");

        if(video == false){
          video = true;
          // Green
          //leds[0] = CRGB::Green; // Audio
          leds[1] = CRGB::Green; // Video
          //leds[2] = CRGB::Green; // Screenshare
          //leds[3] = CRGB::Green; // Hand
          FastLED.show();

        } else {
          video = false;
          // Red
          //leds[0] = CRGB::Red; // Audio
          leds[1] = CRGB::Red; // Video
          //leds[2] = CRGB::Red; // Screenshare
          //leds[3] = CRGB::Red; // Hand
          FastLED.show();

        }
}

void toggle_screenshare(){
  KEYMAP map = {0x16, 0x04};

  // create input report
        InputReport report = {
            .modifiers = map.modifier,
            .reserved = 0,
            .pressedKeys = {
                map.usage,
                0, 0, 0, 0, 0
            }
        };

        // send the input report
        input->setValue((uint8_t*)&report, sizeof(report));
        input->notify();

        delay(5);

        // release all keys between two characters; otherwise two identical
        // consecutive characters are treated as just one key press
        input->setValue((uint8_t*)&NO_KEY_PRESSED, sizeof(NO_KEY_PRESSED));
        input->notify();

        delay(5);

        Serial.println("toggeling Screenshare");

        if(screenshare == false){
          screenshare = true;
          // Green
          //leds[0] = CRGB::Green; // Audio
          //leds[1] = CRGB::Green; // Video
          leds[2] = CRGB::Green; // Screenshare
          //leds[3] = CRGB::Green; // Hand
          FastLED.show();

        } else {
          screenshare = false;
          // Red
          //leds[0] = CRGB::Red; // Audio
          //leds[1] = CRGB::Red; // Video
          leds[2] = CRGB::Red; // Screenshare
          //leds[3] = CRGB::Red; // Hand
          FastLED.show();

        }
}

void toggle_hand(){
  KEYMAP map = {0x1c, 0x04};

  // create input report
        InputReport report = {
            .modifiers = map.modifier,
            .reserved = 0,
            .pressedKeys = {
                map.usage,
                0, 0, 0, 0, 0
            }
        };

        // send the input report
        input->setValue((uint8_t*)&report, sizeof(report));
        input->notify();

        delay(5);

        // release all keys between two characters; otherwise two identical
        // consecutive characters are treated as just one key press
        input->setValue((uint8_t*)&NO_KEY_PRESSED, sizeof(NO_KEY_PRESSED));
        input->notify();

        delay(5);

        Serial.println("toggeling Hand");

        if(screenshare == false){
          screenshare = true;
          // Green
          //leds[0] = CRGB::Green; // Audio
          //leds[1] = CRGB::Green; // Video
          //leds[2] = CRGB::Green; // Screenshare
          leds[3] = CRGB::Green; // Hand
          FastLED.show();

        } else {
          screenshare = false;
          // Red
          //leds[0] = CRGB::Red; // Audio
          //leds[1] = CRGB::Red; // Video
          //leds[2] = CRGB::Red; // Screenshare
          leds[3] = CRGB::Red; // Hand
          FastLED.show();

        }
}

void typeText(const char* text) {
    int len = strlen(text);
    for (int i = 0; i < len; i++) {

        // translate character to key combination
        uint8_t val = (uint8_t)text[i];
        if (val > KEYMAP_SIZE)
            continue; // character not available on keyboard - skip
        KEYMAP map = keymap[val];
        

        // create input report
        InputReport report = {
            .modifiers = map.modifier,
            .reserved = 0,
            .pressedKeys = {
                map.usage,
                0, 0, 0, 0, 0
            }
        };

        // send the input report
        input->setValue((uint8_t*)&report, sizeof(report));
        input->notify();

        delay(5);

        // release all keys between two characters; otherwise two identical
        // consecutive characters are treated as just one key press
        input->setValue((uint8_t*)&NO_KEY_PRESSED, sizeof(NO_KEY_PRESSED));
        input->notify();

        delay(5);
    }
}