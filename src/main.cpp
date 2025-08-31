#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>

#define SERVICE_UUID        "c9239c9e-6fc9-4168-b3aa-53105eb990b0"
#define CHARACTERISTIC_UUID "458d4dc9-349f-401d-b092-a2b1c55f5319"

#define DELAY 2000
#define BLINK_SPEED 500
#define COUNTDOWN_DURATION 10000  // 10 seconds in milliseconds

const int ledPin = D7; 
const int shutterPin = D6; // Pin for trigger

int incoming;
unsigned long now;
unsigned long timestampButton;
unsigned long timestampButton2;
unsigned long countdownStartTime;
bool countdownActive = false;

BLEServer *pServer = nullptr;
bool deviceConnected = false;
bool oldDeviceConnected = false;

void openShutter() {
    digitalWrite(shutterPin, HIGH);
}

void closeShutter() {
    digitalWrite(shutterPin, LOW);
}

void triggerShutter() {
    Serial.println("Trigger shutter");
    openShutter();
    delay(100);
    closeShutter();
}

class MyServerCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer *pServer) override {
        deviceConnected = true;
    };

    void onDisconnect(BLEServer *pServer) override {
        deviceConnected = false;
    };

};

class MyCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) override {
        now = millis(); // Store current time
        incoming = pCharacteristic->getData()[0];
        Serial.print("Incoming = ");
        Serial.println(incoming);

        int button = floor(incoming / 10);
        int value = incoming % 10;

        switch (button) {
            case 1:
                Serial.print("Button 1, value = ");
                Serial.println(value);
                if (value == 1) {
                    digitalWrite(ledPin, HIGH);
                    timestampButton = now;
                } else {
                    digitalWrite(ledPin, LOW);
                }
                triggerShutter();
                break;
            case 2:
                Serial.print("Button 2, value = ");
                Serial.println(value);
                if (value == 1) {
                    digitalWrite(ledPin, HIGH);
                    timestampButton2 = now;
                } else {
                    digitalWrite(ledPin, LOW);
                }
                break;
            case 3:
                Serial.print("Button 3, value = ");
                Serial.println(value);
                if (value == 1) {
                    // Start Arduino countdown - ensure clean state first
                    countdownActive = false;
                    countdownStartTime = millis();
                    countdownActive = true;
                    digitalWrite(ledPin, HIGH);
                    Serial.print("Starting 10-second countdown on Arduino at time: ");
                    Serial.println(countdownStartTime);
                } else {
                    // Cancel Arduino countdown
                    countdownActive = false;
                    countdownStartTime = 0;
                    digitalWrite(ledPin, LOW);
                    Serial.print("Countdown cancelled. Reset timestamp to: ");
                    Serial.println(countdownStartTime);
                }
                break;
        }
    }
};

void checkToReconnect() //added
{
    // disconnected so advertise
    if (!deviceConnected && oldDeviceConnected) {
        delay(500); // give the bluetooth stack the chance to get things ready
        pServer->startAdvertising(); // restart advertising
        Serial.println("Disconnected: start advertising");
        oldDeviceConnected = deviceConnected;
    }
    // connected so reset boolean control
    if (deviceConnected && !oldDeviceConnected) {
        // do stuff here on connecting
        Serial.println("Reconnected");
        oldDeviceConnected = deviceConnected;
    }
}

void setup() {
    Serial.begin(19200);
    Serial.println("Hello from Open RZ67 Trigger!");

    BLEDevice::init("Open RZ67 Trigger");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new MyServerCallbacks());

    BLEService *pService = pServer->createService(SERVICE_UUID);
    BLECharacteristic *pCharacteristic = pService->createCharacteristic(
            CHARACTERISTIC_UUID,
            BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_WRITE_NR
    );
    pCharacteristic->setCallbacks(new MyCallbacks());

    pCharacteristic->setValue("Hello from Open RZ67 Trigger!");
    pService->start();

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  // functions that help with iPhone connections issue
    pAdvertising->setMinPreferred(0x12);
    BLEDevice::startAdvertising();

    pinMode(ledPin, OUTPUT);
    pinMode(shutterPin, OUTPUT);
}


void loop() {

    checkToReconnect();

    if (deviceConnected) {
        now = millis(); // Store current time

        if (incoming == 11 and digitalRead(ledPin) and now > timestampButton + DELAY) {
            // Shutter has fired, disable LED
            digitalWrite(ledPin, LOW);
            Serial.println("Button 1 timeout reached");
        } else if (incoming == 21 and now > timestampButton2 + BLINK_SPEED) {
            // Blink LED to indicate countdown
            digitalWrite(ledPin, !digitalRead(ledPin));
            timestampButton2 = now;
            Serial.println("Button 2: Blink LED");
        }
    }
    
    // Handle Arduino countdown - runs regardless of connection status
    if (countdownActive && countdownStartTime > 0) {
        unsigned long currentTime = millis();
        unsigned long elapsed = currentTime - countdownStartTime;
        Serial.print("Countdown running - active: ");
        Serial.print(countdownActive);
        Serial.print(", startTime: ");
        Serial.print(countdownStartTime);
        Serial.print(", elapsed: ");
        Serial.println(elapsed);
        
        if (elapsed >= COUNTDOWN_DURATION) {
            // Countdown complete - trigger shutter
            countdownActive = false;
            countdownStartTime = 0;
            triggerShutter();
            digitalWrite(ledPin, LOW);
            Serial.print("Open RZ67 Trigger countdown complete - shutter triggered. Elapsed: ");
            Serial.print(elapsed);
            Serial.print("ms at time: ");
            Serial.println(currentTime);
        } else {
            // Blink LED during countdown (faster blink than button 2)
            if (elapsed % 250 < 125) {  // 250ms cycle, on for first 125ms
                digitalWrite(ledPin, HIGH);
            } else {
                digitalWrite(ledPin, LOW);
            }
        }
    }
}
