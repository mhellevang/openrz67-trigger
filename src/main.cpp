#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <esp_pm.h>

#define SERVICE_UUID        "c9239c9e-6fc9-4168-b3aa-53105eb990b0"
#define CHARACTERISTIC_UUID "458d4dc9-349f-401d-b092-a2b1c55f5319"

#define DELAY 2000
#define BLINK_SPEED 500
#define COUNTDOWN_DURATION 10000  // 10 seconds in milliseconds

constexpr int ledPin = 20; // GPIO20 (U0RXD)
constexpr int shutterPinD1 = 3; // GPIO03
constexpr int shutterPinD6 = 21; // GPIO21 (U0TXD)

int incoming;
unsigned long now;
unsigned long timestampButton;
unsigned long countdownStartTime;
bool countdownActive = false;
bool bulbModeActive = false;
unsigned long lastCountdownPrint = 0;

BLEServer *pServer = nullptr;
bool deviceConnected = false;
bool oldDeviceConnected = false;

void cancelBulbMode();

void openShutter() {
    Serial.println("Setting shutterPins HIGH...");
    digitalWrite(shutterPinD1, HIGH);
    digitalWrite(shutterPinD6, HIGH);
}

void closeShutter() {
    Serial.println("Setting shutterPins LOW...");
    digitalWrite(shutterPinD1, LOW);
    digitalWrite(shutterPinD6, LOW);
}

void triggerShutter() {
    Serial.println("Trigger shutter");
    cancelBulbMode();
    openShutter();
    delay(100);
    closeShutter();
}

void startBulbMode() {
    Serial.println("Starting bulb mode - shutter opening");
    bulbModeActive = true;
    openShutter();
}

void endBulbMode() {
    Serial.println("Ending bulb mode - shutter closing");
    bulbModeActive = false;
    closeShutter();
}

void cancelBulbMode() {
    if (bulbModeActive) {
        Serial.println("Cancelling bulb mode due to mode switch");
        bulbModeActive = false;
    }
}

class BleServerCallback : public BLEServerCallbacks {
    void onConnect(BLEServer *pServer) override {
        Serial.println("*** BLE CLIENT CONNECTED ***");
        deviceConnected = true;
    };

    void onDisconnect(BLEServer *pServer) override {
        Serial.println("*** BLE CLIENT DISCONNECTED ***");
        deviceConnected = false;
        
        // Immediately restart advertising for new connections
        delay(100); // Minimal delay for cleanup
        pServer->startAdvertising();
        Serial.println("*** RESTARTED ADVERTISING ***");
    };

};

class BLECharacteristicCallback : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) override {
        now = millis();
        incoming = pCharacteristic->getData()[0];

        Serial.print("BLE command received: ");
        Serial.println(incoming);

        int button = floor(incoming / 10);
        int value = incoming % 10;

        switch (button) {
            case 1:
                Serial.print("Button 1, value = ");
                Serial.println(value);
                if (value == 1) {
                    // Button 1 PRESS - trigger shutter
                    digitalWrite(ledPin, HIGH);
                    timestampButton = now;
                    triggerShutter();
                } else {
                    // Button 1 RELEASE - just turn off LED, don't trigger again
                    digitalWrite(ledPin, LOW);
                }
                break;
            case 2:
                Serial.print("Button 2 (Bulb Mode), value = ");
                Serial.println(value);
                if (value == 1) {
                    // Start bulb mode
                    digitalWrite(ledPin, HIGH);
                    startBulbMode();
                } else {
                    // End bulb mode
                    digitalWrite(ledPin, LOW);
                    endBulbMode();
                }
                break;
            case 3:
                Serial.print("Button 3, value = ");
                Serial.println(value);
                if (value == 1) {
                    // Start Arduino countdown - ensure clean state first
                    cancelBulbMode();
                    countdownActive = false;
                    countdownStartTime = millis();
                    lastCountdownPrint = 0; // Reset print timer
                    countdownActive = true;
                    digitalWrite(ledPin, HIGH);
                    Serial.println("Starting 10-second countdown...");
                } else {
                    // Cancel Arduino countdown
                    countdownActive = false;
                    countdownStartTime = 0;
                    digitalWrite(ledPin, LOW);
                    Serial.println("Countdown cancelled");
                }
                break;
            default:
                Serial.println("Unknown button triggered");
        }
    }
};

void setupBLE() {
    Serial.println("Initializing BLE...");
    
    BLEDevice::init("OpenRZ67");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new BleServerCallback());

    BLEService *pService = pServer->createService(SERVICE_UUID);
    
    BLECharacteristic *pCharacteristic = pService->createCharacteristic(
            CHARACTERISTIC_UUID,
            BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_WRITE_NR
    );
    pCharacteristic->setCallbacks(new BLECharacteristicCallback());
    pCharacteristic->setValue("Hello from OpenRZ67!");
    
    pService->start();
    Serial.println("BLE service and characteristic configured");

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->setScanResponse(true);
    pAdvertising->setMinPreferred(0x06);  // 7.5ms
    pAdvertising->setMaxPreferred(0x12);  // 22.5ms
    
    // Additional advertising configuration for better discoverability
    BLEAdvertisementData adData;
    adData.setName("OpenRZ67");
    adData.setCompleteServices(BLEUUID(SERVICE_UUID));
    pAdvertising->setAdvertisementData(adData);
    
    BLEAdvertisementData scanResponseData;
    scanResponseData.setName("OpenRZ67");
    scanResponseData.setCompleteServices(BLEUUID(SERVICE_UUID));
    pAdvertising->setScanResponseData(scanResponseData);
    
    Serial.println("BLE advertising configured");
    
    // Set lower TX power for better battery life
    
    // Set advertising power (most important for battery life)
    esp_err_t adv_power = esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_N0);  // 0dBm
    Serial.print("Set advertising power to 0dBm: ");
    Serial.println(adv_power == ESP_OK ? "SUCCESS" : "FAILED");
    
    // Set default power for other operations
    esp_err_t default_power = esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_N0);  // 0dBm
    Serial.print("Set default power to 0dBm: ");
    Serial.println(default_power == ESP_OK ? "SUCCESS" : "FAILED");
    
    // Start advertising 
    pAdvertising->start();
    Serial.println("*** BLE ADVERTISING STARTED - Device visible as 'OpenRZ67' ***");
}

void checkToReconnect() //added
{
    if (deviceConnected && !oldDeviceConnected) {
        Serial.println("Reconnected");
        oldDeviceConnected = deviceConnected;
    }
    if (!deviceConnected && oldDeviceConnected) {
        Serial.println("Connection state updated: disconnected");
        oldDeviceConnected = deviceConnected;
    }
}

void configurePowerManagement() {
    esp_pm_config_esp32c3_t pm_config = {
        .max_freq_mhz = 160,
        .min_freq_mhz = 10,
        .light_sleep_enable = false
    };
    esp_pm_configure(&pm_config);
    
    Serial.println("Power management configured for frequency scaling");
}

void setup() {
    
#if ARDUINO_USB_CDC_ON_BOOT
    Serial.setTxTimeoutMs(0); // Don't block when no serial monitor connected
#endif
    Serial.begin(115200);
    delay(500);  // Brief delay for serial stability

    setupBLE();

    Serial.println("=== OPENRZ67 TRIGGER STARTING ===");
    Serial.print("ESP32 Chip Model: ");
    Serial.println(ESP.getChipModel());
    Serial.print("ESP32 Chip Revision: ");
    Serial.println(ESP.getChipRevision());
    Serial.print("Flash Size: ");
    Serial.print(ESP.getFlashChipSize() / (1024 * 1024));
    Serial.println(" MB");
    Serial.print("Free Heap: ");
    Serial.print(ESP.getFreeHeap());
    Serial.println(" bytes");

    Serial.println("Configuring GPIO pins...");
    pinMode(ledPin, OUTPUT);
    pinMode(shutterPinD6, OUTPUT);
    pinMode(shutterPinD1, OUTPUT);
    Serial.print("LED pin configured: GPIO");
    Serial.println(ledPin);
    Serial.print("Shutter pin configured: GPIO");
    Serial.println(shutterPinD1);
    Serial.print("Shutter pin configured: GPIO");
    Serial.println(shutterPinD6);
    
    // Configure power management for frequency scaling
    configurePowerManagement();

    Serial.println("=== SETUP COMPLETE - SYSTEM READY ===");
    Serial.println("Hello from Open RZ67 Trigger!");
}


void loop() {

    checkToReconnect();

    if (deviceConnected) {
        now = millis(); // Store current time

        if (incoming == 11 and digitalRead(ledPin) and now > timestampButton + DELAY) {
            // Shutter has fired, disable LED
            digitalWrite(ledPin, LOW);
            Serial.println("Button 1 timeout reached");
        }
    }
    
    // Handle Arduino countdown - runs regardless of connection status
    if (countdownActive && countdownStartTime > 0) {
        unsigned long currentTime = millis();
        unsigned long elapsed = currentTime - countdownStartTime;
        
        // Print countdown status only once per second to avoid spam
        if (currentTime - lastCountdownPrint >= 1000) {
            unsigned long secondsRemaining = (COUNTDOWN_DURATION - elapsed) / 1000;
            Serial.print("Countdown: ");
            Serial.print(secondsRemaining);
            Serial.println(" seconds remaining");
            lastCountdownPrint = currentTime;
        }
        
        if (elapsed >= COUNTDOWN_DURATION) {
            // Countdown complete - trigger shutter
            countdownActive = false;
            countdownStartTime = 0;
            triggerShutter();
            digitalWrite(ledPin, LOW);
            Serial.println("Countdown complete - shutter triggered!");
        } else {
            // Blink LED during countdown (faster blink than button 2)
            if (elapsed % 250 < 125) {  // 250ms cycle, on for first 125ms
                digitalWrite(ledPin, HIGH);
            } else {
                digitalWrite(ledPin, LOW);
            }
        }
    }
    
    // Handle bulb mode LED indication - steady light when active
    if (bulbModeActive) {
        digitalWrite(ledPin, HIGH);
    }
    
}
