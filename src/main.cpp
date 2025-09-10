#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEAdvertisedDevice.h>
#include <BLEScan.h>

#define SERVICE_UUID        "c9239c9e-6fc9-4168-b3aa-53105eb990b0"
#define CHARACTERISTIC_UUID "458d4dc9-349f-401d-b092-a2b1c55f5319"

#define DELAY 2000
#define BLINK_SPEED 500
#define COUNTDOWN_DURATION 10000  // 10 seconds in milliseconds

const int ledPin = 20; // GPIO20 (U0RXD) 
const int shutterPin = 21; // GPIO21 (U0TXD) - Pin for trigger

int incoming;
unsigned long now;
unsigned long timestampButton;
unsigned long timestampButton2;
unsigned long countdownStartTime;
bool countdownActive = false;

BLEServer *pServer = nullptr;
BLEScan *pBLEScan = nullptr;
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
        Serial.println("*** BLE CLIENT CONNECTION ATTEMPT ***");
        Serial.flush();
        deviceConnected = true;
        Serial.println("*** BLE CLIENT CONNECTED SUCCESSFULLY ***");
        Serial.flush();
    };

    void onDisconnect(BLEServer *pServer) override {
        Serial.println("*** BLE CLIENT DISCONNECTING ***");
        Serial.flush();
        deviceConnected = false;
        Serial.println("*** BLE CLIENT DISCONNECTED ***");
        Serial.flush();
    };

};

class MyCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) override {
        Serial.println("*** BLE CHARACTERISTIC WRITE RECEIVED ***");
        Serial.flush();
        
        now = millis(); // Store current time
        incoming = pCharacteristic->getData()[0];
        Serial.print("Incoming = ");
        Serial.println(incoming);
        Serial.flush();

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

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        Serial.print("BLE Device found: ");
        Serial.print(advertisedDevice.getName().c_str());
        Serial.print(" (");
        Serial.print(advertisedDevice.getAddress().toString().c_str());
        Serial.print(") RSSI: ");
        Serial.print(advertisedDevice.getRSSI());
        Serial.println(" dBm");
    }
};

void performBLEScan() {
    Serial.println("=== STARTING BLE SCAN TO VERIFY RADIO WORKS ===");
    
    // Initialize BLE first (required before scanning)
    BLEDevice::init("OpenRZ67-Scanner");
    Serial.println("BLE initialized for scanning...");
    
    pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setActiveScan(true);
    pBLEScan->setInterval(100);
    pBLEScan->setWindow(99);
    
    Serial.println("Scanning for BLE devices for 10 seconds...");
    BLEScanResults foundDevices = pBLEScan->start(10, false);
    
    Serial.print("Scan complete. Found ");
    Serial.print(foundDevices.getCount());
    Serial.println(" devices.");
    
    pBLEScan->clearResults();
    
    // Deinitialize BLE after scanning so we can reinitialize for server mode
    BLEDevice::deinit(true);
    Serial.println("=== BLE SCAN COMPLETE - REINITIALIZING FOR SERVER MODE ===");
}

void setupBLE() {
    Serial.println("Initializing BLE...");
    Serial.flush();
    
    // Add small delay before BLE init to ensure system is stable
    delay(500);
    
    BLEDevice::init("OpenRZ67");
    Serial.println("BLE device initialized with name: OpenRZ67");
    Serial.flush();
    
    // Small delay after init to let BLE stack stabilize
    delay(200);
    
    Serial.println("Creating BLE server...");
    Serial.flush();
    pServer = BLEDevice::createServer();
    Serial.println("BLE server object created");
    Serial.flush();
    
    // Delay after server creation
    delay(100);
    
    pServer->setCallbacks(new MyServerCallbacks());
    Serial.println("BLE server callbacks set");
    Serial.flush();

    BLEService *pService = pServer->createService(SERVICE_UUID);
    Serial.print("BLE service created with UUID: ");
    Serial.println(SERVICE_UUID);
    
    BLECharacteristic *pCharacteristic = pService->createCharacteristic(
            CHARACTERISTIC_UUID,
            BLECharacteristic::PROPERTY_READ |
            BLECharacteristic::PROPERTY_WRITE_NR
    );
    pCharacteristic->setCallbacks(new MyCallbacks());
    pCharacteristic->setValue("Hello from OpenRZ67!");
    Serial.print("BLE characteristic created with UUID: ");
    Serial.println(CHARACTERISTIC_UUID);
    
    pService->start();
    Serial.println("BLE service started");

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
    
    Serial.println("BLE advertising configured with enhanced discoverability");
    
    // Set optimal TX power for both advertising and connection stability
    Serial.println("Setting optimal TX power levels...");
    
    // Set advertising power (for discovery)
    esp_err_t adv_power = esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_P3);  // +3dBm
    Serial.print("Set advertising power to +3dBm: ");
    Serial.println(adv_power == ESP_OK ? "SUCCESS" : "FAILED");
    
    // Set connection power (for stable data transfer) 
    esp_err_t conn_power = esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL0, ESP_PWR_LVL_P3);  // +3dBm
    Serial.print("Set connection power to +3dBm: ");
    Serial.println(conn_power == ESP_OK ? "SUCCESS" : "FAILED");
    
    // Set default power for other operations
    esp_err_t default_power = esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_P3);  // +3dBm
    Serial.print("Set default power to +3dBm: ");
    Serial.println(default_power == ESP_OK ? "SUCCESS" : "FAILED");
    
    if (adv_power != ESP_OK || conn_power != ESP_OK) {
        Serial.println("High power failed, trying moderate levels...");
        
        // Fallback to 0dBm if +3dBm fails
        adv_power = esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_N0);  // 0dBm
        conn_power = esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_CONN_HDL0, ESP_PWR_LVL_N0);  // 0dBm
        Serial.print("Set advertising/connection power to 0dBm: ");
        Serial.println((adv_power == ESP_OK && conn_power == ESP_OK) ? "SUCCESS" : "FAILED");
    }
    
    // Start advertising 
    Serial.println("Starting BLE advertising...");
    pAdvertising->start();
    Serial.println("BLE advertising start command sent");
    
    Serial.println("*** BLE ADVERTISING STARTED - Device should now be visible as 'OpenRZ67' ***");
}

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
    // Initialize BLE FIRST, before any Serial/USB CDC activity
    setupBLE();
    
    // Then initialize Serial after BLE is running
    Serial.begin(115200);
    Serial.setTxTimeoutMs(0);  // Don't block when no serial monitor connected
    delay(2000);

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
    pinMode(shutterPin, OUTPUT);
    Serial.print("LED pin configured: GPIO");
    Serial.println(ledPin);
    Serial.print("Shutter pin configured: GPIO");
    Serial.println(shutterPin);

    // Skip BLE scan for now - double init may be causing crash
    // performBLEScan();
    
    // BLE already initialized at start of setup()
    // setupBLE();

    Serial.println("=== SETUP COMPLETE - SYSTEM READY ===");
    Serial.println("Hello from Open RZ67 Trigger!");
}


void loop() {
    static unsigned long lastHeartbeat = 0;
    unsigned long currentTime = millis();
    
    // Heartbeat every 30 seconds to show system is alive
    if (currentTime - lastHeartbeat > 30000) {
        Serial.print("System heartbeat - Uptime: ");
        Serial.print(currentTime / 1000);
        Serial.print("s, Free heap: ");
        Serial.print(ESP.getFreeHeap());
        Serial.print(" bytes, BLE connected: ");
        Serial.println(deviceConnected ? "YES" : "NO");
        lastHeartbeat = currentTime;
    }

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
