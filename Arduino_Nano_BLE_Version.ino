#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>


// BLE Service and Characteristic UUIDs
BLEService accelService("19B10000-E8F2-537E-4F6C-D104768A1214");
BLECharacteristic accelCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLENotify | BLEWrite, 6);

void setup() {
  Serial.begin(9600);

  // Initialize the LSM9DS1 sensor
  if (!IMU.begin()) {
    Serial.println("Failed to initialize the LSM9DS1 sensor!");
    while (1)
      ;
  }

  // Initialize the BLE library
  if (!BLE.begin()) {
    Serial.println("Failed to initialize BLE!");
    while (1)
      ;
  }

  // Set the local name for the BLE device
  BLE.setLocalName("GUDER");
  BLE.setAdvertisedService(accelService);

  // Add the characteristic to the service
  accelService.addCharacteristic(accelCharacteristic);

  // Add the service to the BLE
  BLE.addService(accelService);

  // Start advertising
  BLE.advertise();

  Serial.println("BLE Accelerometer Sensor Initialized");
}

void loop() {
  float x, y, z;
  // Check if a central device is connected
  BLEDevice central = BLE.central();

  // If connected
  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());

    // Continuously read accelerometer values and send them over BLE
    while (central.connected()) {
      // Read accelerometer values
      if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(x, y, z);

        Serial.print(x);
        Serial.print('\t');
        Serial.print(y);
        Serial.print('\t');
        Serial.println(z);

        // Convert accelerometer values to bytes
        int16_t xInt = static_cast<int16_t>(x * 100);
        int16_t yInt = static_cast<int16_t>(y * 100);
        int16_t zInt = static_cast<int16_t>(z * 100);
        int16_t pInt = static_cast<int16_t>(z * 100);
    


        // Create a byte array to store the accelerometer values
        byte accelData[8];
        accelData[0] = (byte)(xInt & 0xFF);
        accelData[1] = (byte)((xInt >> 8) & 0xFF);
        accelData[2] = (byte)(yInt & 0xFF);
        accelData[3] = (byte)((yInt >> 8) & 0xFF);
        accelData[4] = (byte)(zInt & 0xFF);
        accelData[5] = (byte)((zInt >> 8) & 0xFF);
        accelData[6] = (byte)(pInt & 0xFF);
        accelData[7] = (byte)((pInt >> 8) & 0xFF);

        // Update the BLE characteristic value
        accelCharacteristic.writeValue(accelData, sizeof(accelData));

        delay(100);  // Delay between readings
      }
    }

    // When the central device disconnects
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
  }
}

