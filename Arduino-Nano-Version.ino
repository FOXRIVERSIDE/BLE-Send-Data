#include <bluefruit.h>
#include <Arduino_LSM9DS1.h>

// max concurrent connections supported by this example
#define MAX_PRPH_CONNECTION 2

// Use on-board button if available, else use A0 pin
#ifdef PIN_BUTTON1
uint8_t button = PIN_BUTTON1;
#else
uint8_t button = A0;
#endif

// CPB button active state is HIGH, all other is low
#ifdef ARDUINO_NRF52840_CIRCUITPLAY
#define BUTTON_ACTIVE HIGH
#else
#define BUTTON_ACTIVE LOW
#endif

uint8_t connection_count = 0;
uint8_t buttonState;
float x, y, z;
bool isConnected = false;

/* Nordic accel service
 * accel service : 19B10000-E8F2-537E-4F6C-D104768A1214
 * accel characteristic: 19B10001-E8F2-537E-4F6C-D104768A1214 
 * LBS Button : 00001524-1212-EFDE-1523-785FEABCD123
 * LBS LED    : 00001525-1212-EFDE-1523-785FEABCD123
 */

const uint8_t ACCEL_UUID_SERVICE[] = {
  0x14, 0x12, 0x8A, 0x76, 0x04, 0xD1, 0x6C, 0x4F,
  0x7E, 0x53, 0xF2, 0xE8, 0x00, 0x00, 0xB1, 0x19
};

// const uint8_t ACCEL_UUID_SERVICE[] =
// {
//   0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15,
//   0xDE, 0xEF, 0x12, 0x12, 0x23, 0x15, 0x00, 0x00
// };

const uint8_t ACCEL_UUID_CHARACTERISTIC[] = {
  0x14, 0x12, 0x8A, 0x76, 0x04, 0xD1, 0x6C, 0x4F,
  0x7E, 0x53, 0xF2, 0xE8, 0x01, 0x00, 0xB1, 0x19
};

const uint8_t ACCEL_UUID_CHR_LED[] = {
  0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15,
  0xDE, 0xEF, 0x12, 0x12, 0x25, 0x15, 0x00, 0x00
};

const uint8_t LBS_UUID_CHR_BUTTON[] = {
  0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15,
  0xDE, 0xEF, 0x12, 0x12, 0x24, 0x15, 0x00, 0x00
};

BLEService accelBLEService(ACCEL_UUID_SERVICE);
BLECharacteristic accelChar(ACCEL_UUID_CHARACTERISTIC);
BLECharacteristic accelLEDChar(ACCEL_UUID_CHR_LED);
BLECharacteristic accelButtonChar(LBS_UUID_CHR_BUTTON);

byte accelData[8];

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);  // led off

  Serial.begin(9600);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize the LSM9DS1 sensor!");
    while (1)
      ;
  }

  // Initialize Bluefruit with max concurrent connections as Peripheral = MAX_PRPH_CONNECTION, Central = 0
  Serial.println("Initialise the Bluefruit nRF52 module");
  Bluefruit.begin(MAX_PRPH_CONNECTION, 0);
  Bluefruit.setName("GUDER");
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // Setup the LED-Button service using
  Serial.println("Configuring the LED-Button Service");

  // Note: You must call .begin() on the BLEService before calling .begin() on
  // any characteristic(s) within that service definition.. Calling .begin() on
  // a BLECharacteristic will cause it to be added to the last BLEService that
  // was 'begin()'ed!
  accelBLEService.begin();

  // Configure Button characteristic
  // Properties = Read + Notify
  // Permission = Open to read, cannot write
  // Fixed Len  = 1 (button state)
  accelChar.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY | CHR_PROPS_WRITE);
  accelChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  accelChar.setFixedLen(8);
  accelChar.begin();
  accelChar.write(accelData, sizeof(accelData));

  // accelChar.setWriteCallback(accel_write_callback);

  // Configure the LED characteristic
  // Properties = Read + Write
  // Permission = Open to read, Open to write
  // Fixed Len  = 1 (LED state)
  accelLEDChar.setProperties(CHR_PROPS_READ | CHR_PROPS_WRITE);
  accelLEDChar.setPermission(SECMODE_OPEN, SECMODE_OPEN);
  accelLEDChar.setFixedLen(1);
  accelLEDChar.begin();
  accelLEDChar.write8(0x01);  // led = on when connected

  accelLEDChar.setWriteCallback(led_write_callback);

  // Configure Button characteristic
  // Properties = Read + Notify
  // Permission = Open to read, cannot write
  // Fixed Len  = 1 (button state)
  accelButtonChar.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
  accelButtonChar.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  accelButtonChar.setFixedLen(1);
  accelButtonChar.begin();
  accelButtonChar.write8(buttonState);

  // Setup the advertising packet(s)
  Serial.println("Setting up the advertising");
  startAdv();
}

void startAdv(void) {
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include HRM Service UUID
  Bluefruit.Advertising.addService(accelBLEService);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();

  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);  // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);    // number of seconds in fast mode
  Bluefruit.Advertising.start(0);              // 0 = Don't stop advertising after n seconds
}

void setLED(bool on) {
  // data = 1 -> LED = On
  // data = 0 -> LED = Off
  digitalWrite(LED_BUILTIN, on ? LED_STATE_ON : (1 - LED_STATE_ON));
}

void led_write_callback(uint16_t conn_hdl, BLECharacteristic* chr, uint8_t* data, uint16_t len) {
  (void)conn_hdl;
  (void)chr;
  (void)len;  // len should be 1

  // data = 1 -> LED = On
  // data = 0 -> LED = Off
  setLED(data[0]);
}

void readAccel() {
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x, y, z);
    int sensorValue = analogRead(A2);
    float voltage = sensorValue * (5.0 / 1023.0);



    Serial.println(voltage);
    

    // Serial.print("Sensor Value: ");
    // Serial.print(sensorValue);
    // Serial.print(", Voltage: ");
    //Serial.println(voltage);

    // Convert accelerometer values to bytes
    int16_t xInt = static_cast<int16_t>(x * 100);
    int16_t yInt = static_cast<int16_t>(y * 100);
    int16_t zInt = static_cast<int16_t>(z * 100);
    int16_t pInt = static_cast<int16_t>(voltage* 100);

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
    accelChar.notify(accelData, sizeof(accelData));

    delay(100);  // Delay between readings
  }
}

void accel_read_callback(uint16_t conn_hdl, BLECharacteristic* chr) {
  (void)conn_hdl;
  (void)chr;

  // data = 1 -> LED = On
  // data = 0 -> LED = Off
  readAccel();
}


void loop() {
    // poll button every 10 ms
  if (isConnected) {
    readAccel();
  }


  uint8_t newState = (BUTTON_ACTIVE == digitalRead(button));

  // only notify if button state changes
  if (newState != buttonState) {
    buttonState = newState;
    accelButtonChar.write8(buttonState);

    // notify all connected clients
    for (uint16_t conn_hdl = 0; conn_hdl < MAX_PRPH_CONNECTION; conn_hdl++) {
      if (Bluefruit.connected(conn_hdl) && accelButtonChar.notifyEnabled(conn_hdl)) {
        accelButtonChar.notify8(conn_hdl, buttonState);
      }
    }
  }
}

// callback invoked when central connects
void connect_callback(uint16_t conn_handle) {
  (void)conn_handle;

  isConnected = true;

  setLED(true);
  accelLEDChar.write8(0x01);

  connection_count++;
  Serial.print("Connection count: ");
  Serial.println(connection_count);

  // Keep advertising if not reaching max
  if (connection_count < MAX_PRPH_CONNECTION) {
    Serial.println("Keep advertising");
    Bluefruit.Advertising.start(0);
  }
}

/**
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE which can be found in ble_hci.h
 */
void disconnect_callback(uint16_t conn_handle, uint8_t reason) {
  (void)conn_handle;
  (void)reason;

  isConnected = false;

  setLED(false);
  accelLEDChar.write8(0x00);

  Serial.println();
  Serial.print("Disconnected, reason = 0x");
  Serial.println(reason, HEX);

  connection_count--;
}
