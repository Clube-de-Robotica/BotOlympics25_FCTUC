#include "FCTUC.h"

/* Wireless Communications */

WiFiUDP udp;
#ifdef TELNET_ENABLED
ESPTelnetStream telnet;
#endif

/* Static Class Definitions */

TaskHandle_t FCTUC::batteryTaskHandle;
TaskHandle_t FCTUC::rfidTaskHandle;
TaskHandle_t FCTUC::wifiTaskHandle;
TaskHandle_t FCTUC::mainTaskHandle;

constexpr char FCTUC::WIFI_SSID[];
constexpr char FCTUC::WIFI_PWD[];
constexpr char FCTUC::LOCAL_IP[];
constexpr char FCTUC::GATEWAY_IP[];
constexpr char FCTUC::SUBNET_MASK[];
constexpr char FCTUC::OTA_PWD[];

MFRC522 FCTUC::deviceRFIDRight(PIN_RFID_SDA_R, MFRC522::UNUSED_PIN);
MFRC522 FCTUC::deviceRFIDLeft(PIN_RFID_SDA_L, MFRC522::UNUSED_PIN);

Adafruit_NeoPixel FCTUC::deviceNeoPixel(1, FCTUC::PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

VL53L0X FCTUC::deviceLidarRight;
VL53L0X FCTUC::deviceLidarFront;
VL53L0X FCTUC::deviceLidarLeft;

float FCTUC::motorCoefficientLeft = 1.0;
float FCTUC::motorCoefficientRight = 1.0;

/* Class Implementation */

void FCTUC::setupButton() {
    pinMode(PIN_BUTTON, INPUT);
}

void FCTUC::setupBuzzer() {
    pinMode(PIN_BUZZER, OUTPUT);
}

void FCTUC::setupNeopixel() {
    deviceNeoPixel.begin();
}

void FCTUC::setupMotors() {
    pinMode(PIN_MOTOR_L_1, OUTPUT);
    pinMode(PIN_MOTOR_L_2, OUTPUT);
    pinMode(PIN_MOTOR_R_1, OUTPUT);
    pinMode(PIN_MOTOR_R_2, OUTPUT);

    analogWriteResolution(PWM_RESOLUTION_BITS);
}

void FCTUC::setupLidar() {
    Wire.begin();

    pinMode(PIN_XSHUT_RIGHT, OUTPUT);
    pinMode(PIN_XSHUT_FRONT, OUTPUT);
    pinMode(PIN_XSHUT_LEFT, OUTPUT);

    digitalWrite(PIN_XSHUT_RIGHT, LOW);
    delay(200);
    digitalWrite(PIN_XSHUT_RIGHT, HIGH);
    delay(200);
    deviceLidarRight.setAddress(ADDR_LIDAR_RIGHT);
    deviceLidarRight.setTimeout(500);
    deviceLidarRight.init(true);

    digitalWrite(PIN_XSHUT_FRONT, LOW);
    delay(200);
    digitalWrite(PIN_XSHUT_FRONT, HIGH);
    delay(200);
    deviceLidarFront.setAddress(ADDR_LIDAR_FRONT);
    deviceLidarFront.setTimeout(500);
    deviceLidarFront.init(true);

    digitalWrite(PIN_XSHUT_LEFT, LOW);
    delay(200);
    digitalWrite(PIN_XSHUT_LEFT, HIGH);
    delay(200);
    deviceLidarLeft.setAddress(ADDR_LIDAR_LEFT);
    deviceLidarLeft.setTimeout(500);
    deviceLidarLeft.init(true);

    deviceLidarRight.startContinuous(0);
    deviceLidarFront.startContinuous(0);
    deviceLidarLeft.startContinuous(0);
}

void FCTUC::setupRFID() {
    SPI.begin(); 
    deviceRFIDRight.PCD_Init();
    deviceRFIDLeft.PCD_Init();
}

void FCTUC::setupWifi() {
    IPAddress local, gateway, subnet;
    local.fromString(LOCAL_IP);
    gateway.fromString(GATEWAY_IP);
    subnet.fromString(SUBNET_MASK);

    WiFi.begin(WIFI_SSID, WIFI_PWD);
    WiFi.config(local, gateway, subnet);

    int attempts = 10;
    while ((WiFi.status() != WL_CONNECTED) && (attempts-- > 0)) {
        delay(1000);
        Serial.println("[INFO] - Waiting for WiFi connection...");
    }

    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[WARN] - Failed to establish WiFi connection.");
    }

    Serial.print("[INFO] - FCTUC's IP is: ");
    Serial.println(WiFi.localIP());
}

/**
 * @brief Initialize the hardware interface. Must be called to interact with robot.
 */
void FCTUC::begin() {
    setupMotors();
    setupNeopixel();
    setupButton();
    setupBuzzer();
    setupLidar();
    setupRFID();
    setupWifi();

    // Self-test failed procedure
    if (doSelfTest() == false) {
        // while (true);
    }

    mainTaskHandle = xTaskGetCurrentTaskHandle();
    xTaskCreatePinnedToCore(taskReadActiveRFIDValue, "TASK_RFID", 2000, nullptr, 1, &rfidTaskHandle, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(taskMonitorBatteryValue, "TASK_BATT", 2000, nullptr, 1, &batteryTaskHandle, tskNO_AFFINITY);
    xTaskCreatePinnedToCore(taskMonitorWirelessComms, "TASK_COMM", 2000, nullptr, 1, &wifiTaskHandle, tskNO_AFFINITY);

    Serial.println("[INFO] - FCTUC battery voltage: " + String(getBatteryVoltage()) + "v");
}

/**
 * @brief Immobilizes the bot and stops the default loop() task.
 */
void FCTUC::terminate() {
    moveMotors(0, 0);
    vTaskSuspendAll();
    while (1) {};
}

void FCTUC::taskMonitorBatteryValue(void*) {
    while (true) {
        if (getBatteryVoltage() < MIN_BAT_VOLTAGE) {
            Serial.println("[ERROR] - FCTUC's battery has run out! - Request a new one from a mentor or technical team member!");
            deviceNeoPixel.setPixelColor(0, 100, 50, 0);
            deviceNeoPixel.show();
            terminate();
        }
        delay(3000);
    }
}

void FCTUC::taskReadActiveRFIDValue(void*) {
    // We can do this because the objects are preallocated
    static constexpr MFRC522* deviceRFIDSelect[] = {&deviceRFIDRight, &deviceRFIDLeft};
    static MFRC522::MIFARE_Key TAG_KEY = {0xAA, 0xAA, 0xAA, 0xAA, 0xAA, 0xAA}; // public key, only allows read operations
    uint8_t buffer_size = 18; // must be 18 as the 2 extra bytes are used internally by the MFRC522 library for the read operation

    bool& isTagDetected = getTagDetected();
    bool& isTagReadSuccess = getTagReadSuccess();
    uint8_t* buffer = getTagData();

    bool active = false;

    while (true) {
        MFRC522::StatusCode status = MFRC522::STATUS_ERROR;

        delay(TAG_ANTENNA_SWITCH_MS);

        // Simple and effective way to toggle active RFID
        active = !active;
        deviceRFIDSelect[active]->PCD_AntennaOn();
        deviceRFIDSelect[!active]->PCD_AntennaOff();
        
        // RFID detected a new card, perform read
        isTagDetected = deviceRFIDSelect[active]->PICC_IsNewCardPresent() && deviceRFIDSelect[active]->PICC_ReadCardSerial();
        isTagReadSuccess = false;

        if (!isTagDetected) {
            continue;
        }

        status = deviceRFIDSelect[active]->PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, TAG_BLOCK_POS, &TAG_KEY, &(deviceRFIDSelect[active]->uid)); //authenticate with our public key for reading

        if (status == MFRC522::STATUS_OK) {
            status = deviceRFIDSelect[active]->MIFARE_Read(TAG_BLOCK_POS, buffer, &buffer_size); //try to read block
        }

        if (status == MFRC522::STATUS_OK) {
            isTagReadSuccess = true;
        } else {
            Serial.print("[ERROR] - ");
            Serial.print(deviceRFIDSelect[active] == &deviceRFIDRight ? "Right" : "Left");
            Serial.print(" RFID Read Fail - PCD_Authenticate() failed: ");
            Serial.println(deviceRFIDSelect[active]->GetStatusCodeName(status));
        }

        deviceRFIDSelect[active]->PICC_HaltA();
        deviceRFIDSelect[active]->PCD_StopCrypto1();
    }
}

void FCTUC::taskMonitorWirelessComms(void*) {
#ifdef OTA_ENABLED
    ArduinoOTA.setPort(OTA_PORT);
    ArduinoOTA.setPassword(OTA_PWD);
    ArduinoOTA.onStart([]() {
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH)
            type = "sketch";
        else // U_SPIFFS
            type = "filesystem";

        Serial.println("Start updating " + type);
    }).onEnd([]() {
        Serial.println("\nEnd");
    }).onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    }).onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

    ArduinoOTA.begin();
#endif

#ifdef TELNET_ENABLED
    telnet.begin(TELNET_PORT);
#endif
    
    uint8_t* udp_buffer = getUDPData();
    udp.begin(UDP_PORT);

    while (true) {
                
#ifdef OTA_ENABLED
        ArduinoOTA.handle();
#endif

#ifdef TELNET_ENABLED
        telnet.loop();
#endif        
        if (udp.parsePacket()) {
            // Set first byte as data length, copy data after.
            udp_buffer[0] = udp.read(udp_buffer + 1, 255);
            // Null terminate the buffer, just in case someone has the bright idea to print it.
            udp_buffer[udp_buffer[0]] = 0;
        }

        udp.flush();

        delay(100);
    }
}

/**
 * @brief Get if a position tag has been detected. **Note: due to our double reader setup this may return true more than once in the same position**
 * @return True if yes, false if no
 */
bool& FCTUC::getTagDetected() {
    static bool isTagDetected = false;
    return isTagDetected;
}

/**
 * @brief Get the read status of last detected tag
 * @return True if successful, false if not successful
 */
bool& FCTUC::getTagReadSuccess() {
    static bool isTagReadSuccess = false;
    return isTagReadSuccess;
}

/**
 * @brief Get the data stored in the tag
 * @return uint8_t[32]
 */
uint8_t* FCTUC::getTagData() {
    static uint8_t buffer[32];
    return buffer;
}

/**
 * @brief Get the data from UDP message. First byte is always length of data.
 * @return uint8_t[256]
 */
uint8_t* FCTUC::getUDPData() {
    static uint8_t buffer[256];
    return buffer;
}

/**
 * @brief Test connection to peripherals
 */
bool FCTUC::doSelfTest() {
    static constexpr char HexToCharMap[] = "0123456789ABCDEF";
    //Look for I2C devices by scanning witin the expected address range
    static constexpr uint8_t EXPECTED_I2C_DEVICES = 3;

    byte I2CCount = 0;
    byte I2CAddr[EXPECTED_I2C_DEVICES] = {};

    for (byte i = 1; i < 120; i++) {
        Wire.beginTransmission(i);

        if (Wire.endTransmission() == 0) {
            I2CAddr[I2CCount] = i;
            I2CCount++;
        }
    }

    if(I2CCount < EXPECTED_I2C_DEVICES){
        Serial.println("Error during self-test: Only found " + String(I2CCount) +  " I2C devices, expected " + String(EXPECTED_I2C_DEVICES) + " ! :");

        for(uint8_t i = 0; i < I2CCount; i++){
            Serial.print(String(i) + ": 0x");
            Serial.print(HexToCharMap[I2CAddr[i] & 0xF]);          //print out the first half of the hex (bitwise operator with 1111)
            Serial.print(HexToCharMap[(I2CAddr[i] & 0xF0) >> 4]);  //print out the second half of the hex (bitwise operator with 11110000, then shift 4 bits to the right)

            if(I2CAddr[i] == ADDR_LIDAR_LEFT){Serial.print(" (Left Lidar)");}
            else if(I2CAddr[i] == ADDR_LIDAR_RIGHT){Serial.print(" (Right Lidar)");}
            else if(I2CAddr[i] == ADDR_LIDAR_FRONT){Serial.print(" (Front Lidar)");}
            Serial.print('\n');
        }

        return false;
    }


    //Test connection to RFID readers by using the MFRC522's built in CRC coprocessor. A timeout most likely means a bad connection.
    //Initially I was just reading the firmware version register, but that proved to be unreliable.

    //
    //
    //  NOT YET WORKING, do CTRL+F in the library header file for "STATUS_CRC_WRONG" and implement checking if the calculated CRC matches what we should receive
    //
    //

    byte CRCData[2] = {0xFF, 0xFF};

    MFRC522::StatusCode RFIDStat = deviceRFIDLeft.PCD_CalculateCRC(CRCData, 2, CRCData);

    if (RFIDStat != MFRC522::STATUS_OK) {
        Serial.println("Error during self-test: Failed to communicated with Left RFID Reader!");
        return false;
    }

    RFIDStat = deviceRFIDRight.PCD_CalculateCRC(CRCData, 2, CRCData);

    if (RFIDStat != MFRC522::STATUS_OK) {
        Serial.println("Error during self-test: Failed to communicated with Right RFID Reader!");
        return false;
    }

    Serial.println("Self test pass!");
    return true;
}

/**
 * @brief Waits until the button is pressed. This will block execution.
 */
void FCTUC::waitStart() {
    Serial.println("[INFO] - FCTUC is waiting to start!");

    uint32_t tick = millis();
    
    // Note that logic is inverted due to PULLUP resistor!
    while (digitalRead(PIN_BUTTON)) {    
        setPixelColor(0, (millis() % 1000 >= 500) * 128, 0);
        delay(100); // Delay for task scheduler
    }

    setPixelColor(0, 0, 0);
}

/**
  @brief Read the button state.
  @return true if button is pressed, false if not
  */
bool FCTUC::readButton() {
    return digitalRead(PIN_BUTTON);
}

/**
  @brief Play a tone on the buzzer.
  @param frequency Frequency of square wave, in Hz
  @param duration Time to play the tone for, in milliseconds 
 */
void FCTUC::buzzer(uint16_t frequency, uint16_t duration = 0) {
    tone(PIN_BUZZER, frequency, duration);
}

/**
  @brief Set the NeoPixel's color.
  @param red value between [0, 255]
  @param green value between [0, 255]
  @param blue value between [0, 255]
 */
void FCTUC::setPixelColor(uint8_t red, uint8_t green, uint8_t blue) {
    deviceNeoPixel.setPixelColor(0, red, green, blue); // (pixelID, color)
    deviceNeoPixel.show();
}

/**
  @brief Set the velocity coefficients of each motor due to inconsistent motor design. PLEASE CAN WE FIX THE FUCKING POTENTIOMETERS AND GET RID OF THIS :)
  @param left value between [0.0, 1.0]
  @param right value between [0.0, 1.0]
 */
void FCTUC::setMotorCompensation(float left, float right) {
    motorCoefficientLeft = constrain(left, 0.0, 1.0);
    motorCoefficientRight = constrain(right, 0.0, 1.0);
}

/**
  @brief Control left motor speed.
  @param duty desired duty cycle for the motor, value between [-511, 511]
 */
void FCTUC::moveMotorLeft(int16_t duty) {
    duty = constrain(duty * motorCoefficientLeft, -DUTY_MOTOR_MAX, DUTY_MOTOR_MAX);

    if (duty <= 0) {
        digitalWrite(PIN_MOTOR_L_1, LOW);
    } else {
        duty = DUTY_MOTOR_MAX - duty;
        digitalWrite(PIN_MOTOR_L_1, HIGH);
    }

    analogWrite(PIN_MOTOR_L_2, abs(duty));
}

/**
  @brief Control right motor speed.
  @param duty desired duty cycle for the motor, value between [-511, 511]
 */
void FCTUC::moveMotorRight(int16_t duty) {
    duty = constrain(duty * motorCoefficientRight, -DUTY_MOTOR_MAX, DUTY_MOTOR_MAX);

    if (duty <= 0) {
        digitalWrite(PIN_MOTOR_R_1, LOW);
    } else {
        duty = DUTY_MOTOR_MAX - duty;
        digitalWrite(PIN_MOTOR_R_1, HIGH);
    }

    analogWrite(PIN_MOTOR_R_2, abs(duty));
}

/**
  @brief Control both motors simultaneously.
  @param dutyMotorLeft desired duty cycle for left motor, value between [-511, 511]
  @param dutyMotorRight desired duty cycle for right motor, value between [-511, 511]
 */
void FCTUC::moveMotors(int16_t dutyMotorLeft, int16_t dutyMotorRight) {
    moveMotorLeft(dutyMotorLeft);
    moveMotorRight(dutyMotorRight);
}

/**
 * @brief Get the right LiDAR distance value, in millimeters.
 * @return value between [0, 2600] (mm)
 */
uint16_t FCTUC::getLidarRightDistance() {
    uint16_t result = deviceLidarRight.readRangeContinuousMillimeters();
    return constrain(result, DIST_LIDAR_MIN, DIST_LIDAR_MAX);
}

/**
 * @brief Get the front LiDAR distance value, in millimeters.
 * @return value between [0, 2600] 
 */
uint16_t FCTUC::getLidarFrontDistance() {
    uint16_t result = deviceLidarFront.readRangeContinuousMillimeters();
    return constrain(result, DIST_LIDAR_MIN, DIST_LIDAR_MAX);
}

/**
 * @brief Get the left LiDAR distance value, in millimeters.
 * @return value between [0, 2600] 
 */
uint16_t FCTUC::getLidarLeftDistance() {
    uint16_t result = deviceLidarLeft.readRangeContinuousMillimeters();
    return constrain(result, DIST_LIDAR_MIN, DIST_LIDAR_MAX);
}

/**
 * @brief Returns the battery level, in volts.
 */
float FCTUC::getBatteryVoltage() {
    return map(analogRead(PIN_BAT_SENSE), 0, 3630, 0, 8200) / 1000.0f;
}

/**
  @brief Print all detected I2C devices.
 */
void FCTUC::printI2C() {
    Serial.println("I2C scanner. Scanning ...");
    byte count = 0;

    for (byte i = 1; i < 120; i++) {
        Wire.beginTransmission(i);

        if (Wire.endTransmission() == 0) {
            Serial.print("Found address: ");
            Serial.print(i, DEC);
            Serial.print(" (0x");
            Serial.print(i, HEX);
            Serial.println(")");
            count++;
            delay(1);
        }
    }

    Serial.println("Done.");
    Serial.print("Found ");
    Serial.print(count, DEC);
    Serial.println(" device(s).");
}

/**
  @brief Print all LiDAR distance values.
 */
void FCTUC::printLidarValue() {
    uint16_t left = getLidarLeftDistance();
    uint16_t front = getLidarFrontDistance();
    uint16_t right = getLidarRightDistance();

    Serial.println(
        "Left : Front : Right - " + String(left) + " : " +  String(front)  + " : " + String(right) + " (mm)"
    );
}

/**
  @brief Print (to serial ONLY) the detected RFID reader firmware versions. Useful for detecting connection issues.
 */
void FCTUC::printRfidPcdFw() {
    Serial.print("Left: ");
    deviceRFIDLeft.PCD_DumpVersionToSerial();
    Serial.print("Right: ");
    deviceRFIDRight.PCD_DumpVersionToSerial();
}