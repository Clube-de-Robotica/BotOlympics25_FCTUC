// =============================================================
// = Header File FCTUC - BotOlympics 2025
// =
// = Contributors:
// =    João Vasco @ https://github.com/JNDVasco (-2022)
// =    Tobias Hulland @ https://github.com/errorcodecritical (2023-2025)
// =    Martim Pegueiro @ https://github.com/LittleNyanCat (2024-2025)
// =    Tiago Furtado @ https://github.com/tiagoluis19 (2025)
// =============================================================

#ifndef BOT_FCTUC_H
#define BOT_FCTUC_H

#define STRSTR(y) #y
#define STR(x) STRSTR(x)

/* Communication Libraries */
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include <esp_task_wdt.h>

/* OTA Upload */
#ifdef OTA_ENABLED
#include <ArduinoOTA.h>
#endif

/* Remote Console */
#ifdef TELNET_ENABLED
#include <ESPTelnetStream.h>
#endif

/* Peripheral Drivers */
#include <VL53L0X.h>
#include <MFRC522.h>
#include <Adafruit_NeoPixel.h>

/*Json Enocder/Decoder*/
#include <ArduinoJson.h>


/* Wireless Communications */
extern WiFiUDP udp;
#ifdef TELNET_ENABLED
extern ESPTelnetStream telnet;
#endif

/**
 * @brief Class to interface with robot hardware.
 */
class FCTUC {
private:
    /* Pin Mapping */
    static constexpr uint16_t PIN_MOTOR_L_1         = 17;
    static constexpr uint16_t PIN_MOTOR_L_2         = 16;
    static constexpr uint16_t PIN_MOTOR_R_1         = 13;
    static constexpr uint16_t PIN_MOTOR_R_2         = 27;
    static constexpr uint16_t PIN_NEOPIXEL          = 14;
    static constexpr uint16_t PIN_BUZZER            = 26;
    static constexpr uint16_t PIN_XSHUT_RIGHT       = 25;
    static constexpr uint16_t PIN_XSHUT_FRONT       = 33;
    static constexpr uint16_t PIN_XSHUT_LEFT        = 32;
    static constexpr uint16_t PIN_BUTTON            = 39;
    static constexpr uint16_t PIN_BAT_SENSE         = 36;
    static constexpr uint16_t PIN_RFID_SDA_L        = 5;
    static constexpr uint16_t PIN_RFID_SDA_R        = 12;

    /* LIDAR I2C Address Mapping */
    static constexpr uint16_t ADDR_LIDAR_LEFT       = 0x70;
    static constexpr uint16_t ADDR_LIDAR_FRONT      = 0x71;
    static constexpr uint16_t ADDR_LIDAR_RIGHT      = 0x72;

    /* LIDAR Range */
    static constexpr uint16_t DIST_LIDAR_MIN        = 0;
    static constexpr uint16_t DIST_LIDAR_MAX        = 2600;

    /* Battery Warning Threshold */
    static constexpr float MIN_BAT_VOLTAGE         = 5.7f;

    /* Motor Config */
    static constexpr uint8_t PWM_RESOLUTION_BITS    = 9;
    static constexpr int16_t DUTY_MOTOR_MAX         = 512 - 1;

    /* WiFi Config - Please check 'platformio.ini' to define these macros. */
    static constexpr char WIFI_SSID[]               = STR(SET_WIFI_SSID);
    static constexpr char WIFI_PWD[]                = STR(SET_WIFI_PWD);
    static constexpr char LOCAL_IP[]                = STR(SET_LOCAL_IP);
    static constexpr char GATEWAY_IP[]              = STR(SET_GATEWAY_IP);
    static constexpr char SUBNET_MASK[]             = STR(SET_SUBNET_MASK);
    
    /* OTA, UDP, Telnet Config */
    static constexpr uint16_t TELNET_PORT           = 23;
    static constexpr uint16_t UDP_PORT              = 1234;
    static constexpr uint16_t OTA_PORT              = 3232;
    static constexpr char OTA_PWD[]                 = STR(SET_OTA_PWD);

    /* RFID Config */
    static constexpr uint8_t TAG_BLOCK_POS          = 61;
    static constexpr uint8_t TAG_ANTENNA_SWITCH_MS  = 25;

    /* Internal Task Handles */
    static TaskHandle_t mainTaskHandle;
    static TaskHandle_t batteryTaskHandle;
    static TaskHandle_t rfidTaskHandle;
    static TaskHandle_t wifiTaskHandle;
    static TaskHandle_t idleTaskHandle;

    /* Peripheral Interfaces */
    static Adafruit_NeoPixel deviceNeoPixel;

    static MFRC522 deviceRFIDRight;
    static MFRC522 deviceRFIDLeft;

    static VL53L0X deviceLidarRight;
    static VL53L0X deviceLidarFront;
    static VL53L0X deviceLidarLeft;

    static float motorCoefficientLeft;
    static float motorCoefficientRight;

    /* Initialization Methods */
    static void setupButton();
    static void setupBuzzer();
    static void setupNeopixel();
    static void setupMotors();
    static void setupLidar();
    static void setupRFID();
    static void setupWifi();

    /* Internal Task Routines */
    static void taskMonitorBatteryValue(void*);
    static void taskReadActiveRFIDValue(void*);
    static void taskMonitorWirelessComms(void*);
    static void taskIdle(void*) {while(1) {esp_task_wdt_reset();}}

    /* IR Sensor Values*/
    static constexpr uint16_t PIN_IR_SENSOR = 34;
    static constexpr uint16_t IR_MIN = 0;
    static constexpr uint16_t IR_MAX = 4096;


public:
    static void begin();
    static void waitStart();
    static void terminate();
    static bool doSelfTest();
    static bool readButton();
    static void buzzer(uint16_t, uint16_t);
    static void setPixelColor(uint8_t, uint8_t, uint8_t);
    static void setMotorCompensation(float, float);

    static void moveMotorLeft(int16_t);
    static void moveMotorRight(int16_t);
    static void moveMotors(int16_t, int16_t);

    static uint16_t getLidarLeftDistance();
    static uint16_t getLidarFrontDistance();
    static uint16_t getLidarRightDistance();

    static float getBatteryVoltage();

    static bool& getTagDetected();
    static bool& getTagReadSuccess();
    static uint8_t* getTagData();

    static uint8_t* getUDPData();

    static void printI2C();
    static void printLidarValue();
    static void printRfidPcdFw();
    static uint16_t getIRSensorValue();

    static void STOP();
    
};
#endif