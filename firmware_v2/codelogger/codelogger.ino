/*************************************************************************
* Arduino OBD-II/G-Force Data Logger and Transmitter
* Distributed under GPL v2.0
* Copyright (c) 2013-14 Stanley Huang <stanleyhuangyc@gmail.com>
* All rights reserved.
*************************************************************************/

#include <Arduino.h>
#include <OBD.h>
#include <SD.h>
#include <Wire.h>
#include <SPI.h>
#include <MPU6050.h>
#include "config.h"
#if USE_SOFTSERIAL
#include <SoftwareSerial.h>
#endif
#if USE_GPS && LOG_GPS_PARSED_DATA
#include <TinyGPS.h>
#endif
#include "codelogger.h"

#if LCD_DISPLAY
   #include <LiquidCrystal_I2C.h>
#endif

// logger states
#define STATE_SD_READY 0x1
#define STATE_OBD_READY 0x2
#define STATE_GPS_FOUND 0x4
#define STATE_GPS_READY 0x8
#define STATE_ACC_READY 0x10
#define STATE_SLEEPING 0x20

#if USE_SOFTSERIAL
#if VERBOSE && !ENABLE_DATA_OUT
#if defined(__AVR_ATmega2560__) || defined(__AVR_ATmega1280__)
SoftwareSerial SerialInfo(A8, A9); /* for BLE Shield on MEGA*/
#else
SoftwareSerial SerialInfo(A2, A3); /* for BLE Shield on UNO/leonardo*/
#endif
#endif
#else
#define SerialInfo Serial
#endif

#if LCD_DISPLAY
   LiquidCrystal_I2C lcd(LCD_ADDRESS, 16, 2);
#endif

#define PMTK_SET_NMEA_UPDATE_1HZ  "$PMTK220,1000*1F\r"
#define PMTK_SET_NMEA_UPDATE_5HZ  "$PMTK220,200*2C\r"
#define PMTK_SET_NMEA_UPDATE_10HZ "$PMTK220,100*2F\r"

#if USE_GPS && LOG_GPS_PARSED_DATA
  TinyGPS gps;
#endif

static uint16_t lastFileSize = 0;
static uint16_t fileIndex = 0;

static byte pidTier1[]= {PID_RPM, PID_SPEED, PID_ENGINE_LOAD, PID_THROTTLE};
static byte pidTier2[] = {PID_COOLANT_TEMP, PID_INTAKE_TEMP, PID_AMBIENT_TEMP, PID_FUEL_LEVEL, PID_BAROMETRIC, PID_DISTANCE, PID_RUNTIME};

#define TIER_NUM1 sizeof(pidTier1)
#define TIER_NUM2 sizeof(pidTier2)

class COBDLogger : public COBD, public CDataLogger
{
public:
    COBDLogger():state(0) {}
    void setup()
    {
        state = 0;
        
#if USE_ACCEL
        Wire.begin();
        if (MPU6050_init() == 0) {
            state |= STATE_ACC_READY;
        }
#endif

        for (;;) {
            if (init()) {
                state |= STATE_OBD_READY;
                break;
            }
        }

#if USE_GPS
        if (initGPS()) {
            state |= STATE_GPS_FOUND;
        }
#endif

        showStates();
        //showECUCap();
        //queryDTCCount();
    }
#if USE_GPS
    void logGPSData()
    {
#if LOG_GPS_PARSED_DATA
        bool isUpdated = false;
#endif
        char lastChar;
        // issue the command to get NMEA data (one line per request)
        write("ATGRR\r");
        dataTime = millis();
        logTimeElapsed();
        for (;;) {
            if (available()) {
                char c = read();
                if (c == '>') {
                    // prompt char received
                    break;
                } else {
#if VERBOSE
                    SerialInfo.write(c);
#endif
#if LOG_GPS_PARSED_DATA
                    if (gps.encode(c)) {
                        isUpdated = true;
                        state |= STATE_GPS_READY;
                    }
#elif LOG_GPS_NMEA_DATA
                    logData(c);
                    lastChar = c;
#endif

                }
            } else if (millis() - dataTime > 100) {
                // timeout
                break;
            }
        }
#if LOG_GPS_PARSED_DATA
        if (isUpdated) {
            uint32_t date, time;
            gps.get_datetime(&date, &time, 0);
            logData(PID_GPS_TIME, (int32_t)time);
            int32_t lat, lon;
            gps.get_position(&lat, &lon, 0);
            logData(PID_GPS_LATITUDE, lat);
            logData(PID_GPS_LONGITUDE, lon);
            logData(PID_GPS_ALTITUDE, (int)(gps.altitude() / 100));
            int kph = gps.speed() * 1852 / 100000;
            logData(PID_GPS_SPEED, kph);
        }
#elif LOG_GPS_NMEA_DATA
        if (lastChar != '\r') {
            logData('\r');
        }

#endif
    }
#endif
#if USE_ACCEL
    void logACCData()
    {
        if ((state & STATE_ACC_READY)) {
            accel_t_gyro_union accData;
            MPU6050_readout(&accData);
            dataTime = millis();
#if VERBOSE
            SerialInfo.print("ACC:");
            SerialInfo.print(accData.reg.x_accel_h);
            SerialInfo.print('/');
            SerialInfo.print(accData.reg.y_accel_h);
            SerialInfo.print('/');
            SerialInfo.println(accData.reg.z_accel_h);
#endif
            // log x/y/z of accelerometer
            logData(PID_ACC, accData.value.x_accel >> 4, accData.value.y_accel >> 4, accData.value.z_accel >> 4);
            // log x/y/z of gyro meter
            //logData(PID_GYRO, accData.value.x_gyro, accData.value.y_gyro, accData.value.z_gyro);
        }
    }
#endif
    void logOBDData()
    {
        static byte index1 = 0;
        static byte index2 = 0;

        if (index1 == TIER_NUM1) {
            index1 = 0;
            queryOBDData(pidTier2[index2]);
            index2 = (index2 + 1) % TIER_NUM2;
        } else {
            queryOBDData(pidTier1[index1++]);
        }
        if (errors >= 5) {
            reconnect();
        }
    }
#if ENABLE_DATA_LOG
    bool initSD()
    {
        state &= ~STATE_SD_READY;
        pinMode(SS, OUTPUT);
        Sd2Card card;
        if (card.init(SPI_HALF_SPEED, SD_CS_PIN)) {
#if VERBOSE
            const char* type;
            switch(card.type()) {
            case SD_CARD_TYPE_SD1:
                type = "SD1";
                break;
            case SD_CARD_TYPE_SD2:
                type = "SD2";
                break;
            case SD_CARD_TYPE_SDHC:
                type = "SDHC";
                break;
            default:
                type = "SDx";
            }

            SerialInfo.print("SD type: ");
            SerialInfo.println(type);

            SdVolume volume;
            if (!volume.init(card)) {
                SerialInfo.println("No FAT!");
                return false;
            }

            uint32_t volumesize = volume.blocksPerCluster();
            volumesize >>= 1; // 512 bytes per block
            volumesize *= volume.clusterCount();
            volumesize >>= 10;
            SerialInfo.print("SD size: ");
            SerialInfo.print((int)((volumesize + 511) / 1000));
            SerialInfo.println("GB");
#endif
        }

        if (!SD.begin(SD_CS_PIN)) {
            return false;
        }

        uint16_t index = openFile();
        if (index) {
#if VERBOSE
            SerialInfo.print("File ID: ");
            SerialInfo.println(index);
#endif
            state |= STATE_SD_READY;
        } else {
#if VERBOSE
            SerialInfo.println("File error");
#endif
        }
        return true;
    }
    void flushData()
    {
        // flush SD data every 1KB
        if ((state & STATE_SD_READY) && (uint16_t)dataSize - lastFileSize >= 1024) {
            flushFile();
            lastFileSize = (uint16_t)dataSize;
#if VERBOSE
            // display logged data size
            SerialInfo.print("Logged KB:");
            SerialInfo.println((int)(dataSize >> 10));
#endif
        }
    }
#endif
#if USE_GPS
    bool initGPS()
    {
        char buf[OBD_RECV_BUF_SIZE];
        // setting GPS baudrate
        write("ATBR2 38400\r");
        if (receive(buf) && strstr(buf, "OK")) {
            /*
            write("ATSGC ");
            write(PMTK_SET_NMEA_UPDATE_10HZ);
            receive();
            */
            return true;
        } else {
            return false;
        }
    }
#endif


/* 
 * queryDTCCount
 *  send request for number of DTC codes available. Will also return value if MIL ("Check Engine") light is on.
 *  if number of codes is > 0, then request the actual codes as well.
 */    
    void queryDTCCount()
    {
      char buffer[64];
      char val3[3];
      write("01 01\r");  // send OBD request for DTCs
      int bytesReceived = receive(buffer,1000);
      buffer[bytesReceived+1] = 0;

      if (bytesReceived>0) {

         // Response should be "41 01 xx yy yy yy yy"
         // looking for the value in xx

         //const char *buffer = "41 01 81 00 00 00 00";  // to debug

         int response = strncmp(buffer,"41",2);
         if (response == 0) {

            // Extract the numver of codes from the string
            memcpy( val3, &buffer[6], 2 );
            val3[2] = '\0';

            int numCodes = strtol(val3,NULL,16);  // Convert hex string to decimal

            free(val3);
          
            if ((numCodes & 128)>0) { // MIL is on
              numCodes = numCodes - 128;
            }
            queryDTCs(numCodes);
            
         } else {

#if LCD_DISPLAY
           lcd.print("No DTCs");
#endif
// Use for debugging at computer
//            queryDTCs(-9);

         }
         
      }

    }



    byte state;
private:
    void queryOBDData(byte pid)
    {
        int value;

        // send a query command
        sendQuery(pid);
        pid = 0; // this lets PID also obtained and filled from response
        // receive and parse the response
        if (getResult(pid, value)) {
            dataTime = millis();
            showData(pid, value);
            // log data to SD card
            logData(0x100 | pid, value);
            errors = 0;
        } else {
            errors++;
        }
    }


/* queryDTCs
 *  Send request for trouble codes.  Will receive multiple codes in one message.
 *
 * Called by public queryDTCCount procedure
 *
 */
   void queryDTCs(int numCodes) 
   {
      char buffer[64];
      write("03\r");  // send OBD request for DTCs
      int bytesReceived = receive(buffer,1000);
      buffer[bytesReceived+1] = '\0';

      if (bytesReceived>0) {
         
         int response = strncmp(buffer,"43",2);
         if (response == 0) {  // != 0 for debugging
           
            // Debugging code
            if (numCodes == -9) {
               char buffer[] = "43 04 41 00 00 00 00\r43 01 33 00 00 00 00\r";  // Example return for 2 codes
               numCodes = 2;
            }

#if ENABLE_DATA_LOG
            sdfile.println("DTC Response");
            sdfile.println(buffer);
#endif
            
            int i=0, o=0;
            char response[64];
            // Remove spaces from the string
            while (buffer[i]!= '\0') {
              if (buffer[i]!=' ') response[o++]=buffer[i];
              i++;
            }
            response[o] = '\0';

            char oneCode[14];
            char category[2];
            char codeType[3];
            char codeTypes[5] = "PCBU";
            char code[4];  // Actual trouble code
            char codeOutput[10]; // formatted output

            // Loop through each code
            for (int codeCount=1; codeCount<=numCodes; codeCount++) {

               // Grab one chunk of codes at a time
               memcpy( oneCode, &response[15*(codeCount-1)], 14 );
               oneCode[14] = '\0';               

               memcpy( category, &oneCode[2], 1 );
               category[2] = '\0';
               int numCategory = strtol(category, NULL, 16);
               free(category);
               
               int n = sprintf(codeType, "%c%d", codeTypes[(numCategory/4)], (numCategory%4));
               
               memcpy( code, &oneCode[3], 3 );
               code[3] = '\0';

               n = sprintf(codeOutput, "%d/%d:%s%s", codeCount, numCodes, codeType, code);

#if LCD_DISPLAY
               lcd.setCursor(0,(codeCount-1)); // next line down for each code
               lcd.print(codeOutput);
#endif               

#if ENABLE_DATA_LOG
            sdfile.println(codeOutput);
#endif

              delay(20);
              
               //free(code);
            }

            //free(oneCode);
            
         } else {
#if LCD_DISPLAY
           lcd.print("No DTC codes");  // Should never reach here
           lcd.println(buffer);
#endif         
         }

#if ENABLE_DATA_LOG
        if (state & STATE_SD_READY) {
          flushFile();
        }
#endif
      }     
   }
    
    void showECUCap()
    {
#if VERBOSE
        byte pidlist[] = {PID_RPM, PID_SPEED, PID_THROTTLE, PID_ENGINE_LOAD, PID_CONTROL_MODULE_VOLTAGE, PID_MAF_FLOW, PID_INTAKE_MAP, PID_FUEL_LEVEL, PID_FUEL_PRESSURE, PID_COOLANT_TEMP, PID_INTAKE_TEMP, PID_AMBIENT_TEMP, PID_TIMING_ADVANCE, PID_BAROMETRIC};
        const char* namelist[] = {"RPM", "SPEED", "THROTTLE", "ENG.LOAD", "CTRL VOLT", "MAF", "MAP", "FUEL LV.", "FUEL PRE.", "COOLANT", "INTAKE","AMBIENT", "IGNITION", "BARO"};
        for (byte i = 0; i < sizeof(pidlist) / sizeof(pidlist[0]); i++) {
            SerialInfo.print(namelist[i]);
            SerialInfo.print(':');
            SerialInfo.println(isValidPID(pidlist[i]) ? 'Yes' : 'No');
        }
#endif
    }
    void reconnect()
    {
#if ENABLE_DATA_LOG
        closeFile();
#endif
        state &= ~STATE_OBD_READY;
        state |= STATE_SLEEPING;
        //digitalWrite(SD_CS_PIN, LOW);
#if VERBOSE
        SerialInfo.print("Reconnecting ");
#endif
        while (!init()) {
#if VERBOSE
            SerialInfo.write('.');
#endif
        }
        state &= ~STATE_SLEEPING;
#if ENABLE_DATA_LOG
        if (!openFile()) {
            state &= ~STATE_SD_READY;
        }
#endif
    }
    // screen layout related stuff
    void showStates()
    {
#if VERBOSE
        SerialInfo.print("OBD:");
        SerialInfo.print((state & STATE_OBD_READY) ? "Yes" : "No");
        SerialInfo.print(" ACC:");
        SerialInfo.print((state & STATE_ACC_READY) ? "Yes" : "No");
        SerialInfo.print(" GPS:");
        SerialInfo.println((state & STATE_GPS_FOUND) ? "Yes" : "No");
        delay(1000);
#endif
    }
    void showData(byte pid, int value)
    {
#if VERBOSE
        SerialInfo.print('[');
        SerialInfo.print(millis());
        SerialInfo.print("] [");
        SerialInfo.print(pid, HEX);
        SerialInfo.print("]=");
        SerialInfo.println(value);
#endif
    }
};

static COBDLogger logger;

void setup()
{
#if VERBOSE
    SerialInfo.begin(STREAM_BAUDRATE);
#endif

#if LCD_DISPLAY
    lcd.init();
    lcd.backlight();
#endif

    logger.begin();
    logger.initSender();
    logger.setup();

#if ENABLE_DATA_LOG
    logger.initSD();
#endif

}

void loop()
{
    uint32_t t = millis();

    logger.logOBDData();
#if USE_ACCEL
    logger.logACCData();
#endif
#if USE_GPS
    if (logger.state & STATE_GPS_FOUND) {
        logger.logGPSData();
    }
#endif

#if MONITOR_DTC
  // Run DTC task every DTC_INTERVAL (allow for 2s between checks)
  if ((t%DTC_INTERVAL)<=2000) {
    logger.queryDTCCount();
  }
#endif

#if ENABLE_DATA_LOG
    logger.flushData();
#endif

#if LOOP_INTERVAL
    // get time elapsed
    t = millis() - t;
    if (t < LOOP_INTERVAL) {
        delay(LOOP_INTERVAL - t);
    }
#endif
}
