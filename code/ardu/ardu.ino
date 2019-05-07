/*-----------------------------------------------------------------------------
 *  Author: Felipe G. Nievinski <fgnievinski@gmail.com>
 *  Author: Nelso G. Jost <nelsojost@gmail.com>
 *  Author: Cristthian Marafigo Arpino <cristthian.m.arpino@protonmail.com>
 *  Author: Lucas Doria de Carvalo <lucas.doria@ufrgs.br>
 *  Author: Iuri Mendonça Tinti <iuri.tinti@inf.ufrgs.br>
 *  License: GPLv2
 *  Purpose: Firmware to perform simple datalog of GPS raw data with the
 *           Adalogger Feather board and GPS Feathering module.
 *           Arduino IDE and PlatformIO compatible.
 *---------------------------------------------------------------------------*/
#include <SPI.h>
#include <SD.h>

#define GPSSerial          Serial1
#define GPSSerialEvent     serialEvent1

#define GPS_BAUD_RATE_DEFAULT    9600   // original firmware
#define GPS_BAUD_RATE  115200

// time between GPS readings:
#define  GPS_UPDATE_INTERVAL  1000          // in milliseconds
// suported values: 1000 ms   (1 Hz rate)
//                  10000 ms  (100 mHz rate)
// ATTENTION: SD flush/close takes about 200 ms!!
//            Thus higher rates such as 5 Hz or 10 Hz may not comply.

// time to wait for the GPS to finish sending data:
#define  IDLE_THRESHOLD      10             // milliseconds
// It should be greater than 5 ms, which is 
// the maximum time between GPS serial events (determined experimentally).
// It should be smaller than GPS_UPDATE_INTERVAL, 
// otherwise the buffer might have to hold more than one block of GPS data.

#define  BLINK_LED_IF_OKAY  // give visual indication that writing is okay
//#define WAIT_FOR_VALID_GPS  // if not defined, we'll write to default file.

#define  GPS_BUFFER_SIZE_TYPICAL 512  // typical buffer size, for pre-allocation

#define MAX_BASENAME_LEN 8+1  // FAT limit: 8.3
#define MAX_FILENAME_LEN MAX_BASENAME_LEN-1+1+3+1  // basename-null+dot+ext+null

// check if USB serial port is open before printing:
#define serialPrint if(Serial)Serial.print
#define serialPrintln if(Serial)Serial.println

const int SDCard = 4;  // SD card ID
const char basenameDefault[] = "DEFAULT";
String GPSBuffer = "";   // to hold the incoming GPS data
unsigned long bufferTime = millis();  // timer to indicate when GPS was last read
const char fileDuration = 'D';     // Use 'H' to save by Hour or 'D' to save by Day

// battery settings:
#define BATTERY_UPDATE_INTERVAL  60000      // in milliseconds; zero to disable.
//#define BATTERY_UPDATE_INTERVAL  1000      // in milliseconds; zero to disable.
#define VBATPIN A9  // battery voltage pin
unsigned long batteryTimeEnd = millis();  // timer to indicate when battery was last checked
unsigned long batteryTimeBusy = 0;  // time duration spent busy writing battery

int numBlk=0;  // number of GPS data blocks per file
char basenameOld[MAX_BASENAME_LEN];  // last used basename

// function prototypes:
void configGPS(void);
bool initSD(void);
void datalog(const char basename[]);
void getBasename(char basename[], const char dateTime[], bool GPSActive);
bool getDateTime(char stringOriginal[], char dateTime[]);
void vbatlog(const char basename[], const char dateTime []);
void writeBatteryVoltage(const char basename[], const char dateTime[]);
float readBatteryVoltage (void);
void blinkLED(void);
const char* nth_strchr(const char* s, int c, int n);

void setup()
{
    Serial.begin(115200);

    initSD();
    configGPS();
    delay(1000);
    
    GPSBuffer.reserve(GPS_BUFFER_SIZE_TYPICAL);
    GPSSerial.flush();
}

/* Rationale: GPS keeps sending characters via a serial event;
 * We store the GPS characters in a string (GPSBuffer).
 * When the GPS stops talking, then we log the data to SD card.
 */

void loop()
{
    bool GPSActive;
    char dateTime[6+6+1];  // yymmddHHMMSS\0
    char basename[MAX_BASENAME_LEN];
    unsigned long startTime=millis();

    // time interval elapsed since last GPS serial read:
    unsigned long idleTime = startTime - bufferTime;

    // we will write only when there is no char coming from the GPS:
    if (idleTime < IDLE_THRESHOLD || GPSBuffer.length() < 3) { return; }

    // GPS serial went silent, now log data to SD card.
 
    serialPrintln("[DEBUG] starting.");
    serialPrint(GPSBuffer);

    GPSActive = getDateTime(GPSBuffer.c_str(), dateTime);
    if (!GPSActive)
    {
#ifdef WAIT_FOR_VALID_GPS
        serialPrintln("[DEBUG] waiting for GPS");
        return;
#endif
    }

    getBasename(basename, dateTime, GPSActive);
    
    datalog(basename);

    vbatlog(basename, dateTime);

    // calculate number of blocks per file:
    if (strcmp(basename, basenameOld) != 0)
    {
        strncpy(basenameOld, basename, strlen(basename)+1);
        numBlk=0;
    }
    numBlk++;
    serialPrintln("[DEBUG] number of blocks: " + String(numBlk));

    // clear the buffer and preallocate it:
    GPSBuffer = "";
    GPSBuffer.reserve(GPS_BUFFER_SIZE_TYPICAL);
 
    serialPrintln("[DEBUG] done; time ellapsed: " +
                   String(millis() - startTime) + " ms.\n");
}
    
void vbatlog(const char basename[], const char dateTime [])
{
    if( BATTERY_UPDATE_INTERVAL == 0 ) { return; }
    unsigned long batteryTimeStart = millis();
    unsigned long batteryTimeIdle = batteryTimeStart - batteryTimeEnd;
    unsigned long batteryTimeTol = BATTERY_UPDATE_INTERVAL - batteryTimeBusy*0.9;
    if( batteryTimeIdle < batteryTimeTol ) { return; }
    writeBatteryVoltage(basename, dateTime);
    batteryTimeEnd = millis();
    batteryTimeBusy = batteryTimeEnd - batteryTimeStart;
}

void writeBatteryVoltage (const char basename[], const char dateTime[])
{
    char filename[MAX_FILENAME_LEN];
    strncpy(filename, basename, strlen(basename)+1);
    strcat(filename, ".bat");
    serialPrintln("[DEBUG] filename (battery): " + String(filename));

    File file = SD.open(filename, FILE_WRITE);
    if (!file)
    {
        serialPrintln("[ERROR] Unable to open file '" + String(filename) + "'.");
        return;
    }

    file.print(dateTime);
    file.print("\t");
    file.print(readBatteryVoltage());
    file.println("");

    file.close();
}

float readBatteryVoltage (void)
{
    float measuredvbat = analogRead(VBATPIN);
    measuredvbat *= 2;    // we divided by 2, so multiply back
    measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
    measuredvbat /= 1024; // convert to voltage
    return measuredvbat;
}


// called in between loops, every time a new character appears at GPSSerial
void GPSSerialEvent()
{
    char c = GPSSerial.read();

    // store the caracter in the buffer
    GPSBuffer += c;

    // restart the idle timer count
    bufferTime = millis();
}


// configure the GPS settings
void configGPS(void)
{
    //GPSSerial.begin(GPS_BAUD_RATE);
    configGPSBaudRate();

    switch (GPS_UPDATE_INTERVAL)
    {
        case 100:
            //PMTK_SET_NMEA_UPDATE_10HZ
            GPSSerial.println("$PMTK220,100*2F\r\n");
            break;
        case 1000:
            // PMTK_SET_NMEA_UPDATE_1HZ
            GPSSerial.println("$PMTK220,1000*1F\r\n");
            break;
        case 10000:
            // PMTK_SET_NMEA_UPDATE_100_MILLIHERTZ
            GPSSerial.println("$PMTK220,10000*2F\r\n");
            break;
    }
    
    // GPSSerial.println("$PGCMD,33,1*6C");  // PGCMD_ANTENNA
    GPSSerial.println("$PGCMD,33,0*6D");  // PGCMD_NOANTENNA

    // PMTK_SET_NMEA_OUTPUT_RMCGGAGSV:
    GPSSerial.println("$PMTK314,0,1,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0*29");
}

void configGPSBaudRate(void) {
    if (GPS_BAUD_RATE == GPS_BAUD_RATE_DEFAULT) {
        GPSSerial.begin(GPS_BAUD_RATE);
        return;
    }
    
    // open GPS at default baud rate (9600) then 
    // change it to the new rate (115200) and reconnect:
    // (if it was already at 115200, it will be ignored.)
    GPSSerial.begin(GPS_BAUD_RATE_DEFAULT);
    switch (GPS_BAUD_RATE)
    {
        case 57600:
            // PMTK_SET_BAUD_57600
            GPSSerial.println("$PMTK251,57600*2C");
            break;
        case 115200:
            // PMTK_SET_BAUD_115200
            GPSSerial.println("$PMTK251,115200*1F");
            break;
    }

    delay(1000);
    GPSSerial.begin(GPS_BAUD_RATE);
}

// initialize SD card
bool initSD(void)
{
    // WARNING: SD.begin() can only be called once due to a bug at the
    // Arduino SD.h standard library

    if (!SD.begin(SDCard))
    {
        serialPrintln("[ERROR] Unable to initialized SD card.");
        return false;
    }

    return true;
}

// write data do SD card
void datalog(const char basename[])
{
    char filename[MAX_FILENAME_LEN];
    strncpy(filename, basename, strlen(basename)+1);
    strcat(filename, ".log");
    serialPrintln("[DEBUG] filename (GPS): " + String(filename));
    
    // open the file
    File file = SD.open(filename, FILE_WRITE);
    if (!file)
    {
        serialPrintln("[ERROR] Unable to open file '" + String(filename) + "'.");
        //return;
    }

    // get the size of string to check latter if it has been all written:
    byte len1 = GPSBuffer.length();
    
    // write whole GPS buffer on the SD file at once:
    byte len2 = file.print(GPSBuffer);
    
    // insert a blank line between blocks:
    file.println();

    // closing the file to guarantee integrity:
    // (it costs about 200 ms, same as flush())
    //return true;
    file.close();

    if (len2 == len1) { blinkLED(); }
    String prefix = "[DEBUG]";
    if (len2 != len1) { prefix = "[ERROR]"; }
    serialPrintln(prefix + " " + (100*len2/len1) + "%" + " of " + String(len1) + " bytes written to SD file '" + String(filename) + "'");
}

// blink the light emitting diode:
void blinkLED(void) {
#ifndef BLINK_LED_IF_OKAY
    return;
#endif
    digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(10);              // wait a little
    digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
}

// obtain basename:
void getBasename(char basename[], const char dateTime[], bool GPSActive)
{
    if(!GPSActive)
    {
        // use default basename:
        strncpy(basename, basenameDefault, strlen(basenameDefault)+1);
        return;
    }

    // decide how many characters to copy:
    int num;
    switch (fileDuration)
    {
    case 'H':  // hour
        num = 8;
        break;
    case 'D':  // day
        num = 6;
        break;
    default:
        serialPrintln("[ERROR] Filename duration not recognized; assuming hourly.");
        num = 8;
    }

    // copy date/time (YYMMDDhh) for the basename:
    strncpy(basename, dateTime, num);
    
    // null-terminate string:
    basename[num]='\0';
    
    serialPrintln("[DEBUG] basename: " + String(basename));
}

// fills a string with date and time from the GPS:
bool getDateTime(const char stringOriginal[], char dateTimeOut[])
{
    char  strTemp[82];  // as per <https://en.wikipedia.org/wiki/NMEA_0183>
    char* limInf=NULL;
    char* limSup=NULL;
    const char *dateIn;
    const char *timeIn;
    int len;

    /* here we handle a string like this: 
     *    $GPRMC,144016.000,A,3001.2778,S,05113.2839,W,0.01,37.24,090318,,,A*59
     * then find the date (090318) and time (144016, discarding decimal seconds),
     * and finally reorder it as YYMMDDhhmmss: 180309124016.
    */
    
    if(strlen(stringOriginal)==0)
    {
        serialPrintln("[DEBUG] Empty GPS string.");
        return false;
    }

    // find beginning of $GPRMC sentence:
    limInf=strstr(stringOriginal, "$GPRMC");
    if(!limInf)
    {
        serialPrintln("[DEBUG] $GPRMC not found.");
        return false;
    }
    serialPrint(limInf);

    // verify if GPS is active:
    if((limInf[18]!='A'))
    {
        serialPrintln("[DEBUG] GPS not valid.");
        return false;
    }

    // find end of $GPRMC sentence:
    limSup=strchr(limInf, '\n');
    len=(limSup-limInf)+1;
    strncpy(strTemp, limInf, min(len, sizeof(strTemp)-1));
    strTemp[len]='\0';

    // extract date, reordering components (DDMMYY -> YYMMDD):
    dateIn = nth_strchr(strTemp, ',', 9) + 1;
    dateTimeOut[0] = dateIn[4];  // year
    dateTimeOut[1] = dateIn[5];  // year
    dateTimeOut[2] = dateIn[2];  // month
    dateTimeOut[3] = dateIn[3];  // month
    dateTimeOut[4] = dateIn[0];  // day
    dateTimeOut[5] = dateIn[1];  // day

    // extract time, keeping the order (hhmmss):
    timeIn = &strTemp[7];
    strncpy(&dateTimeOut[6], timeIn, 6);

    // terminate string:
    dateTimeOut[12]='\0';
    serialPrintln("[DEBUG] dateTime: " + String(dateTimeOut));

    return true;
}

// returns a pointer to the nth character in a string:
const char* nth_strchr(const char* s, int c, int n)
{
    int c_count;
    char* nth_ptr;

    for (c_count=1,nth_ptr=strchr(s,c);
         nth_ptr != NULL && c_count < n && c!=0;
         c_count++)
    {
         nth_ptr = strchr(nth_ptr+1, c);
    }

    return nth_ptr;
}

