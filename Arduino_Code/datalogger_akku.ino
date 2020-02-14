#include "SD.h"
#include "FS.h"
#include <SPI.h>
#include <TinyGPS++.h>
#include "Adafruit_MAX31855.h"
#include <Wire.h>
#include <HardwareSerial.h>
#include "BluetoothSerial.h"


// Pindefinition für ESP-WROOM-32 DEV KIT

const int hallPinFront = 34;        // Hall-Sensor vorne
const int hallPinRear = 35;         // Hall-Sensor hinten
const int pressurePinFront = 39;    // Druck-Sensor vorne
const int pressurePinBack = 36;     // Druck-Sensor hinten
const int breakPin = 25;            // Bremshebel vorne/hinten
const int gpsRxPin = 16;            // GPS RX für Hardware-Serial
const int gpsTxPin = 17;            // GPS TX für Hardware-Serial
const int sdChipSelect = 32;        // SD SPI Chip-Select
const int blueLED = 4;              // Blaue LED
const int greenLED = 0;             // Grüne LED
const int yellowLED = 2;            // Gelbe LED
const int wakeUpPin = 13;             // Wake-Up-Pin



// GPS-Daten werden mittels Hardware Serial an den ESP32 übermittelt. Hierfür wird die HardwareSerial.h Bibliothek verwendet.
// Das parsen der NMEA-Strings wird von der TinyGPS++.h Bibliothek übernommen. 
// Hierfür muss ledigleich eine Instanz des TinyGPSPlus Objekts erstellt werden.

TinyGPSPlus gps;
HardwareSerial SerialGPS(2);

// Schematische Darstellung des gpsData Structs im Speicher des ESP32:
// 
// Durch optimales Anordnen der Einträge, kann automatisches Padding durch den Compiler vermieden werden.
// Die Einträge müssen einzeln oder zu mehreren immer ein vielfaches des größten Mitglieds des Structs sein.
// In diesem Fall werden die Einträge auf 8-bytes ausgerichtet, da der größte Eintrag ein double ist.
// 
// Falls es nötig sein sollte, kann der Speicherbedarf durch das nutzen von floats reduziert werden.
// Das Allignment muss dann nochmals überprüft und die Reihenfolge der Einträge gegebenenfalls geändert werden.
// Es bietet sich jedoch an für die Positionsdaten des GPS doubles zu verwenden, da ansonsten Informationen verloren gehen.
// 
//      |---------------------------------------| |---------------------------------------| --->
//      |gpsLat                                 | |gpsLon                                 | --->
//      |---------------------------------------| |---------------------------------------| --->
//      |doubel                                 | |doubel                                 | --->
//      |---------------------------------------| |---------------------------------------| --->
// ---> |-------------------|-------------------| |---------|----|----|----|----|----|----|
// ---> |gpsAlt             |gpsSpd             | |JJJJ     |MM  |TT  |hh  |mm  |ss  |Sat |
// ---> |-------------------|-------------------| |---------|----|----|----|----|----|----|
// ---> |float              |float              | |int16    |int8|int8|int8|int8|int8|int8|
// ---> |-------------------|-------------------| |---------|----|----|----|----|----|----|

struct gpsDataStruct_t {
    double          gpsLat;       //  8 byte GPS Breitengrad
    double          gpsLon;       //  8 byte GPS Längengrad
    float           gpsAlt;       //  4 byte GPS Höhe ü.n.N.
    float           gpsSpd;       //  4 byte GPS Geschwindigkeit
    uint16_t        gpsYear;      //  2 byte |
    uint8_t         gpsMonth;     //  1 byte |
    uint8_t         gpsDay;       //  1 byte |--> GPS Zeitstempel
    uint8_t         gpsHour;      //  1 byte |
    uint8_t         gpsMin;       //  1 byte |
    uint8_t         gpsSec;       //  1 byte |
    uint8_t         gpsSat;       //  1 byte GPS Satelliten-Verbindungen
} gpsData;                         // 32 byte

// Anlegen einer Objekt-Instanz für den MAX31855 Temp.-Sensor.
// Kommunikation mit ESP32 über SPI-Schnittstelle.
// Die Messung erfolgt mittels Thermoelementen.

#define MAXCS_DISC   26
#define MAXCS_LINING 27
Adafruit_MAX31855 thermocoupleDisc(MAXCS_DISC);
Adafruit_MAX31855 thermocoupleLining(MAXCS_LINING);

// Daten-Struct für die Temperaturmessung.

struct tmpDataStruct_t {
    double          tmpFrontDisc;     //  8 byte
    double          tmpFrontLining;   //  8 byte
    double          tmpAmbDisc;          //  8 byte
    double          tmpAmbLining;          //  8 byte
} tmpData;                            // 32 byte

// Das CAN-Signal wird zusammen mit der Spannungsversorgung von der Leitung zwischen Motor und Tacho abgegriffen.
// Hierfür wurde aus MCP2515 und TJA1050 ein einfacher CAN-Sniffer auf der Platine platziert,
// der die gelesenen Daten über SPI-Kommunikation an den ESP32 schickt.
// 
// In dieser Variante wird die ID 0x101 gelesen, welche Informationen über den Batteriezustand enthält.
// Weitere IDs können im Struct hinterlegt werden, und müssen dann analog zur 0x101 im Funktionsaufruf "getCanData()"
// angelegt werden.

// ACHTUNG! Muss auch bei Akku-Version bleiben, da ansonsten das Parsing fehlschlägt.


struct canDataStruct_t {
    byte            canId0x101[8];  //  8 byte
} canData;                          //  8 byte

// Das folgende Struct wird in der  

struct logDataStruct_t {
    gpsDataStruct_t gpsLogData;     // 32 byte
    tmpDataStruct_t tmpLogData;     // 32 byte
    canDataStruct_t canLogData;     //  8 byte
    uint32_t        tstmp;          //  4 byte
    uint16_t        presFront;      //  2 byte
    uint16_t        presBack;       //  2 byte
    uint32_t        rpmFront;       //  4 byte
    uint8_t         logBreak;       //  1 byte
                                    //  3 byte padding
                                    //--------
                                    // 88 byte   
} logData;

// Um die Datenmenge schnell genug auf die SD-Karte schreiben zu können, werden die einzelnen logData-Structs
// in einen von zwei Puffer geschrieben. Erreicht der aktuelle Puffer seine definierte größe "buffSize" wird er
// mit einem Mal auf die SD-Karte geschrieben. Parallel werden neue Daten auf den zweiten Puffer geschrieben.
// Die hierfür nötigen globalen Structs und State-Variablen werden im Folgenden angelegt.

// SD-Schreibgeschwindigkeit [byte/s] (Von in2p ermittelt. Kann wenn nötig vorsichtig erhöht werden)
const uint16_t sdMax = 3000;

const int buffSize = (int) (sdMax / sizeof(logData)); 

int buffCounter = 0;
logDataStruct_t buffOne[buffSize];
bool buffOneFull = 0;
bool fillBuffOne = 1;
logDataStruct_t buffTwo[buffSize];
bool buffTwoFull = 0;

// Globale Variablen für das File-Management der SD-Karte.
File logFile;
char currLogFile[11];
char currLogFolder[13];
char currLogPath[25];

// Anlegen einer Instanz des BluetoothSerial Objekts für die Live-Kommunikation mit dem
// Postprocessing/GUI am PC oder Handy.
BluetoothSerial SerialBT;

// Globale Bool-Varibale für den Bremszustand
volatile uint32_t breaking = 0;

// Globale Variabe für die Drehzahlmessung
volatile uint32_t currentRPM = 0;

// Globale Varibale für den Timeout des Loggers
volatile bool hallTriggered = false;
#define TIMEOUT 60000; // Timeout nach 1 min

// Anlegen einer Instanz des Timer Objrekts für den Timer-Interrupt
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;



/*************************************************************************************************************************************************/
/**************************************************************** FUNCTIONS **********************************************************************/
/*************************************************************************************************************************************************/



/***********************************************************/
/********************* INTERRUPTS **************************/
/***********************************************************/


// Die Interrupt Funktionen stellen den Kern der Echtzeit-Datenerfassung dar. Da die Laufzeit der Interrupt-Service-Routinen (ISR) 
// möglichst kurz sein müssen, können hier nur die wichtigsten Werte ermittelt werden. Teilweise wäre eine Echtzeiterfassung anderer
// Werte auch nicht sinnvoll, da diese eine wesenlich langsamere Aktualisierungsrate haben als die schnelle Abtastrate der Messung.
// 
// Weiterhin befinden sich alle ISRs im internen RAM des ESP32, was die Geschwindigkeit nochmals deutlich anhebt.
// Dies kann man am Attribut "IRAM_ATTR" vor dem Funktionsnamen erkennen.


// Hauptfunktion zur Erfassung der Echtzeitmesswerte und Verwaltung der Puffer
void IRAM_ATTR onTimer() {
    static uint8_t interruptCounter = 0;
    static uint16_t blinkCounter = 0;
    static bool blinking = false;
    interruptCounter++;
    blinkCounter++;

    // Folgede Messerte sind immer aktuell (Echtzeitmessung!):
    //  - Bremsdruck (vorne/hinten)
    //  - Drehzahl (vorne/hinten)
    //  - Bremsstatus (Ja/Nein)
    // Da die ISR (Interrupt-Service-Routine) nicht länger als 5ms dauern darf, werden die zeitintensiven
    // Daten nicht in Echtzeit erhoben und beim schreiben per memcpy() von den entsprechenen Adressen kopiert.
    // Dies ist eine extrem schnelle und robste Methode, da die Adressen der Structs schon zu Beginn festgelegt wurden
    // und sich diese somit auch nicht ändern können.

    
    // Falluntscheidung Bremsen / nicht Bremsen:
    // - Hohe Abtastrate (200Hz / 5ms) wenn gebremst wird.
    // - Niedrige Abtastrate (5Hz / 200ms) wenn nicht gebremst wird.     
    
    if (breaking || interruptCounter >= 40) {
        interruptCounter = 0;
                
        logData.tstmp = millis();
        memcpy((byte *) &logData.gpsLogData, (byte *) &gpsData, sizeof(gpsData));
        memcpy((byte *) &logData.tmpLogData, (byte *) &tmpData, sizeof(tmpData));

        logData.presFront = analogRead(pressurePinFront);
        logData.presBack = analogRead(pressurePinBack);
        
        memcpy((byte *) &logData.rpmFront, (byte *) &currentRPM, sizeof(currentRPM));
        memcpy((byte *) &logData.logBreak, (byte *) &breaking, sizeof(breaking));
        
        // Bufferverwaltung
        if (fillBuffOne) {
            buffOne[buffCounter] = logData;
            buffCounter++;
            
            if (buffCounter >= buffSize) {
                buffOneFull = 1;
                fillBuffOne = 0;
                buffCounter = 0;
            }
            
        } else {
            buffTwo[buffCounter] = logData;
            buffCounter++;
  
            if (buffCounter >= buffSize) {
                buffTwoFull = 1;
                fillBuffOne = 1;
                buffCounter = 0;
            }   
        }              
    }

    // Ansteuerung der Status LED
    // Langsames blinken, wenn nicht gebremst wird,
    // schnelles blinken wenn gebremst wird.
    if (blinkCounter > 25 && breaking) {
        blinkCounter = 0;
        blinking = !blinking;
        digitalWrite(blueLED, blinking);
    } else if (blinkCounter > 100) {
        blinkCounter = 0;
        blinking = !blinking;
        digitalWrite(blueLED, blinking);
    }
}


// Bremserkennungs-ISR:
void IRAM_ATTR detectBreaking() {
    bool breakSignal = digitalRead(breakPin);
    portENTER_CRITICAL(&timerMux);
    breaking = !breakSignal;
    portEXIT_CRITICAL(&timerMux);
}


// Hall-Sensor ISR:
// Zeit zwischen zwei Flanken wird aus der letzten Zeit seit einer Flanke und der aktuellen Zeit berechnent
// und in der Variable "currentRPM" gespeichet.
// Anders als bei der E-Bike Version, wird hier zusätzlich eine globale Variable hinzugenommen, die im Main-Loop
// einen Timer zurücksetzt. So kann der Logger bei inaktivität abgeschaltet werden.
void IRAM_ATTR hallInterrupt() {
    hallTriggered = true;
    uint32_t currentTime = micros();
    static bool firstCall = 1;
    static uint32_t lastTime;
    if (firstCall) {
        lastTime = currentTime;
        firstCall = 0;
    } else {
        portENTER_CRITICAL(&timerMux);
        currentRPM = currentTime - lastTime;
        portEXIT_CRITICAL(&timerMux);
        lastTime = currentTime;
    }
}
/***********************************************************/
/********************** FUNKTIONEN *************************/
/***********************************************************/


void createNewLogFile() {
    bool yellowLedState = 0;
    uint16_t ledDelay;
    uint32_t ledBlinkCount = 0;
    uint32_t gpsEncodeTimer = millis();
    uint32_t yellowLedTimer = millis();
    double startGpsTimeSignal = millis();
    uint32_t timeout = 10000; // 10 sek in ms (für Tests)
    // uint32_t timeout = 600000; // 10 min in ms

    // Im Folgenden wird auf den Zeitstempel des GPS gewartet. Dieser wird für die Erstellung der Log-Datei verwendet.
    // Nach einem Timeout von 10min (Variable "timeout") wird ein Dateiname mit fortlaufender Nummer (XXXX.BIN)
    // im Ordner /FAILED_GPS erstellt.
     
    while (gps.date.year() == 2000 && (millis() - startGpsTimeSignal) < timeout) {
        if (millis() > gpsEncodeTimer) {
            // encode Befehl alle 1000ms
            gpsEncodeTimer = millis() + 1000;
            while (SerialGPS.available() > 0) {
                gps.encode(SerialGPS.read());
            }
        }
        // Gelbe Status-LED blinkt zweimal kurz, dann eine Sekunde nicht.
        if (millis() > yellowLedTimer) {
            if (ledBlinkCount % 4 == 0) {
                ledDelay = 1000;
            } else {
                ledDelay = 100;
            }
            yellowLedTimer = millis() + ledDelay;
            yellowLedState = !yellowLedState;
            digitalWrite(yellowLED, yellowLedState);   
            ledBlinkCount++;
        }
    } digitalWrite(yellowLED, 0);
    Serial.print("Zeit Für GPS-Zeit-Signal: "); Serial.println(millis() - startGpsTimeSignal);

    // Nachfolgend wird der Dateiname für ungültigen GPS-Zeitstempel generiert.
    if (gps.date.year() == 2000) {
        String fileName;
        uint16_t fileCount = 0; 
        File root = SD.open("/FAILED_GPS");
        
        File file = root.openNextFile();
        while(file) {
            fileName = file.name();
            
            int numZero = fileName[12] - '0';
            int numOne = fileName[13] - '0';
            int numTwo = fileName[14] - '0';
            int numThree = fileName[15] - '0';
            
            uint16_t currFileCount = numZero * 1000 + numOne * 100 + numTwo * 10 + numThree + 1;
            if (currFileCount > fileCount) {
                fileCount = currFileCount; 
            }
            
            file = root.openNextFile();
        }
        
        String newCount = String(fileCount);
        while (newCount.length() < 4) {
            newCount = '0' + newCount;
        }
        
        fileName[12] = newCount[0];
        fileName[13] = newCount[1];
        fileName[14] = newCount[2];
        fileName[15] = newCount[3];
        fileName.toCharArray(currLogPath, 24);
        Serial.println(currLogPath);

    // Hier wird aus einem gültigen GPS-Zeitstempel der Ordner und der Dateiname für die log-Datei generiert.
    } else {
        String gpsStrYear = (String) gps.date.year();
        String gpsStrMonth = (String) gps.date.month();
        String gpsStrDay = (String) gps.date.day();
        String gpsStrHour = (String) gps.time.hour();
        String gpsStrMin = (String) gps.time.minute();
        String gpsStrSec = (String) gps.time.second();
    
        if (gpsStrMonth.length() < 2) {
            gpsStrMonth = '0' + gpsStrMonth;
        }
        if (gpsStrDay.length() < 2) {
            gpsStrDay = '0' + gpsStrDay;
        }
        if (gpsStrHour.length() < 2) {
            gpsStrHour = '0' + gpsStrHour;
        }
        if (gpsStrMin.length() < 2) {
            gpsStrMin = '0' + gpsStrMin;
        }
        if (gpsStrSec.length() < 2) {
            gpsStrSec = '0' + gpsStrSec;
        }
    
        // currLogFolder   0  1  2  3  4  5  6  7  8  9 10    currLogFile   0  1  2  3  4  5  6  7  8  9 10 11 12      
        // Path            0  1  2  3  4  5  6  7  8  9 10                 11 12 13 14 15 16 17 18 19 20 21 22 23      
        // Format         [/][j][j][j][j][_][m][m][_][t][t]                [/][h][h][_][m][m][_][s][s][.][b][i][n]
        
        // Code etwas ausschweifend. Kann auch eleganter gelößt werden...
        currLogFolder[0] = '/';
        currLogFolder[1] = gpsStrYear[0];
        currLogFolder[2] = gpsStrYear[1];
        currLogFolder[3] = gpsStrYear[2];
        currLogFolder[4] = gpsStrYear[3];
        currLogFolder[5] = '_';
        currLogFolder[6] = gpsStrMonth[0];
        currLogFolder[7] = gpsStrMonth[1];
        currLogFolder[8] = '_';
        currLogFolder[9] = gpsStrDay[0];
        currLogFolder[10] = gpsStrDay[1];
        Serial.print("Foldername: ");Serial.println(currLogFolder);
        
        createDir(SD, currLogFolder);
    
        currLogFile[0] = '/';
        currLogFile[1] = gpsStrHour[0];
        currLogFile[2] = gpsStrHour[1];
        currLogFile[3] = '_';
        currLogFile[4] = gpsStrMin[0];
        currLogFile[5] = gpsStrMin[1];
        currLogFile[6] = '_';
        currLogFile[7] = gpsStrSec[0];
        currLogFile[8] = gpsStrSec[1];
        currLogFile[9] = '.';
        currLogFile[10] = 'b';
        currLogFile[11] = 'i';
        currLogFile[12] = 'n';
        Serial.print("Filename: ");Serial.println(currLogFile);
    
        currLogPath[0] = currLogFolder[0];
        currLogPath[1] = currLogFolder[1];
        currLogPath[2] = currLogFolder[2];
        currLogPath[3] = currLogFolder[3];
        currLogPath[4] = currLogFolder[4];
        currLogPath[5] = currLogFolder[5];
        currLogPath[6] = currLogFolder[6];
        currLogPath[7] = currLogFolder[7];
        currLogPath[8] = currLogFolder[8];
        currLogPath[9] = currLogFolder[9];
        currLogPath[10] = currLogFolder[10];
        currLogPath[11] = currLogFile[0];
        currLogPath[12] = currLogFile[1];
        currLogPath[13] = currLogFile[2];
        currLogPath[14] = currLogFile[3];
        currLogPath[15] = currLogFile[4];
        currLogPath[16] = currLogFile[5];
        currLogPath[17] = currLogFile[6];
        currLogPath[18] = currLogFile[7];
        currLogPath[19] = currLogFile[8];
        currLogPath[20] = currLogFile[9];
        currLogPath[21] = currLogFile[10];
        currLogPath[22] = currLogFile[11];
        currLogPath[23] = currLogFile[12];
        currLogPath[24] = '\0';
        Serial.print("Pfad: ");Serial.println(currLogPath);
    }    
}

// Funktion für die Erstellung der Files und Ordner
void createDir(fs::FS &fs, const char * path){
    Serial.printf("Creating Dir: %s\n", path);
    if(fs.mkdir(path)){
        Serial.println("Dir created");
    } else {
        Serial.println("mkdir failed");
    }
}

// GPS-Daten werden abgerufen und im gpsData-Struct abgelegt.
void getGpsData() {
    while (SerialGPS.available() > 0) {
      gps.encode(SerialGPS.read());
    }
    
    gpsData.gpsLat = gps.location.lat();
    gpsData.gpsLon = gps.location.lng();
    gpsData.gpsAlt = gps.altitude.meters();
    gpsData.gpsSpd = gps.speed.kmph();
    gpsData.gpsYear = gps.date.year();
    gpsData.gpsMonth = gps.date.month();
    gpsData.gpsDay = gps.date.day();
    gpsData.gpsHour = gps.time.hour();
    gpsData.gpsMin = gps.time.minute();
    gpsData.gpsSec = gps.time.second();
    gpsData.gpsSat = gps.satellites.value();
}

// Temperatur-Werte werden vom Thermomodul abgerufen und im tmpData-Struct abgelegt.
void getTmpData() {

    double c = thermocoupleDisc.readCelsius();
    if (isnan(c)){
        tmpData.tmpFrontDisc = 0.0;
    } else {
        tmpData.tmpFrontDisc = c;
    }
    
    c = thermocoupleLining.readCelsius();
    if (isnan(c)){
        tmpData.tmpFrontLining = 0.0;
    } else {
        tmpData.tmpFrontLining = c;
    }

    tmpData.tmpAmbDisc = thermocoupleDisc.readInternal();
    tmpData.tmpAmbLining = thermocoupleLining.readInternal();
}


/*************************************************************************************************************************************************/
/****************************************************************** SETUP ************************************************************************/
/*************************************************************************************************************************************************/

void setup() {
    // Signal für DC/DC-Wandler
    digitalWrite(wakeUpPin, 1);

    pinMode(blueLED, OUTPUT);
    pinMode(yellowLED, OUTPUT);
    pinMode(greenLED, OUTPUT);

    bool blueLedState = 1;
    bool yellowLedState = 1;
    bool greenLedState = 1;
    //Status LEDs gehen an (Funktionstest)
    digitalWrite(blueLED, blueLedState);
    digitalWrite(yellowLED, yellowLedState);
    digitalWrite(greenLED, greenLedState);

    // Serial-Verbindung wird für debugging verwendet
    Serial.begin(115200);

    // Bluetooth Initialisierung
    SerialBT.begin("DATENLOGGER");

    // GPS-Serial Initialisierung
    SerialGPS.begin(9600, SERIAL_8N1, gpsRxPin, gpsTxPin);
    Serial.println("GPS initialisiert!");

    blueLedState = 0;
    yellowLedState = 0;
    greenLedState = 0;
    //Status LEDs gehen aus
    digitalWrite(blueLED, blueLedState);
    digitalWrite(yellowLED, yellowLedState);
    digitalWrite(greenLED, greenLedState);
    
    // SD-Karten Initialisierung.
    // Programm wartet auf SD-Karte, falls diese nicht eingesteckt ist.
    // Gelbe Status-LED blinkt bei fehlender SD-Karte.
    if(!SD.begin(sdChipSelect)) {
        Serial.println("Keine SD-Karte gefunden!");
        while(!SD.begin(sdChipSelect)) {
            yellowLedState = !yellowLedState;
            digitalWrite(yellowLED, yellowLedState);
        }
    } digitalWrite(yellowLED, 0);
    Serial.println("SD-Karte initialisiert!");

    // Log-File wird erzeugt.
    createNewLogFile();
    
    // Pin-Interrupt des Bremshebel-Trigers
    pinMode(breakPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(breakPin), detectBreaking, CHANGE);
    
    // Hall-Interrupt
    pinMode(hallPinFront, INPUT);
    attachInterrupt(digitalPinToInterrupt(hallPinFront), hallInterrupt, RISING);

    Serial.println("Interrupts aktiv!");    

    // Grüne Status-LED blinkt, um erfolgreichen Setup zu signalisieren
    for (int i = 0; i <= 5; i++) {
        greenLedState = !greenLedState;
        digitalWrite(greenLED, greenLedState);
        delay(200);
    }
    
    // 1. Timer-Interrupt (alle 5ms)
    // - Daten in Log-Struct speichern:
    //    ° bei jedem Interrupt, wenn gebremst wird
    //    ° bei jedem 40. Interrupt (5ms * 40 = 200ms) wenn nicht gebremst wird
    // - Log-Struct in Puffer-Struct speichern
    // - Puffer-Structs verwalten
    
    timer = timerBegin(0, 8000, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 50, true); // 5ms
    timerAlarmEnable(timer);

}

/*************************************************************************************************************************************************/
/******************************************************************* LOOP ************************************************************************/
/*************************************************************************************************************************************************/


void loop() {
    static double startTimestamp;
    static double gpsTimer = millis();
    static double tmpTimer = millis();
    static double canTimer = millis();
    static double btSerialTimer = millis();
    static double espTimeout = millis() + TIMEOUT;
    static bool yellowLedState = 0;
    
    // GPS-Abfrage alle 200ms
    if (millis() > gpsTimer) {
        gpsTimer = millis() + 200;
        getGpsData();
    }

    // Temperatur-Abfrage alle 200ms
    if (millis() > tmpTimer) {
        tmpTimer = millis() + 200;
        getTmpData();
    }
    
    // Puffer werden auf die SD-Karte geschrieben, wenn diese voll sind.
    if (buffOneFull) {
        startTimestamp = millis();
        logFile = SD.open(currLogPath, FILE_APPEND);
        buffOneFull = false;
        logFile.write((byte *)&buffOne, sizeof(buffOne));
        logFile.close();
        Serial.print("Buffer 1 geschrieben: ");
        Serial.println(millis() - startTimestamp);
        
    } else if (buffTwoFull) {
        startTimestamp = millis();
        logFile = SD.open(currLogPath, FILE_APPEND);
        buffTwoFull = false;
        logFile.write((byte *)&buffTwo, sizeof(buffTwo));
        logFile.close();
        Serial.print("Buffer 2 geschrieben: ");
        Serial.println(millis() - startTimestamp);
    }
    
    if (gps.location.isValid()) {
        yellowLedState = 1;          
    } else {
        yellowLedState = 0;
    } digitalWrite(yellowLED, yellowLedState);
    
    // Bluetooth-Kommunikaton alle 100ms.
    // Der Aktuelle Datensatz wird via Serial-Bluetooth zum PC oder Handy übermittelt.
    // Dort kann er dann auf die selbe Weise decoded werden wie die binären Rohdaten.
    if (btSerialTimer < millis()) {
        btSerialTimer = millis() + 100;
        SerialBT.write((byte *) &logData, sizeof(logData));
    }

    // Nach 60 sek ohne Hall-Interrupt, wird der Wake-Up-Pin auf Low gezogen und der DC/DC-Wandler
    // schaltet aus. Eine Reaktivierung kann durch den Bremshebel-Trigger erfolgen. 
    if (hallTriggered) {
        hallTriggered = false;
        espTimeout = millis() + TIMEOUT;
    }
    if (espTimeout < millis()) {
        yellowLedState = 0;
        bool blueLedState = 0;
        bool greenLedState = 0;

        for (int i = 0; i < 5; i++) {
            digitalWrite(yellowLED, 1);
            digitalWrite(blueLED, 1);
            digitalWrite(greenLED, 1);
        }
        digitalWrite(wakeUpPin, 0);
    }
}