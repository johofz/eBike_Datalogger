#include <SPI.h>
#include "Adafruit_MAX31855.h"
#include "BluetoothSerial.h"

#define MAXCS_DISC   26
#define MAXCS_LINING 27

Adafruit_MAX31855 thermocoupleDisc(MAXCS_DISC);
Adafruit_MAX31855 thermocoupleLining(MAXCS_LINING);

// Daten-Struct für die Temperaturmessung.

struct tmpDataStruct_t {
    double          tmpFrontDisc;     //  8 byte
    double          tmpFrontLining;   //  8 byte
    double          tmpAmbDisc;       //  8 byte
    double          tmpAmbLining;     //  8 byte
} tmpData;                            // 32 byte


// Anlegen einer Instanz des BluetoothSerial Objekts für die Live-Kommunikation mit dem
// Postprocessing/GUI am PC oder Handy.
BluetoothSerial SerialBT;


// Temperatur-Werte werden vom Thermomodul abgerufen und im tmpData-Struct abgelegt.
void getTmpData() {
    double c = thermocoupleDisc.readCelsius();
    if (isnan(c) || c > 500) {
        tmpData.tmpFrontDisc = 0.0;
    } else {
        tmpData.tmpFrontDisc = c;
    }
    
    c = thermocoupleLining.readCelsius();
    if (isnan(c) || c > 500) {
        tmpData.tmpFrontLining = 0.0;
    } else {
        tmpData.tmpFrontLining = c;
    }

    tmpData.tmpAmbDisc = thermocoupleDisc.readInternal();
    tmpData.tmpAmbLining = thermocoupleLining.readInternal();
}

/***************************************************************************************************************************************/
/************************************************************* SETUP *******************************************************************/
/***************************************************************************************************************************************/

void setup() {
    // Serial-Verbindung wird für debugging verwendet
    Serial.begin(115200);

    // Bluetooth Initialisierung
    SerialBT.begin("DATENLOGGER");

    delay(500)
}

/***************************************************************************************************************************************/
/************************************************************** LOOP *******************************************************************/
/***************************************************************************************************************************************/


void loop() {
    static double tmpTimer = millis();
    static double btSerialTimer = millis();

    // Temperatur-Abfrage alle 200ms
    if (millis() > tmpTimer) {
        tmpTimer = millis() + 50;
        getTmpData();
    }

    // Bluetooth-Kommunikaton alle 50.
    // Der Aktuelle Datensatz wird via Serial-Bluetooth zum PC oder Handy übermittelt.
    // Dort kann er dann auf die selbe Weise decoded werden wie die binären Rohdaten.
    if (btSerialTimer < millis()) {
        btSerialTimer = millis() + 50;
        SerialBT.write((byte *) &logData, sizeof(logData));
    }
}