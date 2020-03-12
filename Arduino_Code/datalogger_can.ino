#include "mcp_can.h"
#include "BluetoothSerial.h"
#include <SPI.h>


// Pindefinition für ESP-WROOM-32 DEV KIT
const int canChipSelect = 13;       // CAN SPI Chip-Select


MCP_CAN CAN(canChipSelect);

unsigned char len = 0;
unsigned char buf[8];

struct canDataStruct_t {
    byte            canId0x101[8];  //  8 byte
} canData;                          //  8 byte


// Anlegen einer Instanz des BluetoothSerial Objekts für die Live-Kommunikation mit dem
// Postprocessing/GUI am PC oder Handy.
BluetoothSerial SerialBT;


// CAN ISR
// Wir im Moment nicht verwendet, da es zu Konflikten mit der Haupt-ISR kam
void IRAM_ATTR canInterrupt() {
    while (CAN.checkReceive() == CAN_MSGAVAIL) {
        CAN.readMsgBuf(&len, buf);
        portENTER_CRITICAL(&timerMux);
        memcpy((byte *) &canData, (byte *) &buf, sizeof(buf));
        portEXIT_CRITICAL(&timerMux);
    }
}


// Can-Buffer wird ausgelesen und nach den gesuchten IDs mit Logik gefiltert.
// Kann auch mittels Hardware Masken und Filtern im Setup eingestellt werden.
void getCanData() {
    while (CAN.checkReceive() == CAN_MSGAVAIL) {
        CAN.readMsgBuf(&len, buf);
        unsigned long canId = CAN.getCanId();
        if (canId == 0x101) {
            for (int i = 0; i < sizeof(buf); i++) {
                canData.canId0x101[i] = (byte)buf[i];
            }
        }
    }
}


/***************************************************************************************************************************************/
/************************************************************* SETUP *******************************************************************/
/***************************************************************************************************************************************/

void setup() {

    // Serial-Verbindung wird für debugging verwendet
    Serial.begin(115200);
    
    // Bluetooth Initialisierung
    SerialBT.begin("DATENLOGGER");
    
    // CAN Initialisierung
    while (CAN.begin(CAN_500KBPS) != CAN_OK) {
        delay(100);
    } Serial.println("CAN-Sniffer initialisiert!");

    // CAN-Maske, CAN-Filter
    CAN.init_Mask(0, 0, 0x3ff);
    CAN.init_Mask(1, 0, 0x3ff);
    CAN.init_Filt(0, 0, 0x101);

    // Can-Interrupt (Im Moment nicht verwendet!)
    // pinMode(canIntPin, INPUT);
    // attachInterrupt(digitalPinToInterrupt(canIntPin), canInterrupt, FALLING);

    delay(500)
}

/***************************************************************************************************************************************/
/************************************************************** LOOP *******************************************************************/
/***************************************************************************************************************************************/

void loop() {
    static double canTimer = millis();
    static double btSerialTimer = millis();

    // CAN-Abfrage alle 200ms
    if (millis() > canTimer) {
        canTimer = millis() + 200;
        getCanData();
    }
    
    // Bluetooth-Kommunikaton alle 50ms.
    // Der Aktuelle Datensatz wird via Serial-Bluetooth zum PC oder Handy übermittelt.
    // Dort kann er dann auf die selbe Weise decoded werden wie die binären Rohdaten.
    if (btSerialTimer < millis()) {
        btSerialTimer = millis() + 50;
        SerialBT.write((byte *) &canData, sizeof(canData));
    }
}