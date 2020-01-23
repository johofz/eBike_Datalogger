#include "SD.h"
#include "FS.h"
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;
File file;
char filename[] = "/ordner/datei.bin";
const int buttonPin;
const int sdChipSelect;
volatile uint32_t buttonPressedTime = 0;
volatile bool buttonPressed = 0;


void IRAM_ATTR buttonISR() {
    uint32_t currentTime = millis()
    static uint32_t startTime;
    bool buttonState = digitalRead(buttonPin);

    if (buttonState) {
        startTime = currentTime;
    } else {
        buttonPressedTime = startTime - currentTime
        buttonPressed = 1;
    }
}


bool transferFile(char* filename) {
    file = SD.open(filename, FILE_READ)
    if (file) {
        while (file.available()) {
            Serial.write(file.read(512));
        }
        return 1
    } else {
        return 0
    }
}


void setup() {
    Serial.begin(115200);

    if(!SD.begin(sdChipSelect)) {
        Serial.println("Keine SD-Karte gefunden!");
        }
    } Serial.println("SD-Karte initialisiert!");

    pinMode(buttonPin, INPUT);
    attachInterrupt(digitalPinToInterrupt(buttonPin), buttonISR, CHANGE);
}


void loop() {
    if (buttonPressed) {
        noInterrupts();
        bool transferd = transferFile(logFileName);
        if (transferd) {
            Serial.println("Datei wurde erfolgreich übertragen!")
        } else {
            Serial.println("Fehler! Datei wurde nicht übertragen!")
        }
        interrupts();
        buttonPressedTime = 0;
        buttonPressed = 0;
    }
}