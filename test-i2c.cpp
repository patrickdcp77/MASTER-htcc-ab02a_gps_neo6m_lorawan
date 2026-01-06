#include "Arduino.h"
#include <Wire.h>

void setup() {
  Serial.begin(9600);
  while (!Serial) { delay(10); }

  Serial.println("Scan I2C en cours...");

  Wire.begin();

  for (uint8_t address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    uint8_t error = Wire.endTransmission();

    if (error == 0) {
      Serial.print("I2C device trouvé à l'adresse 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println();
    } else if (error == 4) {
      Serial.print("Erreur inconnue à l'adresse 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println();
    }
  }
  Serial.println("Scan terminé.");
}

void loop() {
  // Rien à faire ici
}