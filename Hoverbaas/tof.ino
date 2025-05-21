#include "Adafruit_VL53L0X.h"
#include <math.h>

#define XSHUT1 47
#define XSHUT2 48
#define XSHUT3 49

Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();


void setup_tof(){
  // Zet alle XSHUT-pinnen op OUTPUT en initieel LOW (uit)
  pinMode(XSHUT1, OUTPUT);
  pinMode(XSHUT2, OUTPUT);
  pinMode(XSHUT3, OUTPUT);

  digitalWrite(XSHUT1, LOW);
  digitalWrite(XSHUT2, LOW);
  digitalWrite(XSHUT3, LOW);
  delay(1);

  // Sensor 1 aanzetten en adres toewijzen
  digitalWrite(XSHUT1, HIGH);
  delay(10);
  if (!lox1.begin()) {
    Serial.println(F("Sensor 1 niet gevonden!"));
    while (1);
  }
  lox1.setAddress(0x30);  // Nieuw adres

  // Sensor 2 aanzetten en adres toewijzen
  digitalWrite(XSHUT2, HIGH);
  delay(10);
  if (!lox2.begin()) {
    Serial.println(F("Sensor 2 niet gevonden!"));
    while (1);
  }
  lox2.setAddress(0x31);  // Nieuw adres

  // Sensor 3 aanzetten en adres toewijzen
  digitalWrite(XSHUT3, HIGH);
  delay(10);
  if (!lox3.begin()) {
    Serial.println(F("Sensor 3 niet gevonden!"));
    while (1);
  }
  lox3.setAddress(0x32);  // Nieuw adres

  Serial.println("Alle VL53L0X sensoren zijn opgestart en hebben unieke I2C-adressen.");
}