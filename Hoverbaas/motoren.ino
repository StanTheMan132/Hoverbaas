#include <Arduino.h>

// Functie om PWM te berekenen uit de motornaam en kracht in Newton
// Positieve kracht = vooruit, negatieve kracht = achteruit
int berekenPWM(String motorNaam, float krachtN) {
  float absKracht = abs(krachtN);  // Neem absolute waarde van kracht
  float a = 0.0, b = 0.0;          // Parameters voor lineaire benadering PWM = a * kracht + b

  // Bepaal richting als string (vooruit of achteruit)
  String richting = (krachtN >= 0) ? "vooruit" : "achteruit";
  String motorRichting = motorNaam + "_" + richting;

  // Stel parameters a en b in op basis van motor en richting met piecewise lineaire benadering
  // Deze waarden zijn afgeleid uit jouw experimentdata om PWM goed te schatten
  if (motorRichting == "Maxon1_vooruit") {
    if (absKracht < 0.06928) { a = 869.69; b = 0.0; }
    else if (absKracht < 0.29584) { a = 308.97; b = 38.59; }
    else { a = 166.0; b = 80.89; }
  }
  else if (motorRichting == "Maxon1_achteruit") {
    if (absKracht < 0.04984) { a = 1208.7; b = 0.0; }
    else if (absKracht < 0.21854) { a = 414.94; b = 39.32; }
    else { a = 233.02; b = 79.08; }
  }
  else if (motorRichting == "Maxon2_vooruit") {
    if (absKracht < 0.06322) { a = 949.07; b = 0.0; }
    else if (absKracht < 0.28042) { a = 322.28; b = 39.63; }
    else { a = 169.0; b = 82.61; }
  }
  else if (motorRichting == "Maxon2_achteruit") {
    if (absKracht < 0.04731) { a = 1389.21; b = 0.0; }
    else if (absKracht < 0.21716) { a = 412.13; b = 40.5; }
    else { a = 237.3; b = 78.47; }
  }
  else if (motorRichting == "DC1_vooruit") {
    if (absKracht < 0.02678) { a = 1120.24; b = 0.0; }
    else if (absKracht < 0.07205) { a = 773.14; b = 9.3; }
    else { a = 729.27; b = 12.46; }
  }
  else if (motorRichting == "DC1_achteruit") {
    if (absKracht < 0.02446) { a = 1226.49; b = 0.0; }
    else if (absKracht < 0.05677) { a = 1083.26; b = 3.5; }
    else { a = 1073.32; b = 4.07; }
  }

  // Bereken PWM waarde uit de formule: PWM = a * kracht + b
  int pwm = (int)(a * absKracht + b);
  // Constrain PWM tot max waarde afhankelijk van motor:
  // Max 255 voor Maxon motoren, max 130 voor DC motor
  if (motorNaam == "DC1") {
    return constrain(pwm, 0, 130);
  } else {
    return constrain(pwm, 0, 255);
  }
}