#include "Adafruit_VL53L0X.h"


#define maxon1_pin 11
#define maxon2_pin 9
#define dc_motor_pin 5

#define XSHUT1 47
#define XSHUT2 48
#define XSHUT3 49

#define INT_DIEPONTLADING 2
#define RELAIS_BLOWERS    3
#define RELAIS_SDC        4

Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox3 = Adafruit_VL53L0X();


struct {
  //4 bytes
  int stateNr = 0;
  //4 bytes
  float gyroDir;
  float motor_one_force;
  float motor_two_force;
  float motor_middle_force;
  float current;
  // 2 bytes
  uint16_t tof_front;
  uint16_t tof_right_front;
  uint16_t tof_right_back;
  //1 byte
  bool blowers;
} state;

struct {
  int stateNr;
  float set_motor_one_force = 0.6;
  float set_motor_two_force = 0.6;
  float set_motor_middle_force;
  bool set_blowers;
} set_state;

char recievedChar;
uint32_t prevSerial;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  // Serial.println("Starting");
  setup_tof();
  // Serial.println("Done TOF");
  pinMode(maxon1_pin, OUTPUT);
  pinMode(maxon2_pin, OUTPUT);
  // pinMode(dc_motor_pin, OUTPUT);
  pinMode(6, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(INT_DIEPONTLADING), diepontladingInterrupt, FALLING);
  pinMode(RELAIS_BLOWERS, OUTPUT);
  pinMode(RELAIS_SDC, OUTPUT);
  digitalWrite(RELAIS_BLOWERS, HIGH);
  digitalWrite(RELAIS_SDC, HIGH);
}

void diepontladingInterrupt() {
  //Serial.println("Celspanning < 3V: relais worden uitgeschakeld.");
  digitalWrite(RELAIS_BLOWERS, LOW);
  digitalWrite(RELAIS_SDC, LOW);
}

void update_state(){
  lox1.startRangeContinuous();
  lox2.startRangeContinuous();
  lox3.startRangeContinuous();
  state.motor_one_force = set_state.set_motor_one_force;
  state.motor_two_force = set_state.set_motor_two_force;
  state.current = 0;
  state.blowers = set_state.set_blowers;
  while(!lox1.isRangeComplete()) {
    delay(1);
  }
  state.tof_right_back = lox1.readRange();
  while(!lox2.isRangeComplete()) {
    delay(1);
  }
  state.tof_front = lox2.readRange();
  while(!lox3.isRangeComplete()) {
    delay(1);
  }
  state.tof_right_front = lox3.readRange();

}

void update_hardware(){
  digitalWrite(RELAIS_BLOWERS, set_state.set_blowers);

  analogWrite(maxon1_pin, 255 - berekenPWM("Maxon1", set_state.set_motor_one_force));
  analogWrite(maxon2_pin, 255 - berekenPWM("Maxon2", set_state.set_motor_two_force));
  analogWrite(dc_motor_pin, berekenPWM("DC", set_state.set_motor_middle_force));
}

void update_dashboard(){
  if(Serial.available() > 0){
    recievedChar = Serial.read();
    switch(recievedChar){
      case '1':
      state.stateNr = 1;
      break;
      case '2':
      state.stateNr = 18;
      break;
      case '3':
      state.stateNr = 19;
      break;
      case '4':
      state.stateNr = 20;
      break;
      case '5':
      state.stateNr = 21;
      break;
      case '6':
      state.stateNr = 22;
      break;
      case '7':
      state.stateNr = 23;
      break;
      case '8':
      state.stateNr=24;
    }
  }
  if (millis() - prevSerial > 100){
    Serial.write(0xAA);
    Serial.write((uint8_t *)&state, sizeof(state));
    prevSerial = millis();
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  // Serial.println("UPDATING STATE");
  update_state();
  // Serial.println("UPDATING dashboard");
  update_dashboard();
  // Serial.println("UPDATING hardware");
  update_hardware();

  switch(state.stateNr){
  case 0:
  //start-up
  break;
  case 1:
  //stand-by
  set_state.set_motor_one_force = 0.0;
  set_state.set_motor_two_force = 0.0;
  break;
  case 18:
  //afmeren tof
  regelaar18();
  set_state.set_motor_one_force = 0.6;
  set_state.set_motor_two_force = 0.6;
  break;
  case 19:
  //parallel tof
  set_state.set_blowers = 1;
  break;
  case 20: 
  //parallel gyro
  set_state.set_blowers = 0;
  // digitalWrite(2, 0);
  break;
  case 21:
  //vooruit
  break;
  case 22:
  //stoppen
  break;
  case 23:
  //90 graden rotatie
  break;
  case 24:
  //rpi
  break;

  }

}
