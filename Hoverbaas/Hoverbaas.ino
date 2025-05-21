#define maxon1_pin 1
#define maxon2_pin 2
#define dc_motor_pin 3

struct {
  //4 bytes
  int stateNr = 5;
  //4 bytes
  float gyroDir;
  float tof_front;
  float tof_right_front;
  float tof_right_back;
  float motor_one_force;
  float motor_two_force;
  float motor_middle_force;
  float current;
  //1 byte
  bool blowers;
} state;

struct {
  int stateNr;
  float set_motor_one_force;
  float set_motor_two_force;
  float set_motor_middle_force;
  bool set_blowers;
} set_state;

char recievedChar;
bool blowers_enabled = false;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  setup_tof();
  pinMode(52, OUTPUT);

  
}

void update_state(){

}

void update_hardware(){
  digitalWrite(52, blowers_enabled);

  analogWrite(maxon1_pin, berekenPWM("maxon1", set_state.set_motor_one_force));
  analogWrite(maxon2_pin, berekenPWM("maxon1", set_state.set_motor_two_force));
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
  if (millis() % 300 == 0){
    Serial.write(0xAA);
    Serial.write((uint8_t *)&state, sizeof(state));
  }
}

void loop() {
  // put your main code here, to run repeatedly:
  update_state();
  update_dashboard();
  update_hardware();

  switch(state.stateNr){
  case 0:
  //start-up
  break;
  case 1:
  //stand-by
  break;
  case 18:
  //afmeren tof
  break;
  case 19:
  //parallel tof
  break;
  case 20: 
  //parallel gyro
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
