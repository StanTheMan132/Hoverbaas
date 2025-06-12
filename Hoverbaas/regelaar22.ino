

void regelaar22(){
    float Kp = 9.0;
    float Kd = 4.2;
    float Ki = 0.0;
    float sp = 300; // setpoint in meter (300 mm)
    float error;
    float Fx;
    static float error_oud = 0.0, error_som = 0.0;
  error = sp - state.tof_front;
  error_som += error * dt;
  Fx = Kp * error + Kd * (error - error_oud) / dt + Ki * error_som;
  error_oud = error;

  // begrens kracht
  Fx = constrain(Fx, -0.5, 0.5);

  set_state.set_motor_one_force = -Fx; // motor aansturen met kracht Fx
  set_state.set_motor_two_force = -Fx; // motor aansturen met kracht Fx
}