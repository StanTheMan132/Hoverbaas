void regelaar19() {
  const float Kp = 0.8, Ki = 0.0, Kd = 1.5;

  static float fout_integratie = 0;
  static float vorige_fout = 0;



  float fout = state.tof_right_back - state.tof_right_front;
  fout_integratie += fout * dt;
  float fout_afgeleide = (fout - vorige_fout) / dt;
  vorige_fout = fout;

  float delta = Kp * fout + Ki * fout_integratie + Kd * fout_afgeleide;
  delta = constrain(delta, -0.5, 0.5);

  float base = 0.0;
  set_state.set_motor_one_force = base + delta;
  set_state.set_motor_two_force = base - delta;

}

