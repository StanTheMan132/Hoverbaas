void regelaar19() {
  const float Kp = 0.5, Ki = 0.0, Kd = 0.6;

  // static float fout_integratie = 0;
  static float vorige_fout = 0;


  
  // float fout = (state.tof_right_back - state.tof_right_front) / 20.0;
  float fout = (static_cast<int>(state.tof_right_back) - static_cast<int>(state.tof_right_front)) / 20.0;

  fout = (fout / 180.0) * 2.0 * 3.1415;
  // fout_integratie += fout * dt;
  float fout_afgeleide = (fout - vorige_fout) / dt;
  vorige_fout = fout;

  // float delta = Kp * fout + Ki * fout_integratie + Kd * fout_afgeleide;
  float delta = Kp * fout + Kd * fout_afgeleide;

  delta = constrain(delta, -0.5, 0.5);

  float base = 0.0;

  set_state.set_motor_one_force = base + delta;
  set_state.set_motor_two_force = base - delta;



}

