

void regelaar18(){
  // PD-regeling hovercraft met ToF-sensor (of simulatie)
  const bool simulatie = false;           // Zet op true om te simuleren, false voor echte ToF meting
  const float m = 1.056;                  // massa hovercraft in kg
  const float Fmax = 0.11733;             // maximale kracht in N (bij PWM = 130)
  const float Fmin = -0.11733;

  float Kp = 4.0;
  float Kd = 2.0;

  float F = 0.0;                          // kracht
  float a = 0.0;                          // versnelling
  float v = 0.0;                          // snelheid
  float x = 0.05;                         // afstand tot muur, beginwaarde (0.05m = 5cm)
  float ref = 0.20;                       // gewenste afstand tot muur (20 cm)






    // === MEET X (afstand tot muur) ===
    if (!simulatie) {
      x = ((state.tof_right_front + state.tof_right_back) / 2.0) / 1000.0; // in meter
    }

    // === PD-berekening ===
    error = ref - x;
    float fout_afgeleide = (error - error_oud) / (dt / 1000);
    error_oud = error;

    F = Kp * error + Kd * fout_afgeleide;
    F = constrain(F, Fmin, Fmax);

    set_state.set_motor_middle_force = F; // kracht doorsturen
}