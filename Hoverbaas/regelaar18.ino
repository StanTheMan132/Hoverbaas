

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
  float ref = 200;                       // gewenste afstand tot muur (20 cm)

  static float error = 0.0;
  static float error_oud = 0.0;






    // === PD-berekening ===

    error = ref - (state.tof_right_front + state.tof_right_back)/2;
    float fout_afgeleide = (error - error_oud) / dt;
    error_oud = error;

    F = Kp * error + Kd * fout_afgeleide;
    F = constrain(F, Fmin, Fmax);

    set_state.set_motor_middle_force = F;  // kracht doorsturen


  
  


}