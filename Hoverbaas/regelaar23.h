#ifndef REGELAAR23_H
#define REGELAAR23_H

// struct {
//   //4 bytes
//   int stateNr = 0;
//   //4 bytes
//   float gyroDir;
//   float motor_one_force;
//   float motor_two_force;
//   float motor_middle_force;
//   float current;
//   // 2 bytes
//   uint16_t tof_front;
//   uint16_t tof_right_front;
//   uint16_t tof_right_back;
//   //1 byte
//   bool blowers;
// } state;

class regelaar {
public:
    double Kp;
    double Kd;
    double Ki;
    double error_last;
    double error_sum;
    float basis_kracht;
    float max_krachtverschil;

    regelaar(double Kp = 0.0, double Kd = 0.0, double Ki = 0.0);
    virtual ~regelaar() = default;
};

class regelaar23 : public regelaar {
public:
    regelaar23(double Kp = 0.0, double Kd = 0.0, double Ki = 0.0);
    void step(float theta, float setpoint, float dt, float& m1_force, float& m2_force); // theta, m1 en m2 van state, m1&m2 by reference
};

#endif // REGELAAR23_H
