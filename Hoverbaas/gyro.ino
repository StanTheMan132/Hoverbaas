

void setup_gyro(){
    Wire.begin();
 
    gyroSensor.setWire(&Wire);
    gyroSensor.beginAccel();
    gyroSensor.beginGyro();
    gyroSensor.beginMag();
}