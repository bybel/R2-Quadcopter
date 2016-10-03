Adafruit_BNO055 bno = Adafruit_BNO055();
float ypr[3];

void mpu_initialisation() {


  bno.begin();
  bno.setExtCrystalUse(true);
}


void mpu_get_values() {
  imu::Vector<3> gyroscope = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  roll_angle = gyroscope.x() * 180 / 3.14159265359;
}

