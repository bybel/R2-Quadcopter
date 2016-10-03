MPU6050 mpu;
uint8_t mpuIntStatus;
uint16_t packetSize;
uint16_t fifoCount;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];


void mpu_initialisation() {


  Wire.begin();
  TWBR = 24;
  mpu.initialize();
  mpu.dmpInitialize();
  mpu.setXAccelOffset(-1343);
  mpu.setYAccelOffset(-1155);
  mpu.setZAccelOffset(1033);
  mpu.setXGyroOffset(65);
  mpu.setYGyroOffset(-10);
  mpu.setZGyroOffset(25);
  mpu.setDMPEnabled(true);
  packetSize = mpu.dmpGetFIFOPacketSize();
}


void mpu_get_values() {
  while (fifoCount < packetSize) {
    fifoCount = mpu.getFIFOCount();
  }

  if (fifoCount == 1024) mpu.resetFIFO();

  else {
    fifoCount = mpu.getFIFOCount();

    mpu.getFIFOBytes(fifoBuffer, packetSize);


    mpu.resetFIFO();

    fifoCount -= packetSize;


    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    roll_angle = ypr[1] * 180 / PI;

  }


}

