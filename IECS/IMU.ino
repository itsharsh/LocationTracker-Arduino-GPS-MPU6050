void initIMU() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
//  initMag();
  pinMode(INTERRUPT_PIN, INPUT);

  //HMC5833L
//  Wire.beginTransmission(magAddr); //start talking
//  Wire.write(0x02); // Set the Register
//  Wire.write(0x00); // Tell the HMC5883 to Continuously Measure
//  Wire.endTransmission();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

}

void getIMUData() {
//  getMagData();
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    if (mpuInterrupt && fifoCount < packetSize) {
      // try to get out of the infinite loop
      fifoCount = mpu.getFIFOCount();
    }
    // other program behavior stuff here
    // .
    // .
    // .
    // if you are really paranoid you can frequently test in between other
    // stuff to see if mpuInterrupt is true, and if so, "break;" from the
    // while() loop to immediately process the MPU data
    // .
    // .
    // .
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    fifoCount = mpu.getFIFOCount();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;


    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
}

void printIMUData() {
  // display quaternion values in easy matrix form: w x y z
  mpu.dmpGetQuaternion(&q, fifoBuffer);

  // display Euler angles in degrees
  //  mpu.dmpGetEuler(euler, &q);

  // display Euler angles in degrees
  //  mpu.dmpGetGravity(&gravity, &q);
  //  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

  // display  acceleration
  //  mpu.dmpGetAccel(&aa, fifoBuffer);
  //  mpu.dmpGetGravity(&gravity, &q);
  //  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

  //   display real acceleration, adjusted to remove gravity
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);

  convertToAcc();

  // display  gyro
  mpu.dmpGetGyro(&gyro, fifoBuffer);

#ifdef OUTPUT_READABLE_QUATERNION
  Serial.print("quat\t");
  Serial.print(q.w);  Serial.print("\t");
  Serial.print(q.x);  Serial.print("\t");
  Serial.print(q.y);  Serial.print("\t");
  Serial.print(q.z);  Serial.print("\t");
#endif

#ifdef OUTPUT_READABLE_EULER
  Serial.print("euler\t");
  Serial.print(euler[0] * 180 / M_PI);  Serial.print("\t");
  Serial.print(euler[1] * 180 / M_PI);  Serial.print("\t");
  Serial.print(euler[2] * 180 / M_PI);  Serial.print("\t");
#endif

#ifdef OUTPUT_READABLE_YAWPITCHROLL
  Serial.print("ypr\t");
  Serial.print(ypr[0] * 180 / M_PI);  Serial.print("\t");
  Serial.print(ypr[1] * 180 / M_PI);  Serial.print("\t");
  Serial.print(ypr[2] * 180 / M_PI);  Serial.print("\t");
#endif

#ifdef OUTPUT_ACCEL
  Serial.print("aa\t");
  Serial.print(aa.x);  Serial.print("\t");
  Serial.print(aa.y);  Serial.print("\t");
  Serial.print(aa.z);  Serial.print("\t");
#endif

#ifdef OUTPUT_READABLE_REALACCEL
  Serial.print("areal\t");
  Serial.print(aaReal.x);  Serial.print("\t");
  Serial.print(aaReal.y);  Serial.print("\t");
  Serial.print(aaReal.z);  Serial.print("\t");
#endif

#ifdef OUTPUT_GYRO
  Serial.print("gyro\t");
  Serial.print(gyro.x);  Serial.print("\t");
  Serial.print(gyro.y);  Serial.print("\t");
  Serial.print(gyro.z);  Serial.print("\t");
#endif

#ifdef OUTPUT_MAG
  Serial.print("mag\t");
  Serial.print(mag.x);  Serial.print("\t");
  Serial.print(mag.y);  Serial.print("\t");
  Serial.print(mag.z);  Serial.print("\t");
#endif

  Serial.println();
}

//void getMagData() {
//  Vector norm = compass.readNormalize();
//
//  // Calculate heading
//  float heading = atan2(norm.YAxis, norm.XAxis);
//
//  // Set declination angle on your location and fix heading
//  // You can find your declination on: http://magnetic-declination.com/
//  // (+) Positive or (-) for negative
//  // For Bytom / Poland declination angle is 4'26E (positive)
//  // Formula: (deg + (min / 60.0)) / (180 / M_PI);
//  float declinationAngle = (4.0 + (26.0 / 60.0)) / (180 / M_PI);
//  heading += declinationAngle;
//
//  // Correct for heading < 0deg and heading > 360deg
//  if (heading < 0)  {
//    heading += 2 * PI;
//  }
//
//  if (heading > 2 * PI)  {
//    heading -= 2 * PI;
//  }
//
//  // Convert to degrees
//  float headingDegrees = heading * 180 / M_PI;
//
//  // Output
//  Serial.print(" Heading = ");
//  Serial.print(heading);
//  Serial.print(" Degress = ");
//  Serial.print(headingDegrees);
//  Serial.println();
//}

void convertToAcc() {
  float accSensitivity = 0.061;
  aa.x *= accSensitivity;
  aa.y *= accSensitivity;
  aa.z *= accSensitivity;

  aaReal.x *= accSensitivity;
  aaReal.y *= accSensitivity;
  aaReal.z *= accSensitivity;
}
