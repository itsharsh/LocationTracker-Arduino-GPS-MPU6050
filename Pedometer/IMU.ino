//void writeTo(byte device_address, byte register_address, byte val) {
//  Wire.beginTransmission(device_address);
//  Wire.write(register_address);
//  Wire.write(val);
//  Wire.endTransmission();
//}
//
//void initIMU() {
//  Wire.begin();
//
//  writeTo(MPU_add, 0x6B, 0x00); //initializing a reset
//  writeTo(MPU_add, 0x6B, 0x03); //initializing the power register
//  writeTo(MPU_add, 0x1B, 0x00); //configuring gyroscope
//  writeTo(MPU_add, 0x1C, 0x00); //configuring accerometer
//  writeTo(MPU_add, 0x1A, 0x00); //DLPF (Digital Low Pass Filter)
//  compass.init();
//  compass.setSamplingRate(50);
//
//  //configuring magnetometer
//  //  Wire.beginTransmission(mag_add);
//  //  Wire.write(0x0B);
//  //  Wire.write(0x01);
//  //  Wire.endTransmission(true);
//  //  Wire.beginTransmission(mag_add);
//  //  Wire.write(0x09);
//  //  Wire.write(0x1D);
//  //  Wire.endTransmission();
//}
//
//void getDataFromIMU(byte device_address, byte register_address, int16_t data_raw[], int16_t data_raw_prev[], float data[], float gain, float offset[], int range) {
//  Wire.beginTransmission(device_address);
//  Wire.write(register_address);
//  Wire.endTransmission(false);
//  Wire.requestFrom(MPU_add, 6, true);
//  for (int i = 0; i < 3; i++) {
//    data_raw_prev[i] = data_raw[i];
//    data_raw[i] =  (Wire.read() << 8 | Wire.read());
//    data_raw[i] = (gain * data_raw_prev[i]) + ((1 - gain) * data_raw[i]);
//    data[i] = (float) (data_raw[i] - offset[i]) / afs_range;
//  }
//}
//
//void getMagData() {
//  int16_t mtemp;
//  for (int i = 0; i < 3; i++) {
//    mag_raw_prev[i] = mag_raw[i];
//  }
//  compass.readRaw(&mag_raw[0], &mag_raw[1], &mag_raw[2], &mtemp);
//  for (int i = 0; i < 3; i++) {
//    mag_raw[i] = (mag_gain * mag_raw_prev[i]) + ((1 - mag_gain) * mag_raw[i]);
//    mag[i] = mag_raw[i];
//  }
//  //  x_mag_raw_prev = x_mag_raw;
//  //  y_mag_raw_prev = y_mag_raw;
//  //  z_mag_raw_prev = z_mag_raw;
//  //  compass.readRaw(&x_mag_raw, &y_mag_raw, &z_mag_raw, &mtemp);
//  //  x_mag_raw = (mag_gain * x_mag_raw_prev) + ((1 - mag_gain) * x_mag_raw);
//  //  y_mag_raw = (mag_gain * y_mag_raw_prev) + ((1 - mag_gain) * y_mag_raw);
//  //  z_mag_raw = (mag_gain * z_mag_raw_prev) + ((1 - mag_gain) * z_mag_raw);
//
//
//
//  heading_prev = heading;
//  heading = compass.readHeading();
//  heading = (mag_gain * heading_prev) + ((1 - mag_gain) * heading);
//}
//
//void getTempData() {
//  Wire.beginTransmission(MPU_add);
//  Wire.write(0x41);
//  Wire.endTransmission(false);
//  Wire.requestFrom(MPU_add, 2, true);
//  int16_t tempFull = Wire.read() << 8 | Wire.read();
//  temp_prev = temp;
//  temp = (temp_gain * temp_prev) + ((1 - temp_gain) * tempFull);
//  temp = (float) temp / 340.0 + 36.53;
//}
//
//void calibrateIMU() {
//  int readings = 1000;
//  for (int i = 0 ; i < readings; i++) {
//    if (i % 200 == 0) Serial.println("Calculating .....");
//    getDataFromIMU(MPU_add, acc_add, acc_raw, acc_raw_prev, acc, acc_gain, acc_offset, afs_range);        //Accelerometer Data
//    getDataFromIMU(MPU_add, gyro_add, gyro_raw, gyro_raw_prev, gyro, gyro_gain, gyro_offset, gfs_range);  //Gyroscope Data
//    delay(10);
//
//    for (int i = 0; i < 3; i++) {
//      cal_acc[i] += acc[i];
//      cal_gyro[i] += gyro[i];
//    }
//  }
//  for (int i = 0; i < 3; i++) {
//    cal_acc[i] /= readings;
//    cal_gyro[i] /= readings;
//  }
//  Serial.print("End of Calculation");
//  Serial.print("\nAcc:\t");
//  for (int i = 0; i < 3; i++) {
//    Serial.print(cal_acc[i]); Serial.print("\t");
//  }
//  Serial.print("\nGyro:\t");
//  for (int i = 0; i < 3; i++) {
//    Serial.print(cal_gyro[i]); Serial.print("\t");
//  }
//  Serial.println();
//}
//
//void printArray(float data[], int array_length) {
//  for (int i = 0; i < array_length; i++) {
//    Serial.print(data[i]); Serial.print("\t");
//  }
//}
//
//void printTempData() {
//  Serial.print("Temp\t");
//  Serial.print(temp); Serial.print("\t");
//}
