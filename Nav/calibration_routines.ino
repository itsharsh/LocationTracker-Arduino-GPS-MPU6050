/**
  \defgroup calib Calibration routines
  \brief Calibration routines. Only accelerometer calibration is implemented
  since basic gyro calibration (bias) is trivial and is available on the IMU.

  @{
*/

/*! \brief  Function that estimates the accelerometer biases given a matrix of
  the mean of the measured acceleration at different orientations.

  @param[out] max_v   Largest value of the input vector.
  @param[out] index   Index of the vector element holding the largest value.
  @param[in]  arg_vec   The input vector.
*/
void estimate_accelerometer_biases(void) {

  vec3 tmp1;          //Aiding variable
  vec3 tmp2;          //Aiding variable
  precision tmp3;       //Aiding variable
  uint8_t index;        //Variable indicating the element index of the maximum value of a vector
  precision maximum_value;  //The maximum value of a vector
  uint8_t orientation_check;  //Variable used in the check of the goodness of the user chosen orientations.

  // Set the magnitude of the gravity magnitude based upon the latitude and height of the navigation system.
  gravity();

  //*********************** Check if the excitation is sufficient *****************************************//
  /*
    This is a add-hoc way to check if the excitation to the accelerometer bias estimation algorithm is sufficient.
    Check that the projections of the gravity vector onto the axes of the accelerometer cluster during the calibration have been such that:
    1) The largest elements have been along the x,y, and z axes.
    2) The magnitude of the largest element at each orientation is close to the gravity magnitude g.

  */
  for (uint8_t orientation_ctr = 0; orientation_ctr < nr_of_calibration_orientations; orientation_ctr++) {

    tmp1[0] = absf(acceleration_mean_matrix[0][orientation_ctr]);
    tmp1[1] = absf(acceleration_mean_matrix[1][orientation_ctr]);
    tmp1[2] = absf(acceleration_mean_matrix[2][orientation_ctr]);

    max_value(&maximum_value, &index, tmp1);

    //Test if the absolute value of the largest element in the  acceleration mean vector at orientation given by "orientation_ctr" is within cos(pi/18) of g.
    if ((maximum_value - cos(M_PI / 18)*g) > 0)    {
      orientation_check |= (1 << index);
    }
  }

  // If the orientation_check variable is not equal to 7 then we should send an error message to the user
  // telling him that the estimated accelerometer bias values may be of poor quality
  if (orientation_check != 7)  {
    error_signal = ACC_CALIBRATION_ILLCONDITIONED;
  }


  //***************************** NOW LET US ESTIMATES THE BIASES *************************//

  // Reset the accelerometer biases
  accelerometer_biases[0] = 0;
  accelerometer_biases[1] = 0;
  accelerometer_biases[2] = 0;


  // Outer loop of the estimation algorithm
  for (uint8_t itr_ctr = 0; itr_ctr < 30; itr_ctr++) {

    tmp1[0] = 0;
    tmp1[1] = 0;
    tmp1[2] = 0;

    //Inner loop in the estimation algorithm
    for (uint8_t orientation_ctr = 0; orientation_ctr < nr_of_calibration_orientations; orientation_ctr++) {

      // Update the aiding variable tmp2
      tmp2[0] = acceleration_mean_matrix[0][orientation_ctr] - accelerometer_biases[0];
      tmp2[1] = acceleration_mean_matrix[1][orientation_ctr] - accelerometer_biases[1];
      tmp2[2] = acceleration_mean_matrix[2][orientation_ctr] - accelerometer_biases[2];

      //Update the cumulative sum of the "new" accelerometer bias
      tmp3 = sqrt_hf(vecnorm2(tmp2, 3));
      tmp1[0] = tmp1[0] + acceleration_mean_matrix[0][orientation_ctr] - g * (tmp2[0] / tmp3);
      tmp1[1] = tmp1[1] + acceleration_mean_matrix[1][orientation_ctr] - g * (tmp2[1] / tmp3);
      tmp1[2] = tmp1[2] + acceleration_mean_matrix[2][orientation_ctr] - g * (tmp2[2] / tmp3);

    }

    // Calculate the "new" bias estimate from the cumulative sum
    accelerometer_biases[0] = tmp1[0] / nr_of_calibration_orientations;
    accelerometer_biases[1] = tmp1[1] / nr_of_calibration_orientations;
    accelerometer_biases[2] = tmp1[2] / nr_of_calibration_orientations;
  }
}

/*! \brief Function for calibrating the accelerometer biases

  \details This function is used to calibrate the biases of the accelerometers in the IMU and requires the users to place
  the IMU into at least three different orientations. The calibration method is based upon the calibration algorithm described
  in the paper <A href="https://eeweb01.ee.kth.se/upload/publications/reports/2010/IR-EE-SB_2010_046.pdf">Calibration of the
  Accelerometer Triad of an Inertial Measurement Unit, Maximum Likelihood Estimation and Cramer-Rao Bound</A>, but does only
  calibrate the accelerometer bias.

  Before the calibration is started the flags \a new_orientation_flag and \a acc_calibration_successful_flag should be set to false.
  Then the function should called every time new IMU-data have been read from the IMU and as long as the new flags \a new_orientation_flag
  and \a acc_calibration_successful_flag are false. When the flag \a new_orientation_flag becomes true a message should be sent
  to the user, which then should place the IMU in a new orientation and reset the flag. When the flag \a acc_calibration_successful_flag
  becomes true the calibration is finished and the accelerometer calibration parameters have been written into the memory of the IMU.


  \note The function can return one error and one warning message that are stored in the variable #error_vec. The error message may
  be sent if the was no stationary during one of the calibration phases and new accelerometer data most be recorded with the IMU in
  the same orientation. A warning message may be sent if the orientations the IMU was placed in may have caused a poor estimate of
  the accelerometer biases.

   @param[out]    error_vec           A variable that is set non zero value of an error/warning has occurred during the
   @param[in,out]   acc_calibration_successful_flag A flag that should be false when calibration is started and that becomes
                            true when the calibration is finished.
   @param[in,out]   new_orientation_flag      A flag that should be false when calibration is started and that becomes
                            true when IMU should be placed in a new orientation. When the IMU has been placed in
                            a new orientation it should be set to false.
                            execution of the function.
   @param[in]     accelerations_in        The from the IMU latest read acceleration data.
*/
void calibrate_accelerometers(void) {

  static uint32_t sample_ctr = 0;
  static uint8_t orientation_ctr = 0;
  static vec3 acceleration_second_moment = {0, 0, 0};       //Vector of the second order moments (the power) of the accelerometer measurements during the calibration [(m/s^2)^2]
  vec3 acceleration_variance;                     //Vector of the variances of of the accelerometer measurements during the calibration [(m/s^2)^2]
  precision tmp;

  // While the number of samples taken at the current orientation are less then the value specified. Do the following
  if (sample_ctr < nr_of_calibration_samples) {

    tmp = ((precision)(sample_ctr)) / (sample_ctr + 1);

    // Update the recursively calculated mean of the acceleration measurements
    acceleration_mean_matrix[0][orientation_ctr] = tmp * acceleration_mean_matrix[0][orientation_ctr] + accelerations_in[0] / (sample_ctr + 1);
    acceleration_mean_matrix[1][orientation_ctr] = tmp * acceleration_mean_matrix[1][orientation_ctr] + accelerations_in[1] / (sample_ctr + 1);
    acceleration_mean_matrix[2][orientation_ctr] = tmp * acceleration_mean_matrix[2][orientation_ctr] + accelerations_in[2] / (sample_ctr + 1);

    // Update the recursively calculated variance of the acceleration measurements
    acceleration_second_moment[0] = tmp * acceleration_second_moment[0] + (accelerations_in[0] * accelerations_in[0]) / (sample_ctr + 1);
    acceleration_second_moment[1] = tmp * acceleration_second_moment[1] + (accelerations_in[1] * accelerations_in[1]) / (sample_ctr + 1);
    acceleration_second_moment[2] = tmp * acceleration_second_moment[2] + (accelerations_in[2] * accelerations_in[2]) / (sample_ctr + 1);

    //Update the sample counter
    sample_ctr = sample_ctr + 1;
  }

  // If the number of samples taken at the current orientation are equal to the value specified. Do the following
  if (sample_ctr == nr_of_calibration_samples)
  {
    //Reset the sample counter
    sample_ctr = 0;

    //Calculate the variance
    acceleration_variance[0] = acceleration_second_moment[0] - acceleration_mean_matrix[0][orientation_ctr] * acceleration_mean_matrix[0][orientation_ctr];
    acceleration_variance[1] = acceleration_second_moment[1] - acceleration_mean_matrix[1][orientation_ctr] * acceleration_mean_matrix[1][orientation_ctr];
    acceleration_variance[2] = acceleration_second_moment[2] - acceleration_mean_matrix[2][orientation_ctr] * acceleration_mean_matrix[2][orientation_ctr];

    //Check that IMU has been stationary during the calibration. If so do the following
    if ((acceleration_variance[0] < acceleration_variance_threshold) & (acceleration_variance[1] < acceleration_variance_threshold) & (acceleration_variance[2] < acceleration_variance_threshold)) {
      //Check if this was the last orientation in the calibration procedure, then do the following
      if (orientation_ctr == (nr_of_calibration_orientations - 1))      {

        // Estimate the bias estimation algorithm
        estimate_accelerometer_biases();

        //Set the flag that signals that the calibration was successful.
        acc_calibration_finished_flag = true;

        //Reset the orientation counter
        orientation_ctr = 0;
      }
      else      {

        //Send a message to the user that data was successfully calculated and the IMU should be place in a new orientation
        new_orientation_flag = true;

        //Update the orientation counter
        orientation_ctr = orientation_ctr + 1;
      }
    }
    else    {

      //Send a message to the user that the IMU wasn't stationary during the data collection at the current orientation
      error_signal = ACC_STATIONARITY_ERROR;
    }
  }
}
