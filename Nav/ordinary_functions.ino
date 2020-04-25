/**
  \defgroup main_func ZUPT aided INS functions
  \brief the main functions for running the ZUPT aided INS.
  @{
*/

/*! \brief Function that updates the IMU data buffers with the latest values read from the IMU, and writes
  the IMU data to that should be process at the current iteration to the processing variables.
  \details The function updates the IMU data buffers \a acc_buffer_x_axis, \a acc_buffer_y_axis, \a acc_buffer_z_axis, \a gyro_buffer_x_axis, \a gyro_buffer_y_axis,
  \a gyro_buffer_z_axis with the values stored in the vectors \a accelerations_in and \a angular_rates_in. The function also updates the vectors
  \a accelerations_out and \a angular_rates_out. The data stored in these vectors is the data processed in the next iteration of the navigation algorithm.

  \note This function should be called after data have been read from the IMU through the SPI interface and before the navigation algorithm is processed.

   @param[out] accelerations_out The acceleration data that is to be used in the next iteration of the navigation algorithm.
   @param[out] angular_rates_out The angular rate data that is to be used in the next iteration of the navigation algorithm.
   @param[in] accelerations_in The from the IMU latest read acceleration data.
   @param[in] angular_rates_in The from the IMU latest read angular rate data.
*/
void update_imu_data_buffers(void) {

  // The index of the IMU data buffers which the data from the IMU should be written to.
  static uint8_t in_data_buffer_index;

  // The index of the IMU data buffers which the out data should be read from.
  int16_t out_data_buffer_index;

  //Update the IMU data buffer with the latest read IMU data.
  acc_buffer_x_axis[in_data_buffer_index] = accelerations_in[0];
  acc_buffer_y_axis[in_data_buffer_index] = accelerations_in[1];
  acc_buffer_z_axis[in_data_buffer_index] = accelerations_in[2];
  gyro_buffer_x_axis[in_data_buffer_index] = angular_rates_in[0];
  gyro_buffer_y_axis[in_data_buffer_index] = angular_rates_in[1];
  gyro_buffer_z_axis[in_data_buffer_index] = angular_rates_in[2];

  /* Read the ''middle'' samples from the IMU data buffer.
    This is used to update the navigation equations at this iteration. */

  //Calculate the out data index.
  out_data_buffer_index = in_data_buffer_index - (detector_Window_size / 2);

  if (out_data_buffer_index < 0) {
    out_data_buffer_index = in_data_buffer_index + (detector_Window_size / 2) + 1;
  }

  //Update the global variables that are used in the update the navigation equations.
  accelerations_out[0] = acc_buffer_x_axis[out_data_buffer_index];
  accelerations_out[1] = acc_buffer_y_axis[out_data_buffer_index];
  accelerations_out[2] = acc_buffer_z_axis[out_data_buffer_index];
  angular_rates_out[0] = gyro_buffer_x_axis[out_data_buffer_index];
  angular_rates_out[1] = gyro_buffer_y_axis[out_data_buffer_index];
  angular_rates_out[2] = gyro_buffer_z_axis[out_data_buffer_index];

  //Update the in data buffer counter. If the in data buffer counter is equal to length of the buffer, reset the counter.
  if (in_data_buffer_index == (detector_Window_size - 1))  {
    in_data_buffer_index = 0;
  }
  else  {
    in_data_buffer_index = in_data_buffer_index + 1;
  }
}

/*! \brief Function for doing a time update of the mechanized navigation equations.


  \details When called the function does a time update of the mechanized inertial navigation system equations. That is, first the quaternions stored in \a quaternions
  is updated using the angular rate measurements in \a angular_rates_out. Then the position and velocity state vectors \a position and \a velocity are updated
  using the acceleration measurements in \a accelerations_out. The function also updates the ''aiding'' vector \a Rb2t (body to navigation coordinate system rotation matrix) which is
  used in the Kalman filter.

  \note   This is a rudimentary mechanization of the inertial navigation equations which only is suitable for
      use with systems that uses low-cost/low-performance inertial sensors and where only short periods of
      free inertial navigation is expected.

   @param[in,out] position    The position estimate of the navigation system.
   @param[in,out] velocity    The velocity estimate of the navigation system.
   @param[in,out] quaternions   The orientation estimate of the navigation system.
   @param[out] Rb2t       The body to navigation coordinate system rotation matrix estimate.
   @param[in] accelerations_out The acceleration measurements used in the update of the inertial navigation system equations.
   @param[in] angular_rates_out The angular rate measurements used in the update of the inertial navigation system equations.
   @param[in] dt          The sampling period of the system.
   @param[in] g         The magnitude of the local gravity vector.
*/
void strapdown_mechanisation_equations(void) {

  /*
    The inputs and outputs are the following:

    position - position vector in the n-frame [m]
    velocity - velocity vector in the n-frame [m]
    quaternions - quaternions describing the orientation of the navigation platform
    Rb2t - The rotation matrix Rb2t (Includes the same information as the quaternionsuaternions but we use it other functions, i.e., we saves computations)
    accelerations - Measured accelerations vector (acceleration) in the b-frame [m/s^2]
    angular_rates - Measured angular-rate vector in the b-frame [rad/s]

    The following global variables most be defined in the main function:

    g - The gravity constant [m/s^2]
    dt - The sampling period [s]

    Note that this is a rudimentary mechanization of the navigation equaternionsuations which only is suitable for
    use with systems that uses low-cost/low-performance inertial sensors and where only short periods of
    free inertial navigation is expected.
  */

  // Working variables
  vec3 an_hat;
  vec3 angular_rates_dt;
  quat_vec quat_tmp;
  precision cos_v;
  precision sin_v;

  //********** Integrate angular rates  **********//

  // If we are measuring some angular rates, update the quaternion vector.
  if ((angular_rates_out[0] != 0) | (angular_rates_out[1] != 0) | (angular_rates_out[2] != 0)) {
    // Multiply the angular rates with the sampling period dt.
    angular_rates_dt[0] = angular_rates_out[0] * dt;
    angular_rates_dt[1] = angular_rates_out[1] * dt;
    angular_rates_dt[2] = angular_rates_out[2] * dt;

    // Calculate the norm of the vector angular_rates_dt
    precision v = ( sqrt_hf( vecnorm2(angular_rates_dt, 3) ) );

    cos_v = cos(v / 2);
    sin_v = (sin(v / 2) / v);

    // Time update of the quaternions
    quat_tmp[0] = cos_v * quaternions[0] + sin_v * (angular_rates_dt[2] * quaternions[1] - angular_rates_dt[1] * quaternions[2] + angular_rates_dt[0] * quaternions[3]); // w_tb(2)*quaternions(1)-w_tb(1)*quaternions(2)+w_tb(0)*quaternions(3)
    quat_tmp[1] = cos_v * quaternions[1] + sin_v * (-angular_rates_dt[2] * quaternions[0] + angular_rates_dt[0] * quaternions[2] + angular_rates_dt[1] * quaternions[3]); //-w_tb(2)*quaternions(0)+w_tb(0)*quaternions(2)+w_tb(1)*quaternions(3)
    quat_tmp[2] = cos_v * quaternions[2] + sin_v * (angular_rates_dt[1] * quaternions[0] - angular_rates_dt[0] * quaternions[1] + angular_rates_dt[2] * quaternions[3]); //w_tb(1)*quaternions(0)-w_tb(0)*quaternions(1)+w_tb(2)*quaternions(3)
    quat_tmp[3] = cos_v * quaternions[3] + sin_v * (-angular_rates_dt[0] * quaternions[0] - angular_rates_dt[1] * quaternions[1] - angular_rates_dt[2] * quaternions[2]); //-w_tb(0)*quaternions(0)-w_tb(1)*quaternions(1)-w_tb(2)*quaternions(2)

    // Re-normalize the quaternions and update the global variable
    v = sqrt_hf(vecnorm2(quat_tmp, 4));
    quaternions[0] = quat_tmp[0] / v;
    quaternions[1] = quat_tmp[1] / v;
    quaternions[2] = quat_tmp[2] / v;
    quaternions[3] = quat_tmp[3] / v;
  }
  //*****************************************************//

  //******** Update the position and velocity *******//

  // Convert quaternions to rotation matrix
  quat2rotation(Rb2t, quaternions); //Rb2t

  // Compute acceleration in navigation coordinate frame and subtract the acceleration due to the earth gravity force.
  an_hat[0] = Rb2t[0] * accelerations_out[0] + Rb2t[1] * accelerations_out[1] + Rb2t[2] * accelerations_out[2];
  an_hat[1] = Rb2t[3] * accelerations_out[0] + Rb2t[4] * accelerations_out[1] + Rb2t[5] * accelerations_out[2];
  an_hat[2] = Rb2t[6] * accelerations_out[0] + Rb2t[7] * accelerations_out[1] + Rb2t[8] * accelerations_out[2] + g;

  // Integrate the acceleration to get the velocity
  velocity[0] = velocity[0] + an_hat[0] * dt;
  velocity[1] = velocity[1] + an_hat[1] * dt;
  velocity[2] = velocity[2] + an_hat[2] * dt;

  // Integrate the velocity to get the position
  position[0] = position[0] + velocity[0] * dt;
  position[1] = position[1] + velocity[1] * dt;
  position[2] = position[2] + velocity[2] * dt;
  //******************************************************//
}

/*! \brief Function for doing a time update of the Kalman filter state covariance.

  \details When called the function does a time update of the Kalman filter state covariance matrix stored in the vector \a cov_vector.
   @param[in,out] cov_vector    The vector representation of the Kalman filter covariance matrix.
   @param[in] dt          The sampling period of the system.
   @param[in] sigma_acceleration  The standard deviation of the accelerometer process noise.
   @param[in] sigma_gyroscope   The standard deviation of the gyroscope process noise.
   @param[in] accelerations_out The acceleration measurements used in the update of the inertial navigation system equations.
   @param[in] Rb2t        The vector current body to navigation coordinate system rotation matrix estimate.
*/
void time_up_data(void) {

  //Working variables
  uint8_t ctr = 0;
  precision dt2_sigma2_acc = (dt * dt) * (sigma_acceleration * sigma_acceleration);
  precision dt2_sigma2_gyro = (dt * dt) * (sigma_gyroscope * sigma_gyroscope);
  mat9sym ppvec;    //Temporary vector holding the update covariances
  vec3 s;       //Specific accelerations vector in the n-frame.

  // Calculate the acceleration (specific-force) vector "s" in the n-frame
  s[0] = Rb2t[0] * accelerations_out[0] + Rb2t[1] * accelerations_out[1] + Rb2t[2] * accelerations_out[2];
  s[1] = Rb2t[3] * accelerations_out[0] + Rb2t[4] * accelerations_out[1] + Rb2t[5] * accelerations_out[2];
  s[2] = Rb2t[6] * accelerations_out[0] + Rb2t[7] * accelerations_out[1] + Rb2t[8] * accelerations_out[2];

  // First row of the covariance matrix
  ppvec[0] = cov_vector[0] + dt * (2 * cov_vector[3] + dt * cov_vector[24]);
  ppvec[1] = cov_vector[1] + dt * (cov_vector[4] + cov_vector[11] + dt * cov_vector[25]);
  ppvec[2] = cov_vector[2] + dt * (cov_vector[5] + cov_vector[18] + dt * cov_vector[26]);
  ppvec[3] = cov_vector[3] + dt * (cov_vector[24] + cov_vector[8] * s[1] + cov_vector[29] * (s[1] * dt) - s[2] * (cov_vector[7] + cov_vector[28] * dt));
  ppvec[4] = cov_vector[4] + dt * cov_vector[25] + (s[2] * dt) * (cov_vector[6] + dt * cov_vector[27]) - (s[0] * dt) * (cov_vector[8] + dt * cov_vector[29]);
  ppvec[5] = cov_vector[5] + dt * (cov_vector[26] + cov_vector[7] * s[0] + cov_vector[28] * (s[0] * dt) - s[1] * (cov_vector[6] + cov_vector[27] * dt));
  ppvec[6] = cov_vector[6] + cov_vector[27] * dt;
  ppvec[7] = cov_vector[7] + cov_vector[28] * dt;
  ppvec[8] = cov_vector[8] + cov_vector[29] * dt;

  // Second row of the covariance matrix
  ppvec[9] = cov_vector[9] + dt * (2 * cov_vector[12] + cov_vector[30] * dt);
  ppvec[10] = cov_vector[10] + dt * (cov_vector[13] + cov_vector[19] + cov_vector[31] * dt);
  ppvec[11] = cov_vector[11] + dt * (cov_vector[25] + cov_vector[16] * s[1] + cov_vector[34] * (s[1] * dt) - s[2] * (cov_vector[15] + cov_vector[33] * dt));
  ppvec[12] = cov_vector[12] + cov_vector[30] * dt + s[2] * dt * (cov_vector[14] + cov_vector[32] * dt) - (s[0] * dt) * ( cov_vector[16] + cov_vector[34] * dt );
  ppvec[13] = cov_vector[13] + dt * (cov_vector[31] + cov_vector[15] * s[0] + cov_vector[33] * (s[0] * dt) - s[1] * (cov_vector[14] + cov_vector[32] * dt) );
  ppvec[14] = cov_vector[14] + cov_vector[32] * dt;
  ppvec[15] = cov_vector[15] + cov_vector[33] * dt;
  ppvec[16] = cov_vector[16] + cov_vector[34] * dt;

  // Third row of the covariance matrix
  ppvec[17] = cov_vector[17] + dt * (2 * cov_vector[20] + cov_vector[35] * dt);
  ppvec[18] = cov_vector[18] + dt * (cov_vector[26] + cov_vector[23] * s[1] + cov_vector[38] * s[1] * dt - s[2] * (cov_vector[22] + cov_vector[37] * dt));
  ppvec[19] = cov_vector[19] + cov_vector[31] * dt + s[2] * dt * (cov_vector[21] + cov_vector[36] * dt) - (s[0] * dt) * (cov_vector[23] + cov_vector[38] * dt);
  ppvec[20] = cov_vector[20] + dt * (cov_vector[35] + cov_vector[22] * s[0] + cov_vector[37] * (s[0] * dt) - s[1] * (cov_vector[21] + cov_vector[36] * dt));
  ppvec[21] = cov_vector[21] + cov_vector[36] * dt;
  ppvec[22] = cov_vector[22] + cov_vector[37] * dt;
  ppvec[23] = cov_vector[23] + cov_vector[38] * dt;

  // Forth row of the covariance matrix
  ppvec[24] = cov_vector[24] + dt * (2 * (cov_vector[29] * s[1]) + (cov_vector[44] * s[1]) * (s[1] * dt) + s[2] * (-2 * cov_vector[28] - (2 * cov_vector[43]) * (s[1] * dt) + cov_vector[42] * (s[2] * dt))) + dt2_sigma2_acc;
  ppvec[25] = cov_vector[25] + dt * (-cov_vector[29] * s[0] + cov_vector[34] * s[1] - (cov_vector[44] * s[0]) * (s[1] * dt) + s[2] * (cov_vector[27] - cov_vector[33] + cov_vector[43] * (s[0] * dt) + cov_vector[41] * (s[1] * dt) - cov_vector[40] * (s[2] * dt)));
  ppvec[26] = cov_vector[26] + cov_vector[38] * (s[1] * dt) - cov_vector[37] * (s[2] * dt) - (s[1] * dt) * (cov_vector[27] + cov_vector[41] * (s[1] * dt) - cov_vector[40] * (s[2] * dt)) + (s[0] * dt) * (cov_vector[28] + cov_vector[43] * (s[1] * dt) - cov_vector[42] * (s[2] * dt));
  ppvec[27] = cov_vector[27] + cov_vector[41] * (s[1] * dt) - cov_vector[40] * (s[2] * dt);
  ppvec[28] = cov_vector[28] + cov_vector[43] * (s[1] * dt) - cov_vector[42] * (s[2] * dt);
  ppvec[29] = cov_vector[29] + cov_vector[44] * (s[1] * dt) - cov_vector[43] * (s[2] * dt);

  // Fifth row of the covariance matrix
  ppvec[30] = cov_vector[30] + dt * (-2 * (cov_vector[34] * s[0]) + (cov_vector[44] * s[0]) * (s[0] * dt) + s[2] * (2 * cov_vector[32] - (2 * cov_vector[41]) * (s[0] * dt) + cov_vector[39] * (s[2] * dt))) + dt2_sigma2_acc;
  ppvec[31] = cov_vector[31] - cov_vector[38] * (s[0] * dt) + cov_vector[36] * (s[2] * dt) - (s[1] * dt) * (cov_vector[32] - cov_vector[41] * (s[0] * dt) + cov_vector[39] * (s[2] * dt)) + (s[0] * dt) * (cov_vector[33] - cov_vector[43] * s[0] * dt + cov_vector[40] * (s[2] * dt));
  ppvec[32] = cov_vector[32] - cov_vector[41] * s[0] * dt + cov_vector[39] * (s[2] * dt);
  ppvec[33] = cov_vector[33] - cov_vector[43] * s[0] * dt + cov_vector[40] * (s[2] * dt);

  ppvec[34] = cov_vector[34] - cov_vector[44] * (s[0] * dt) + cov_vector[41] * (s[2] * dt);

  // Sixth row of the covariance matrix
  ppvec[35] = cov_vector[35] + dt * (2 * (cov_vector[37] * s[0]) + (cov_vector[42] * s[0]) * (s[0] * dt) + s[1] * (-2 * cov_vector[36] - 2 * cov_vector[40] * s[0] * dt + cov_vector[39] * s[1] * dt)) + dt2_sigma2_acc;
  ppvec[36] = cov_vector[36] + cov_vector[40] * s[0] * dt - cov_vector[39] * s[1] * dt;
  ppvec[37] = cov_vector[37] + cov_vector[42] * s[0] * dt - cov_vector[40] * s[1] * dt;
  ppvec[38] = cov_vector[38] + cov_vector[43] * s[0] * dt - cov_vector[41] * s[1] * dt;

  // Seventh row of the covariance matrix
  ppvec[39] = cov_vector[39] + dt2_sigma2_gyro;
  ppvec[40] = cov_vector[40];
  ppvec[41] = cov_vector[41];

  // Eight row of the covariance matrix
  ppvec[42] = cov_vector[42] + dt2_sigma2_gyro;
  ppvec[43] = cov_vector[43];

  // Ninth row of the covariance matrix
  ppvec[44] = cov_vector[44] + dt2_sigma2_gyro;

  // Copy the temporary vector to the global covariance vector
  for (ctr = 0; ctr < 45; ctr++) {
    cov_vector[ctr] = ppvec[ctr];
  }
}

/*! \brief Function for calculating the Kalman filter gain matrix.

  \details When called the function calculates the Kalman filter gain and store it in the vector \a kalman_gain.
  To calculate the gain the function uses the Kalman filter covariance and the pseudo velocity measurement noise
  standard deviation.

   @param[out] kalman_gain    The vector representation of the Kalman filter gain matrix.
   @param[in] cov_vector      The vector representation of the Kalman filter covariance matrix.
   @param[in] sigma_velocity    The standard deviation of the pseudo velocity measurement error (Note: This parameters enters the function through a sub-function.).
*/
void gain_matrix(void) {

  mat3sym Re;     //Innovation matrix
  mat3sym invRe;    //Inverse of the innovation matrix

  /************ Calculate the Kalman filter innovation matrix *******************/
  innovation_cov(Re, cov_vector);

  /************ Calculate the inverse of the innovation matrix *********/
  invmat3sys(invRe, Re);

  /******************* Calculate the Kalman filter gain **************************/

  // First row of the gain matrix
  kalman_gain[0] = cov_vector[3] * invRe[0] + cov_vector[4] * invRe[1] + cov_vector[5] * invRe[2];
  kalman_gain[1] = cov_vector[3] * invRe[1] + cov_vector[4] * invRe[3] + cov_vector[5] * invRe[4];
  kalman_gain[2] = cov_vector[3] * invRe[2] + cov_vector[4] * invRe[4] + cov_vector[5] * invRe[5];

  // Second row of the gain matrix
  kalman_gain[3] = cov_vector[11] * invRe[0] + cov_vector[12] * invRe[1] + cov_vector[13] * invRe[2];
  kalman_gain[4] = cov_vector[11] * invRe[1] + cov_vector[12] * invRe[3] + cov_vector[13] * invRe[4];
  kalman_gain[5] = cov_vector[11] * invRe[2] + cov_vector[12] * invRe[4] + cov_vector[13] * invRe[5];

  // Third row of the gain matrix
  kalman_gain[6] = cov_vector[18] * invRe[0] + cov_vector[19] * invRe[1] + cov_vector[20] * invRe[2];
  kalman_gain[7] = cov_vector[18] * invRe[1] + cov_vector[19] * invRe[3] + cov_vector[20] * invRe[4];
  kalman_gain[8] = cov_vector[18] * invRe[2] + cov_vector[19] * invRe[4] + cov_vector[20] * invRe[5];

  //Forth row of the gain matrix
  kalman_gain[9] = cov_vector[24] * invRe[0] + cov_vector[25] * invRe[1] + cov_vector[26] * invRe[2];
  kalman_gain[10] = cov_vector[24] * invRe[1] + cov_vector[25] * invRe[3] + cov_vector[26] * invRe[4];
  kalman_gain[11] = cov_vector[24] * invRe[2] + cov_vector[25] * invRe[4] + cov_vector[26] * invRe[5];

  //Fifth row of the gain matrix
  kalman_gain[12] = cov_vector[25] * invRe[0] + cov_vector[30] * invRe[1] + cov_vector[31] * invRe[2];
  kalman_gain[13] = cov_vector[25] * invRe[1] + cov_vector[30] * invRe[3] + cov_vector[31] * invRe[4];
  kalman_gain[14] = cov_vector[25] * invRe[2] + cov_vector[30] * invRe[4] + cov_vector[31] * invRe[5];

  //Sixth row of the gain matrix
  kalman_gain[15] = cov_vector[26] * invRe[0] + cov_vector[31] * invRe[1] + cov_vector[35] * invRe[2];
  kalman_gain[16] = cov_vector[26] * invRe[1] + cov_vector[31] * invRe[3] + cov_vector[35] * invRe[4];
  kalman_gain[17] = cov_vector[26] * invRe[2] + cov_vector[31] * invRe[4] + cov_vector[35] * invRe[5];

  //Seventh row of the gain matrix
  kalman_gain[18] = cov_vector[27] * invRe[0] + cov_vector[32] * invRe[1] + cov_vector[36] * invRe[2];
  kalman_gain[19] = cov_vector[27] * invRe[1] + cov_vector[32] * invRe[3] + cov_vector[36] * invRe[4];
  kalman_gain[20] = cov_vector[27] * invRe[2] + cov_vector[32] * invRe[4] + cov_vector[36] * invRe[5];

  //Eight row of the gain matrix
  kalman_gain[21] = cov_vector[28] * invRe[0] + cov_vector[33] * invRe[1] + cov_vector[37] * invRe[2];
  kalman_gain[22] = cov_vector[28] * invRe[1] + cov_vector[33] * invRe[3] + cov_vector[37] * invRe[4];
  kalman_gain[23] = cov_vector[28] * invRe[2] + cov_vector[33] * invRe[4] + cov_vector[37] * invRe[5];

  //Ninth row of the gain matrix
  kalman_gain[24] = cov_vector[29] * invRe[0] + cov_vector[34] * invRe[1] + cov_vector[38] * invRe[2];
  kalman_gain[25] = cov_vector[29] * invRe[1] + cov_vector[34] * invRe[3] + cov_vector[38] * invRe[4];
  kalman_gain[26] = cov_vector[29] * invRe[2] + cov_vector[34] * invRe[4] + cov_vector[38] * invRe[5];
}

/*! \brief Function for correcting the navigation states given a zero-velocity detection.

  \details When called this function takes the velocity estimate of the navigation system and treat it as an observation of
  the current velocity error of the system. The errors in all navigation states (position, velocity, and attitude) are then
  estimated by multiplying the "velocity error" with the Kalman gain. The estimated errors in the navigation states are
  then used to correct the navigation states.

   @param[in,out] position    The position estimate of the navigation system.
   @param[in,out] velocity    The velocity estimate of the navigation system.
   @param[out]  quaternions   The orientation estimate of the navigation system.
   @param[in]   Rb2t      The vector representation of the body to navigation coordinate system rotation matrix estimate.
   @param[in]   kalman_gain   The vector representation of the Kalman filter gain matrix.
*/
void correct_navigation_states(void) {

  vec3 velocity_tmp; // Temporary vector holding the corrected velocity state.

  // Correct the position and velocity
  position[0] = position[0] - kalman_gain[0] * velocity[0] - kalman_gain[1] * velocity[1] - kalman_gain[2] * velocity[2];
  position[1] = position[1] - kalman_gain[3] * velocity[0] - kalman_gain[4] * velocity[1] - kalman_gain[5] * velocity[2];
  position[2] = position[2] - kalman_gain[6] * velocity[0] - kalman_gain[7] * velocity[1] - kalman_gain[8] * velocity[2];
  velocity_tmp[0] = velocity[0] - kalman_gain[9] * velocity[0] - kalman_gain[10] * velocity[1] - kalman_gain[11] * velocity[2];
  velocity_tmp[1] = velocity[1] - kalman_gain[12] * velocity[0] - kalman_gain[13] * velocity[1] - kalman_gain[14] * velocity[2];
  velocity_tmp[2] = velocity[2] - kalman_gain[15] * velocity[0] - kalman_gain[16] * velocity[1] - kalman_gain[17] * velocity[2];

  precision delta_roll = kalman_gain[18] * velocity[0] + kalman_gain[19] * velocity[1] + kalman_gain[20] * velocity[2];
  precision delta_pitch = kalman_gain[21] * velocity[0] + kalman_gain[22] * velocity[1] + kalman_gain[23] * velocity[2];
  precision delta_yaw = kalman_gain[24] * velocity[0] + kalman_gain[25] * velocity[1] + kalman_gain[26] * velocity[2];

  //Update the velocity state with the temporary velocity state
  velocity[0] = velocity_tmp[0];
  velocity[1] = velocity_tmp[1];
  velocity[2] = velocity_tmp[2];

  // Correct the rotation matrix
  mat3 new_rotmat;

  new_rotmat[0] = Rb2t[0] - delta_yaw * Rb2t[3] + delta_pitch * Rb2t[6];
  new_rotmat[1] = Rb2t[1] - delta_yaw * Rb2t[4] + delta_pitch * Rb2t[7];
  new_rotmat[2] = Rb2t[2] - delta_yaw * Rb2t[5] + delta_pitch * Rb2t[8];
  new_rotmat[3] = delta_yaw * Rb2t[0] + Rb2t[3] - delta_roll * Rb2t[6];
  new_rotmat[4] = delta_yaw * Rb2t[1] + Rb2t[4] - delta_roll * Rb2t[7];
  new_rotmat[5] = delta_yaw * Rb2t[2] + Rb2t[5] - delta_roll * Rb2t[8];
  new_rotmat[6] = -delta_pitch * Rb2t[0] + delta_roll * Rb2t[3] + Rb2t[6];
  new_rotmat[7] = -delta_pitch * Rb2t[1] + delta_roll * Rb2t[4] + Rb2t[7];
  new_rotmat[8] = -delta_pitch * Rb2t[2] + delta_roll * Rb2t[5] + Rb2t[8];

  // Calculate the corrected quaternions
  rotation2quat(quaternions, new_rotmat);
}

/*! \brief Function for doing a measurement update of the Kalman filter covariance.

  \details When called the function does a measurement update of the Kalman filter state covariance matrix stored in the vector \a cov_vector.
   @param[in,out] cov_vector    The vector representation of the Kalman filter covariance matrix.
   @param[in] kalman_gain     The vector representation of the Kalman filter gain matrix.
*/
void measurement_update(void) {

  uint8_t ctr = 0;
  mat9sym ppvec;    //Temporary vector holding the update covariances

  // First row
  ppvec[0] = cov_vector[0] - kalman_gain[0] * cov_vector[3] - kalman_gain[1] * cov_vector[4] - kalman_gain[2] * cov_vector[5];
  ppvec[1] = cov_vector[1] - kalman_gain[0] * cov_vector[11] - kalman_gain[1] * cov_vector[12] - kalman_gain[2] * cov_vector[13];
  ppvec[2] = cov_vector[2] - kalman_gain[0] * cov_vector[18] - kalman_gain[1] * cov_vector[19] - kalman_gain[2] * cov_vector[20];
  ppvec[3] = cov_vector[3] - kalman_gain[0] * cov_vector[24] - kalman_gain[1] * cov_vector[25] - kalman_gain[2] * cov_vector[26];
  ppvec[4] = cov_vector[4] - kalman_gain[0] * cov_vector[25] - kalman_gain[1] * cov_vector[30] - kalman_gain[2] * cov_vector[31];
  ppvec[5] = cov_vector[5] - kalman_gain[0] * cov_vector[26] - kalman_gain[1] * cov_vector[31] - kalman_gain[2] * cov_vector[35];
  ppvec[6] = cov_vector[6] - kalman_gain[0] * cov_vector[27] - kalman_gain[1] * cov_vector[32] - kalman_gain[2] * cov_vector[36];
  ppvec[7] = cov_vector[7] - kalman_gain[0] * cov_vector[28] - kalman_gain[1] * cov_vector[33] - kalman_gain[2] * cov_vector[37];
  ppvec[8] = cov_vector[8] - kalman_gain[0] * cov_vector[29] - kalman_gain[1] * cov_vector[34] - kalman_gain[2] * cov_vector[38];

  // Second row
  ppvec[9] = cov_vector[9] - kalman_gain[3] * cov_vector[11] - kalman_gain[4] * cov_vector[12] - kalman_gain[5] * cov_vector[13];
  ppvec[10] = cov_vector[10] - kalman_gain[3] * cov_vector[18] - kalman_gain[4] * cov_vector[19] - kalman_gain[5] * cov_vector[20];
  ppvec[11] = cov_vector[11] - kalman_gain[3] * cov_vector[24] - kalman_gain[4] * cov_vector[25] - kalman_gain[5] * cov_vector[26];
  ppvec[12] = cov_vector[12] - kalman_gain[3] * cov_vector[25] - kalman_gain[4] * cov_vector[30] - kalman_gain[5] * cov_vector[31];
  ppvec[13] = cov_vector[13] - kalman_gain[3] * cov_vector[26] - kalman_gain[4] * cov_vector[31] - kalman_gain[5] * cov_vector[35];
  ppvec[14] = cov_vector[14] - kalman_gain[3] * cov_vector[27] - kalman_gain[4] * cov_vector[32] - kalman_gain[5] * cov_vector[36];
  ppvec[15] = cov_vector[15] - kalman_gain[3] * cov_vector[28] - kalman_gain[4] * cov_vector[33] - kalman_gain[5] * cov_vector[37];
  ppvec[16] = cov_vector[16] - kalman_gain[3] * cov_vector[29] - kalman_gain[4] * cov_vector[34] - kalman_gain[5] * cov_vector[38];

  // Third row
  ppvec[17] = cov_vector[17] - kalman_gain[6] * cov_vector[18] - kalman_gain[7] * cov_vector[19] - kalman_gain[8] * cov_vector[20];
  ppvec[18] = cov_vector[18] - kalman_gain[6] * cov_vector[24] - kalman_gain[7] * cov_vector[25] - kalman_gain[8] * cov_vector[26];
  ppvec[19] = cov_vector[19] - kalman_gain[6] * cov_vector[25] - kalman_gain[7] * cov_vector[30] - kalman_gain[8] * cov_vector[31];
  ppvec[20] = cov_vector[20] - kalman_gain[6] * cov_vector[26] - kalman_gain[7] * cov_vector[31] - kalman_gain[8] * cov_vector[35];
  ppvec[21] = cov_vector[21] - kalman_gain[6] * cov_vector[27] - kalman_gain[7] * cov_vector[32] - kalman_gain[8] * cov_vector[36];
  ppvec[22] = cov_vector[22] - kalman_gain[6] * cov_vector[28] - kalman_gain[7] * cov_vector[33] - kalman_gain[8] * cov_vector[37];
  ppvec[23] = cov_vector[23] - kalman_gain[6] * cov_vector[29] - kalman_gain[7] * cov_vector[34] - kalman_gain[8] * cov_vector[38];

  // Forth row
  ppvec[24] = cov_vector[24] - kalman_gain[9] * cov_vector[24] - kalman_gain[10] * cov_vector[25] - kalman_gain[11] * cov_vector[26];
  ppvec[25] = cov_vector[25] - kalman_gain[9] * cov_vector[25] - kalman_gain[10] * cov_vector[30] - kalman_gain[11] * cov_vector[31];
  ppvec[26] = cov_vector[26] - kalman_gain[9] * cov_vector[26] - kalman_gain[10] * cov_vector[31] - kalman_gain[11] * cov_vector[35];
  ppvec[27] = cov_vector[27] - kalman_gain[9] * cov_vector[27] - kalman_gain[10] * cov_vector[32] - kalman_gain[11] * cov_vector[36];
  ppvec[28] = cov_vector[28] - kalman_gain[9] * cov_vector[28] - kalman_gain[10] * cov_vector[33] - kalman_gain[11] * cov_vector[37];
  ppvec[29] = cov_vector[29] - kalman_gain[9] * cov_vector[29] - kalman_gain[10] * cov_vector[34] - kalman_gain[11] * cov_vector[38];

  // Fifth row
  ppvec[30] = cov_vector[30] - kalman_gain[12] * cov_vector[25] - kalman_gain[13] * cov_vector[30] - kalman_gain[14] * cov_vector[31];
  ppvec[31] = cov_vector[31] - kalman_gain[12] * cov_vector[26] - kalman_gain[13] * cov_vector[31] - kalman_gain[14] * cov_vector[35];
  ppvec[32] = cov_vector[32] - kalman_gain[12] * cov_vector[27] - kalman_gain[13] * cov_vector[32] - kalman_gain[14] * cov_vector[36];
  ppvec[33] = cov_vector[33] - kalman_gain[12] * cov_vector[28] - kalman_gain[13] * cov_vector[33] - kalman_gain[14] * cov_vector[37];
  ppvec[34] = cov_vector[34] - kalman_gain[12] * cov_vector[29] - kalman_gain[13] * cov_vector[34] - kalman_gain[14] * cov_vector[38];

  // Sixth row
  ppvec[35] = cov_vector[35] - kalman_gain[15] * cov_vector[26] - kalman_gain[16] * cov_vector[31] - kalman_gain[17] * cov_vector[35];
  ppvec[36] = cov_vector[36] - kalman_gain[15] * cov_vector[27] - kalman_gain[16] * cov_vector[32] - kalman_gain[17] * cov_vector[36];
  ppvec[37] = cov_vector[37] - kalman_gain[15] * cov_vector[28] - kalman_gain[16] * cov_vector[33] - kalman_gain[17] * cov_vector[37];
  ppvec[38] = cov_vector[38] - kalman_gain[15] * cov_vector[29] - kalman_gain[16] * cov_vector[34] - kalman_gain[17] * cov_vector[38];

  // Seventh row
  ppvec[39] = cov_vector[39] - kalman_gain[18] * cov_vector[27] - kalman_gain[19] * cov_vector[32] - kalman_gain[20] * cov_vector[36];
  ppvec[40] = cov_vector[40] - kalman_gain[18] * cov_vector[28] - kalman_gain[19] * cov_vector[33] - kalman_gain[20] * cov_vector[37];
  ppvec[41] = cov_vector[41] - kalman_gain[18] * cov_vector[29] - kalman_gain[19] * cov_vector[34] - kalman_gain[20] * cov_vector[38];

  // Eight row
  ppvec[42] = cov_vector[42] - kalman_gain[21] * cov_vector[28] - kalman_gain[22] * cov_vector[33] - kalman_gain[23] * cov_vector[37];
  ppvec[43] = cov_vector[43] - kalman_gain[21] * cov_vector[29] - kalman_gain[22] * cov_vector[34] - kalman_gain[23] * cov_vector[38];

  // Ninth row
  ppvec[44] = cov_vector[44] - kalman_gain[24] * cov_vector[29] - kalman_gain[25] * cov_vector[34] - kalman_gain[26] * cov_vector[38];

  // Copy the temporary vector to the covariance vector
  for (ctr = 0; ctr < 45; ctr++) {
    cov_vector[ctr] = ppvec[ctr];
  }
}

/*! \brief Function for detecting when the system has zero-velocity.


  \details When called this function takes the acceleration and angular rate measurements stored in the IMU data buffers and
  runs a generalized likelihood ratio test to determine if the navigation system is stationary or moving. More information
  about the detector and it characteristics can be found in following papers:

  \li <A href="https://eeweb01.ee.kth.se/upload/publications/reports/2010/IR-EE-SB_2010_038.pdf">Zero-Velocity Detection -- An Algorithm Evaluation</A>
  \li <A href="https://eeweb01.ee.kth.se/upload/publications/reports/2010/IR-EE-SB_2010_043.pdf">Evaluation of Zero-Velocity Detectors for Foot-Mounted Inertial Navigation Systems</A>


   @param[out]  zupt          The zero-velocity detection flag
   @param[in]   detector_Window_size  The window size of the zero-velocity detector.
   @param[in]   detector_threshold    The threshold used in the detector.
   @param[in]   sigma_acc_det     The standard deviation figure used to control the importance of the accelerometer measurements in the detection algorithm.
   @param[in]   sigma_gyro_det      The standard deviation figure used to control the importance of the gyroscope measurements in the detection algorithm.
   @param[in]   g           The magnitude of the local gravity vector.
*/
void ZUPT_detector(void) {

  /************ Calculate the mean of the accelerations in the in-data buffer ***********/
  uint8_t ctr;
  vec3 acceleration_mean = {0, 0, 0}; //Mean acceleration within the data window

  for (ctr = 0; ctr < detector_Window_size; ctr++) {
    acceleration_mean[0] = acceleration_mean[0] + acc_buffer_x_axis[ctr];
    acceleration_mean[1] = acceleration_mean[1] + acc_buffer_y_axis[ctr];
    acceleration_mean[2] = acceleration_mean[2] + acc_buffer_z_axis[ctr];
  }

  acceleration_mean[0] = acceleration_mean[0] / detector_Window_size;
  acceleration_mean[1] = acceleration_mean[1] / detector_Window_size;
  acceleration_mean[2] = acceleration_mean[2] / detector_Window_size;

  /****************** Calculate the likelihood ratio  *******************************/

  precision acceleration_mean_norm = sqrt_hf(vecnorm2(acceleration_mean, 3));
  precision sigma2_acc_det = sigma_acc_det * sigma_acc_det;
  precision sigma2_gyro_det = sigma_gyro_det * sigma_gyro_det;
  vec3 acceleration_mean_normalized;
  vec3 tmp1;
  vec3 tmp2;

  acceleration_mean_normalized[0] = (g * acceleration_mean[0]) / acceleration_mean_norm;
  acceleration_mean_normalized[1] = (g * acceleration_mean[1]) / acceleration_mean_norm;
  acceleration_mean_normalized[2] = (g * acceleration_mean[2]) / acceleration_mean_norm;

  Test_statistics = 0;
  for (ctr = 0; ctr < detector_Window_size; ctr++) {

    tmp1[0] = acc_buffer_x_axis[ctr] - acceleration_mean_normalized[0];
    tmp1[1] = acc_buffer_y_axis[ctr] - acceleration_mean_normalized[1];
    tmp1[2] = acc_buffer_z_axis[ctr] - acceleration_mean_normalized[2];

    tmp2[0] = gyro_buffer_x_axis[ctr];
    tmp2[1] = gyro_buffer_y_axis[ctr];
    tmp2[2] = gyro_buffer_z_axis[ctr];

    Test_statistics = Test_statistics + vecnorm2(tmp1, 3) / sigma2_acc_det + vecnorm2(tmp2, 3) / sigma2_gyro_det;
  }
  Test_statistics = Test_statistics / detector_Window_size;


  /******************** Check if the test statistics T are below or above the detector threshold ******************/
  if (Test_statistics < detector_threshold) {
    zupt = true;
  }
  else {
    zupt = false;
  }
}
