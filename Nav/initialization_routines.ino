/**
  \defgroup init Initialization routines
  \brief Routines for initializing the system. Only coarse initial alignment is implemented

  @{
*/
/*! \brief Function for initializing the navigation algorithm.

  \details This function initializes the navigation algorithm and should during the initialization of the
  navigation system be called every time new IMU-data have been read from the IMU. Before the initialization is
  started the flag \a initialize_flag should be set to true, and the counter \a initialize_sample_ctr to zero.
  The initialization is finished when the flag \a initialize_flag becomes false.

  The initialization function first runs an initial alignment of the navigation system, where the
  roll and pitch are estimated from the average of the accelerometer readings. Then, the function sets the
  initial navigation states (position, velocity, and quaternions) and the initial Kalman filter covariance.

  \note The navigation system most be stationary during the initialization, and the number of samples used in the
      initial alignment most be larger than the length of the zero-velocity detector window.

   @param[out]  position            The position estimate of the navigation system.
   @param[out]  velocity            The velocity estimate of the navigation system.
   @param[out]  quaternions           The orientation estimate of the navigation system.
   @param[out]  cov_vector            The vector representation of the Kalman filter covariance matrix.
   @param[in,out] initialize_flag         A flag that should be set to true when initialization is started and that becomes false when the initialization is finished.
   @param[in]   nr_of_inital_alignment_samples  The number of samples used in the initial alignment.
   @param[in]   initial_heading         The initial heading of the navigation system.
   @param[in]   initial_pos           The initial position of the navigation system.
   @param[in]   sigma_initial_position      The standard deviations of the uncertainties in the initial position.
   @param[in]   sigma_initial_velocity      The standard deviations of the uncertainties in the initial velocity.
   @param[in]   sigma_initial_attitude      The standard deviations of the uncertainties in the initial attitude.

*/
void initialize_navigation_algorithm(void) {

  // Counter that counts the number of initialization samples that have been processed.
  static uint8_t initialize_sample_ctr;

  // Mean acceleration vector used in the initial alignment.
  vec3 acceleration_mean;

  // Reset the mean if we start a new initialization
  if (initialize_sample_ctr == 0)  {
    acceleration_mean[0] = 0;
    acceleration_mean[1] = 0;
    acceleration_mean[2] = 0;
  }

  //Calculate the cumulative sum of the accelerometer readings until we have the number
  //of samples specified in the initial alignment settings
  if (initialize_sample_ctr < nr_of_inital_alignment_samples) {

    //Cumulative sum

    acceleration_mean[0] = acceleration_mean[0] + accelerations_in[0];
    acceleration_mean[1] = acceleration_mean[1] + accelerations_in[1];
    acceleration_mean[2] = acceleration_mean[2] + accelerations_in[2];

    //Increase the counter
    initialize_sample_ctr = initialize_sample_ctr + 1;
  }


  // If we are at the last iteration of the initial alignment, do this:
  if (initialize_sample_ctr == nr_of_inital_alignment_samples) {

    vec3 initial_attitude;
    /************* Initialize the navigation states *************/

    // Calculate the mean acceleration from the cumulative sum.
    acceleration_mean[0] = acceleration_mean[0] / nr_of_inital_alignment_samples;
    acceleration_mean[1] = acceleration_mean[1] / nr_of_inital_alignment_samples;
    acceleration_mean[2] = acceleration_mean[2] / nr_of_inital_alignment_samples;

    //Calculate the roll and pitch
    initial_attitude[0] = atan2(-acceleration_mean[1], -acceleration_mean[2]); //roll
    initial_attitude[1] = atan2(acceleration_mean[0], sqrt_hf((acceleration_mean[1] * acceleration_mean[1]) + (acceleration_mean[2] * acceleration_mean[2]))); //pitch

    //Set the initial heading
    initial_attitude[2] = initial_heading;

    // Calculate the initial rotation matrix, used as temporary variable in the calculation of the initial quaternions
    mat3 initial_rotmat;
    euler2rotation(initial_rotmat, initial_attitude);

    //Set the initial quaternions using the initial rotation matrix
    rotation2quat(quaternions, initial_rotmat);

    //Set the initial velocity (must be zero)
    velocity[0] = 0;
    velocity[1] = 0;
    velocity[2] = 0;

    //Set the initial position
    position[0] = initial_pos[1];
    position[1] = initial_pos[1];
    position[2] = initial_pos[2];

    // Set the gravity magnitude based upon the latitude and height of the navigation platform.
    gravity();

    /*************************************************************/

    /************** Initialize the filter covariance *************/
    cov_vector[0] = sigma_initial_position[0] * sigma_initial_position[0];
    cov_vector[9] = sigma_initial_position[1] * sigma_initial_position[1];
    cov_vector[17] = sigma_initial_position[2] * sigma_initial_position[2];

    cov_vector[24] = sigma_initial_velocity[0] * sigma_initial_velocity[0];
    cov_vector[30] = sigma_initial_velocity[1] * sigma_initial_velocity[1];
    cov_vector[35] = sigma_initial_velocity[2] * sigma_initial_velocity[2];

    cov_vector[39] = sigma_initial_attitude[0] * sigma_initial_attitude[0];
    cov_vector[42] = sigma_initial_attitude[1] * sigma_initial_attitude[1];
    cov_vector[44] = sigma_initial_attitude[2] * sigma_initial_attitude[2];
    /*************************************************************/

    //Reset the initialization ctr
    initialize_sample_ctr = 0;

    //Turn of the initialization flag
    initialize_flag = false; 
  }
}
