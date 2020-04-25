/**
  \defgroup miscel Miscellaneous
  \brief Miscellaneous routines.

  @{
*/

/*! \brief Function that calculates the magnitude of the local gravity vector based upon the WGS84 gravity model.

   @param[out] g      The magnitude of the local gravity vector.
   @param[in] latitude  The latitude of the navigation system.
   @param[in] altitude  The altitude of the navigation system.

*/
inline void gravity(void) {

  precision lambda = M_PI / 180.0 * latitude; //latitude [rad]
  g = 9.780327 * (1 + 0.0053024 * (sin(lambda) * sin(lambda)) - 0.0000058 * (sin(2 * lambda) * sin(2 * lambda))) - (0.0000030877 - 0.000000004 * (sin(lambda) * sin(lambda))) * altitude + 0.000000000000072 * (altitude * altitude);
}

/*! \brief  Wrapper function that checks if a zero-velocity update should be done, and then calls all navigation algorithm
      functions that should be executed during a zero-velocity update.

  \details Wrapper function that checks if a zero-velocity update should be done, and then calls all navigation algorithm
      functions that should be executed during a zero-velocity update. The function first calls \a ZUPT_detector. If
      then flag \a zupt is set to true, it also calls the following functions:

      \li gain_matrix
      \li correct_navigation_states
      \li measurement_update
*/
/// Routine collecting the functions which need to be run to make a ZUPT update.
void zupt_update(void) {

  if (zupt)  {

    //Calculate the Kalman filter gain
    gain_matrix();

    //Correct the navigation states
    correct_navigation_states();

    //Update the covariance matrix
    measurement_update();
  }
}
