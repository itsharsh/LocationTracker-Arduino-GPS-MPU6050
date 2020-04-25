#include "nav_util.h"
//
//#include <Wire.h>
//#include <QMC5883L.h>
//
//#define MPU_add 0x68
//#define acc_add 0x3B
//#define gyro_add 0x43
////#define mag_add 0x0D
//
//#define afs_range 16384         //for +- 2g, 16384
//#define gfs_range 131           //131
//#define mfs_range 3000          //3000
//
//#define acc_gain 0            // 1 means dependency on prev data
//#define gyro_gain 0
//#define mag_gain 0
//#define temp_gain 0
//
//float pitch, roll, yaw;
//float deltat;
//
//float acc_offset[3] = {0, 0, 0}, gyro_offset[3] = {0, 0, 0}, mag_offset[3] = {0, 0, 0};
//float acc[3], gyro[3], mag[3], velocity[3], dist[3];
//float cal_acc[3], cal_gyro[3];
//float acc_prev[3], gyro_prev[3], mag_prev[3];
//int16_t acc_raw[3], gyro_raw[3], mag_raw[3];
//int16_t acc_raw_prev[3], gyro_raw_prev[3], mag_raw_prev[3];
//
//float velocity_prev[3];
//long time_prev[3];
//
//QMC5883L compass;
//int heading, heading_prev;
//
//float temp, temp_prev;
//
////*************************************************************ZUPT Parameters**************************************************************
//#define N 256u
//#define NR_OF_INITIAL_ALIGNMENT_SAMPLES (N)
//
//#define KALMAN_SIGMA_2_ACC (0.7f*0.7f)
//#define KALMAN_SIGMA_2_GYRO (0.017f*0.017f)
//#define KALMAN_SIGMA_2_ZUPT (0.1f*0.1f)
//#define RESET_COV_THRESHOLD 0.0003f
//
///// Rough latitude of the system [\f$degrees\f$]. (Used to calculate the magnitude of the gravity vector)
//precision latitude = 13;
//
///// Rough altitude of the system [\f$m\f$]. (Used to calculate the magnitude of the gravity vector)
//precision altitude = 920;
//
///// Magnitude of the local gravity acceleration [\f$m/s^2\f$]
//precision g = 9.81f; //9.782940329221166;
//
///// Error signaling vector. If zero no error has occurred.
//uint8_t error_signal;
//
///*!
//  \name Initialization control parameters
//  Parameters controlling the initialization of the navigation algorithm,
//  i.e., the initial states of the inertial navigation system equations
//  and the initial Kalman filter covariance matrix.
//*/
////@{
///// A flag that should be set to true when initialization is started and that becomes false when the initialization is finished.
//bool init_done = false;
//
///// Initial heading [\f$rad\f$]
//#define INITIAL_HEADING 0.0f
//
///// Initial position (North, East, Down) [\f$m\f$]
//#define INITIAL_POSX 0.0f
//#define INITIAL_POSY 0.0f
//#define INITIAL_POSZ 0.0f
//
///// Standard deviations in the initial position uncertainties [\f$m\f$].
//#define SIGMA2_INIT_POSX (0.00001f*0.00001f)
//#define SIGMA2_INIT_POSY (0.00001f*0.00001f)
//#define SIGMA2_INIT_POSZ (0.00001f*0.00001f)
//
///// Standard deviations in the initial velocity uncertainties [\f$m/s\f$].
//#define SIGMA2_INIT_VELX (0.01f*0.01f)
//#define SIGMA2_INIT_VELY (0.01f*0.01f)
//#define SIGMA2_INIT_VELZ (0.01f*0.01f)
//
///// Standard deviations in the initial attitude uncertainties [\f$rad\f$].
//#define SIGMA2_INIT_ROLL  (0.00174f*0.00174f)
//#define SIGMA2_INIT_PITCH (0.00174f*0.00174f)
//#define SIGMA2_INIT_YAW   (0.00174f*0.00174f)
////@}


/*!
  \name Kalman filter control parameters

  Parameters controlling the behavior of the Kalman filter.
  The parameters can be changed while the filter is running
  to adapt the filter to the current motion dynamics.

  \note The default noise standard deviation figures are not set to
  reflect the true noise figures of the IMU sensors, but
  rather to model the sum of all the errors (biases,
  scale factors, nonlinearities, etc.) in the system and
  the measurement model.

*/
//@{
/// Accelerometer process noise standard deviation [\f$m/s^2\f$]
//precision sigma_acceleration=KALMAN_SIGMA_2_ACC;

/// Gyroscope process noise standard deviation [\f$rad/s\f$]
//precision sigma_gyroscope=KALMAN_SIGMA_2_GYRO;

/// Pseudo zero-velocity measurement noise standard deviations (north, east, down) [\f$m/s\f$]
//vec3 sigma_velocity={KALMAN_SIGMA_2_ZUPT,KALMAN_SIGMA_2_ZUPT,KALMAN_SIGMA_2_ZUPT};
//@}

/*!
  \name Navigation and filter state variables.

  Vectors that holds the current navigation state estimate and the covariance and gain of the Kalman filter.

  //*/
////@{
/////  Position estimate (North,East,Down) [\f$m\f$].
//vec3 pos;
//vec3 dpos;
//
///// Velocity estimate (North,East,Down) [\f$m/s\f$]
//vec3 vel;
//vec3 dvel;
//
///// Attitude (quaternions) estimate
//quat_vec quat;
//vec3 deuler;
//
///// Rotation matrix used as an "aiding" variable in the filter algorithm. Holds the same information as the quaternions.
//mat3 Rb2t;
//
///// Vector representation of the Kalman filter covariance matrix.
//mat9sym P;
//
///// Vector representation of the Kalman filter gain matrix.
//mat9by3 K;
////@}
//
////@}
///*!
//  \name Step-wise dead-reckoning parameters and variables
//
//  Parameters and variables controlling the resets of the step-wise dead-reckoning mode.
//
//*/
////@{
//
///// Covariance threshold for filter reset
//precision reset_cov_threshold = RESET_COV_THRESHOLD;
//
///// Minimum period between filter resets (time)
//float min_time_between_resets = 0.75f; //[s]
//
///// Flag signaling that the filter has been reset
//bool filter_reset_flag = false;
//
///// Maximum stationary period before a pending filter reset is done
//float max_time_reset_pending = 0.25f; //[s]
//
///// Displacement and heading change vector
//vec4 dx;
//
///// Displacement and heading change covariance vector
//mat4sym dP;
//
//uint16_t step_counter = 0;
//
////@}
void setup() {
  Serial.begin(115200);
  for (int i = 1; i < 20; i++) {
        Serial.println(sqrt_hf(i));
  }
  //    initIMU();
  //  //  calibrateIMU();
}

void loop() {
  //  getDataFromIMU(MPU_add, acc_add, acc_raw, acc_raw_prev, acc, acc_gain, acc_offset, afs_range);        //Accelerometer Data
  //  getDataFromIMU(MPU_add, gyro_add, gyro_raw, gyro_raw_prev, gyro, gyro_gain, gyro_offset, gfs_range);  //Gyroscope Data
  //  gyro[0] *= DEG_TO_RAD;
  //  gyro[1] *= DEG_TO_RAD;
  //  gyro[2] *= DEG_TO_RAD;
  //  getMagData();
  //
  //  for (int i = 0; i < 3; i++) {
  //    Serial.print(acc[i]);
  //    Serial.print("\t");
  //  }
  //  for (int i = 0; i < 3; i++) {
  //    Serial.print(gyro[i]);
  //    Serial.print("\t");
  //  }
  //  for (int i = 0; i < 3; i++) {
  //    Serial.print(mag[i]);
  //    Serial.print("\t");
  //  }
  //  Serial.println();
}
