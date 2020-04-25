#include <math.h>
#include <stdint.h>

// Define the "precision" of our processor
typedef float precision;
typedef precision vec3[3];
typedef precision mat3[9];
typedef precision mat3sym[6];
typedef precision mat9sym[45];
typedef precision quat_vec[4];
typedef precision mat9by3[27];

/// Maximum numbers of orientations that can be used in the accelerometer calibration
#define MAX_ORIENTATIONS 12

/// Minimum numbers of orientations that can be used in the accelerometer calibration
#define MIN_ORIENTATIONS 3

/// Value returned in the error message if an error occurred in the matrix inversion.
#define MATRIX_INVERSION_ERROR 1

/// Value returned in the error message if the IMU was not stationary during the accelerometer calibration.
#define ACC_STATIONARITY_ERROR 2

/// Value returned in the error message if the orientations used in the accelerometer calibration were poorly chosen.
#define ACC_CALIBRATION_ILLCONDITIONED 3

/// Value returned in the error message if the number of orientations specified for the accelerometer calibration is to large. It has been changed to 12.
#define NUMBER_OF_ORIENTATIONS_TO_LARGE 4

/// Value returned in the error message if the number of orientations specified for the accelerometer calibration is to few. It has been changed to 3.
#define NUMBER_OF_ORIENTATIONS_TO_SMALL 5

/// Absolute value of a floating point variable.
#define absf(a)(a>0 ? a:-a)

/*!
  \name Accelerometer calibration parameters.
  Parameters controlling accelerometer calibration, and vectors and matrices used store the biases.
*/
/// Accelerometer biases (x,y,z-axis) [\f$m/s^2\f$].
vec3 accelerometer_biases;

/// Matrix holding the mean of the accelerometer measurements at each calibration orientation [\f$m/s^2\f$].
static precision acceleration_mean_matrix[3][MAX_ORIENTATIONS];

///Threshold used to check that accelerometers were stationary during the calibration [\f$(m/s^2)^2\f$].
precision acceleration_variance_threshold = 0.002;

/// Number of samples used at each orientation in the calibration procedure.
uint32_t nr_of_calibration_samples = 800;

/// Number of orientations used in the accelerometer calibration procedure. OBS! Most be at least 3 and less than 13.
uint8_t nr_of_calibration_orientations = 6;

/// Flag that is set to true when the IMU should be place in a new orientation. Should be set to false when the calibration procedure is started, and when the IMU has been placed in a new orientation by the user.
boolean new_orientation_flag = false;

/// Flag that is set to true when the calibration is finished. Must be set to false before the calibration is started.
boolean acc_calibration_finished_flag = false;


/*!
  \name General control parameters.
  Parameters controlling general settings of the navigation algorithm
*/
/// Rough latitude of the system [\f$degrees\f$]. (Used to calculate the magnitude of the gravity vector)
precision latitude = 13;

/// Rough altitude of the system [\f$m\f$]. (Used to calculate the magnitude of the gravity vector)
precision altitude = 920;

/// Magnitude of the local gravity acceleration [\f$m/s^2\f$]
precision g = 9.782940329221166;

/// Sampling period [\f$s\f$]
precision dt = 0.001220703125000;

/// Error signaling vector. If zero no error has occurred.
uint8_t error_signal;


/*!
  \name IMU data buffer variables.
  Vectors and variables related to the IMU data buffer.

*/

/// Buffer for the x-axis accelerometer readings.
static precision acc_buffer_x_axis[UINT8_MAX];

/// Buffer for the y-axis accelerometer readings.
static precision acc_buffer_y_axis[UINT8_MAX];

/// Buffer for the z-axis accelerometer readings.
static precision acc_buffer_z_axis[UINT8_MAX];

/// Buffer for the x-axis gyroscope readings.
static precision gyro_buffer_x_axis[UINT8_MAX];

/// Buffer for the y-axis gyroscope readings.
static precision gyro_buffer_y_axis[UINT8_MAX];

/// Buffer for the z-axis gyroscope readings.
static precision gyro_buffer_z_axis[UINT8_MAX];

/// Accelerations read from the IMU [\f$m/s^2\f$]. These are written into the IMU data buffer.
vec3 accelerations_in;

/// Angular rates read from the IMU [\f$rad/s\f$]. These are written into the IMU data buffer.
vec3 angular_rates_in;

/// Accelerations outputted from the IMU data buffer [\f$m/s^2\f$].
vec3 accelerations_out;

/// Angular rates outputted from the IMU data buffer [\f$rad/s\f$].
vec3 angular_rates_out;


/*!
  \name Initialization control parameters
  Parameters controlling the initialization of the navigation algorithm,
  i.e., the initial states of the inertial navigation system equations
  and the initial Kalman filter covariance matrix.
*/

/// A flag that should be set to true when initialization is started and that becomes false when the initialization is finished.
boolean initialize_flag = true;

/// Number of samples used in the initial alignment.
uint8_t nr_of_inital_alignment_samples = 16; //4;

/// Initial heading [\f$rad\f$]
precision initial_heading = 0;

/// Initial position (North, East, Down) [\f$m\f$]
vec3 initial_pos = {0, 0, 0};

/// Standard deviations in the initial position uncertainties [\f$m\f$].
vec3 sigma_initial_position = {0.00001, 0.00001, 0.00001};

/// Standard deviations in the initial velocity uncertainties [\f$m/s\f$].
vec3 sigma_initial_velocity = {0.01, 0.01, 0.01};

/// Standard deviations in the initial attitude uncertainties [\f$rad\f$].
vec3 sigma_initial_attitude = {0.00174, 0.00174, 0.00174};


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

/// Accelerometer process noise standard deviation [\f$m/s^2\f$]
precision sigma_acceleration = 0.7;

/// Gyroscope process noise standard deviation [\f$rad/s\f$]
precision sigma_gyroscope = 0.005235987755983;

/// Pseudo zero-velocity measurement noise standard deviations (north, east, down) [\f$m/s\f$]
vec3 sigma_velocity = {0.1, 0.1, 0.1};


/*!
  \name Navigation and filter state variables.
  Vectors that holds the current navigation state estimate and the covariance and gain of the Kalman filter.
*/
///  Position estimate (North,East,Down) [\f$m\f$].
vec3 position;

/// Velocity estimate (North,East,Down) [\f$m/s\f$]
vec3 velocity;

/// Attitude (quaternions) estimate
quat_vec quaternions;

/// Rotation matrix used as an "aiding" variable in the filter algorithm. Holds the same information as the quaternions.
mat3 Rb2t;

/// Vector representation of the Kalman filter covariance matrix.
mat9sym cov_vector;

/// Vector representation of the Kalman filter gain matrix.
mat9by3 kalman_gain;


/*!
  \name Zero-velocity detector control parameters

  Parameters controlling the behavior of the zero-velocity detector. All the detector
  control parameters, except the \a detector_Window_size may be changed will the navigation algorithm is running in
  order to adapt the behavior of the detector to the current motion dynamics.

*/
/// Accelerometer noise standard deviation figure [\f$m/s^2\f$], which is used to control how much the detector should trusts the accelerometer data.
precision sigma_acc_det = 0.035; //0.01;

/// Gyroscope noise standard deviation figure [\f$rad/s\f$], which is used to control how much the detector should trusts the gyroscope data.
precision sigma_gyro_det = 0.006; //0.001745329251994;

/// The data window size used in the detector (OBS! Must be an odd number.).
volatile uint8_t detector_Window_size = 3;

/// Threshold used in the detector.
precision detector_threshold = 50000;

/// Flag that is set to true if a zero-velocity update should be done.
bool zupt = false;

///Variable holding the test statistics for the generalized likelihood ratio test, i.e., the zero-velocity detector.
precision Test_statistics = 0;

void setup() {
  initialize_navigation_algorithm();
  calibrate_accelerometers();
  estimate_accelerometer_biases();
}

void loop() {
  update_imu_data_buffers();
  time_up_data();
  strapdown_mechanisation_equations();
  ZUPT_detector();
  zupt_update();
}
