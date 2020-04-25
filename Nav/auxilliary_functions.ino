/**
  \defgroup aux_func Auxilliary functions
  \brief Auxilliary functions for calculation various matrix and scalar operations.

  @{
*/

/*! \brief Function for calculating the square root.
  \details Function for calculating the square root using the hardware multipliers if the precision used is float,
  otherwise use the standard square root function.
*/
inline precision sqrt_hf(precision arg) {

  if (sizeof(precision) == 4)  {
    float tmp1, tmp2, tmp3;
    //    __asm__ __volatile__ ( "frsqrta.s %0, %1" : "=r" (tmp3) : "r" (arg)); //@itsharsh
    tmp1 = tmp3 * tmp3;
    tmp2 = 3.0f - tmp1 * arg;
    tmp3 = 0.5f * (tmp2 * tmp3);
    tmp1 = tmp3 * tmp3;
    tmp2 = 3.0f - tmp1 * arg;
    tmp3 = 0.5f * (tmp2 * tmp3);
    tmp1 = tmp3 * tmp3;
    tmp2 = 3.0f - tmp1 * arg;
    tmp3 = 0.5f * (tmp2 * tmp3);
    return tmp3 * arg;
  }
  else  {
    return sqrt(arg);
  }
}

/*! \brief Function that calculates the squared Euclidean norm of a vector.

   @param[out] norm2        The squared Euclidean norm of the input vector.
   @param[in] arg_vec       The input vectors
   @param[in] len         The length of the input vector.
*/
inline precision vecnorm2(precision *arg_vec, uint8_t len) {

  precision norm2 = 0;
  uint8_t ctr;

  for (ctr = 0; ctr < len; ctr++) {
    norm2 = norm2 + arg_vec[ctr] * arg_vec[ctr];
  }
  return norm2;
}

/*! \brief Function that converts Euler angles ([roll,pitch,yaw]) into a rotation matrix \f$R_b^t\f$.

   @param[out] rotmat       Vector representation of the rotation matrix.
   @param[in] euler       Vector of euler angles.
*/
inline void euler2rotation(mat3 rotmat, const vec3 euler) {

  // Trigonometric value variables
  precision sin_phi = sin(euler[0]);
  precision cos_phi = cos(euler[0]);
  precision sin_theta = sin(euler[1]);
  precision cos_theta = cos(euler[1]);
  precision sin_psi = sin(euler[2]);
  precision cos_psi = cos(euler[2]);

  rotmat[0] = cos_psi * cos_theta;            //Matrix element [1,1]
  rotmat[3] = sin_psi * cos_theta;            //Matrix element [1,2]
  rotmat[6] = -sin_theta;                 //Matrix element [1,3]
  rotmat[1] = (-sin_psi * cos_phi) + cos_psi * (sin_theta * sin_phi); //Matrix element [2,1]
  rotmat[4] = (cos_psi * cos_phi) + sin_psi * (sin_theta * sin_phi); //Matrix element [2,2]
  rotmat[7] = cos_theta * sin_phi;            //Matrix element [2,3]
  rotmat[2] = (sin_psi * sin_phi) + cos_psi * (sin_theta * cos_phi); //Matrix element [3,1]
  rotmat[5] = (-cos_psi * sin_phi) + sin_psi * (sin_theta * cos_phi); //Matrix element [3,2]
  rotmat[8] = cos_theta * cos_phi;            //Matrix element [3,3]
}

/*! \brief Function for converting a rotation matrix \f$R_b^t\f$ to Euler angles ([roll,pitch,yaw]).

  @param[out] euler       Vector of euler angles.
  @param[in] rotmat       Vector of representation of the rotation matrix.
*/
inline void  rotation2euler(vec3 euler, const mat3 rotmat) {

  // Compute Euler angles [WARNING! check atan2]
  euler[0] = atan2(rotmat[7], rotmat[8]); //atan2( R(3,2), R(3,3) );
  euler[1] = asin(-rotmat[6]);      //asin( -R(3,1) );
  euler[2] = -atan2(rotmat[3], rotmat[0]); //atan2( R(2,1), R(1,1));
}

/*! \brief Function for converting quaternions to a rotation matrix \f$R_b^t\f$.

  @param[out] rotmat        Vector of representation of the rotation matrix.
   @param[in] q         Vector of quaternions.

*/
inline void quat2rotation(mat3 rotmat, const quat_vec q) {

  precision p[6];

  p[0] = q[0] * q[0];
  p[1] = q[1] * q[1];
  p[2] = q[2] * q[2];
  p[3] = q[3] * q[3];
  p[4] = p[1] + p[2];
  if (p[0] + p[3] + p[4] != 0)  {
    p[5] = 2 / (p[0] + p[3] + p[4]);
  }
  else  {
    p[5] = 0;
  }

  rotmat[0] = 1 - p[5] * p[4]; //R(1,1)=1-p(6)*p(5);
  rotmat[4] = 1 - p[5] * (p[0] + p[2]); //R(2,2)=1-p(6)*(p(1)+p(3));
  rotmat[8] = 1 - p[5] * (p[0] + p[1]); //R(3,3)=1-p(6)*(p(1)+p(2));

  p[0] = p[5] * q[0];     //p(1)=p(6)*q(1);
  p[1] = p[5] * q[1];     //p(2)=p(6)*q(2);
  p[4] = p[5] * q[2] * q[3]; //p(5)=p(6)*q(3)*q(4);
  p[5] = p[0] * q[1];     //p(6)=p(1)*q(2);

  rotmat[1] = p[5] - p[4];  //R(1,2)=p(6)-p(5);
  rotmat[3] = p[5] + p[4];  //R(2,1)=p(6)+p(5);

  p[4] = p[1] * q[3];     //p(5)=p(2)*q(4);
  p[5] = p[0] * q[2];     //p(6)=p(1)*q(3);

  rotmat[2] = p[5] + p[4];  //R(1,3)=p(6)+p(5);
  rotmat[6] = p[5] - p[4];  //R(3,1)=p(6)-p(5);

  p[4] = p[0] * q[3];     //p(5)=p(1)*q(4);
  p[5] = p[1] * q[2];     //p(6)=p(2)*q(3);

  rotmat[5] = p[5] - p[4];  //R(2,3)=p(6)-p(5);
  rotmat[7] = p[5] + p[4];  //R(3,2)=p(6)+p(5);
}

/*! \brief Function for converting a rotation matrix \f$R_b^t\f$ to quaternions.

   @param[out] q          Vector of quaternions.
   @param[in] rotmat        Vector of representation of the rotation matrix.
*/
inline void rotation2quat(quat_vec q, const mat3 rotmat) {

  // For checking robustness of DCM, diagonal elements
  precision T = 1 + rotmat[0] + rotmat[4] + rotmat[8]; // 1+diag(R)

  // Calculate quaternion based on largest diagonal element
  if (T > (0.00000001)) {
    precision S = 0.5 / sqrt_hf(T);
    q[3] = 0.25 / S;
    q[0] = (rotmat[7] - rotmat[5]) * S; //(R(3,2) - R(2,3))*S
    q[1] = (rotmat[2] - rotmat[6]) * S; //( R(1,3) - R(3,1) ) * S;
    q[2] = (rotmat[3] - rotmat[1]) * S; // ( R(2,1) - R(1,2) ) * S;
  }
  else if ((rotmat[0] > rotmat[4] ) && (rotmat[0] > rotmat[8]) ) { //(R(1,1) > R(2,2)) && (R(1,1) > R(3,3))
    precision S = sqrt_hf(1 + rotmat[0] - rotmat[4] - rotmat[8]) * 2; //S=sqrt_hf(1 + R(1,1) - R(2,2) - R(3,3)) * 2
    q[3] = (rotmat[6] - rotmat[5]) / S; //(R(3,1) - R(2,3)) / S;
    q[0] = 0.25 * S;
    q[1] = (rotmat[1] + rotmat[3]) / S; //(R(1,2) + R(2,1)) / S;
    q[2] = (rotmat[2] + rotmat[6]) / S; //(R(1,3) + R(3,1)) / S;
  }

  else if (rotmat[4] > rotmat[8]) { //(R(2,2) > R(3,3))
    precision S = sqrt_hf(1 + rotmat[4] - rotmat[0] - rotmat[8]) * 2; //sqrt_hf( 1 + R(2,2) - R(1,1) - R(3,3) ) * 2;
    q[3] = (rotmat[2] - rotmat[6]) / S;       //(R(1,3) - R(3,1)) / S;
    q[0] =  (rotmat[1] + rotmat[3]) / S;      //(R(1,2) + R(2,1)) / S;
    q[1] = 0.25 * S;
    q[2] = (rotmat[5] + rotmat[7]) / S;       //(R(2,3) + R(3,2)) / S;
  }
  else {
    precision S = sqrt_hf(1 + rotmat[8] - rotmat[0] - rotmat[4]) * 2; //S = sqrt_hf( 1 + R(3,3) - R(1,1) - R(2,2) ) * 2;
    q[3] = (rotmat[3] - rotmat[1]) / S;       //(R(2,1) - R(1,2)) / S;
    q[0] = (rotmat[2] + rotmat[6]) / S;       //(R(1,3) + R(3,1)) / S;
    q[1] = (rotmat[1] + rotmat[3]) / S;       //(R(1,2) + R(2,1)) / S;
    q[2] = 0.25 * S;
  }

  //Normalize the quaternion
  T = sqrt_hf(vecnorm2(q, 4));

  q[0] = q[0] / T;
  q[1] = q[1] / T;
  q[2] = q[2] / T;
  q[3] = q[3] / T;
}

/*! \brief Function for inverting a 3 by 3 matrix hermitian matrix.

  @param[out] ainv    Vector representation of the inverted matrix.
  @param[out] error   Error signaling vector.
  @param[in]  a     Vector representation of the matrix to be inverted.
*/
inline void invmat3sys(mat3sym ainv, mat3sym a) {

  // Calculate the determinant of matrix
  precision det = -a[2] * (a[2] * a[3]) + 2 * a[1] * (a[2] * a[4]) - a[0] * (a[4] * a[4]) - a[1] * (a[1] * a[5]) + a[0] * (a[3] * a[5]);

  // Check the size of the determinant and send a error message if it is to small.
  if (absf(det) < 0.0000001)  {
    error_signal = MATRIX_INVERSION_ERROR;
  }
  ainv[0] = (a[3] * a[5] - a[4] * a[4]) / det;
  ainv[1] = (a[2] * a[4] - a[1] * a[5]) / det;
  ainv[2] = (a[1] * a[4] - a[2] * a[3]) / det;
  ainv[3] = (a[0] * a[5] - a[2] * a[2]) / det;
  ainv[4] = (a[1] * a[2] - a[0] * a[4]) / det;
  ainv[5] = (a[0] * a[3] - a[1] * a[1]) / det;
}

/*! \brief  Function that calculates the maximum value of a vector and returns
      the max value and the index of the vector element holding the maximum value.

  @param[out] max_v   Largest value of the input vector.
  @param[out] index   Index of the vector element holding the largest value.
  @param[in]  arg_vec   The input vector.
*/
inline void max_value(precision *max_v, uint8_t *index, precision *arg_vec) {

  // Set the initial max value and vector element index
  *max_v = arg_vec[0];
  *index = 0;

  //Iterate through the vector
  for (uint8_t ctr = 1; ctr < (sizeof(arg_vec) / sizeof(arg_vec[0])); ctr++)  {
    //If the current element of the vector is larger than the previous element, update the max value and the index.
    if (arg_vec[ctr] > arg_vec[ctr - 1])   {
      *max_v = arg_vec[ctr];
      *index = ctr;
    }
  }
}

/*! \brief Function for calculating the Kalman filter innovation covariance.

  @param[out] re    Vector representation of the innovation covariance matrix.
  @param[in] pvec   Vector representation of the Kalman filter covariance matrix.
  @param[in] sigma  Vector representation of pseudo zero-velocity measurement noise standard deviations.
*/
inline void innovation_cov(mat3sym re, mat9sym pvec) {

  re[0] = pvec[24] + sigma_velocity[0] * sigma_velocity[0];
  re[1] = pvec[25];
  re[2] = pvec[26];
  re[3] = pvec[30] + sigma_velocity[1] * sigma_velocity[1];
  re[4] = pvec[31];
  re[5] = pvec[35] + sigma_velocity[2] * sigma_velocity[2];
}
