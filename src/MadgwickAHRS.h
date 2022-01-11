//=============================================================================================
// MadgwickAHRS.h
//=============================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// From the x-io website "Open-source resources available on this website are
// provided under the GNU General Public Licence unless an alternative licence
// is provided in source."
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=============================================================================================
#ifndef MadgwickAHRS_h
#define MadgwickAHRS_h
#include <math.h>

//--------------------------------------------------------------------------------------------
// Variable declaration
class Madgwick
{
private:
  static float invSqrt(float x);
  static float dot(float ax, float ay, float az, float bx, float by, float bz);
  static void cross(float ax, float ay, float az, float bx, float by, float bz, float &cx, float &cy, float &cz);
  static void norm(float &ax, float &ay, float &az);
  float beta; // algorithm gain
  float q0;
  float q1;
  float q2;
  float q3; // quaternion of sensor frame relative to auxiliary frame
  float invSampleFreq;
  float roll;
  float pitch;
  float yaw;
  char anglesComputed;
  void computeAngles();
  void align(float ax, float ay, float az, float bx, float by, float bz);
  void combine(float p0, float p1, float p2, float p3);
  void rotate(float &ax, float &ay, float &az);

  //-------------------------------------------------------------------------------------------
  // Function declarations
public:
  Madgwick(void);
  void begin(float sampleFrequency) { invSampleFreq = 1.0f / sampleFrequency; }
  void begin(float sampleFrequency, float gain)
  {
    invSampleFreq = 1.0f / sampleFrequency;
    beta = gain;
  }
  void begin(float sampleFrequency, float gain, float pitch, float roll, float yaw);
  void begin(float sampleFrequency, float gain, float ax, float ay, float az, float mx, float my, float mz);
  void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
  void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);

  float getRoll()
  {
    if (!anglesComputed)
      computeAngles();
    return roll * 57.29578f;
  }
  float getPitch()
  {
    if (!anglesComputed)
      computeAngles();
    return pitch * 57.29578f;
  }
  float getYaw()
  {
    if (!anglesComputed)
      computeAngles();
    return yaw * 57.29578f + 180.0f; // TODO: test
  }
  float getRollRadians()
  {
    if (!anglesComputed)
      computeAngles();
    return roll;
  }
  float getPitchRadians()
  {
    if (!anglesComputed)
      computeAngles();
    return pitch;
  }
  float getYawRadians()
  {
    if (!anglesComputed)
      computeAngles();
    return yaw;
  }
  void getQuaternion(float *q_0, float *q_1, float *q_2, float *q_3)
  {
    *q_0 = q0;
    *q_1 = q1;
    *q_2 = q2;
    *q_3 = q3;
  }
};
#endif
