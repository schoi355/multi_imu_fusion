#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "dtypes.h"
#include "PropagateQuaternion.h"

#ifndef PI
  #define PI 3.14159265358979f
#endif

#define DEG2RAD (PI/180.0f)
#define RAD2DEG (180.0f/PI)


static Quat4f quat = {0.0f, 0.0f, 0.0f, 1.0f};

int main(void) {
  printf(" -- attitude estimator unit test begins ... \n");
  printf(" -- start quaternion: [%.4f %.4f %.4f %.4f] \n", quat.x, quat.y, quat.z, quat.w);

  float dt = 1;

  // rotate along x axis by 1 degree 360 times
  for (int i=0; i<360; i++) {    
    Axis3f gyro;
    gyro.x = 1*DEG2RAD;
    gyro.y = 0*DEG2RAD;
    gyro.z = 0*DEG2RAD;
    propagateQuaternion(&quat, &gyro, dt);
  }
  
  // rotate along y axis by 1 degree 360 times
  for (int i=0; i<360; i++) {    
    Axis3f gyro;
    gyro.x = 0*DEG2RAD;
    gyro.y = 1*DEG2RAD;
    gyro.z = 0*DEG2RAD;
    propagateQuaternion(&quat, &gyro, dt);
  }

  // rotate along z axis by 1 degree 360 times
  for (int i=0; i<360; i++) {    
    Axis3f gyro;
    gyro.x = 0*DEG2RAD;
    gyro.y = 0*DEG2RAD;
    gyro.z = 1*DEG2RAD;
    propagateQuaternion(&quat, &gyro, dt);
  }

  printf(" -- end quaternion: [%.4f %.4f %.4f %.4f] \n", quat.x, quat.y, quat.z, quat.w);
  
  return 0;
}