/*
 *    MIT License
 *    
 *    Copyright (c) 2023 Jongwon Lee (jongwon5@illinois.edu)
 *    
 *    Permission is hereby granted, free of charge, to any person obtaining a copy
 *    of this software and associated documentation files (the "Software"), to deal
 *    in the Software without restriction, including without limitation the rights
 *    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *    copies of the Software, and to permit persons to whom the Software is
 *    furnished to do so, subject to the following conditions:
 *    
 *    The above copyright notice and this permission notice shall be included in all
 *    copies or substantial portions of the Software.
 *    
 *    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *    SOFTWARE.
 */
#pragma once

#include <math.h>
#include "dtypes.h"

#define EPS (1e-6f)


/**
 * @brief Get a norm of a unit Hamiltonian quaternion
 *
 * @param[in] this a unit quaternion
 * @return float-type norm
 */
static inline float quatNorm(Quat4f* this)
{
  return sqrtf(this->x*this->x + this->y*this->y + this->z*this->z + this->w*this->w) + EPS;
}

/**
 * @brief Normalize a unit Hamiltonian quaternion by an angular velocity
 *
 * @param[in] this a unit quaternion to be normalized
 */
static inline void normalize(Quat4f* this)
{
  // normalize and store the result
  float norm = quatNorm(this);
  
  this->x = this->x/norm;
  this->y = this->y/norm;
  this->z = this->z/norm;
  this->w = this->w/norm;
  
  if (this->w < 0) {
    this->x = -this->x;
    this->y = -this->y;
    this->z = -this->z;
    this->w = -this->w;
  }
}

/**
 * @brief Propagate a unit Hamiltonian quaternion by an angular velocity
 *
 * See equation (102) of trawny tech report [Indirect Kalman Filter for 3D Attitude Estimation](http://mars.cs.umn.edu/tr/reports/Trawny05b.pdf).
 * BEWARE: The reference uses JPL quaternion but the following holds for Hamiltonian quaternion as well.
 *
 * @param[in] this a unit quaternion to be propagated; it becomes q_ItoIp1, a quaternion transforming any entity in the previous local frame to the new one
 * @param[in] gyro angular velocity
 * @param[in] dt temporal increment
 */
void propagateQuaternion(Quat4f* this, Axis3f *gyro, float dt)
{
  float dt2 = dt*dt;

  // attitude update (rotate by gyroscope), we do this in quaternions
  // this is the gyroscope angular velocity integrated over the sample period
  float dtwx = dt*gyro->x;
  float dtwy = dt*gyro->y;
  float dtwz = dt*gyro->z;

  // compute the quaternion values in [w,x,y,z] order
  float angle = sqrtf(dtwx*dtwx + dtwy*dtwy + dtwz*dtwz) + EPS;
  float ca = cosf(angle/2.0f);
  float sa = sinf(angle/2.0f);
  Quat4f dq = {sa*dtwx/angle , sa*dtwy/angle , sa*dtwz/angle, ca};

  // rotate the quad's attitude by the delta quaternion vector computed above
  this->x = dq.x*this->w + dq.w*this->x + dq.z*this->y - dq.y*this->z;
  this->y = dq.y*this->w - dq.z*this->x + dq.w*this->y + dq.x*this->z;
  this->z = dq.z*this->w + dq.y*this->x - dq.x*this->y + dq.w*this->z;
  this->w = dq.w*this->w - dq.x*this->x - dq.y*this->y - dq.z*this->z;

  normalize(this);

}
