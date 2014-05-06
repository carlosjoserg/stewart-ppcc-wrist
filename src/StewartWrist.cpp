#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Geometry>

#include "StewartWrist.h"

#define PI_GRECO 3.141592653589793

using namespace std;

StewartWrist::StewartWrist()
{
	// default initialization for parameters comming from the building
    
  //------------------------------------------------------
  // JACOBIAN MATRIX w.r.t. the sensor reference frame
  //------------------------------------------------------ 
  // 0.4390949  -0.8156991   0.3766041   0.4390949  -0.8156991   0.3766041  
  //-0.6883766  -0.0360791   0.7244556  -0.6883766  -0.0360791   0.7244556  
  // 0.5773503   0.5773503   0.5773503   0.5773503   0.5773503   0.5773503  
  // 80.521629   38.44844   -48.526287   42.60139   -31.995335  -81.049823  
  // 9.5441487  -71.390064   64.961697   68.992354  -74.505853   2.3977035  
  //-49.859969   49.859971  -49.859966   49.85997   -49.859972   49.859965 
  J_.resize(6,6); // just for resizing

  J_(0,0) = 0.4390949;
  J_(1,0) = -0.6883766;
  J_(2,0) = 0.5773503;
  J_(3,0) = 80.521629;
  J_(4,0) = 9.5441487;
  J_(5,0) = -49.859969;

  J_(0,1) = -0.8156991;
  J_(1,1) = -0.0360791;
  J_(2,1) = 0.5773503;
  J_(3,1) = 38.44844;
  J_(4,1) = -71.390064;
  J_(5,1) = 49.859971;

  J_(0,2) = 0.3766041;
  J_(1,2) = 0.7244556;
  J_(2,2) = 0.5773503;
  J_(3,2) = -48.526287;
  J_(4,2) = 64.961697;
  J_(5,2) = -49.859966;

  J_(0,3) = 0.4390949;
  J_(1,3) = -0.6883766;
  J_(2,3) = 0.5773503;
  J_(3,3) = 42.60139;
  J_(4,3) = 68.992354;
  J_(5,3) = 49.85997;

  J_(0,4) = -0.8156991;
  J_(1,4) = -0.0360791;
  J_(2,4) = 0.5773503;
  J_(3,4) = -31.995335;
  J_(4,4) = -74.505853;
  J_(5,4) = -49.859972;

  J_(0,5) = 0.3766041;
  J_(1,5) = 0.7244556;
  J_(2,5) = 0.5773503;
  J_(3,5) = -81.049823;
  J_(4,5) = 2.3977035;
  J_(5,5) = 49.859965;

  //----------------------------------------------------------------------------
  // STIFFNESS MATRIX in N, mm, all mixed up w.r.t. the sensor reference frame
  //----------------------------------------------------------------------------
  //  4.0500568   -0.0000000   -0.0000002    0.0000053    362.19652    0.0000032  
  // -0.0000000    4.0500566   -0.0000002   -362.19651   -0.0000050   -0.0000024  
  // -0.0000002   -0.0000002    4.0500572    0.0000168   -0.0000174   -0.0000003  
  //  0.0000053   -362.19651    0.0000168    39942.611    0.0004058    0.0007158  
  //  362.19652   -0.0000050   -0.0000174    0.0004058    39942.612    0.0009061  
  //  0.0000032   -0.0000024   -0.0000003    0.0007158    0.0009061    30205.524
  K_.resize(6,6);

  K_(0,0) = 4.0500568;
  K_(1,0) = -0.0000000;
  K_(2,0) = -0.0000002;
  K_(3,0) = 0.0000053;
  K_(4,0) = 362.19652;
  K_(5,0) = 0.0000032;

  K_(0,1) = -0.0000000;
  K_(1,1) = 4.0500566;
  K_(2,1) = -0.0000002;
  K_(3,1) = -362.19651;
  K_(4,1) = -0.0000050;
  K_(5,1) = -0.0000024;

  K_(0,2) = -0.0000002;
  K_(1,2) = -0.0000002;
  K_(2,2) = 4.0500572;
  K_(3,2) = 0.0000168;
  K_(4,2) = -0.0000174;
  K_(5,2) = -0.0000003;

  K_(0,3) = 0.0000053;
  K_(1,3) = -362.19651;
  K_(2,3) = 0.0000168;
  K_(3,3) = 39942.611;
  K_(4,3) = 0.0004058;
  K_(5,3) = 0.0007158;

  K_(0,4) = 362.19652;
  K_(1,4) = -0.0000050;
  K_(2,4) = -0.0000174;
  K_(3,4) = 0.0004058;
  K_(4,4) = 39942.612;
  K_(5,4) = 0.0009061;

  K_(0,5) = 0.0000032;
  K_(1,5) = -0.0000024;
  K_(2,5) = -0.0000003;
  K_(3,5) = 0.0007158;
  K_(4,5) = 0.0009061;
  K_(5,5) = 30205.524;

  //------------------------------------------
  // PLATFORM WEIGHT component
  //------------------------------------------

  // Default orientation, REMEMBER to set the orientation before doing the calculation !
  Eigen::MatrixXf RR;
  RR.setZero(6,6);
  
  RR(0,0) = 1.0;
  RR(1,0) = 0.0;
  RR(2,0) = 0.0;

  RR(0,1) = 0.0;
  RR(1,1) = 1.0;
  RR(2,1) = 0.0;
 
  RR(0,2) = 0.0;
  RR(1,2) = 0.0;
  RR(2,2) = 1.0;

  RR(3,3) = 1.0;
  RR(4,3) = 0.0;
  RR(5,3) = 0.0;
  
  RR(3,4) = 0.0;
  RR(4,4) = 1.0;
  RR(5,4) = 0.0;

  RR(3,5) = 0.0;
  RR(4,5) = 0.0;
  RR(5,5) = 1.0;

  RR_ = RR.transpose().eval();

  // center of mass
  Eigen::VectorXf cm_platform;
  cm_platform.resize(3,1);
  cm_platform(0) = 0.023981346;
  cm_platform(1) = 0.015794487;
  cm_platform(2) = -55.446556;
  
  // Wrench mapping transform
  P_.setIdentity(6,6);
  // this is the skew-symmetric part
  P_(3,1) = -55.446556;
  P_(3,2) = 0.015794487;
  P_(4,0) = -55.446556;
  P_(4,2) = -0.023981346;
  P_(5,0) = -0.015794487;
  P_(5,1) = 0.023981346;

  // Default weight of the platform in world coordinates, typically with Z pointing up.
  w_w_platform_.resize(6);
  w_w_platform_(0) = 0.0;
  w_w_platform_(1) = 0.0;
  w_w_platform_(2) = -3.0;
  w_w_platform_(3) = 0.0;
  w_w_platform_(4) = 0.0;
  w_w_platform_(5) = 0.0;

  // and finally obtain the weight in the platform reference system
  w_platform_.resize(6);
  w_platform_ = P_ * RR_ * w_w_platform_;


  //------------------------------------------
  // PRELOAD component
  //------------------------------------------   
  float k_l = 2.5;  // [N]
  float x_preload = 1; // [mm]
  float f_preload = k_l*x_preload;

  Eigen::VectorXf preload;
  preload.resize(6);
  preload(0) = 2.5;
  preload(1) = 2.5;
  preload(2) = 2.5;
  preload(3) = 2.5;
  preload(4) = 2.5;
  preload(5) = 2.5;

  w_preload_.resize(6);
  w_preload_ = J_*preload;

  //------------------------------------------------
  // EXTERNAL WRENCH initialized to zero
  //------------------------------------------------
  w_external_.resize(6);
  w_external_(0) = 0.0;
  w_external_(1) = 0.0;
  w_external_(2) = 0.0;
  w_external_(3) = 0.0;
  w_external_(4) = 0.0;
  w_external_(5) = 0.0;  

  //--------------------------------------------------
  // DISPLACEMENT resizing initizializing
  //--------------------------------------------------
  displacement_.resize(6);

}


void StewartWrist::setOrientation(Matrix3f R)
{
  
  // Note that the transposing is implemented when filling the matriz
  RR_(0,0) = R(0,0);
  RR_(1,0) = R(1,0);
  RR_(2,0) = R(2,0);

  RR_(0,1) = R(0,1);
  RR_(1,1) = R(1,1);
  RR_(2,1) = R(2,1);

  RR_(0,2) = R(0,2);
  RR_(1,2) = R(1,2);
  RR_(2,2) = R(2,2);

  RR_(3,3) = R(0,0);
  RR_(4,3) = R(1,0);
  RR_(5,3) = R(2,0);

  RR_(3,4) = R(0,1);
  RR_(4,4) = R(1,1);
  RR_(5,4) = R(2,1);

  RR_(3,5) = R(0,2);
  RR_(4,5) = R(1,2);
  RR_(5,5) = R(2,2);

}

void StewartWrist::setExternalWrench(Eigen::Vector3f f, Eigen::Vector3f m)
{
  w_external_(0) = f(0);
  w_external_(1) = f(1);
  w_external_(2) = f(2);
  w_external_(3) = m(0);
  w_external_(4) = m(1);
  w_external_(5) = m(2);
}

void StewartWrist::updatePlatformWeight()
{
  w_platform_ = P_ * RR_ * w_w_platform_;
}

void StewartWrist::doCalculation(float ft[6], Eigen::Matrix3f R)
{
    setSensedForce((double)ft[0], (double)ft[1], (double)ft[2]);
    setSensedTorque((double)ft[3],(double)ft[4],(double)ft[5]);
    setExternalWrench(sensor_force_, sensor_torque_);
    setOrientation(R);
    updatePlatformWeight();
    
    Eigen::VectorXf d(6);
    d = K_.inverse()*(w_external_ + w_platform_ + w_preload_);

    // a positive value means nothing because the platform can not go beyond zero, the springs do not allow it
    if(d(2)>0)
    {
      d(2) = 0;
    };

    setDisplacement(d);
}

void StewartWrist::setDisplacement(Eigen::VectorXf d)
{
  displacement_(0) = d(0);
  displacement_(1) = d(1);
  displacement_(2) = d(2);
  displacement_(3) = d(3);
  displacement_(4) = d(4);
  displacement_(5) = d(5);
  
  // printf("contact point %f %f %f\n", app(0,2), app(1,2), app(2,2));
  //printf("contact point %f %f %f\n", contact_point_(0), contact_point_(1), contact_point_(2));
}

Eigen::VectorXf StewartWrist::getDisplacement()
{
  Eigen::VectorXf d;
  d.resize(6);
  d(0) = displacement_(0);
  d(1) = displacement_(1);
  d(2) = displacement_(2);
  d(3) = displacement_(3);
  d(4) = displacement_(4);
  d(5) = displacement_(5);
  
// printf("contact point %f %f %f\n", app(0,2), app(1,2), app(2,2));
//printf("contact point %f %f %f\n", contact_point_(0), contact_point_(1), contact_point_(2));
  return d;
}