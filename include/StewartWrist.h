#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <string.h>
#include <stdlib.h>
#include <Eigen/Core>

using namespace std;
using namespace Eigen;

class StewartWrist{

private:  
  // point coordinates, force and torque at the contact point, the force should be the same as the sensed force
  VectorXf displacement_;
  
  // the force/torque at the sensor frame
  Vector3f sensor_force_;
  Vector3f sensor_torque_;
  VectorXf w_external_;

  // wrnech due to the weight of the platform in the world reference frame, it must be update if the platform orientation changes to R
  MatrixXf RR_;
  MatrixXf P_;
  VectorXf w_w_platform_;
  VectorXf w_platform_;
  
  // these are created at the constructor since they are constant values for the platform
  // 6x6 stiffness matrix
  MatrixXf K_;

  // the jacobian at the built configuration
  MatrixXf J_;

  // preload wrench on the platform
  VectorXf w_preload_;
  
public:  
  StewartWrist();
  void setSensedForce(double fx, double fy, double fz){ sensor_force_ = Vector3f(fx, fy, fz); };
  void setSensedTorque(double tx, double ty, double tz){ sensor_torque_ = Vector3f(tx, ty, tz); };
  void setOrientation(Matrix3f R);
  void setExternalWrench(Vector3f f, Vector3f m);
  void updatePlatformWeight();
  void doCalculation(float ft[6], Matrix3f R);
  void setDisplacement(VectorXf d);
  VectorXf getDisplacement();
  
};