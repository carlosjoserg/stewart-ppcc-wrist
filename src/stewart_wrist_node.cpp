#include <ros/ros.h>
#include <ros/message_operations.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/Empty.h>

// Stewart Wrist class definition
#include "StewartWrist.h"

namespace tactile_perception {

class PlatformPublisher
{
  private:
    //! The node handle
    ros::NodeHandle nh_;
    //! Node handle in the private namespace
    ros::NodeHandle priv_nh_;

    //! The class that does the processing 
    StewartWrist* stewart_wrist_;
    
    //! Subscriber for sensor readings
    ros::Subscriber sub_sensor_readings_;
    
    //! Publisher for tactile info
    ros::Publisher pub_stewart_processing_;
    
  public:
    //------------------ Callbacks -------------------
    // Callback for performing the tactile computations
    void doPlatformComputations(const geometry_msgs::Wrench & msg);

    //! Subscribes to and advertises topics
    PlatformPublisher(ros::NodeHandle nh) : nh_(nh), priv_nh_("~")
    {
      // Create a new tactile object
      stewart_wrist_ = new StewartWrist();
      
      // Subscribe to the topic where readings are being published
      sub_sensor_readings_ = nh_.subscribe(nh_.resolveName("FT_sensor_readings"), 10, &PlatformPublisher::doPlatformComputations, this);

      pub_stewart_processing_ = nh_.advertise<geometry_msgs::Twist>(nh_.resolveName("stewart_info"), 10);

    }

    //! Empty stub
    ~PlatformPublisher() {}

};

void PlatformPublisher::doPlatformComputations(const geometry_msgs::Wrench & msg )
{
  // this is just for testing, but we should read the orientation from the robot measurements
  Matrix3f R;
  R.setIdentity(3,3);

  // get the sensor measurements
  float ft[6];
  ft[0] = msg.force.x;
  ft[1] = msg.force.y;
  ft[2] = msg.force.z;
  ft[3] = msg.torque.x;
  ft[4] = msg.torque.y;
  ft[5] = msg.torque.z;

  // give it to the intrinsic tactile object
  stewart_wrist_->doCalculation(ft, R);

  // retrieve the computed data
  VectorXf d_data(6);
  d_data = stewart_wrist_->getDisplacement();

  // fill the message with the tactile info
  geometry_msgs::Twist displacement_info;
  displacement_info.linear.x = d_data(0);
  displacement_info.linear.y = d_data(1);
  displacement_info.linear.z = d_data(2);
  displacement_info.angular.x = d_data(3);
  displacement_info.angular.y = d_data(4);
  displacement_info.angular.z = d_data(5);

  ROS_INFO("Displacement: %f %f %f %f %f %f", d_data(0), d_data(1), d_data(2), d_data(3), d_data(4), d_data(5));
  
  // and publish the data!
  pub_stewart_processing_.publish(displacement_info);


  
}

} // namespace tactile_perception

int main(int argc, char **argv) 
{
  ros::init(argc, argv, "stewart_wrist_node");
  ros::NodeHandle nh;

  tactile_perception::PlatformPublisher node(nh);

  ros::spin();  
  
  return 0;
}