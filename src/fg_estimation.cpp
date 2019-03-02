#include <ros/ros.h>
#include "sensor_msgs/Imu.h"
#include "fg_estimation.hpp"

using namespace std;

#include "matrix/math.hpp"
#include "Math/Quaternion.h"
using namespace SLR;
using matrix::Vector;
using matrix::Matrix;
using matrix::SquareMatrix;

#include "Eigen/Dense"
#include "Eigen/SVD"
using Eigen::MatrixXf;
using Eigen::VectorXf;



/*--------------------------------------------------------------------
   4  * FG_Estimation()
   5  * Constructor.
   6  *------------------------------------------------------------------*/
flightgoggles_gnc::FG_Estimation::FG_Estimation(void) : 
  n()   /*,
  Q(QUAD_EKF_NUM_STATES, QUAD_EKF_NUM_STATES),
  R_GPS(6, 6),
  R_Mag(1, 1),
  ekfState(QUAD_EKF_NUM_STATES),
  ekfCov(QUAD_EKF_NUM_STATES, QUAD_EKF_NUM_STATES),
  trueError(QUAD_EKF_NUM_STATES) */
{

  //pub_vel = n.advertise<mav_msgs::RateThrust>("/uav/input/rateThrust", 1);
  sub_imu = n.subscribe("/uav/sensors/imu", 1, &FG_Estimation::imuCallback, this);

  

}


/*--------------------------------------------------------------------
  13  * ~FG_Estimation()
  14  * Destructor.
  15  *------------------------------------------------------------------*/
flightgoggles_gnc::FG_Estimation::~FG_Estimation(void)
{

    
}


void flightgoggles_gnc::FG_Estimation::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  ROS_INFO("Imu Seq: [%d]", msg->header.seq);
  ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
  ROS_INFO("Imu angular_velocity x: [%f], y: [%f], z: [%f], w: [%f]", msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z);
  ROS_INFO("Imu linear_acceleration x: [%f], y: [%f], z: [%f], w: [%f]", msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z);

}

void flightgoggles_gnc::FG_Estimation::estimate(void)
{

}
















// ----------
// -- MAIN --
// ----------
using namespace flightgoggles_gnc;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "fg_estimation");

  FG_Estimation fgc;

  ros::Rate r(20);
  while(ros::ok()) {
    ros::spinOnce();
    fgc.estimate();
    r.sleep();
  }
}






