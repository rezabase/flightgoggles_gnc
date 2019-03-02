#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <flightgoggles_gnc/TrajPoint.h> //Custom message types that I created




// ROS Timer
ros::Timer reset_timer;
void timerCallback(const ros::TimerEvent& event);        

//Reset related
ros::Publisher pub_reset;
float TIMER = 5; //Runs every 10 seconds



//Trajecotryrelated
ros::Publisher pub_trajectory;
class TrajPoints
{
  public:

    flightgoggles_gnc::TrajPoint traj_test_array[4]; //Contains a list of trajectory points for the drone to follow
    int max_traj_points;
    int current_traj_point; //Remembers which one was last used.

    TrajPoints()
    {
      //Creating a few test trajectory points:  
      traj_test_array[0] = new_traj_point(0, 0, 3); //take off
      traj_test_array[1] = new_traj_point(2, 0, 3); //move North and increase altitude
      traj_test_array[2] = new_traj_point(-2, 2, 2); //move South West and decrease altitude
      traj_test_array[3] = new_traj_point(0, 0, 2); //back to starting position
      traj_test_array[4] = new_traj_point(0, 0, 2); //back to starting position
      max_traj_points = 5;
      current_traj_point = 0;
    }

    bool publish_next_trajpoint()
    {
      ROS_INFO(" publish_next_trajpoint: >>>>> Sending new traj nr %d <<<<<< ", current_traj_point);

      if(current_traj_point < max_traj_points){

        //flightgoggles_gnc::TrajPoint msg; // = new_traj_point(0,0,4);
        //pub_trajectory.publish(msg);

        traj_test_array[current_traj_point].header.frame_id = "uav/imu";
        traj_test_array[current_traj_point].header.stamp = ros::Time::now();
        pub_trajectory.publish(traj_test_array[current_traj_point]);
    
        current_traj_point++;
        return true;
      }

      current_traj_point = 0; //resets
      return false;
    }


    flightgoggles_gnc::TrajPoint new_traj_point(float x, float y, float z, float yaw=0, float vx=0, float vy=0, float vz=0)
    {
      flightgoggles_gnc::TrajPoint traj_point;

      //Set trjectory values.
      traj_point.position.x = x;
      traj_point.position.y = y;
      traj_point.position.z = z;

      traj_point.velocity.x = vx;
      traj_point.velocity.y = vy;
      traj_point.velocity.z = vz;

      traj_point.yaw = yaw;  
      return traj_point;
    }

};
TrajPoints my_trajpoints;




void timerCallback(const ros::TimerEvent& event) //Note: this function requires REZA modifications in flightgoggles_uav_dynamics_node
{
  //ROS_INFO(" TRAJ TEST: >>>>> Current traj: %d, max_traj: %d <<<<<< ", current_traj_point, max_traj_points);
  if(!my_trajpoints.publish_next_trajpoint() ){
    //Send reset topic to the flightgoggles_dynamic
    std_msgs::Empty reset_msg;
    pub_reset.publish(reset_msg);
    ROS_INFO(" TRAJ TEST: >>>>> Sent reset <<<<<< ");
  }
}











// ----------
// -- MAIN --
// ----------
using namespace flightgoggles_gnc;
int main(int argc, char** argv)
{
  ros::init(argc, argv, "control_manual_tune");
  ros::NodeHandle n;
  
  //Publishing reset commands to FlightGoggles_dynamics. NOTE: Must use Rezas custom flightgoggles_dynamic code that subscibes to reset commands. 
  pub_reset = n.advertise<std_msgs::Empty>("/uav/input/reset", 1);

  pub_trajectory = n.advertise<flightgoggles_gnc::TrajPoint>("/traj_point", 2);
  
  reset_timer = n.createTimer(ros::Duration(TIMER), timerCallback);

  ros::Rate r(10);
  while(ros::ok()) {
    ros::spinOnce();

    r.sleep();
  }
}
