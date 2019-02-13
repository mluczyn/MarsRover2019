#include <chrono>
#include <queue>
#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Twist.h"

struct MotionPhase
{
  std::chrono::duration<double> duration;
  double linVelocity;
  double angVelocity;
};
//Constants
#define ACCEL 0 // Linear acceleration
#define DECEL 0 // Linear deceleration
#define ANGV 0 // Angular velocity (rate of rotation)
#define MAXLV 0 // Max linear velocity

//State Values
double cVelocity; // Current linear velocity
std::chrono::steady_clock::time_point phaseStartTime;
std::queue<MotionPhase> phaseQueue;

void ballCoordCallback(const geometry_msgs::Vector3::ConstPtr& coords)
{
  double optimalVelocity = MAXLV;
  double omega = ANGV;

  std::queue<MotionPhase> newQueue;
  //First phase: accelerate to optimal linear velocity
  double dur1 = optimalVelocity/ ACCEL;
  newQueue.push_back({std::chrono::duration<double>(dur1), optimalVelocity, 0.0f});
  double newy = coords.y - dur1 * optimalVelocity / 2.0; 

  //Second phase: match heading
  double theta = atan(coords.x/newy);
  newQueue.push_back({std::chrono::duration<double>(theta/omega), optimalVelocity, omega});

  double chordLen = 2.0 * (optimalVelocity / omega) * sin(theta / 2);
  double ballDist = newy * sec(theta);
  double phi = theta / 2;
  double dist3 = sqrt((chordLen ^ 2) + (ballDist ^ 2) - 2 * chordLen * ballDist * cos(phi));
  double dur3 = dist3 / optimalVelocity;
  newQueue.push_back({std::chrono::duration<double>(dur3), optimalVelocity, 0.0f});
  
  newQueue.push_back({std::chrono::duration<double>(0), 0, 0});
  phaseQueue = newQueue;
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "ball_follow");

  ros::NodeHandle n;
  ros::Subscriber coordSubscriber = n.subscribe("ball_coord", 1, ballCoordCallback);
  ros::Publisher driveSysPub = n.advertise<geometry_msgs::Twist>("TODO", 1);
  ros::Rate updateFrequency(1000);

  while(ros::ok())
  {
    ros::spinOnce();
    
    if(phaseQueue.empty()) continue;
 
    if((std::chrono::steady_clock::now() - phaseStartTime) >= phaseQueue.front().duration)
    {
      phaseQueue.pop();
      if(phaseQueue.empty()) continue;
      
      geometry_msgs::Twist msg;
      msg.linear.y = phaseQueue.front().linVelocity;
      msg.angular.z = phaseQueue.front().angVelocity;
      driveSysPub.publish(msg);
      phaseStartTime = std::chrono::steady_clock::now();
    }
    updateFrequency.sleep();
  }

}
