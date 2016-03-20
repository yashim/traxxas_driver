#include "ros/ros.h"
#include "std_msgs/String.h"
#include "ackermann_msgs/AckermannDrive.h"
#include <string>
#include <stdlib.h>     /* abs */
#include "JHPWMPCA9685.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

PCA9685 *pca9685 ;

// The Steering Servo is plugged into the follow PWM channel
#define STEERING_CHANNEL 0
// The ESC is plugged into the following PWM channel
#define ESC_CHANNEL 1


#define PWM_STEERING_FULL_REVERSE 204
#define PWM_STEERING_NEUTRAL 307
#define PWM_STEERING_FULL_FORWARD 409

#define PWM_THROTTLE_FULL_REVERSE 279
#define PWM_THROTTLE_NEUTRAL 288
#define PWM_THROTTLE_FULL_FORWARD 297

float currentSteeringOutput;
float currentThrottleOutput;

float targetSteeringOutput;


void turn(float degrees, int side) {
    pca9685->setPWM(STEERING_CHANNEL,0,PWM_STEERING_NEUTRAL + 100 * degrees);
} 

void throttle(float amount, int direction) {
    pca9685->setPWM(ESC_CHANNEL,0,PWM_THROTTLE_NEUTRAL + 9 * direction);
}

void callback(const ackermann_msgs::AckermannDrive::ConstPtr& msg)
{
    if (msg->speed > 0) {
       throttle(1.0, 1);
    } else if (msg->speed < 0) {
       throttle(1.0, -1);
    } else {
       throttle(1.0, 0);
    } 

    if (msg->steering_angle > 0) {  
       turn(msg->steering_angle, 1);
    } else if (msg->steering_angle < 0) {
       turn(msg->steering_angle, -1);
    } else {
       turn(0.0, 0);
    }
}

void checkSystems() 
{
  pca9685->setAllPWM(0,0) ;
  pca9685->reset() ;
  pca9685->setPWMFrequency(50) ;

  sleep(1.0) ;

  pca9685->setPWM(STEERING_CHANNEL,0,PWM_STEERING_NEUTRAL);

  sleep(1.0);

  targetSteeringOutput = PWM_STEERING_NEUTRAL + 70;
  currentSteeringOutput = PWM_STEERING_NEUTRAL;

  while (abs(currentSteeringOutput -targetSteeringOutput) > 0.5f) {
     currentSteeringOutput += 0.2f;
     pca9685->setPWM(STEERING_CHANNEL,0,currentSteeringOutput);
  }

  sleep(1.0);
  
  pca9685->setPWM(STEERING_CHANNEL,0,PWM_STEERING_NEUTRAL);

  sleep(1.0);

  targetSteeringOutput = PWM_STEERING_NEUTRAL - 70;
  currentSteeringOutput = PWM_STEERING_NEUTRAL;



  while (abs(currentSteeringOutput -targetSteeringOutput) > 0.5f) {
     currentSteeringOutput -= 0.2f;
     pca9685->setPWM(STEERING_CHANNEL,0,currentSteeringOutput);
  }

  sleep(1.0);

  pca9685->setPWM(STEERING_CHANNEL,0,PWM_STEERING_NEUTRAL);
}

int main(int argc, char **argv)
{

  pca9685 = new PCA9685() ;
  int err = pca9685->openPCA9685();
  if (err < 0){
      printf("Error: %d", pca9685->error);
  }
  printf("PCA9685 Device Address: 0x%02X\n",pca9685->kI2CAddress) ;
  
  checkSystems();

  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "listener");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("drivecmd", 1000, callback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}
