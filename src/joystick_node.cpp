/*
 * Joystick Teleop for mapping desired values
 * Output message is of type geometry_msgs::Twist
 * Mapping for output message:
 * linear.x => desired_velocity [0.0 - 1.0] in propotional
 * linear.z => NavMode
 * angular.z => steering angle [0.0 - 1.0] in propotional
**/ 

#include <math.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <autoware_msgs/VehicleCmd.h>
#include <sensor_msgs/Joy.h>

enum class NavMode
{
  Brake,
  FailSafe,
  Manual,
  Autonomous
};

class JoystickTeleop
{
public:
  JoystickTeleop();

private:
  bool is_healthy_ = true;
  double max_speed_fwd_;                // [m/s]
  double max_speed_rev_;                // [m/s]
  double max_steering_angle_;           // [deg]
  double speed_limit_;                  // [m/s]
  std::string joy_type_;
  std::string control_setting_;
  
  NavMode current_nav_mode_;
  geometry_msgs::Twist cmd_vel_out_;
  autoware_msgs::Gear current_gear_;
  autoware_msgs::VehicleCmd vehicle_cmd_;

  ros::NodeHandle nh;
  ros::Subscriber joystick_sub;
  ros::Subscriber autonomous_cmd_sub;
  ros::Subscriber health_monitor_sub;
  ros::Publisher cmd_vel_pub;
  ros::Publisher vehicle_cmd_pub;

  void joystickCallback(const sensor_msgs::Joy::ConstPtr& joy_msg);
  void autonomousCmdVelCallback(const geometry_msgs::Twist::ConstPtr& autonomous_vel_msg);
  // void healthMonitorCallback(const agv::HealthMonitor::ConstPtr& health_msg);
};

JoystickTeleop::JoystickTeleop()
{
  ros::NodeHandle private_nh("~");

  std::string joy_topic;
  std::string autonomous_cmd_topic;
  std::string cmd_vel_out_topic;
  std::string vehicle_cmd_topic;
  // std::string health_monitor_topic;

  ROS_ASSERT(private_nh.getParam("joy_topic", joy_topic));
  ROS_ASSERT(private_nh.getParam("joy_type", joy_type_));
  ROS_ASSERT(private_nh.getParam("control_setting", control_setting_));
  
  ROS_ASSERT(private_nh.getParam("cmd_vel_out_topic", cmd_vel_out_topic));
  ROS_ASSERT(private_nh.getParam("vehicle_cmd_topic", vehicle_cmd_topic));
  ROS_ASSERT(private_nh.getParam("autonomous_cmd_topic", autonomous_cmd_topic));
  // ROS_ASSERT(private_nh.getParam("health_monitor_topic", health_monitor_topic));

  ROS_ASSERT(private_nh.getParam("max_speed_fwd", max_speed_fwd_));
  ROS_ASSERT(private_nh.getParam("max_speed_rev", max_speed_rev_));
  ROS_ASSERT(private_nh.getParam("max_steering_angle", max_steering_angle_));

  joystick_sub = nh.subscribe(joy_topic, 1, &JoystickTeleop::joystickCallback, this);
  autonomous_cmd_sub = nh.subscribe(autonomous_cmd_topic, 1, &JoystickTeleop::autonomousCmdVelCallback, this);
  // health_monitor_sub = nh.subscribe(health_monitor_topic, 1, &JoystickTeleop::healthMonitorCallback, this);
  cmd_vel_pub = nh.advertise<geometry_msgs::Twist>(cmd_vel_out_topic, 1);
  vehicle_cmd_pub = nh.advertise<autoware_msgs::VehicleCmd>(vehicle_cmd_topic, 1);

  speed_limit_ = 0.0;
  current_gear_.gear = autoware_msgs::Gear::NONE;
  current_nav_mode_ = NavMode::Brake;
  ROS_INFO("joy_type: using %s\n", joy_type_.c_str());
}


void JoystickTeleop::joystickCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  bool A, B, X, Y, LB, RB, button_stick_left, button_stick_right;
  double LT, RT, LR_axis_stick_L, UD_axis_stick_L, LR_axis_stick_R, UD_axis_stick_R, cross_key_LR, cross_key_UD;
  double forward_axes, reverse_axes, steering_axes;
  
  // Select the joystick type used
  if (joy_type_.compare("F710") == 0) // Logitech F710 XInput Mode
  {
    A = joy_msg->buttons[0];
    B = joy_msg->buttons[1];
    X = joy_msg->buttons[2];
    Y = joy_msg->buttons[3];
    LB = joy_msg->buttons[4];
    RB = joy_msg->buttons[5];
    
    button_stick_left = joy_msg->buttons[10];   // doing nothing
    button_stick_right = joy_msg->buttons[11];  // doing nothing
    
    cross_key_LR = joy_msg->axes[0];
    cross_key_UD = joy_msg->axes[1];
    LT = joy_msg->axes[2];                      // [0, 1], doing nothing
    LR_axis_stick_R = joy_msg->axes[3];
    UD_axis_stick_R = joy_msg->axes[4];
    RT = joy_msg->axes[5];                      // [0, 1], release the full power
    LR_axis_stick_L = joy_msg->axes[6];
    UD_axis_stick_L = joy_msg->axes[7];
  }
  else if (joy_type_.compare("Xbox") == 0)      // default using xbox wired controller
  {
    A = joy_msg->buttons[0];
    B = joy_msg->buttons[1];
    X = joy_msg->buttons[2];
    Y = joy_msg->buttons[3];
    LB = joy_msg->buttons[4];
    RB = joy_msg->buttons[5];

    button_stick_left = joy_msg->buttons[9];    // doing nothing
    button_stick_right = joy_msg->buttons[10];  // doing nothing
    
    LR_axis_stick_L = joy_msg->axes[0];         
    UD_axis_stick_L = joy_msg->axes[1];
    LT = joy_msg->axes[2];                      // [1.0, -1.0], doing nothing
    LR_axis_stick_R = joy_msg->axes[3];
    UD_axis_stick_R = joy_msg->axes[4];
    RT = joy_msg->axes[5];                      // [1.0, -1.0], release the full power
    cross_key_LR = joy_msg->axes[6];
    cross_key_UD = joy_msg->axes[7];
  }
  else
  {
    ROS_ERROR("[joystick_node]: Invalid joystick type (%s) used, 'Xbox' or 'F710' expected", joy_type_.c_str());
    return;
  }

  // Select the joystick control setting used
  if (control_setting_.compare("RightHand") == 0)
  {
    forward_axes = UD_axis_stick_R;
    steering_axes = LR_axis_stick_R;
    // Switch Speed Limit
    speed_limit_ = forward_axes >= 0.0? max_speed_fwd_ : -max_speed_rev_;
  }
  else if (control_setting_.compare("ForzaHorizon") == 0)
  {
    forward_axes = (-RT + 1.0)/2.0;
    reverse_axes = (-LT + 1.0)/2.0;
    steering_axes = LR_axis_stick_L;

    speed_limit_ = reverse_axes > 0.05? -max_speed_rev_ : max_speed_fwd_;
    // forward_axes = reverse_axes > 0.05? forward_axes : reverse_axes;
  }
  else
  {
    ROS_ERROR("[joystick_node]: Invalid control setting (%s) used, 'RightHand' or 'ForzaHorizon' expected", control_setting_.c_str());
  }

  // Switch NavMode
  if (B == true)
  {
    current_nav_mode_ = NavMode::Brake;
    ROS_INFO("[joystick_node] %s: Entered Brake Mode", joy_type_.c_str());
  }
  else if (X == true)
  {
    current_nav_mode_ = NavMode::Manual;
    ROS_INFO("[joystick_node] %s: Entered Manual Mode", joy_type_.c_str());
  }
  else if (A == true)
  {
    if (is_healthy_ == false)
    {
      current_nav_mode_ = NavMode::FailSafe;
      ROS_ERROR("[joystick_node] %s: Unhealthy vehicle! Going to FailSafe Mode", joy_type_.c_str());
    }
    else if (current_nav_mode_ == NavMode::Brake)
    {
      current_nav_mode_ = NavMode::Autonomous;
      ROS_INFO("[joystick_node] %s: Entered Autonomous Mode", joy_type_.c_str());
    }
    else if (current_nav_mode_ == NavMode::Autonomous)
    {
      ROS_INFO("[joystick_node] %s: Already in Autonomous Mode", joy_type_.c_str());
    }
    else
    {
      ROS_INFO("[joystick_node] %s: Can only enter Autonomous Mode from Brake Mode!", joy_type_.c_str());
    }
    return;
  }

  if (current_nav_mode_ == NavMode::Autonomous)
  {
    // Empty Else, Let autonomousCmdVelCallback() function handle publishing autonomous mode messages
    return;
  }
  else if (current_nav_mode_ == NavMode::Manual)
  {
    // Throttle & Steering 
    cmd_vel_out_.linear.x = forward_axes * speed_limit_;
    cmd_vel_out_.angular.z = steering_axes * max_steering_angle_/180*M_PI;
    vehicle_cmd_.mode = (int)current_nav_mode_;
  }
  else 
  {
    cmd_vel_out_.linear.x = 0;
    cmd_vel_out_.angular.z = 0;
    vehicle_cmd_.mode = (int)current_nav_mode_;
  }

  vehicle_cmd_.header = joy_msg->header;
  vehicle_cmd_.header.frame_id = "base_link";
  vehicle_cmd_.twist_cmd.twist = cmd_vel_out_;
  vehicle_cmd_.gear_cmd = current_gear_;
  
  // 
  // vehicle_cmd_.steer_cmd.steer = -1;
  // vehicle_cmd_.accel_cmd.accel = 1;

  // vehicle_cmd_.ctrl_cmd.linear_acceleration = forward_axes * 3;
  // vehicle_cmd_.ctrl_cmd.linear_velocity = forward_axes * speed_limit_;
  // vehicle_cmd_.ctrl_cmd.steering_angle = steering_axes * max_steering_angle_;

  // Publish the final msgs
  cmd_vel_pub.publish(cmd_vel_out_);
  vehicle_cmd_pub.publish(vehicle_cmd_);
  ROS_INFO("[joystick_node] %s: Steering Goal Angle: %.2f [deg] Throttle Goal Speed: %.2f [m/s]", 
            joy_type_.c_str(), cmd_vel_out_.angular.z * max_steering_angle_, cmd_vel_out_.linear.x);
  
  return;
}

void JoystickTeleop::autonomousCmdVelCallback(const geometry_msgs::Twist::ConstPtr& autonomous_vel_msg)
{
  if (current_nav_mode_ == NavMode::Autonomous)
  {
    // if we are healthy: autonomous variable brake.
    if (is_healthy_ == true)
    {
      ROS_INFO("[joystick_node] %s: Autonomous Driving", joy_type_.c_str());
      cmd_vel_out_.linear.x = std::min(autonomous_vel_msg->linear.x, speed_limit_); //set speed to desired speed
      cmd_vel_out_.angular.z = autonomous_vel_msg->angular.z; //steering angle
      cmd_vel_pub.publish(cmd_vel_out_);

      vehicle_cmd_.header.frame_id = "base_link";
      vehicle_cmd_.twist_cmd.twist = cmd_vel_out_;
      vehicle_cmd_pub.publish(vehicle_cmd_);
      ROS_INFO("[joystick_node] %s: Publishing Autonomous Commands", joy_type_.c_str());
    }
    else 
    {
      //if we are unhealthy and in autonomous mode, go to soft brake mode.
      current_nav_mode_ = NavMode::FailSafe;
      cmd_vel_out_.linear.x = 0; //set speed to 0
      cmd_vel_out_.linear.z = (double)current_nav_mode_;
      cmd_vel_out_.angular.z = 0;
      cmd_vel_pub.publish(cmd_vel_out_);

      vehicle_cmd_.header.frame_id = "base_link";
      vehicle_cmd_.twist_cmd.twist = cmd_vel_out_;
      vehicle_cmd_pub.publish(vehicle_cmd_);
      ROS_ERROR("[joystick_node] %s: Unhealthy vehicle! Check sensors! Going to Soft Brake Mode.", joy_type_.c_str());
    }
  }

  ROS_INFO("[joystick_node] %s: Steering Goal Angle: %.2f [deg]  Throttle Goal Speed: %.2f [m/s]", joy_type_.c_str(), cmd_vel_out_.angular.z * max_steering_angle_, cmd_vel_out_.linear.x);

  return;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joystick_node");
  JoystickTeleop joystick_teleop_obj;
  ros::spin();
  return 0;
}

