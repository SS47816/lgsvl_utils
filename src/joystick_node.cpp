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
#include <sensor_msgs/Joy.h>
#include <lgsvl_msgs/VehicleControlData.h>
#include <autoware_msgs/VehicleCmd.h>

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
  // double max_speed_fwd_;                // [m/s]
  // double max_speed_rev_;                // [m/s]
  double max_steering_angle_;           // [deg]
  std::string joy_type_;
  std::string control_setting_;
  std::string steering_mapping_;
  NavMode current_nav_mode_;
  int gear_;

  ros::NodeHandle nh;
  ros::Subscriber joystick_sub;
  ros::Subscriber autonomous_cmd_sub;
  ros::Subscriber health_monitor_sub;
  ros::Publisher vehicle_cmd_pub;

  void joystickCallback(const sensor_msgs::Joy::ConstPtr& joy_msg);
  void autonomousCmdVelCallback(const autoware_msgs::VehicleCmd::ConstPtr& auto_cmd_msg);
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
  ROS_ASSERT(private_nh.getParam("steering_mapping", steering_mapping_));
  
  ROS_ASSERT(private_nh.getParam("vehicle_cmd_topic", vehicle_cmd_topic));
  ROS_ASSERT(private_nh.getParam("autonomous_cmd_topic", autonomous_cmd_topic));
  // ROS_ASSERT(private_nh.getParam("health_monitor_topic", health_monitor_topic));
  ROS_ASSERT(private_nh.getParam("max_steering_angle", max_steering_angle_));

  joystick_sub = nh.subscribe(joy_topic, 1, &JoystickTeleop::joystickCallback, this);
  autonomous_cmd_sub = nh.subscribe(autonomous_cmd_topic, 1, &JoystickTeleop::autonomousCmdVelCallback, this);
  // health_monitor_sub = nh.subscribe(health_monitor_topic, 1, &JoystickTeleop::healthMonitorCallback, this);
  vehicle_cmd_pub = nh.advertise<lgsvl_msgs::VehicleControlData>(vehicle_cmd_topic, 1);

  current_nav_mode_ = NavMode::Brake;
  gear_ == lgsvl_msgs::VehicleControlData::GEAR_DRIVE;
  ROS_INFO("joy_type: using %s\n", joy_type_.c_str());
}


void JoystickTeleop::joystickCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  bool A, B, X, Y, LB, RB, button_stick_left, button_stick_right;
  double LT, RT, LR_axis_stick_L, UD_axis_stick_L, LR_axis_stick_R, UD_axis_stick_R, cross_key_LR, cross_key_UD;
  double forward_axes, reverse_axes, steering_axes;

  lgsvl_msgs::VehicleControlData vehicle_cmd;
  vehicle_cmd.header = joy_msg->header;
  vehicle_cmd.header.frame_id = "base_link";
  
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
  }
  else if (control_setting_.compare("ForzaHorizon") == 0)
  {
    forward_axes = (-RT + 1.0)/2.0;
    reverse_axes = (-LT + 1.0)/2.0;
    steering_axes = LR_axis_stick_L;
  }
  else
  {
    ROS_ERROR("[joystick_node]: Invalid control setting (%s) used, 'RightHand' or 'ForzaHorizon' expected", control_setting_.c_str());
  }

  // Construct the control message
  if (gear_ == lgsvl_msgs::VehicleControlData::GEAR_DRIVE)
  {
    vehicle_cmd.acceleration_pct = forward_axes;
    vehicle_cmd.braking_pct = reverse_axes;
    vehicle_cmd.target_gear = gear_;
  }
  else if (gear_ == lgsvl_msgs::VehicleControlData::GEAR_REVERSE)
  {
    vehicle_cmd.acceleration_pct = reverse_axes;
    vehicle_cmd.braking_pct = forward_axes;
    vehicle_cmd.target_gear = gear_;
  }
  
  // Map steering axes output
  if (control_setting_.compare("Quadratic") == 0)
  {
    vehicle_cmd.target_wheel_angle = -(steering_axes*steering_axes)*max_steering_angle_/180.0*M_PI;
  }
  else
  {
    vehicle_cmd.target_wheel_angle = -steering_axes*max_steering_angle_/180.0*M_PI;
  }
  vehicle_cmd.target_gear = gear_;
  vehicle_cmd.target_wheel_angular_rate = 0.0;

  // Switch between Forward and Reverse
  if (Y)
  {
    if (gear_ == lgsvl_msgs::VehicleControlData::GEAR_DRIVE)
    {
      // Reverse Gear
      ROS_WARN("REVERSE");
      gear_ = lgsvl_msgs::VehicleControlData::GEAR_REVERSE;
    }
    else
    {
      // Forward Gear
      ROS_WARN("FORWARD");
      gear_ = lgsvl_msgs::VehicleControlData::GEAR_DRIVE;
    }
  }

  // Switch NavMode
  if (B)
  {
    current_nav_mode_ = NavMode::Brake;
    ROS_INFO("[Brake Mode ] %s: Entered Brake Mode", joy_type_.c_str());
  }
  else if (X)
  {
    current_nav_mode_ = NavMode::Manual;
    ROS_INFO("[Manual Mode] %s: Entered Manual Mode", joy_type_.c_str());
  }
  else if (A)
  {
    if (is_healthy_ == false)
    {
      current_nav_mode_ = NavMode::FailSafe;
      ROS_ERROR("[ Safe Mode ] %s: Unhealthy vehicle! Going to FailSafe Mode", joy_type_.c_str());
    }
    else if (current_nav_mode_ == NavMode::Brake)
    {
      current_nav_mode_ = NavMode::Autonomous;
      ROS_INFO("[Brake Mode ] %s: Entered Autonomous Mode", joy_type_.c_str());
    }
    else if (current_nav_mode_ == NavMode::Autonomous)
    {
      ROS_INFO("[ Auto Mode ] %s: Already in Autonomous Mode", joy_type_.c_str());
    }
    else
    {
      ROS_INFO("[joystick_node] %s: Can only enter Autonomous Mode from Brake Mode!", joy_type_.c_str());
    }
    return;
  }

  // Publish vehicle control messages based on vehicle mode
  if (current_nav_mode_ == NavMode::Manual)
  {
    vehicle_cmd_pub.publish(vehicle_cmd);
    ROS_INFO("[Manual Mode] %s: Steering Goal Angle: %.1f [deg] Throttle Value: %.2f", 
              joy_type_.c_str(), vehicle_cmd.target_wheel_angle*180.0/M_PI, vehicle_cmd.acceleration_pct);
    return;
  }
  else if (current_nav_mode_ == NavMode::Autonomous)
  {
    // Empty Else, Let autonomousCmdVelCallback() function handle publishing autonomous mode messages
    return;
  }
  else 
  {
    current_nav_mode_ = NavMode::FailSafe;
    return;
  }
}

void JoystickTeleop::autonomousCmdVelCallback(const autoware_msgs::VehicleCmd::ConstPtr& auto_cmd_msg)
{
  if (current_nav_mode_ != NavMode::Autonomous)
  {
    return;
  }

  lgsvl_msgs::VehicleControlData vehicle_cmd;
  vehicle_cmd.header = auto_cmd_msg->header;
  vehicle_cmd.header.frame_id = "base_link";
  vehicle_cmd.target_gear = lgsvl_msgs::VehicleControlData::GEAR_NEUTRAL;
  
  if (is_healthy_)
  {
    vehicle_cmd_pub.publish(vehicle_cmd);
    ROS_INFO("[ Auto Mode ] %s: Steering Goal Angle: %.1f [deg] Throttle Value: %.2f", 
              joy_type_.c_str(), vehicle_cmd.target_wheel_angle*180.0/M_PI, vehicle_cmd.acceleration_pct);
    return;
  }
  else 
  {
    // if we are unhealthy and in autonomous mode, go to soft brake mode.
    current_nav_mode_ = NavMode::FailSafe;
    ROS_ERROR("[ Safe Mode ] %s: Unhealthy vehicle! Check sensors! Going to Soft Brake Mode.", joy_type_.c_str());
    return;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joystick_node");
  JoystickTeleop joystick_teleop_obj;
  ros::spin();
  return 0;
}

