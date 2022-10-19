/* joystick_node.cpp

 * Copyright (C) 2021 SS47816

 * ROS Node for controlling the vehicle using joysticks
 
**/

#include <math.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Joy.h>
#include <lgsvl_msgs/VehicleControlData.h>
#include <lgsvl_msgs/VehicleStateData.h>
#include <autoware_msgs/VehicleCmd.h>


class JoystickTeleop
{
public:
  JoystickTeleop();

private:
  bool is_healthy_ = true;
  int curr_mode_;
  int curr_gear_;
  float deadzone_;
  float steering_limit_;           // [deg]
  std::string joy_type_;
  std::string control_setting_;
  std::string steering_mapping_;
  std_msgs::String curr_mode_name_;
  std_msgs::String curr_gear_name_;

  ros::NodeHandle nh;
  ros::Subscriber joystick_sub;
  ros::Subscriber autonomous_cmd_sub;
  ros::Subscriber health_monitor_sub;
  ros::Publisher vehicle_cmd_pub;
  ros::Publisher vehicle_state_pub;
  ros::Publisher pub_curr_mode;
  ros::Publisher pub_curr_gear;

  void joystickCallback(const sensor_msgs::Joy::ConstPtr& joy_msg);
  void autonomousCmdCallback(const autoware_msgs::VehicleCmd::ConstPtr& auto_cmd_msg);
  // void healthMonitorCallback(const agv::HealthMonitor::ConstPtr& health_msg);
};

JoystickTeleop::JoystickTeleop()
{
  ros::NodeHandle private_nh("~");

  std::string joy_topic;
  std::string autonomous_cmd_topic;
  std::string vehicle_cmd_topic;
  std::string vehicle_state_topic;
  std::string curr_mode_topic;
  std::string curr_gear_topic;
  // std::string health_monitor_topic;

  ROS_ASSERT(private_nh.getParam("joy_topic", joy_topic));
  ROS_ASSERT(private_nh.getParam("joy_type", joy_type_));
  ROS_ASSERT(private_nh.getParam("control_setting", control_setting_));
  ROS_ASSERT(private_nh.getParam("steering_mapping", steering_mapping_));
  
  ROS_ASSERT(private_nh.getParam("autonomous_cmd_topic", autonomous_cmd_topic));
  ROS_ASSERT(private_nh.getParam("vehicle_cmd_topic", vehicle_cmd_topic));
  ROS_ASSERT(private_nh.getParam("vehicle_state_topic", vehicle_state_topic));
  ROS_ASSERT(private_nh.getParam("curr_mode_topic", curr_mode_topic));
  ROS_ASSERT(private_nh.getParam("curr_gear_topic", curr_gear_topic));
  // ROS_ASSERT(private_nh.getParam("health_monitor_topic", health_monitor_topic));
  ROS_ASSERT(private_nh.getParam("steering_limit", steering_limit_));
  ROS_ASSERT(private_nh.getParam("deadzone", deadzone_));

  joystick_sub = nh.subscribe(joy_topic, 1, &JoystickTeleop::joystickCallback, this);
  autonomous_cmd_sub = nh.subscribe(autonomous_cmd_topic, 1, &JoystickTeleop::autonomousCmdCallback, this);
  // health_monitor_sub = nh.subscribe(health_monitor_topic, 1, &JoystickTeleop::healthMonitorCallback, this);
  vehicle_cmd_pub = nh.advertise<lgsvl_msgs::VehicleControlData>(vehicle_cmd_topic, 1);
  vehicle_state_pub = nh.advertise<lgsvl_msgs::VehicleStateData>(vehicle_state_topic, 1);
  pub_curr_mode = nh.advertise<std_msgs::String>(curr_mode_topic, 1);
  pub_curr_gear = nh.advertise<std_msgs::String>(curr_gear_topic, 1);

  curr_mode_name_.data = "MANUAL";
  curr_gear_name_.data = "PARKING";
  curr_mode_ = lgsvl_msgs::VehicleStateData::VEHICLE_MODE_COMPLETE_MANUAL;
  curr_gear_ == lgsvl_msgs::VehicleControlData::GEAR_PARKING;
  ROS_INFO("joy_type: using %s\n", joy_type_.c_str());
}

void JoystickTeleop::joystickCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{
  bool A, B, X, Y, LB, RB, button_stick_left, button_stick_right;
  float LT, RT, LR_axis_stick_L, UD_axis_stick_L, LR_axis_stick_R, UD_axis_stick_R, cross_key_LR, cross_key_UD;
  float accel_axes, brake_axes, steer_axes;

  lgsvl_msgs::VehicleControlData vehicle_cmd;
  vehicle_cmd.header = joy_msg->header;
  vehicle_cmd.header.frame_id = "base_link";
  vehicle_cmd.target_wheel_angular_rate = 0.0;
  vehicle_cmd.acceleration_pct = 0.0;
  vehicle_cmd.braking_pct = 1.0;

  lgsvl_msgs::VehicleStateData vehicle_state;
  vehicle_state.header = joy_msg->header;
  vehicle_state.header.frame_id = "base_link";
  vehicle_state.blinker_state = lgsvl_msgs::VehicleStateData::BLINKERS_OFF;
  vehicle_state.headlight_state = lgsvl_msgs::VehicleStateData::HEADLIGHTS_OFF;
  vehicle_state.wiper_state = lgsvl_msgs::VehicleStateData::WIPERS_OFF;
  vehicle_state.autonomous_mode_active = 0;
  vehicle_state.hand_brake_active = 0;
  vehicle_state.horn_active = 0;
  
  // Select the joystick type used
  if (joy_type_.compare("F710") == 0) // Logitech F710 XInput Mode
  {
    A = joy_msg->buttons[0];
    B = joy_msg->buttons[1];
    X = joy_msg->buttons[2];
    Y = joy_msg->buttons[3];
    // LB = joy_msg->buttons[4];                   // doing nothing
    // RB = joy_msg->buttons[5];                   // doing nothing
    
    // button_stick_left = joy_msg->buttons[10];   // doing nothing
    // button_stick_right = joy_msg->buttons[11];  // doing nothing
    
    cross_key_LR = joy_msg->axes[0];
    cross_key_UD = joy_msg->axes[1];
    LT = joy_msg->axes[2];                      // [0, 1], doing nothing
    LR_axis_stick_R = joy_msg->axes[3];
    UD_axis_stick_R = joy_msg->axes[4];
    RT = joy_msg->axes[5];                      // [0, 1], release the full power

    // LR_axis_stick_L = joy_msg->axes[6];         // doing nothing
    // UD_axis_stick_L = joy_msg->axes[7];         // doing nothing
  }
  else if (joy_type_.compare("Xbox") == 0)      // default using xbox wired controller
  {
    A = joy_msg->buttons[0];
    B = joy_msg->buttons[1];
    X = joy_msg->buttons[2];
    Y = joy_msg->buttons[3];
    // LB = joy_msg->buttons[4];                   // doing nothing
    // RB = joy_msg->buttons[5];                   // doing nothing

    // button_stick_left = joy_msg->buttons[9];    // doing nothing
    // button_stick_right = joy_msg->buttons[10];  // doing nothing
    
    LR_axis_stick_L = joy_msg->axes[0];         
    UD_axis_stick_L = joy_msg->axes[1];
    LT = joy_msg->axes[2];                      // [1.0, -1.0], doing nothing
    LR_axis_stick_R = joy_msg->axes[3];
    UD_axis_stick_R = joy_msg->axes[4];
    RT = joy_msg->axes[5];                      // [1.0, -1.0], release the full power

    // cross_key_LR = joy_msg->axes[6];            // doing nothing
    // cross_key_UD = joy_msg->axes[7];            // doing nothing
  }
  else
  {
    ROS_ERROR("[joystick_node]: Invalid joystick type (%s) used, 'Xbox' or 'F710' expected", joy_type_.c_str());
    return;
  }

  // Select the joystick control setting used
  if (control_setting_.compare("ForzaHorizon") == 0)
  {
    accel_axes = (-RT + 1.0)/2.0;
    brake_axes = (-LT + 1.0)/2.0;
    steer_axes = LR_axis_stick_L;
  }
  else if (control_setting_.compare("JapanHand") == 0)
  {
    accel_axes = UD_axis_stick_R;
    brake_axes = -accel_axes;
    steer_axes = LR_axis_stick_L;
  }
  else if (control_setting_.compare("USAHand") == 0)
  {
    accel_axes = UD_axis_stick_L;
    brake_axes = -accel_axes;
    steer_axes = LR_axis_stick_R;
  }
  else
  {
    ROS_ERROR("[joystick_node]: Invalid control setting (%s) used, 'ForzaHorizon', 'JapanHand' or 'USAHand' expected", control_setting_.c_str());
    return;
  }

  // Switch Vehicle Mode and Gear 
  if (B)
  {
    // Manual Mode, Parking Gear
    curr_mode_name_.data = "MANUAL";
    curr_mode_ = lgsvl_msgs::VehicleStateData::VEHICLE_MODE_COMPLETE_MANUAL;
    curr_gear_name_.data = "PARKING";
    curr_gear_ = lgsvl_msgs::VehicleControlData::GEAR_PARKING;
  }
  else if (X)
  {
    // Manual Mode, Forward Gear
    curr_mode_name_.data = "MANUAL";
    curr_mode_ = lgsvl_msgs::VehicleStateData::VEHICLE_MODE_COMPLETE_MANUAL;
    curr_gear_name_.data = "DRIVE";
    curr_gear_ = lgsvl_msgs::VehicleControlData::GEAR_DRIVE;
  }
  else if (Y)
  {
    // Manual Mode, Reverse Gear
    curr_mode_name_.data = "MANUAL";
    curr_mode_ = lgsvl_msgs::VehicleStateData::VEHICLE_MODE_COMPLETE_MANUAL;
    curr_gear_name_.data = "REVERSE";
    curr_gear_ = lgsvl_msgs::VehicleControlData::GEAR_REVERSE;
  }
  else if (A)
  {
    // Autonomous Mode, Auto Gear
    if (!is_healthy_)
    {
      // Emergency Mode, Neutral Gear
      curr_mode_name_.data = "EMERGENCY";
      curr_mode_ = lgsvl_msgs::VehicleStateData::VEHICLE_MODE_EMERGENCY_MODE;
      curr_gear_name_.data = "NEUTRAL";
      curr_gear_ = lgsvl_msgs::VehicleControlData::GEAR_NEUTRAL;
      vehicle_state.hand_brake_active = 1;
      ROS_ERROR("[Emergency Mode]: Unhealthy vehicle!");
      return;
    }
    else if (curr_mode_ == lgsvl_msgs::VehicleStateData::VEHICLE_MODE_COMPLETE_MANUAL 
          && curr_gear_ == lgsvl_msgs::VehicleControlData::GEAR_PARKING)
    {
      // Autonomous Mode, Drive TBD
      curr_mode_name_.data = "AUTONOMOUS";
      curr_mode_ = lgsvl_msgs::VehicleStateData::VEHICLE_MODE_COMPLETE_AUTO_DRIVE;
    }
    else if (curr_mode_ == lgsvl_msgs::VehicleStateData::VEHICLE_MODE_COMPLETE_AUTO_DRIVE)
    {
      ROS_DEBUG("[ Auto Mode ]: Vehicle Already in Autonomous Mode");
      return;
    }
    else
    {
      ROS_DEBUG("[joystick_node]: Can only enter Autonomous Mode from Brake Mode!");
      return;
    }
  }

  // Publish vehicle control messages based on vehicle mode
  if (curr_mode_ == lgsvl_msgs::VehicleStateData::VEHICLE_MODE_COMPLETE_AUTO_DRIVE)
  {
    // Empty Action, Let autonomousCmdCallback() function handle publishing autonomous mode messages
    pub_curr_mode.publish(curr_mode_name_);
    pub_curr_gear.publish(curr_gear_name_);
    return;
  }
  else if (curr_mode_ == lgsvl_msgs::VehicleStateData::VEHICLE_MODE_COMPLETE_MANUAL)
  {
    // Map Accel & Brake axes output
    if (curr_gear_ == lgsvl_msgs::VehicleControlData::GEAR_DRIVE)
    {
      vehicle_cmd.acceleration_pct = accel_axes;
      vehicle_cmd.braking_pct = brake_axes;
    }
    else if (curr_gear_ == lgsvl_msgs::VehicleControlData::GEAR_REVERSE)
    {
      vehicle_cmd.acceleration_pct = brake_axes;
      vehicle_cmd.braking_pct = accel_axes;
    }
    
    // Map Steering axes output
    if (control_setting_.compare("Quadratic") == 0)
    {
      vehicle_cmd.target_wheel_angle = -(steer_axes*steer_axes)*steering_limit_/180.0*M_PI;
    }
    else
    {
      vehicle_cmd.target_wheel_angle = -steer_axes*steering_limit_/180.0*M_PI;
    }
    // ROS_DEBUG("[Manual Mode]: Steering Goal Angle: %.1f [deg] Throttle Value: %.2f", 
    //           joy_type_.c_str(), vehicle_cmd.target_wheel_angle*180.0/M_PI, vehicle_cmd.acceleration_pct);
  }
  else 
  {
    // Emergency Mode, Neutral Gear
    curr_mode_name_.data = "EMERGENCY";
    curr_mode_ = lgsvl_msgs::VehicleStateData::VEHICLE_MODE_EMERGENCY_MODE;
    curr_gear_name_.data = "NEUTRAL";
    curr_gear_ = lgsvl_msgs::VehicleControlData::GEAR_NEUTRAL;
    vehicle_state.hand_brake_active = 1;
    ROS_ERROR("[Emergency Mode]: Unhealthy vehicle!");
  }

  // Publish final vehicle command, state, mode, and gear messages
  vehicle_cmd.target_gear = curr_gear_;
  vehicle_cmd_pub.publish(std::move(vehicle_cmd));
  vehicle_state.vehicle_mode = curr_mode_;
  vehicle_state.current_gear = curr_gear_;
  vehicle_state_pub.publish(std::move(vehicle_state));
  pub_curr_mode.publish(curr_mode_name_);
  pub_curr_gear.publish(curr_gear_name_);

  return;
}

void JoystickTeleop::autonomousCmdCallback(const autoware_msgs::VehicleCmd::ConstPtr& auto_cmd_msg)
{
  if (curr_mode_ != lgsvl_msgs::VehicleStateData::VEHICLE_MODE_COMPLETE_AUTO_DRIVE)
  {
    return;
  }

  lgsvl_msgs::VehicleControlData vehicle_cmd;
  vehicle_cmd.header = auto_cmd_msg->header;
  vehicle_cmd.header.frame_id = "base_link";
  
  lgsvl_msgs::VehicleStateData vehicle_state;
  vehicle_state.header = auto_cmd_msg->header;
  vehicle_state.header.frame_id = "base_link";
  vehicle_state.blinker_state = lgsvl_msgs::VehicleStateData::BLINKERS_OFF;
  vehicle_state.headlight_state = lgsvl_msgs::VehicleStateData::HEADLIGHTS_OFF;
  vehicle_state.wiper_state = lgsvl_msgs::VehicleStateData::WIPERS_OFF;
  vehicle_state.vehicle_mode = lgsvl_msgs::VehicleStateData::VEHICLE_MODE_COMPLETE_AUTO_DRIVE;
  vehicle_state.autonomous_mode_active = 1;
  vehicle_state.hand_brake_active = 0;
  vehicle_state.horn_active = 0;

  if (!is_healthy_)
  {
    // Emergency Mode, Neutral Gear
    curr_mode_name_.data = "EMERGENCY";
    curr_mode_ = lgsvl_msgs::VehicleStateData::VEHICLE_MODE_EMERGENCY_MODE;
    curr_gear_name_.data = "NEUTRAL";
    curr_gear_ = lgsvl_msgs::VehicleControlData::GEAR_NEUTRAL;
    vehicle_state.hand_brake_active = 1;
    ROS_ERROR("[Emergency Mode]: Unhealthy vehicle!");
  }
  else if (auto_cmd_msg->gear_cmd.gear == autoware_msgs::Gear::PARK)
  {
    curr_gear_ = lgsvl_msgs::VehicleControlData::GEAR_PARKING;
    curr_gear_name_.data = "PARKING";
  }
  else if (auto_cmd_msg->gear_cmd.gear == autoware_msgs::Gear::DRIVE)
  {
    curr_gear_ = lgsvl_msgs::VehicleControlData::GEAR_DRIVE;
    curr_gear_name_.data = "DRIVE";
    vehicle_cmd.acceleration_pct = std::fabs(std::max(0.0, auto_cmd_msg->twist_cmd.twist.linear.x));
    vehicle_cmd.braking_pct = std::fabs(std::min(0.0, auto_cmd_msg->twist_cmd.twist.linear.x));
    vehicle_cmd.target_wheel_angle = -(auto_cmd_msg->twist_cmd.twist.angular.z);
  }
  else if (auto_cmd_msg->gear_cmd.gear == autoware_msgs::Gear::REVERSE)
  {
    curr_gear_ = lgsvl_msgs::VehicleControlData::GEAR_REVERSE;
    curr_gear_name_.data = "REVERSE";
  }
  else if (auto_cmd_msg->gear_cmd.gear == autoware_msgs::Gear::LOW)
  {
    curr_gear_ = lgsvl_msgs::VehicleControlData::GEAR_LOW;
    curr_gear_name_.data = "LOW";
  }
  else
  {
    curr_gear_ = lgsvl_msgs::VehicleControlData::GEAR_NEUTRAL;
    curr_gear_name_.data = "NEUTRAL";
  }

  // ROS_DEBUG("[ Auto Mode ]: Steering Goal Angle: %.1f [deg] Throttle Value: %.2f", 
  //           joy_type_.c_str(), vehicle_cmd.target_wheel_angle*180.0/M_PI, vehicle_cmd.acceleration_pct);
  vehicle_cmd.target_gear = curr_gear_;
  vehicle_cmd_pub.publish(std::move(vehicle_cmd));
  vehicle_state.current_gear = curr_gear_;
  vehicle_state_pub.publish(std::move(vehicle_state));
  pub_curr_mode.publish(curr_mode_name_);
  pub_curr_gear.publish(curr_gear_name_);
  
  return;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joystick_node");
  JoystickTeleop joystick_teleop_obj;
  ros::spin();
  return 0;
}

