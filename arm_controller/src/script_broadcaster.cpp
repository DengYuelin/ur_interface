#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <geometry_msgs/Twist.h>

const double deg2rad = 0.0174533;

class ScriptController {
 public:
  ScriptController();
  void set_param(double acc, double vel, double radius);
  void joy_callback(const sensor_msgs::Joy::ConstPtr &msg);
  void cmd_callback(const geometry_msgs::Twist::ConstPtr &msg);
  int get_rate();
  void pub_scirpt_speedl();
  void pub_scirpt_servoc();
  void pub_scirpt();

 private:
  bool send_script_;
  int controller_mode;  // choose between different script controllers

  ros::NodeHandle nh_;
  ros::Publisher urscript_pub_;
  ros::Subscriber joy_sub_;
  ros::Subscriber cmd_sub_;

  std_msgs::String ur_script_;
  float joystick_buffer_[6] = {0, 0, 0, 0, 0, 0};
  // ServoC parameters
  double acc_;     // tool acceleration [m/s^2]
  double vel_;     // tool speed [m/s]
  double radius_;  // blend radius (of target pose) [m]

  double x_, y_, z_, rx_, ry_, rz_;        // tool pose in [m], [rad]
  double dx_, dy_, dz_, drx_, dry_, drz_;  // OFFSET in [m], [rad]
};

ScriptController::ScriptController() {
  urscript_pub_ = nh_.advertise<std_msgs::String>(
      "ur_hardware_interface/script_command", 1);
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>(
      "/joy", 1, &ScriptController::joy_callback, this);
  cmd_sub_ = nh_.subscribe<geometry_msgs::Twist>(
      "pose_cmd", 1, &ScriptController::cmd_callback, this);
  send_script_ = false;

  if (nh_.getParam("controller_mode", controller_mode)) {
    ROS_INFO("Controller mode selected: Mode %d", controller_mode);
  } else {
    controller_mode = 0;
    ROS_WARN("Default controller selected");
  }

  // default config
  acc_ = 0.01;
  vel_ = 0.02;
  radius_ = 0.0001;
  nh_.param<double>("default_pose/x", x_, 0.0);
  nh_.param<double>("default_pose/y", y_, 0.0);
  nh_.param<double>("default_pose/z", z_, 0.0);
  nh_.param<double>("default_pose/rx", rx_, 0.0);
  nh_.param<double>("default_pose/ry", ry_, 0.0);
  nh_.param<double>("default_pose/rz", rz_, 0.0);

  // default offset is zero
  dx_ = 0;
  dy_ = 0;
  dz_ = 0;
  drx_ = 0;
  dry_ = 0;
  drz_ = 0;
}

int ScriptController::get_rate() {
  if (controller_mode == 0)       // speedl
    return 50;                    // publish at 50 Hz
  else if (controller_mode == 1)  // servoc
    return 20;
  else
    return 125;
}

void ScriptController::joy_callback(const sensor_msgs::Joy::ConstPtr &msg) {
  joystick_buffer_[0] = msg->axes[0];
  joystick_buffer_[1] = msg->axes[1];
  joystick_buffer_[2] = (msg->axes[2] - msg->axes[5]) / 2;
  joystick_buffer_[3] = msg->axes[3];
  joystick_buffer_[4] = msg->axes[4];
  joystick_buffer_[5] = msg->axes[6];

  if (msg->buttons[0] == 1) {
    dx_ = 0;
    dy_ = 0;
    dz_ = 0;
    drx_ = 0;
    dry_ = 0;
    drz_ = 0;
  }
}

void ScriptController::cmd_callback(const geometry_msgs::Twist::ConstPtr &msg) {
  x_ = msg->linear.x;
  y_ = msg->linear.y;
  z_ = msg->linear.z;
  rx_ = msg->angular.x;
  ry_ = msg->angular.y;
  rz_ = msg->angular.z;
}

// servoc(pose, a=1.2, v=0.25, r=0)
void ScriptController::set_param(double acc, double vel, double radius) {
  acc_ = acc;
  vel_ = vel;
  radius_ = radius;
}

// Joystick control using speedl
void ScriptController::pub_scirpt_speedl() {
  dx_ = -joystick_buffer_[0] * 0.03;
  dy_ = joystick_buffer_[1] * 0.03;
  dz_ = joystick_buffer_[2] * 0.03;
  drx_ = -joystick_buffer_[4] * 0.1;
  dry_ = -joystick_buffer_[3] * 0.1;
  drz_ = joystick_buffer_[5] * 0.1;

  ur_script_.data = "speedl([" + std::to_string(dx_) + "," +
                    std::to_string(dy_) + "," + std::to_string(dz_) + "," +
                    std::to_string(drx_) + "," + std::to_string(dry_) + "," +
                    std::to_string(drz_) + "], 0.5, 0.05, 0.5)";
  urscript_pub_.publish(ur_script_);
}

void ScriptController::pub_scirpt_servoc() {
  // dx_ += -joystick_buffer_[0] * 0.01;
  // dy_ += joystick_buffer_[1] * 0.01;
  // dz_ += joystick_buffer_[2] * 0.01;
  // drx_ += joystick_buffer_[4] * 0.01;
  // dry_ += joystick_buffer_[3] * 0.01;
  // drz_ += joystick_buffer_[5] * 0.01;
  ur_script_.data =
      "servoc(p[" + std::to_string(x_ + dx_) + "," + std::to_string(y_ + dy_) +
      "," + std::to_string(z_ + dz_) + "," + std::to_string(rx_ + drx_) + "," +
      std::to_string(ry_ + dry_) + "," + std::to_string(rz_ + drz_) +
      "], a=" + std::to_string(acc_) + ", v=" + std::to_string(vel_) +
      ", r=" + std::to_string(radius_) + ")";
  urscript_pub_.publish(ur_script_);
}

void ScriptController::pub_scirpt() {
  if (controller_mode == 0)
    pub_scirpt_speedl();
  else if (controller_mode == 1)
    pub_scirpt_servoc();
  else
    ;
}

int main(int argc, char **argv) {
  // Initiate ROS
  ros::init(argc, argv, "script_br_node");

  // Create an object of class Multiplexer that will take care of everything
  ScriptController ScriptController;
  ROS_INFO("Script Controller ready");
  ros::Rate rate(ScriptController.get_rate());  // [Hz]

  while (ros::ok()) {
    ScriptController.pub_scirpt();
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}