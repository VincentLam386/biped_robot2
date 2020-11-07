#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

#include <controller_manager/controller_manager.h>
#include <ros/ros.h>

using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::EffortJointSoftLimitsHandle;
using joint_limits_interface::EffortJointSoftLimitsInterface;

class Biped2_Robot2 : public hardware_interface::RobotHW
{
public:
  Biped2_Robot2(ros::NodeHandle& nh): nh_(nh)
  {
    init();
    cm.reset(new controller_manager::ControllerManager(this,nh_));
    //nh_.param("/ROBOT/hardware_interface/loop_hz", loop_hz_, 0.1);
    loop_hz_ = 100.0;
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
    non_realtime_loop_ = nh_.createTimer(update_freq, &Biped2_Robot2::update, this);
  }

  virtual ~Biped2_Robot2() {}

  bool init(){
    // nh_.getParam("/ROBOT/hardware_interface/joints", joint_names_);
    joint_names_.push_back("left_rear_joint");
    joint_names_.push_back("left_rear_joint2");
    num_joints_ = joint_names_.size();
    pos.resize(num_joints_);
    vel.resize(num_joints_);
    eff.resize(num_joints_);
    cmd.resize(num_joints_);

    for (int i=0;i<num_joints_;++i){
      std::cout << "test" << std::endl;
      // Create joint state interface
      hardware_interface::JointStateHandle jointStateHandle(joint_names_[i], &pos[i],&vel[i],&eff[i]);
      jnt_state_interface.registerHandle(jointStateHandle);
      std::cout << "test2" << std::endl;
      // Create joint position interface
      hardware_interface::JointHandle jointEffortHandle(jointStateHandle, &cmd[i]);
      JointLimits limits;
      SoftJointLimits soft_limits;
      std::cout << "test3" << std::endl;
      // getJointLimits(joint.name, nh_, limits)
      limits = {-2,2,100,100,100,100,true,true,true,true,true,true};
      EffortJointSoftLimitsHandle jointLimitsHandle(
                                     jointEffortHandle, // we read the state and read/write the command
                                     limits,       // Limits spec
                                     soft_limits);  // Soft limits spec
      
      jnt_eff_interface.registerHandle(jointEffortHandle);
      jnt_limits_interface_.registerHandle(jointLimitsHandle);

    }
    // Connect and register all interfaces
    registerInterface(&jnt_state_interface);
    registerInterface(&jnt_eff_interface);
    registerInterface(&jnt_limits_interface_);
  }

  void read()
  {
    std::cout << "Start reading:\n";
    for (int i=0; i<num_joints_;++i){
      // Read actuator state from the hardware and
      // propagate current actuator state to joints...  
      //pos[i] = ROBOT.getJoint(joint_names_[i]).read();

      std::cout<<"read "<<"  "<< pos[i] << "\n";
    }
    std::cout << "End reading\n" << std::endl;
  }

  void update(const ros::TimerEvent& e){
    elapsed_time = ros::Duration(e.current_real - e.last_real);
    read();
    cm->update(ros::Time::now(), elapsed_time); // update controller directly?
    write(elapsed_time);
  }

  void write(const ros::Duration& period)
  {
    // Enforce joint limits for all registered handles
    // Note: one can also enforce limits on a per-handle basis: handle.enforceLimits(period)
    std::cout << "Period: " << period << std::endl;
    //jnt_limits_interface_.enforceLimits(period);
    
    std::cout << "Start writing:\n";
    for (int i=0; i<num_joints_;++i){
      // Propagate joint commands to actuators and
      // Send actuator command to hardware...
      // ROBOT.getJoint(joint_names_[i]).actuate(cmd[i]);

      std::cout<<"write "<<"  "<< pos[i] << "\n";
    }
    std::cout << "End writing\n" << std::endl;

  }

protected:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::EffortJointInterface jnt_eff_interface;
  hardware_interface::EffortJointInterface eff_cmd_interface_;
  joint_limits_interface::EffortJointSoftLimitsInterface jnt_limits_interface_;
  std::shared_ptr<controller_manager::ControllerManager> cm;
  int num_joints_;
  double loop_hz_;
  ros::NodeHandle nh_;
  ros::Timer non_realtime_loop_;
  ros::Duration elapsed_time;
  std::vector<std::string> joint_names_;
  std::vector<double> cmd;
  std::vector<double> pos;
  std::vector<double> vel;
  std::vector<double> eff;
};


