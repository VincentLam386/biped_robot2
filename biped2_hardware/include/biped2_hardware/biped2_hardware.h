#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

#include <joint_limits_interface/joint_limits_interface.h>

using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;

class Biped2_Robot : public hardware_interface::RobotHW
{
public:
  Biped2_Robot()
  {
    // Connect and register the joint state interface
    hardware_interface::JointStateHandle state_handle_left_front("left_front_joint", &pos[0], &vel[0], &eff[0]);  
    jnt_state_interface.registerHandle(state_handle_left_front);
    hardware_interface::JointStateHandle state_handle_left_rear("left_rear_joint", &pos[0], &vel[0], &eff[0]);  
    jnt_state_interface.registerHandle(state_handle_left_rear);

    registerInterface(&jnt_state_interface);

    hardware_interface::JointHandle pos_handle_left_front(jnt_state_interface.getHandle("left_front_joint"), &cmd[0]);
    jnt_pos_interface.registerHandle(pos_handle_left_front);

    hardware_interface::JointHandle pos_handle_left_rear(jnt_state_interface.getHandle("left_rear_joint"), &cmd[1]);
    jnt_pos_interface.registerHandle(pos_handle_left_rear);

    registerInterface(&jnt_pos_interface);
  }

  virtual ~Biped2_Robot() {}

  bool init(){
    // Populate pos_cmd_interface_ with joint handles...

    // Get joint handle of interest
    hardware_interface::JointHandle joint_handle = pos_cmd_interface_.getHandle("left_front_joint");
  
    JointLimits limits;
    SoftJointLimits soft_limits;
    // Populate with any of the methods presented in the previous example ...

    // Register handle in joint limits interface
    PositionJointSoftLimitsHandle handle(joint_handle, // we read the state and read/write the command
                                         limits,       // Limits spec
                                         soft_limits);  // Soft limits spec

    jnt_limits_interface_.registerHandle(handle);
  }

  void read()
  {
    std::cout<<"read "<<"  "<<cmd[0]<<" "<<cmd[1]<<std::endl;
    // Read actuator state from the hardward...
	
    // Propagate current actuator state to joints...  
  }

  void update(const ros::Time& time, const ros::Duration& period){
    
  }

  void write(const ros::Duration& period)
  {
    std::cout<<"write "<<"  "<<pos[0]<<" "<<pos[1]<<std::endl;

    // Enforce joint limits for all registered handles
    // Note: one can also enforce limits on a per-handle basis: handle.enforceLimits(period)
    jnt_limits_interface_.enforceLimits(period);
    
    // Propagate joint commands to actuators...

    // Send actuator command to hardware...
  }

private:
  hardware_interface::JointStateInterface jnt_state_interface;
  hardware_interface::PositionJointInterface jnt_pos_interface;
  hardware_interface::PositionJointInterface pos_cmd_interface_;
  joint_limits_interface::PositionJointSoftLimitsInterface jnt_limits_interface_;
//  std::shared_ptr<controller_manager::ControllerManager> cm;
  double cmd[2];
  double pos[2];
  double vel[2];
  double eff[2];
};


