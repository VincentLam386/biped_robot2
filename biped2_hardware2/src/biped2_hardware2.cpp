#include <biped2_hardware2/biped2_hardware2.h>
//#include <controller_manager/controller_manager.h>
//#include <hardware_interface/actuator_state_interface.h>

//#include <joint_limits_interface/joint_limits_interface.h>

//#include <vector>
//#include <transmission_interface/simple_transmission.h>
//#include <transmission_interface/transmission_interface.h>

#include <ros/callback_queue.h>

//using std::vector;
//using namespace transmission_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;

int main(int argc, char** argv){
  ros::init(argc,argv, "biped_hardware_node");
  ros::NodeHandle nh("/biped2");
  ros::CallbackQueue queue;
  nh.setCallbackQueue(&queue);

  std::cout << "normal" << std::endl;
  Biped2_Robot2 biped(nh);
  std::cout << "fail?" << std::endl;
  //controller_manager::ControllerManager cm(&biped, nh);

  //ros::AsyncSpinner spinner(4, &queue);
  ros::MultiThreadedSpinner spinner(0);
  //spinner.start();
  spinner.spin(&queue);

/*  ros::Time ts = ros::Time::now();

  ros::Rate rate(50);
  biped
  while(ros::ok()){
    ros::Duration d = ros::Time::now() - ts;
    ts = ros::Time::now();
    biped.read();
    cm.update(ts,d);
    biped.write(d);
    rate.sleep();
  }

  spinner.stop();
*/
  return 0;
}
  
