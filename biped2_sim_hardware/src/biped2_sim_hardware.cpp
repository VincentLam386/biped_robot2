//=================================================================================================
// Copyright (c) 2015, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Source Robotics Foundation
 *  Copyright (c) 2013, The Johns Hopkins University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman, Jonathan Bohren
   Desc:   Hardware Interface for any simulated robot in Gazebo
*/


#include <biped2_sim_hardware/biped2_sim_hardware.h>

namespace biped2_sim_hw_ns
{


bool BipedHWSim::initSim(
  const std::string& robot_namespace,
  ros::NodeHandle model_nh,
  gazebo::physics::ModelPtr parent_model,
  const urdf::Model *const urdf_model,
  std::vector<transmission_interface::TransmissionInfo> transmissions)
{
  // Call parent init (dealing with joint interfaces)
  if (!DefaultRobotHWSim::initSim(robot_namespace, model_nh, parent_model, urdf_model, transmissions)){
    return false;
  }

  /** register sensors */
  // IMU
  imu_data.name = "hip_imu";
  imu_data.frame_id = "hip";
  imu_data.orientation = imu_orientation;
  imu_data.angular_velocity = imu_angular_velocity;
  imu_data.linear_acceleration = imu_linear_acceleration;
  hardware_interface::ImuSensorHandle imu_sensor_handle(imu_data);
  imu_interface_.registerHandle(imu_sensor_handle);
  registerInterface(&imu_interface_);

  world_ = parent_model->GetWorld();
  imu_link_ = parent_model->GetLink(imu_data.frame_id);

  return true;
}


void BipedHWSim::readSim(ros::Time time, ros::Duration period)
{
  DefaultRobotHWSim::readSim(time, period);
  
  double dt = period.toSec();

   // Get Pose/Orientation
  ignition::math::Pose3<double> pose = imu_link_->WorldCoGPose();
  ignition::math::Quaternion<double> rot = this->offset_.Rot()*pose.Rot();
  rot.Normalize();

  // get Gravity
  gravity = world_->Gravity();
  double gravity_length = gravity.Length();
  ROS_DEBUG_NAMED("gazebo_ros_imu", "gravity_world = [%g %g %g]", gravity.X(), gravity.Y(), gravity.Z());

  // get Acceleration and Angular Rates
  // the result of GetRelativeLinearAccel() seems to be unreliable (sum of forces added during the current simulation step)?
  //accel = imu_link_->RelativeLinearAccel(); // get acceleration in body frame
  //accel = imu_link_->WorldCoGLinearAcccel();
  ignition::math::Vector3<double> temp = imu_link_->WorldCoGLinearVel(); // get velocity in world frame
  //std::cout << imu_link_->RelativeLinearVel() << " " << imu_link_->WorldLinearVel() << " " << imu_link_->WorldCoGLinearVel() << "\n";
  //if (dt > 0.0) accel = rot.RotateVectorReverse((temp - velocity) / dt - gravity);
  if (dt > 0.0) accel = rot.RotateVectorReverse((temp - velocity) / dt);
  velocity = temp;
  std::cout << velocity << std::endl;

  // calculate angular velocity from delta quaternion
  // note: link->GetRelativeAngularVel() sometimes return nan?
  // rate  = link->GetRelativeAngularVel(); // get angular rate in body frame
  ignition::math::Quaternion<double> delta = this->orientation.Inverse() * rot;
  this->orientation = rot;
  if (dt > 0.0) {
    rate = 2.0 * acos(std::max(std::min(delta.W(), 1.0), -1.0)) * ignition::math::Vector3<double>(delta.X(), delta.Y(), delta.Z()).Normalize() / dt;
  }

  imu_orientation[0] = rot.X();
  imu_orientation[1] = rot.Y();
  imu_orientation[2] = rot.Z();
  imu_orientation[3] = rot.W();

  imu_angular_velocity[0] = rate.X();
  imu_angular_velocity[1] = rate.Y();
  imu_angular_velocity[2] = rate.Z();

  imu_linear_acceleration[0] = accel.X();
  imu_linear_acceleration[1] = accel.Y();
  imu_linear_acceleration[2] = accel.Z();

}


}

PLUGINLIB_EXPORT_CLASS(biped2_sim_hw_ns::BipedHWSim, gazebo_ros_control::RobotHWSim)
