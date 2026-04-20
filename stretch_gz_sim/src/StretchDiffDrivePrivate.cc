/*
 * Copyright (C) 2018 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "stretch_gz_sim/StretchDiffDrivePrivate.hh"

namespace stretch_diff_drive
{
  void StretchDiffDrivePrivate::UpdateOdometry(const gz::sim::UpdateInfo &_info,
      const gz::sim::EntityComponentManager &_ecm)
  {
    GZ_PROFILE("StretchDiffDrive::UpdateOdometry");

    gz::msgs::Odometry msg;
    if (!this->useGroundTruth_){
      // Initialize, if not already initialized.
      if (!this->odom.Initialized())
      {
        this->odom.Init(std::chrono::steady_clock::time_point(_info.simTime));
        return;
      }
    
      if (this->leftJoints.empty() || this->rightJoints.empty())
        return;
    
      // Get the first joint positions for the left and right side.
      auto leftPos = _ecm.Component<gz::sim::components::JointPosition>(this->leftJoints[0]);
      auto rightPos = _ecm.Component<gz::sim::components::JointPosition>(
          this->rightJoints[0]);
    
      // Abort if the joints were not found or just created.
      if (!leftPos || !rightPos || leftPos->Data().empty() ||
          rightPos->Data().empty())
      {
        return;
      }
    
      this->odom.Update(leftPos->Data()[0], rightPos->Data()[0],
          std::chrono::steady_clock::time_point(_info.simTime));
    
      // Throttle publishing
      auto diff = _info.simTime - this->lastOdomPubTime;
      if (diff > std::chrono::steady_clock::duration::zero() &&
          diff < this->odomPubPeriod)
      {
        return;
      }
      this->lastOdomPubTime = _info.simTime;
    
      // Construct the odometry message and publish it.
      msg.mutable_pose()->mutable_position()->set_x(this->odom.X());
      msg.mutable_pose()->mutable_position()->set_y(this->odom.Y());
    
      gz::math::Quaterniond orientation(0, 0, *this->odom.Heading());
      gz::msgs::Set(msg.mutable_pose()->mutable_orientation(), orientation);
    
      msg.mutable_twist()->mutable_linear()->set_x(this->odom.LinearVelocity());
      msg.mutable_twist()->mutable_angular()->set_z(*this->odom.AngularVelocity());
    }
    else{
      // Calculating Ground Truth Data
      gz::math::Pose3d poseRobot_ = _ecm.Component<gz::sim::components::Pose>(this->model.Entity())->Data();
      double poseTime_ = std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime).count();
      double deltaTime_ = poseTime_ - this->prePoseTime_;

      // Calculate LinearVelocity
      gz::math::Vector3d linearVelocity_ = ((poseRobot_.Pos() - this->prePoseRobot_.Pos())/ deltaTime_)* 1e9;

      // Calculate AngularVelocity
      gz::math::Quaterniond deltaRotation_ = poseRobot_.Rot() * this->prePoseRobot_.Rot().Inverse();
      gz::math::Vector3d axis_;
      double angle_;
      deltaRotation_.AxisAngle(axis_, angle_);
      gz::math::Vector3d angularVelocity_ = (axis_ * (angle_ / deltaTime_))* 1e9;

      // Construct the ground truth odometry message and publish it.
      gz::msgs::Set(msg.mutable_pose(), poseRobot_);
      gz::msgs::Set(msg.mutable_twist()->mutable_linear(), linearVelocity_);
      gz::msgs::Set(msg.mutable_twist()->mutable_angular(), angularVelocity_);
    }
  
    // Set the time stamp in the header
    msg.mutable_header()->mutable_stamp()->CopyFrom(
        gz::sim::convert<gz::msgs::Time>(_info.simTime));
  
    // Set the frame id.
    auto frame = msg.mutable_header()->add_data();
    frame->set_key("frame_id");
    if (this->sdfFrameId.empty())
    {
      frame->add_value(this->model.Name(_ecm) + "/odom");
    }
    else
    {
      frame->add_value(this->sdfFrameId);
    }
  
    std::optional<std::string> linkName = this->canonicalLink.Name(_ecm);
    if (this->sdfChildFrameId.empty())
    {
      if (linkName)
      {
        auto childFrame = msg.mutable_header()->add_data();
        childFrame->set_key("child_frame_id");
        childFrame->add_value(this->model.Name(_ecm) + "/" + *linkName);
      }
    }
    else
    {
      auto childFrame = msg.mutable_header()->add_data();
      childFrame->set_key("child_frame_id");
      childFrame->add_value(this->sdfChildFrameId);
    }
  
    // Construct the Pose_V/tf message and publish it.
    gz::msgs::Pose_V tfMsg;
    gz::msgs::Pose *tfMsgPose = tfMsg.add_pose();
    tfMsgPose->mutable_header()->CopyFrom(*msg.mutable_header());
    tfMsgPose->mutable_position()->CopyFrom(msg.mutable_pose()->position());
    tfMsgPose->mutable_orientation()->CopyFrom(msg.mutable_pose()->orientation());
  
    // Publish the messages
    this->odomPub.Publish(msg);
    this->tfPub.Publish(tfMsg);
  }
  
  void StretchDiffDrivePrivate::UpdateVelocity(const gz::sim::UpdateInfo &_info,
      const gz::sim::EntityComponentManager &/*_ecm*/)
  {
    GZ_PROFILE("StretchDiffDrive::UpdateVelocity");
  
    double linVel;
    double angVel;
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      linVel = this->targetVel.linear().x();
      angVel = this->targetVel.angular().z();
    }
  
    // Limit the target velocity if needed.
    this->limiterLin->Limit(
        linVel, this->last0Cmd.lin, this->last1Cmd.lin, _info.dt);
    this->limiterAng->Limit(
        angVel, this->last0Cmd.ang, this->last1Cmd.ang, _info.dt);
  
    // Update history of commands.
    this->last1Cmd = last0Cmd;
    this->last0Cmd.lin = linVel;
    this->last0Cmd.ang = angVel;
  
    // Convert the target velocities to joint velocities.
    this->rightJointSpeed =
      (linVel + angVel * this->wheelSeparation / 2.0) / this->wheelRadius;
    this->leftJointSpeed =
      (linVel - angVel * this->wheelSeparation / 2.0) / this->wheelRadius;
  }
  
  void StretchDiffDrivePrivate::OnCmdVel(const gz::msgs::Twist &_msg)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    if (this->enabled)
    {
      this->targetVel = _msg;
    }
  }
  
  void StretchDiffDrivePrivate::OnEnable(const gz::msgs::Boolean &_msg)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->enabled = _msg.data();
    if (!this->enabled)
    {
      gz::math::Vector3d zeroVector{0, 0, 0};
      gz::msgs::Set(this->targetVel.mutable_linear(), zeroVector);
      gz::msgs::Set(this->targetVel.mutable_angular(), zeroVector);
    }
  }
}