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

#pragma once

#include <limits>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>
#include "gz/sim/Util.hh"
#include <gz/sim/EntityComponentManager.hh>
#include "gz/sim/components/CanonicalLink.hh"
#include "gz/sim/components/JointPosition.hh"
#include <gz/sim/components/Pose.hh>
#include "gz/sim/Link.hh"

#include <gz/transport/Node.hh>

#include <gz/common/Profiler.hh>

#include <gz/math/SpeedLimiter.hh>
#include <gz/math/Quaternion.hh>
#include <gz/math/DiffDriveOdometry.hh>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/time.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/pose_v.pb.h>


namespace stretch_diff_drive
{
  /// \brief Velocity command.
  struct Commands
  {
    /// \brief Linear velocity.
    double lin;

    /// \brief Angular velocity.
    double ang;

    Commands() : lin(0.0), ang(0.0) {}
  };

  class StretchDiffDrivePrivate
  {
    /// \brief Callback for velocity subscription
    /// \param[in] _msg Velocity message
    public: void OnCmdVel(const gz::msgs::Twist &_msg);

    /// \brief Callback for enable/disable subscription
    /// \param[in] _msg Boolean message
    public: void OnEnable(const gz::msgs::Boolean &_msg);

    /// \brief Update odometry and publish an odometry message.
    /// \param[in] _info System update information.
    /// \param[in] _ecm The EntityComponentManager of the given simulation
    /// instance.
    public: void UpdateOdometry(const gz::sim::UpdateInfo &_info,
      const gz::sim::EntityComponentManager &_ecm);

    /// \brief Update the linear and angular velocities.
    /// \param[in] _info System update information.
    /// \param[in] _ecm The EntityComponentManager of the given simulation
    /// instance.
    public: void UpdateVelocity(const gz::sim::UpdateInfo &_info,
      const gz::sim::EntityComponentManager &_ecm);

    /// \brief Gazebo communication node.
    public: gz::transport::Node node;

    /// \brief Entity of the left joint
    public: std::vector<gz::sim::Entity> leftJoints;

    /// \brief Entity of the right joint
    public: std::vector<gz::sim::Entity> rightJoints;

    /// \brief Name of left joint
    public: std::vector<std::string> leftJointNames;

    /// \brief Name of right joint
    public: std::vector<std::string> rightJointNames;

    /// \brief Calculated speed of left joint
    public: double leftJointSpeed{0};

    /// \brief Calculated speed of right joint
    public: double rightJointSpeed{0};

    /// \brief Distance between wheels
    public: double wheelSeparation{1.0};

    /// \brief Wheel radius
    public: double wheelRadius{0.2};

    /// \brief Model interface
    public: gz::sim::Model model{gz::sim::kNullEntity};

    /// \brief The model's canonical link.
    public: gz::sim::Link canonicalLink{gz::sim::kNullEntity};

    /// \brief Update period calculated from <odom__publish_frequency>.
    public: std::chrono::steady_clock::duration odomPubPeriod{0};

    /// \brief Last sim time odom was published.
    public: std::chrono::steady_clock::duration lastOdomPubTime{0};

    /// \brief Diff drive odometry.
    public: gz::math::DiffDriveOdometry odom;

    /// \brief Diff drive odometry message publisher.
    public: gz::transport::Node::Publisher odomPub;

    /// \brief Diff drive tf message publisher.
    public: gz::transport::Node::Publisher tfPub;

    /// \brief Linear velocity limiter.
    public: std::unique_ptr<gz::math::SpeedLimiter> limiterLin;

    /// \brief Angular velocity limiter.
    public: std::unique_ptr<gz::math::SpeedLimiter> limiterAng;

    /// \brief Previous control command.
    public: Commands last0Cmd;

    /// \brief Previous control command to last0Cmd.
    public: Commands last1Cmd;

    /// \brief Last target velocity requested.
    public: gz::msgs::Twist targetVel;

    /// \brief Enable/disable state of the controller.
    public: bool enabled;

    /// \brief A mutex to protect the target velocity command.
    public: std::mutex mutex;

    /// \brief frame_id from sdf.
    public: std::string sdfFrameId;

    /// \brief child_frame_id from sdf.
    public: std::string sdfChildFrameId;

    /// \brief variable to store previous position of the ground truth pose
    public: gz::math::Pose3d prePoseRobot_;
    /// @brief variable to store the time at which the previous ground trith pose was
    public: double prePoseTime_;
    /// @brief determine to use or not the ground truth as odometry
    public: bool useGroundTruth_{false};
  };
}


