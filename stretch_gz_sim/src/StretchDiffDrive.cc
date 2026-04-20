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

#include "stretch_gz_sim/StretchDiffDrive.hh"
 

GZ_ADD_PLUGIN(
  stretch_diff_drive::StretchDiffDrive,
  gz::sim::System,
  stretch_diff_drive::StretchDiffDrive::ISystemConfigure,
  stretch_diff_drive::StretchDiffDrive::ISystemPreUpdate,
  stretch_diff_drive::StretchDiffDrive::ISystemPostUpdate
)


namespace stretch_diff_drive
  {
  
  StretchDiffDrive::StretchDiffDrive()
    : dataPtr(std::make_unique<StretchDiffDrivePrivate>())
  {
  }
  
  void StretchDiffDrive::Configure(const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &/*_eventMgr*/)
  {
    this->dataPtr->model = gz::sim::Model(_entity);
  
    // Get the canonical link
    std::vector<gz::sim::Entity> links = _ecm.ChildrenByComponents(
        this->dataPtr->model.Entity(), gz::sim::components::CanonicalLink());
    if (!links.empty())
      this->dataPtr->canonicalLink = gz::sim::Link(links[0]);
  
    if (!this->dataPtr->model.Valid(_ecm))
    {
      gzerr << "StretchDiffDrive plugin should be attached to a model entity. "
              << "Failed to initialize." << std::endl;
      return;
    }
  
    // Get params from SDF
    auto sdfElem = _sdf->FindElement("left_joint");
    while (sdfElem)
    {
      this->dataPtr->leftJointNames.push_back(sdfElem->Get<std::string>());
      sdfElem = sdfElem->GetNextElement("left_joint");
    }
    sdfElem = _sdf->FindElement("right_joint");
    while (sdfElem)
    {
      this->dataPtr->rightJointNames.push_back(sdfElem->Get<std::string>());
      sdfElem = sdfElem->GetNextElement("right_joint");
    }
  
    this->dataPtr->wheelSeparation = _sdf->Get<double>("wheel_separation",
        this->dataPtr->wheelSeparation).first;
    this->dataPtr->wheelRadius = _sdf->Get<double>("wheel_radius",
        this->dataPtr->wheelRadius).first;
  
    // Instantiate the speed limiters.
    this->dataPtr->limiterLin = std::make_unique<gz::math::SpeedLimiter>();
    this->dataPtr->limiterAng = std::make_unique<gz::math::SpeedLimiter>();
  
    // Parse speed limiter parameters.
  
    // Min Velocity
    if (_sdf->HasElement("min_velocity"))
    {
      const double minVel = _sdf->Get<double>("min_velocity");
      this->dataPtr->limiterLin->SetMinVelocity(minVel);
      this->dataPtr->limiterAng->SetMinVelocity(minVel);
    }
    if (_sdf->HasElement("min_linear_velocity"))
    {
      const double minLinVel = _sdf->Get<double>("min_linear_velocity");
      this->dataPtr->limiterLin->SetMinVelocity(minLinVel);
    }
    if (_sdf->HasElement("min_angular_velocity"))
    {
      const double minAngVel = _sdf->Get<double>("min_angular_velocity");
      this->dataPtr->limiterAng->SetMinVelocity(minAngVel);
    }
  
    // Max Velocity
    if (_sdf->HasElement("max_velocity"))
    {
      const double maxVel = _sdf->Get<double>("max_velocity");
      this->dataPtr->limiterLin->SetMaxVelocity(maxVel);
      this->dataPtr->limiterAng->SetMaxVelocity(maxVel);
    }
    if (_sdf->HasElement("max_linear_velocity"))
    {
      const double maxLinVel = _sdf->Get<double>("max_linear_velocity");
      this->dataPtr->limiterLin->SetMaxVelocity(maxLinVel);
    }
    if (_sdf->HasElement("max_angular_velocity"))
    {
      const double maxAngVel = _sdf->Get<double>("max_angular_velocity");
      this->dataPtr->limiterAng->SetMaxVelocity(maxAngVel);
    }
  
    // Min Acceleration
    if (_sdf->HasElement("min_acceleration"))
    {
      const double minAccel = _sdf->Get<double>("min_acceleration");
      this->dataPtr->limiterLin->SetMinAcceleration(minAccel);
      this->dataPtr->limiterAng->SetMinAcceleration(minAccel);
    }
    if (_sdf->HasElement("min_linear_acceleration"))
    {
      const double minLinAccel = _sdf->Get<double>("min_linear_acceleration");
      this->dataPtr->limiterLin->SetMinAcceleration(minLinAccel);
    }
    if (_sdf->HasElement("min_angular_acceleration"))
    {
      const double minAngAccel = _sdf->Get<double>("min_angular_acceleration");
      this->dataPtr->limiterAng->SetMinAcceleration(minAngAccel);
    }
  
    // Max Acceleration
    if (_sdf->HasElement("max_acceleration"))
    {
      const double maxAccel = _sdf->Get<double>("max_acceleration");
      this->dataPtr->limiterLin->SetMaxAcceleration(maxAccel);
      this->dataPtr->limiterAng->SetMaxAcceleration(maxAccel);
    }
    if (_sdf->HasElement("max_linear_acceleration"))
    {
      const double maxLinAccel = _sdf->Get<double>("max_linear_acceleration");
      this->dataPtr->limiterLin->SetMaxAcceleration(maxLinAccel);
    }
    if (_sdf->HasElement("max_angular_acceleration"))
    {
      const double maxAngAccel = _sdf->Get<double>("max_angular_acceleration");
      this->dataPtr->limiterAng->SetMaxAcceleration(maxAngAccel);
    }
  
    // Min Jerk
    if (_sdf->HasElement("min_jerk"))
    {
      const double minJerk = _sdf->Get<double>("min_jerk");
      this->dataPtr->limiterLin->SetMinJerk(minJerk);
      this->dataPtr->limiterAng->SetMinJerk(minJerk);
    }
    if (_sdf->HasElement("min_linear_jerk"))
    {
      const double minLinJerk = _sdf->Get<double>("min_linear_jerk");
      this->dataPtr->limiterLin->SetMinJerk(minLinJerk);
    }
    if (_sdf->HasElement("min_angular_jerk"))
    {
      const double minAngJerk = _sdf->Get<double>("min_angular_jerk");
      this->dataPtr->limiterAng->SetMinJerk(minAngJerk);
    }
  
    // Max Jerk
    if (_sdf->HasElement("max_jerk"))
    {
      const double maxJerk = _sdf->Get<double>("max_jerk");
      this->dataPtr->limiterLin->SetMaxJerk(maxJerk);
      this->dataPtr->limiterAng->SetMaxJerk(maxJerk);
    }
    if (_sdf->HasElement("max_linear_jerk"))
    {
      const double maxLinJerk = _sdf->Get<double>("max_linear_jerk");
      this->dataPtr->limiterLin->SetMaxJerk(maxLinJerk);
    }
    if (_sdf->HasElement("max_angular_jerk"))
    {
      const double maxAngJerk = _sdf->Get<double>("max_angular_jerk");
      this->dataPtr->limiterAng->SetMaxJerk(maxAngJerk);
    }

    if (_sdf->HasElement("use_ground_truth_odom"))
    {
      this->dataPtr->useGroundTruth_ = true;
    }
  
    double odomFreq = _sdf->Get<double>("odom_publish_frequency", 50).first;
    if (odomFreq > 0)
    {
      std::chrono::duration<double> odomPer{1 / odomFreq};
      this->dataPtr->odomPubPeriod =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(odomPer);
    }
  
    // Setup odometry.
    this->dataPtr->odom.SetWheelParams(this->dataPtr->wheelSeparation,
        this->dataPtr->wheelRadius, this->dataPtr->wheelRadius);
  
    // Subscribe to commands
    std::vector<std::string> topics;
    if (_sdf->HasElement("topic"))
    {
      topics.push_back(_sdf->Get<std::string>("topic"));
    }
    topics.push_back("/model/" + this->dataPtr->model.Name(_ecm) + "/cmd_vel");
    auto topic = gz::sim::validTopic(topics);
  
    this->dataPtr->node.Subscribe(topic, &StretchDiffDrivePrivate::OnCmdVel,
        this->dataPtr.get());
  
    // Subscribe to enable/disable
    std::vector<std::string> enableTopics;
    enableTopics.push_back(
      "/model/" + this->dataPtr->model.Name(_ecm) + "/enable");
    auto enableTopic = gz::sim::validTopic(enableTopics);
  
    if (!enableTopic.empty())
    {
      this->dataPtr->node.Subscribe(enableTopic, &StretchDiffDrivePrivate::OnEnable,
          this->dataPtr.get());
    }
    this->dataPtr->enabled = true;
  
    std::vector<std::string> odomTopics;
    if (_sdf->HasElement("odom_topic"))
    {
      odomTopics.push_back(_sdf->Get<std::string>("odom_topic"));
    }
    odomTopics.push_back("/model/" + this->dataPtr->model.Name(_ecm) +
        "/odometry");
    auto odomTopic = gz::sim::validTopic(odomTopics);
  
    this->dataPtr->odomPub = this->dataPtr->node.Advertise<gz::msgs::Odometry>(
        odomTopic);
  
    std::string tfTopic{"/model/" + this->dataPtr->model.Name(_ecm) +
      "/tf"};
    if (_sdf->HasElement("tf_topic"))
      tfTopic = _sdf->Get<std::string>("tf_topic");
    this->dataPtr->tfPub = this->dataPtr->node.Advertise<gz::msgs::Pose_V>(
        tfTopic);
  
    if (_sdf->HasElement("frame_id"))
      this->dataPtr->sdfFrameId = _sdf->Get<std::string>("frame_id");
  
    if (_sdf->HasElement("child_frame_id"))
      this->dataPtr->sdfChildFrameId = _sdf->Get<std::string>("child_frame_id");
  
    gzmsg << "StretchDiffDrive subscribing to twist messages on [" << topic << "]"
            << std::endl;
    
    // MAKING THE ROBOT HAVE GROUND TRUTH ODOMETRY!!!!!!!!!!!!!!!    
    this->dataPtr->prePoseRobot_ = _ecm.Component<gz::sim::components::Pose>(this->dataPtr->model.Entity())->Data();
    this->dataPtr->prePoseTime_ = 0.0;
  }
  
  //////////////////////////////////////////////////
  void StretchDiffDrive::PreUpdate(const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
  {
    GZ_PROFILE("StretchDiffDrive::PreUpdate");
  
    // \TODO(anyone) Support rewind
    if (_info.dt < std::chrono::steady_clock::duration::zero())
    {
      gzwarn << "Detected jump back in time ["
              << std::chrono::duration<double>(_info.dt).count()
              << "s]. System may not work properly." << std::endl;
    }
  
    // If the joints haven't been identified yet, look for them
    static std::set<std::string> warnedModels;
    auto modelName = this->dataPtr->model.Name(_ecm);
    if (this->dataPtr->leftJoints.empty() ||
        this->dataPtr->rightJoints.empty())
    {
      bool warned{false};
      for (const std::string &name : this->dataPtr->leftJointNames)
      {
        gz::sim::Entity joint = this->dataPtr->model.JointByName(_ecm, name);
        if (joint != gz::sim::kNullEntity)
          this->dataPtr->leftJoints.push_back(joint);
        else if (warnedModels.find(modelName) == warnedModels.end())
        {
          gzwarn << "Failed to find left joint [" << name << "] for model ["
                  << modelName << "]" << std::endl;
          warned = true;
        }
      }
  
      for (const std::string &name : this->dataPtr->rightJointNames)
      {
        gz::sim::Entity joint = this->dataPtr->model.JointByName(_ecm, name);
        if (joint != gz::sim::kNullEntity)
          this->dataPtr->rightJoints.push_back(joint);
        else if (warnedModels.find(modelName) == warnedModels.end())
        {
          gzwarn << "Failed to find right joint [" << name << "] for model ["
                  << modelName << "]" << std::endl;
          warned = true;
        }
      }
      if (warned)
      {
        warnedModels.insert(modelName);
      }
    }
  
    if (this->dataPtr->leftJoints.empty() || this->dataPtr->rightJoints.empty())
      return;
  
    if (warnedModels.find(modelName) != warnedModels.end())
    {
      gzmsg << "Found joints for model [" << modelName
              << "], plugin will start working." << std::endl;
      warnedModels.erase(modelName);
    }
  
    // Nothing left to do if paused.
    if (_info.paused)
      return;
  
    for (gz::sim::Entity joint : this->dataPtr->leftJoints)
    {
      // skip this entity if it has been removed
      if (!_ecm.HasEntity(joint))
        continue;
  
      // Update wheel velocity
      _ecm.SetComponentData<gz::sim::components::JointVelocityCmd>(joint,
        {this->dataPtr->leftJointSpeed});
    }
  
    for (gz::sim::Entity joint : this->dataPtr->rightJoints)
    {
      // skip this entity if it has been removed
      if (!_ecm.HasEntity(joint))
        continue;
  
      // Update wheel velocity
      _ecm.SetComponentData<gz::sim::components::JointVelocityCmd>(joint,
        {this->dataPtr->rightJointSpeed});
    }
  
    // Create the left and right side joint position components if they
    // don't exist.
    auto leftPos = _ecm.Component<gz::sim::components::JointPosition>(
        this->dataPtr->leftJoints[0]);
    if (!leftPos && _ecm.HasEntity(this->dataPtr->leftJoints[0]))
    {
      _ecm.CreateComponent(this->dataPtr->leftJoints[0],
          gz::sim::components::JointPosition());
    }
  
    auto rightPos = _ecm.Component<gz::sim::components::JointPosition>(
        this->dataPtr->rightJoints[0]);
    if (!rightPos && _ecm.HasEntity(this->dataPtr->rightJoints[0]))
    {
      _ecm.CreateComponent(this->dataPtr->rightJoints[0],
          gz::sim::components::JointPosition());
    }
  }
  
  //////////////////////////////////////////////////
  void StretchDiffDrive::PostUpdate(const gz::sim::UpdateInfo &_info,
      const gz::sim::EntityComponentManager &_ecm)
  {
    GZ_PROFILE("StretchDiffDrive::PostUpdate");
    // Nothing left to do if paused.
    if (_info.paused)
      return;
  
    this->dataPtr->UpdateVelocity(_info, _ecm);
    this->dataPtr->UpdateOdometry(_info, _ecm);
  }
}