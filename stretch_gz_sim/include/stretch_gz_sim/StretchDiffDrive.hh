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

#include <memory>
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
#include "gz/sim/components/JointVelocityCmd.hh"
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"

#include <gz/common/Profiler.hh>

#include <gz/math/SpeedLimiter.hh>

#include <gz/plugin/Register.hh>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/time.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/odometry.pb.h>
#include <gz/msgs/pose.pb.h>
#include <gz/msgs/pose_v.pb.h>


#include "StretchDiffDrivePrivate.hh"

namespace stretch_diff_drive
{
  class StretchDiffDrive:
    public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate,
    public gz::sim::ISystemPostUpdate
  {
    public: StretchDiffDrive();
    public: ~StretchDiffDrive() override = default;

    public: void Configure(const gz::sim::Entity &_entity,
                  const std::shared_ptr<const sdf::Element> &_sdf,
                  gz::sim::EntityComponentManager &_ecm,
                  gz::sim::EventManager &_eventMgr) override;

    public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                  gz::sim::EntityComponentManager &_ecm) override;

    public: void PostUpdate(const gz::sim::UpdateInfo &_info,
                    const gz::sim::EntityComponentManager &_ecm) override;

    private: std::unique_ptr<StretchDiffDrivePrivate> dataPtr;
  };
}


