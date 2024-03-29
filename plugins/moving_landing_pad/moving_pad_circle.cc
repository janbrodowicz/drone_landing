/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#include <gazebo/gazebo.hh>
#include <ignition/math.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>

namespace gazebo
{
  class MovingPadCircle : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;

        // create the animation
        gazebo::common::PoseAnimationPtr anim(
              // name the animation "test",
              // make it last 10 seconds,
              // and set it on a repeat loop
              new gazebo::common::PoseAnimation("test", 597.4, true));

        gazebo::common::PoseKeyFrame *key;

        // set starting location of the box
        key = anim->CreateKeyFrame(0);
        key->Translation(ignition::math::Vector3d(0, 0, 0.1));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        key = anim->CreateKeyFrame(20.8);
        key->Translation(ignition::math::Vector3d(1, 0.3, 0.1));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 0.1));

        key = anim->CreateKeyFrame(42.2);
        key->Translation(ignition::math::Vector3d(2, 0.7, 0.1));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 0.3));

        key = anim->CreateKeyFrame(63);
        key->Translation(ignition::math::Vector3d(3, 1, 0.1));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 0.5));

        key = anim->CreateKeyFrame(83.8);
        key->Translation(ignition::math::Vector3d(4, 1.3, 0.1));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 0.3));

        key = anim->CreateKeyFrame(106.2);
        key->Translation(ignition::math::Vector3d(5, 1.8, 0.1));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 0.1));

        key = anim->CreateKeyFrame(128.6);
        key->Translation(ignition::math::Vector3d(6, 2.3, 0.1));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        key = anim->CreateKeyFrame(151.0);
        key->Translation(ignition::math::Vector3d(7, 2.8, 0.1));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        key = anim->CreateKeyFrame(173.4);
        key->Translation(ignition::math::Vector3d(8, 3.3, 0.1));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        key = anim->CreateKeyFrame(197.8);
        key->Translation(ignition::math::Vector3d(9, 4, 0.1));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        key = anim->CreateKeyFrame(221.0);
        key->Translation(ignition::math::Vector3d(10, 4.6, 0.1));

        key = anim->CreateKeyFrame(246.6);
        key->Translation(ignition::math::Vector3d(11, 5.4, 0.1));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 0.1));

        key = anim->CreateKeyFrame(272.2);
        key->Translation(ignition::math::Vector3d(12, 6.2, 0.1));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 0.3));

        key = anim->CreateKeyFrame(297.8);
        key->Translation(ignition::math::Vector3d(13, 7, 0.1));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 0.5));

        key = anim->CreateKeyFrame(326);
        key->Translation(ignition::math::Vector3d(14, 8, 0.1));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 0.3));

        key = anim->CreateKeyFrame(354.2);
        key->Translation(ignition::math::Vector3d(15, 9, 0.1));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 0.1));

        key = anim->CreateKeyFrame(390.2);
        key->Translation(ignition::math::Vector3d(16, 10.5, 0.1));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        key = anim->CreateKeyFrame(429.6);
        key->Translation(ignition::math::Vector3d(17, 12.2, 0.1));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        key = anim->CreateKeyFrame(470.8);
        key->Translation(ignition::math::Vector3d(18, 14, 0.1));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        key = anim->CreateKeyFrame(524.6);
        key->Translation(ignition::math::Vector3d(19, 16.5, 0.1));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        key = anim->CreateKeyFrame(597.4);
        key->Translation(ignition::math::Vector3d(20, 20, 0.1));

        // set the animation
        _parent->SetAnimation(anim);
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(MovingPadCircle)
}
