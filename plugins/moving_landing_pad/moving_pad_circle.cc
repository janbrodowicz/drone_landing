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
              new gazebo::common::PoseAnimation("test", 114.0, true));

        gazebo::common::PoseKeyFrame *key;

        // set starting location of the box
        key = anim->CreateKeyFrame(0);
        key->Translation(ignition::math::Vector3d(0, 0, 0));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        key = anim->CreateKeyFrame(10.0);
        key->Translation(ignition::math::Vector3d(0.5, 0.1, 0));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 0.1));

        key = anim->CreateKeyFrame(20.0);
        key->Translation(ignition::math::Vector3d(1, 0.25, 0));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 0.3));

        key = anim->CreateKeyFrame(30.0);
        key->Translation(ignition::math::Vector3d(1.5, 0.5, 0));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 0.5));

        key = anim->CreateKeyFrame(42.0);
        key->Translation(ignition::math::Vector3d(2, 0.9, 0));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 0.3));

        key = anim->CreateKeyFrame(54.0);
        key->Translation(ignition::math::Vector3d(2.5, 1.4, 0));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 0.1));

        key = anim->CreateKeyFrame(69.0);
        key->Translation(ignition::math::Vector3d(3, 2, 0));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        key = anim->CreateKeyFrame(82.0);
        key->Translation(ignition::math::Vector3d(3.4, 2.5, 0));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        key = anim->CreateKeyFrame(96.0);
        key->Translation(ignition::math::Vector3d(3.8, 3.1, 0));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 0));

        key = anim->CreateKeyFrame(114.0);
        key->Translation(ignition::math::Vector3d(4, 4, 0));
        // key->Rotation(ignition::math::Quaterniond(0, 0, 0));


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
