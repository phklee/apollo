/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#pragma once

#include <string>
#include <vector>

#include "modules/perception/base/frame.h"
#include "modules/perception/fusion/base/base_forward_declaration.h"
#include "modules/perception/fusion/base/scene.h"
#include "modules/perception/fusion/base/sensor_frame.h"
#include "modules/perception/lib/registerer/registerer.h"

namespace apollo {
namespace perception {
namespace fusion {

struct FusionInitOptions {
  std::string main_sensor;
};

struct FusionOptions {};

class BaseFusionSystem {
 public:
  BaseFusionSystem() = default;
  virtual ~BaseFusionSystem() = default;
  BaseFusionSystem(const BaseFusionSystem&) = delete;
  BaseFusionSystem& operator=(const BaseFusionSystem&) = delete;

  virtual bool Init(const FusionInitOptions& options) = 0;

  // @brief: fuse a sensor frame
  // @param [in]: options
  // @param [in]: sensor_frame
  // @param [out]: fused objects
  virtual bool Fuse(const FusionOptions& options,
                    const base::FrameConstPtr& sensor_frame,
                    std::vector<base::ObjectPtr>* fused_objects) = 0;

  virtual std::string Name() const = 0;

 protected:
  std::string main_sensor_;
};

PERCEPTION_REGISTER_REGISTERER(BaseFusionSystem);   // NOTES(lsq): PERCEPTION_REGISTER_REGISTERER(BaseFusionSystem) 这是一个宏定义，用于注册一个注册器类，
                                                    // 将其与基类 BaseFusionSystem 相关联。通常，注册器类用于自动识别和注册其他类。通过调用 PERCEPTION_REGISTER_REGISTERER 宏，
                                                    // 并将基类作为参数，可以将注册器类与基类关联起来，以便在运行时能够自动识别和创建派生类的对象。

// NOTES(lsq): FUSION_REGISTER_FUSIONSYSTEM(name) 这是另一个宏定义，用于注册一个具体的类 name，使其能够被自动识别和创建。在这个宏定义中，name 是具体类的名称，通常是派生自 BaseFusionSystem 的类名。
// 通过调用 FUSION_REGISTER_FUSIONSYSTEM 宏，可以将具体的类与基类 BaseFusionSystem 关联起来，以便在运行时能够自动识别和创建该类的对象。
#define FUSION_REGISTER_FUSIONSYSTEM(name) \
  PERCEPTION_REGISTER_CLASS(BaseFusionSystem, name)

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
