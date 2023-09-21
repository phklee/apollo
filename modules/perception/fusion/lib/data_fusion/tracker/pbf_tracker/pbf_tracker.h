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

#include <memory>
#include <string>

#include "modules/perception/fusion/base/base_init_options.h"
#include "modules/perception/fusion/lib/interface/base_existence_fusion.h"
#include "modules/perception/fusion/lib/interface/base_motion_fusion.h"
#include "modules/perception/fusion/lib/interface/base_shape_fusion.h"
#include "modules/perception/fusion/lib/interface/base_tracker.h"
#include "modules/perception/fusion/lib/interface/base_type_fusion.h"
#include "modules/perception/proto/pbf_tracker_config.pb.h"

namespace apollo {
namespace perception {
namespace fusion {

class PbfTracker : public BaseTracker {
 public:
  PbfTracker();
  virtual ~PbfTracker();

  PbfTracker(const PbfTracker&) = delete;
  PbfTracker& operator=(const PbfTracker&) = delete;

  static bool InitParams();

  bool Init(TrackPtr track, SensorObjectPtr measurement) override;

  void UpdateWithMeasurement(const TrackerOptions& options,
                             const SensorObjectPtr measurement,
                             double target_timestamp) override;

  void UpdateWithoutMeasurement(const TrackerOptions& options,
                                const std::string& sensor_id,
                                double measurement_timestamp,
                                double target_timestamp) override;

  std::string Name() const override;

 protected:
  bool InitMethods();

 protected:
  static std::string s_type_fusion_method_;
  static std::string s_motion_fusion_method_;
  static std::string s_shape_fusion_method_;
  static std::string s_existence_fusion_method_;

  /* NOTES(lsq) */
  // std::unique_ptr 用于管理动态分配的对象的所有权，并在不再需要时自动释放相关资源。
  // 使用 std::unique_ptr 的好处是，它负责自动释放所拥有的对象的内存，无需手动调用 delete。
  // 当 motion_fusion_ 不再需要时，它会自动释放所持有对象的内存
  // 需要注意的是，由于 std::unique_ptr 是独占所有权的智能指针，即不能进行拷贝操作，只能进行移动语义，
  // 因此在使用 std::unique_ptr 时需要谨慎考虑所有权的转移和生命周期管理。
  /* NOTES(lsq) */
  std::unique_ptr<BaseTypeFusion> type_fusion_ = nullptr;
  std::unique_ptr<BaseMotionFusion> motion_fusion_ = nullptr;
  std::unique_ptr<BaseExistenceFusion> existence_fusion_ = nullptr;
  std::unique_ptr<BaseShapeFusion> shape_fusion_ = nullptr;
};

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
