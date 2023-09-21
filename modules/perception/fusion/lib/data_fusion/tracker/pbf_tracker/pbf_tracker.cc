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
#include "modules/perception/fusion/lib/data_fusion/tracker/pbf_tracker/pbf_tracker.h"

#include "cyber/common/file.h"
#include "modules/common/util/string_util.h"
#include "modules/perception/fusion/lib/data_fusion/existence_fusion/dst_existence_fusion/dst_existence_fusion.h"
#include "modules/perception/fusion/lib/data_fusion/motion_fusion/kalman_motion_fusion/kalman_motion_fusion.h"
#include "modules/perception/fusion/lib/data_fusion/shape_fusion/pbf_shape_fusion/pbf_shape_fusion.h"
#include "modules/perception/fusion/lib/data_fusion/type_fusion/dst_type_fusion/dst_type_fusion.h"
#include "modules/perception/lib/config_manager/config_manager.h"

namespace apollo {
namespace perception {
namespace fusion {

using cyber::common::GetAbsolutePath;

// TODO(all) fix the static string lint issue
std::string PbfTracker::s_type_fusion_method_ = "DstTypeFusion";  // NOLINT
std::string PbfTracker::s_existence_fusion_method_ =              // NOLINT
    "DstExistenceFusion";
std::string PbfTracker::s_motion_fusion_method_ =  // NOLINT
    "KalmanMotionFusion";
std::string PbfTracker::s_shape_fusion_method_ = "PbfShapeFusion";  // NOLINT

PbfTracker::PbfTracker() {}

PbfTracker::~PbfTracker() {}

bool PbfTracker::InitParams() {
  BaseInitOptions options;
  if (!GetFusionInitOptions("PbfTracker", &options)) {
    return false;
  }

  std::string woork_root_config = GetAbsolutePath(
      lib::ConfigManager::Instance()->work_root(), options.root_dir);

  std::string config = GetAbsolutePath(woork_root_config, options.conf_file);
  AINFO << "Config file : " << config;
  PbfTrackerConfig params;
  if (!cyber::common::GetProtoFromFile(config, &params)) {
    AERROR << "Read config failed: " << config;
    return false;
  }

  AINFO << "Load PbfTrackerConfig: " << params.type_fusion_method() << ","
        << params.motion_fusion_method() << "," << params.shape_fusion_method()
        << "," << params.existence_fusion_method();
  s_type_fusion_method_ = params.type_fusion_method();
  s_motion_fusion_method_ = params.motion_fusion_method();
  s_existence_fusion_method_ = params.existence_fusion_method();
  s_shape_fusion_method_ = params.shape_fusion_method();

  return true;
}

bool PbfTracker::InitMethods() {
  if (s_type_fusion_method_ == "DstTypeFusion") {
    type_fusion_.reset(new DstTypeFusion(track_));  // base_type_fusion: TrackPtr track_
  } else {
    AERROR << "Unknown type fusion : " << s_type_fusion_method_;
    return false;
  }

  if (s_motion_fusion_method_ == "KalmanMotionFusion") {
    /* NOTES(lsq) */
    // new KalmanMotionFusion(track_) 创建一个新的 KalmanMotionFusion 对象
    // 目的是重新为 motion_fusion_ 指针对象分配内存，并将其指向一个新创建的 KalmanMotionFusion 对象。
    // 通过调用 reset 函数，可以在保留原有指针的情况下，重新初始化指针的值，从而实现对象的替换或重新创建。
    // 需要注意的是，这种用法假设 motion_fusion_ 已经是一个指针对象，而且在调用 reset 之前，可能已经分配了内存或指向了其他对象。
    // 通过调用 reset 函数，旧的内存将被释放，而 motion_fusion_ 将指向新创建的 KalmanMotionFusion 对象。
    /* NOTES(lsq) */
    motion_fusion_.reset(new KalmanMotionFusion(track_));  // NOTES(lsq): base_motion_fusion: TrackPtr track_
  } else {
    AERROR << "Unknown motion fusion : " << s_motion_fusion_method_;
    return false;
  }

  if (s_existence_fusion_method_ == "DstExistenceFusion") {
    existence_fusion_.reset(new DstExistenceFusion(track_));  // NOTES(lsq): base_existence_fusion: TrackPtr track_
  } else {
    AERROR << "Unknown existence fusion : " << s_existence_fusion_method_;
    return false;
  }

  if (s_shape_fusion_method_ == "PbfShapeFusion") {
    shape_fusion_.reset(new PbfShapeFusion(track_));  // NOTES(lsq): base_shape_fusion: TrackPtr track_
  } else {
    AERROR << "Unknown shape fusion : " << s_shape_fusion_method_;
    return false;
  }

  return true;
}

bool PbfTracker::Init(TrackPtr track, SensorObjectPtr measurement) {
  track_ = track;
  if (!InitMethods()) {
    return false;
  }
  motion_fusion_->Init();
  return true;
}

void PbfTracker::UpdateWithMeasurement(const TrackerOptions& options,
                                       const SensorObjectPtr measurement,
                                       double target_timestamp) {
  std::string sensor_id = measurement->GetSensorId();
  ADEBUG << "fusion_updating..." << track_->GetTrackId() << " with "
         << sensor_id << "..." << measurement->GetBaseObject()->track_id << "@"
         << FORMAT_TIMESTAMP(measurement->GetTimestamp());
  existence_fusion_->UpdateWithMeasurement(measurement, target_timestamp,   // NOTES(lsq): DstExistenceFusion(base_existence_fusion): dst_existence_fusion
                                           options.match_distance);
  motion_fusion_->UpdateWithMeasurement(measurement, target_timestamp);     // NOTES(lsq): KalmanMotionFusion(base_shape_fusion): kalman_motion_fusion
  shape_fusion_->UpdateWithMeasurement(measurement, target_timestamp);      // NOTES(lsq): PbfShapeFusion(base_shape_fusion): pdf_shape_fusion
  type_fusion_->UpdateWithMeasurement(measurement, target_timestamp);       // NOTES(lsq): DstTypeFusion(base_type_fusion): dst_type_fusion
  track_->UpdateWithSensorObject(measurement);
}

void PbfTracker::UpdateWithoutMeasurement(const TrackerOptions& options,
                                          const std::string& sensor_id,
                                          double measurement_timestamp,
                                          double target_timestamp) {
  existence_fusion_->UpdateWithoutMeasurement(sensor_id, measurement_timestamp,
                                              target_timestamp,
                                              options.match_distance);
  motion_fusion_->UpdateWithoutMeasurement(sensor_id, measurement_timestamp,
                                           target_timestamp);
  shape_fusion_->UpdateWithoutMeasurement(sensor_id, measurement_timestamp,
                                          target_timestamp);
  type_fusion_->UpdateWithoutMeasurement(sensor_id, measurement_timestamp,
                                         target_timestamp,
                                         options.match_distance);
  track_->UpdateWithoutSensorObject(sensor_id, measurement_timestamp);
}

std::string PbfTracker::Name() const { return "PbfTracker"; }

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
