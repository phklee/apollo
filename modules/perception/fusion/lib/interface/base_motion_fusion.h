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

#include "modules/perception/fusion/base/base_forward_declaration.h"
#include "modules/perception/fusion/base/scene.h"
#include "modules/perception/fusion/base/sensor_frame.h"
#include "modules/perception/lib/registerer/registerer.h"

namespace apollo {
namespace perception {
namespace fusion {

class BaseMotionFusion {
 public:
  explicit BaseMotionFusion(TrackPtr track) : track_ref_(track) {}  // NOTES(lsq): 它使用了 explicit 关键字，表示该构造函数是显式的，即不能通过隐式转换来调用。
  virtual ~BaseMotionFusion() {}   // NOTES(lsq): 虚析构函数在处理继承关系时很有用，确保在删除派生类对象时，析构函数按照正确的顺序被调用。
  BaseMotionFusion(const BaseMotionFusion&) = delete;     // NOTES(lsq): 使用 delete 关键字可以禁用默认生成的拷贝构造函数，阻止对象的拷贝操作。
                                                          // 在这个类中，拷贝构造函数被删除，意味着不能对该类的对象进行拷贝
  BaseMotionFusion& operator=(const BaseMotionFusion&) = delete;  // NOTES(lsq): 通过使用 delete 关键字禁用赋值运算符重载，阻止对象的赋值操作。 
                                                                  // 在这个类中，赋值运算符被删除，意味着不能对该类的对象进行赋值

  virtual bool Init() = 0;

  // @brief: update track state with measurement
  // @param [in]: measurement
  // @param [in]: target_timestamp
  virtual void UpdateWithMeasurement(const SensorObjectConstPtr& measurement,
                                     double target_timestamp) = 0;

  virtual void UpdateWithoutMeasurement(const std::string& sensor_id,
                                        double measurement_timestamp,
                                        double target_timestamp) = 0;

  virtual std::string Name() const = 0;

 protected:
  TrackPtr track_ref_;
};

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
