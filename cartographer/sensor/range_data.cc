/*
 * @Author: LuoChen 1425523063@qq.com
 * @Date: 2022-12-08 15:32:55
 * @LastEditors: LuoChen 1425523063@qq.com
 * @LastEditTime: 2023-02-23 17:06:34
 * @FilePath: /catkin_ws/src/cartographer/cartographer/sensor/range_data.cc
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "cartographer/sensor/range_data.h"

#include "cartographer/sensor/proto/sensor.pb.h"
#include "cartographer/transform/transform.h"

namespace cartographer {
namespace sensor {

pcl::PointCloud<pcl::PointXYZ>::Ptr ToPointCloudMessage(const ::cartographer::sensor::PointCloud& filtered_gravity_aligned_point_cloud){
  // pcl::PointCloud<pcl::PointXYZ> cloud(filtered_gravity_aligned_point_cloud.size(),1);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  cloud->resize(filtered_gravity_aligned_point_cloud.size());
  for(size_t i = 0; i < filtered_gravity_aligned_point_cloud.size(); i++){
    cloud->at(i).x = filtered_gravity_aligned_point_cloud.points().at(i).position[0];
    cloud->at(i).y = filtered_gravity_aligned_point_cloud.points().at(i).position[1];
    cloud->at(i).z = 0;
  } 
  return cloud;
}

RangeData TransformRangeData(const RangeData& range_data,
                             const transform::Rigid3f& transform) {
  return RangeData{
      transform * range_data.origin,
      TransformPointCloud(range_data.returns, transform),
      TransformPointCloud(range_data.misses, transform),
  };
}

RangeData CropRangeData(const RangeData& range_data, const float min_z,
                        const float max_z) {
  return RangeData{range_data.origin,
                   CropPointCloud(range_data.returns, min_z, max_z),
                   CropPointCloud(range_data.misses, min_z, max_z)};
}

proto::RangeData ToProto(const RangeData& range_data) {
  proto::RangeData proto;
  *proto.mutable_origin() = transform::ToProto(range_data.origin);
  proto.mutable_returns()->Reserve(range_data.returns.size());
  for (const RangefinderPoint& point : range_data.returns) {
    *proto.add_returns() = ToProto(point);
  }
  proto.mutable_misses()->Reserve(range_data.misses.size());
  for (const RangefinderPoint& point : range_data.misses) {
    *proto.add_misses() = ToProto(point);
  }
  return proto;
}

RangeData FromProto(const proto::RangeData& proto) {
  std::vector<RangefinderPoint> returns;
  if (proto.returns_size() > 0) {
    returns.reserve(proto.returns().size());
    for (const auto& point_proto : proto.returns()) {
      returns.push_back(FromProto(point_proto));
    }
  } else {
    returns.reserve(proto.returns_legacy().size());
    for (const auto& point_proto : proto.returns_legacy()) {
      returns.push_back({transform::ToEigen(point_proto)});
    }
  }
  std::vector<RangefinderPoint> misses;
  if (proto.misses_size() > 0) {
    misses.reserve(proto.misses().size());
    for (const auto& point_proto : proto.misses()) {
      misses.push_back(FromProto(point_proto));
    }
  } else {
    misses.reserve(proto.misses_legacy().size());
    for (const auto& point_proto : proto.misses_legacy()) {
      misses.push_back({transform::ToEigen(point_proto)});
    }
  }
  return RangeData{transform::ToEigen(proto.origin()), PointCloud(returns),
                   PointCloud(misses)};
}

}  // namespace sensor
}  // namespace cartographer
