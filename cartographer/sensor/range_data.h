/*
 * @Author: LuoChen 1425523063@qq.com
 * @Date: 2022-12-08 15:32:55
 * @LastEditors: LuoChen 1425523063@qq.com
 * @LastEditTime: 2023-02-23 16:01:40
 * @FilePath: /catkin_ws/src/cartographer/cartographer/sensor/range_data.h
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

#ifndef CARTOGRAPHER_SENSOR_RANGE_DATA_H_
#define CARTOGRAPHER_SENSOR_RANGE_DATA_H_

#include "cartographer/common/port.h"
#include "cartographer/sensor/compressed_point_cloud.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/sensor/proto/sensor.pb.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
// #include "pcl_conversions/pcl_conversions.h"
namespace cartographer {
namespace sensor {

// Rays begin at 'origin'. 'returns' are the points where obstructions were
// detected. 'misses' are points in the direction of rays for which no return
// was detected, and were inserted at a configured distance. It is assumed that
// between the 'origin' and 'misses' is free space.
struct RangeData {
  Eigen::Vector3f origin;
  PointCloud returns;
  PointCloud misses;
};
pcl::PointCloud<pcl::PointXYZ>::Ptr ToPointCloudMessage(
  const ::cartographer::sensor::PointCloud& filtered_gravity_aligned_point_cloud);

RangeData TransformRangeData(const RangeData& range_data,
                             const transform::Rigid3f& transform);


// Crops 'range_data' according to the region defined by 'min_z' and 'max_z'.
RangeData CropRangeData(const RangeData& range_data, float min_z, float max_z);

// Converts 'range_data' to a proto::RangeData.
proto::RangeData ToProto(const RangeData& range_data);

// Converts 'proto' to RangeData.
RangeData FromProto(const proto::RangeData& proto);

}  // namespace sensor
}  // namespace cartographer

#endif  // CARTOGRAPHER_SENSOR_RANGE_DATA_H_