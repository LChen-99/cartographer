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

#include "cartographer/mapping/internal/2d/scan_matching/real_time_correlative_scan_matcher_2d.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <pcl/registration/ndt.h>
#include "Eigen/Geometry"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/math.h"
#include "cartographer/mapping/2d/probability_grid.h"
#include "cartographer/mapping/internal/2d/tsdf_2d.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/transform.h"
#include "glog/logging.h"

namespace cartographer {
namespace mapping {
namespace scan_matching {
namespace {

float ComputeCandidateScore(const TSDF2D& tsdf,
                            const DiscreteScan2D& discrete_scan,
                            int x_index_offset, int y_index_offset) {
  float candidate_score = 0.f;
  float summed_weight = 0.f;
  for (const Eigen::Array2i& xy_index : discrete_scan) {
    const Eigen::Array2i proposed_xy_index(xy_index.x() + x_index_offset,
                                           xy_index.y() + y_index_offset);
    const std::pair<float, float> tsd_and_weight =
        tsdf.GetTSDAndWeight(proposed_xy_index);
    const float normalized_tsd_score =
        (tsdf.GetMaxCorrespondenceCost() - std::abs(tsd_and_weight.first)) /
        tsdf.GetMaxCorrespondenceCost();
    const float weight = tsd_and_weight.second;
    candidate_score += normalized_tsd_score * weight;
    summed_weight += weight;
  }
  if (summed_weight == 0.f) return 0.f;
  candidate_score /= summed_weight;
  CHECK_GE(candidate_score, 0.f);
  return candidate_score;
}

float ComputeCandidateScore(const ProbabilityGrid& probability_grid,
                            const DiscreteScan2D& discrete_scan,
                            int x_index_offset, int y_index_offset) {
  float candidate_score = 0.f;
  for (const Eigen::Array2i& xy_index : discrete_scan) {
    const Eigen::Array2i proposed_xy_index(xy_index.x() + x_index_offset,
                                           xy_index.y() + y_index_offset);
    const float probability =
        probability_grid.GetProbability(proposed_xy_index);
    candidate_score += probability;
  }
  candidate_score /= static_cast<float>(discrete_scan.size());
  CHECK_GT(candidate_score, 0.f);
  return candidate_score;
}

}  // namespace

RealTimeCorrelativeScanMatcher2D::RealTimeCorrelativeScanMatcher2D(
    const proto::RealTimeCorrelativeScanMatcherOptions& options)
    : options_(options) {
      
      
    }

std::vector<Candidate2D>
RealTimeCorrelativeScanMatcher2D::GenerateExhaustiveSearchCandidates(
    const SearchParameters& search_parameters) const {
  int num_candidates = 0;
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    const int num_linear_x_candidates =
        (search_parameters.linear_bounds[scan_index].max_x -
         search_parameters.linear_bounds[scan_index].min_x + 1);
    const int num_linear_y_candidates =
        (search_parameters.linear_bounds[scan_index].max_y -
         search_parameters.linear_bounds[scan_index].min_y + 1);
    num_candidates += num_linear_x_candidates * num_linear_y_candidates;
  }
  std::vector<Candidate2D> candidates;
  candidates.reserve(num_candidates);
  for (int scan_index = 0; scan_index != search_parameters.num_scans;
       ++scan_index) {
    for (int x_index_offset = search_parameters.linear_bounds[scan_index].min_x;
         x_index_offset <= search_parameters.linear_bounds[scan_index].max_x;
         ++x_index_offset) {
      for (int y_index_offset =
               search_parameters.linear_bounds[scan_index].min_y;
           y_index_offset <= search_parameters.linear_bounds[scan_index].max_y;
           ++y_index_offset) {
        candidates.emplace_back(scan_index, x_index_offset, y_index_offset,
                                search_parameters);
      }
    }
  }
  CHECK_EQ(candidates.size(), num_candidates);
  return candidates;
}

double RealTimeCorrelativeScanMatcher2D::Match(
    const transform::Rigid2d& initial_pose_estimate,
    const sensor::PointCloud& point_cloud, const Grid2D& grid,
    transform::Rigid2d* pose_estimate) const {
  CHECK(pose_estimate != nullptr);

  const Eigen::Rotation2Dd initial_rotation = initial_pose_estimate.rotation();
  const sensor::PointCloud rotated_point_cloud = sensor::TransformPointCloud(
      point_cloud,
      transform::Rigid3f::Rotation(Eigen::AngleAxisf(
          initial_rotation.cast<float>().angle(), Eigen::Vector3f::UnitZ())));
  const SearchParameters search_parameters(
      options_.linear_search_window(), options_.angular_search_window(),
      rotated_point_cloud, grid.limits().resolution());

  const std::vector<sensor::PointCloud> rotated_scans =
      GenerateRotatedScans(rotated_point_cloud, search_parameters);
  const std::vector<DiscreteScan2D> discrete_scans = DiscretizeScans(
      grid.limits(), rotated_scans,
      Eigen::Translation2f(initial_pose_estimate.translation().x(),
                           initial_pose_estimate.translation().y()));
  std::vector<Candidate2D> candidates =
      GenerateExhaustiveSearchCandidates(search_parameters);
  ScoreCandidates(grid, discrete_scans, search_parameters, &candidates);

  const Candidate2D& best_candidate =
      *std::max_element(candidates.begin(), candidates.end());
  *pose_estimate = transform::Rigid2d(
      {initial_pose_estimate.translation().x() + best_candidate.x,
       initial_pose_estimate.translation().y() + best_candidate.y},
      initial_rotation * Eigen::Rotation2Dd(best_candidate.orientation));
  return best_candidate.score;
}



double RealTimeCorrelativeScanMatcher2D::Match_Nicp(
    const transform::Rigid2d& initial_pose_estimate,
    const sensor::PointCloud& point_cloud, 
    const std::shared_ptr<const Submap2D> matching_submap,
    transform::Rigid2d* pose_estimate,
    const sensor::RangeData& gravity_aligned_range_data){
    CHECK(pose_estimate != nullptr);
    sensor::PointCloud current_cloud;
    for(cartographer::sensor::PointCloud::PointType point : point_cloud.points()){
      point.position.head<2>() = initial_pose_estimate.cast<float>() * point.position.head<2>();
      point.position[2] = 0;
      current_cloud.push_back(point);
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr currentPclCloud = ToPointCloudMessage(current_cloud);

    nicp::PointInformationMatrixCalculator2d pointInformationMatrixCalculator;
    nicp::NormalInformationMatrixCalculator2d normalInformationMatrixCalculator;
    nicp::CloudConverter converter;
      // 最近邻方法寻找对于点对
    nicp::CorrespondenceFinderNN correspondenceFinder;
    // 设置曲率 距离 角度阈值

    // Create Linearizer and Aligner
    nicp::Linearizer2d linearizer;
    nicp::AlignerNN2d aligner;
    pointInformationMatrixCalculator.setCurvatureThreshold(0.02f);
    normalInformationMatrixCalculator.setCurvatureThreshold(0.02f);
    // converter.setNormalInformationMatrixCalculator(&normalInformationMatrixCalculator);
    // converter.setPointInformationMatrixCalculator(&pointInformationMatrixCalculator);
    converter = nicp::CloudConverter(
          &pointInformationMatrixCalculator,
          &normalInformationMatrixCalculator);
    correspondenceFinder.setInlierDistanceThreshold(1.0f);
    correspondenceFinder.setInlierNormalAngularThreshold(0.9f);
    correspondenceFinder.setFlatCurvatureThreshold(0.02f);
    linearizer.setInlierMaxChi2(9e2);
    linearizer.setRobustKernel(true);
    linearizer.setAligner(&aligner);
    //设置 迭代次数 和 LM 算法中的lambda
    aligner.setOuterIterations(options_.nicp_outeriterations());
    aligner.setInnerIterations(20);
    aligner.setLambda(1e3);
    aligner.setCorrespondenceFinder(&correspondenceFinder);
    aligner.setLinearizer(&linearizer);
    Eigen::Isometry2f initialGuess = Eigen::Isometry2f::Identity();
    auto lastPclCloud = matching_submap->GetPointData();
    // currentCloud->clear();
    // referenceCloud->clear();
    nicp::Cloud referenceCloud, currentCloud;
    converter.compute(referenceCloud, lastPclCloud);
    // if(referenceCloud->size() == 0){
    //   auto lastPclCloud = matching_submap->GetPointData();
    //   converter.compute(*referenceCloud, lastPclCloud);
    // }
    converter.compute(currentCloud, currentPclCloud);
    aligner.setReferenceCloud(&referenceCloud);
    aligner.setCurrentCloud(&currentCloud);
    aligner.setInitialGuess(initialGuess);
    aligner.align();
    Eigen::Isometry2f T = aligner.T();

    transform::Rigid2d delta(T.matrix().block<2, 1>(0, 2).cast<double>(), Eigen::Rotation2Dd(T.matrix().block<2, 2>(0, 0).cast<double>()));
    *pose_estimate = delta * initial_pose_estimate;
    return 0;
  }




double RealTimeCorrelativeScanMatcher2D::Match(
    const transform::Rigid2d& initial_pose_estimate,
    const sensor::PointCloud& point_cloud, 
    const std::shared_ptr<const Submap2D> matching_submap,
    transform::Rigid2d* pose_estimate,
    const sensor::RangeData& gravity_aligned_range_data) const{
    pcl::PointCloud<pcl::PointXYZ>::Ptr
    target = matching_submap->GetPointData();
    sensor::PointCloud source_cloud;
    for(cartographer::sensor::PointCloud::PointType point : point_cloud.points()){
      point.position.head<2>() = initial_pose_estimate.cast<float>() * point.position.head<2>();
      point.position[2] = 0;
      source_cloud.push_back(point);
    }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr source (new pcl::PointCloud<pcl::PointXYZ>);
    
    source = ToPointCloudMessage(source_cloud);
    
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    pcl::PointCloud<pcl::PointXYZ> Final;
    pcl::registration::TransformationEstimation2D<pcl::PointXYZ, pcl::PointXYZ>::Ptr est;
    est.reset(new pcl::registration::TransformationEstimation2D<pcl::PointXYZ, pcl::PointXYZ>);
    
    icp.setTransformationEstimation(est);
    //最大匹配距离
    icp.setMaxCorrespondenceDistance(1);
    icp.setMaximumIterations(300);
    icp.setTransformationEpsilon(1e-10);
    icp.setEuclideanFitnessEpsilon(1e-5);
    icp.setRANSACIterations(4);
    icp.setInputSource(source);
    icp.setInputTarget(target);
    
    icp.align(Final);

    Eigen::Matrix4d transformation = icp.getFinalTransformation().cast<double>();
    transform::Rigid2d delta(Eigen::Vector2d(transformation.block<2, 1>(0, 3)), 
                              Eigen::Rotation2Dd(transformation.block<2, 2>(0, 0)));
    *pose_estimate = delta * initial_pose_estimate;
    return icp.getFitnessScore();
    }

double RealTimeCorrelativeScanMatcher2D::Match(
    const transform::Rigid2d& initial_pose_estimate,
    const sensor::PointCloud& point_cloud, const std::shared_ptr<const Submap2D> matching_submap,
    transform::Rigid2d* pose_estimate, int) const{
      pcl::PointCloud<pcl::PointXYZ>::Ptr
    non_target = matching_submap->GetPointData();
    pcl::PointCloud<pcl::PointXYZ>::Ptr target(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::VoxelGrid<pcl::PointXYZ> sor;
    // sor.setInputCloud (non_target);
    // sor.setLeafSize (0.05f, 0.05f, 0.05f);
    // sor.filter (*target);
    sensor::PointCloud source_cloud;
    for(cartographer::sensor::PointCloud::PointType point : point_cloud.points()){
      
      // cartographer::sensor::PointCloud::PointType new_point( Eigen::Vector3f());
      point.position.head<2>() = initial_pose_estimate.cast<float>() * point.position.head<2>();
      point.position[2] = 0;
      source_cloud.push_back(point);
      //  source_cloud.points().at(i).position[2] = 0;
    }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr source (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ> ndt_;
    source = ToPointCloudMessage(source_cloud);
    pcl::registration::TransformationEstimation2D<pcl::PointXYZ, pcl::PointXYZ>::Ptr est;
    est.reset(new pcl::registration::TransformationEstimation2D<pcl::PointXYZ, pcl::PointXYZ>);

    ndt_.setTransformationEstimation(est);
  
    //最大匹配距离
    ndt_.setTransformationEpsilon (1e-10);
  // Setting maximum step size for More-Thuente line search.
    ndt_.setStepSize (0.1);
    //Setting Resolution of NDT grid structure (VoxelGridCovariance).
    ndt_.setResolution (0.5);

    // Setting max number of registration iterations.
    ndt_.setMaximumIterations (100);
    
    pcl::PointCloud<pcl::PointXYZ> Final;
    // Setting point cloud to be aligned.
    ndt_.setInputSource (source);
    // Setting point cloud to be aligned to.
    if(ndt_.getInputTarget() == nullptr){
        ndt_.setInputTarget (target);
    }
    ndt_.align(Final);
    // if(!icp.hasConverged()){
    //   std::cout << "icp nonconverged!" << std::endl;
    // }
    Eigen::Matrix4d transformation = ndt_.getFinalTransformation().cast<double>();
    transform::Rigid2d delta(Eigen::Vector2d(transformation.block<2, 1>(0, 3)), Eigen::Rotation2Dd(transformation.block<2, 2>(0, 0)));
    *pose_estimate = delta * initial_pose_estimate;
    return ndt_.getFitnessScore();
  }

void RealTimeCorrelativeScanMatcher2D::ScoreCandidates(
    const Grid2D& grid, const std::vector<DiscreteScan2D>& discrete_scans,
    const SearchParameters& search_parameters,
    std::vector<Candidate2D>* const candidates) const {
  for (Candidate2D& candidate : *candidates) {
    switch (grid.GetGridType()) {
      case GridType::PROBABILITY_GRID:
        candidate.score = ComputeCandidateScore(
            static_cast<const ProbabilityGrid&>(grid),
            discrete_scans[candidate.scan_index], candidate.x_index_offset,
            candidate.y_index_offset);
        break;
      case GridType::TSDF:
        candidate.score = ComputeCandidateScore(
            static_cast<const TSDF2D&>(grid),
            discrete_scans[candidate.scan_index], candidate.x_index_offset,
            candidate.y_index_offset);
        break;
    }
    candidate.score *=
        std::exp(-common::Pow2(std::hypot(candidate.x, candidate.y) *
                                   options_.translation_delta_cost_weight() +
                               std::abs(candidate.orientation) *
                                   options_.rotation_delta_cost_weight()));
  }
}

}  // namespace scan_matching
}  // namespace mapping
}  // namespace cartographer
