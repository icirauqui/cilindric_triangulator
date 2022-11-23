#ifndef UTILS_HPP
#define UTILS_HPP


#include <iostream>
#include <vector>

#include <algorithm>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include "types.hpp"

pcl::PointCloud<pcl::PointXYZ> convert_to_pcl(std::vector<Point3D> &points) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (int i = 0; i < points.size(); i++) {
        cloud->points.push_back(pcl::PointXYZ(points[i].xyz_[0], points[i].xyz_[1], points[i].xyz_[2]));
    }
    return *cloud;
}


Eigen::Vector4d fit_3d_line(std::vector<Point3D> &points, double distance_threshold = 0.01) {
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(distance_threshold);

    pcl::PointCloud<pcl::PointXYZ> cloud = convert_to_pcl(points);
    seg.setInputCloud(cloud.makeShared());
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() == 0) {
        PCL_ERROR("Could not estimate a model for the given dataset.");
        return Eigen::Vector4d::Zero();
    }

    Eigen::Vector4d line;
    line << coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3];
    return line;
}


pcl::ModelCoefficients::Ptr fit_3d_line_coeffs(std::vector<Point3D> &points, double distance_threshold = 0.01) {
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(distance_threshold);

    pcl::PointCloud<pcl::PointXYZ> cloud = convert_to_pcl(points);
    seg.setInputCloud(cloud.makeShared());
    seg.segment(*inliers, *coefficients);

    return coefficients;
}




std::pair<Eigen::Vector3d, Eigen::Vector3d> ORD(std::vector<Point3D>& points) {
    Eigen::Vector3d avg = Eigen::Vector3d::Zero();
    for (int i = 0; i < points.size(); i++) {
        avg += points[i].xyz_;
    }
    avg /= points.size();

    Eigen::MatrixXd subs(points.size(), 3);
    for (int i = 0; i < points.size(); i++) {
        subs.row(i) = points[i].xyz_ - avg;
    }

    // SVD
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(subs, Eigen::ComputeFullV);
    Eigen::MatrixXd V = svd.matrixV();

    //std::cout << "V: " << V << std::endl;

    // Direction
    Eigen::Vector3d dir = V.col(0);

    return std::make_pair(avg, dir);

    //std::cout << "dir: " << dir << std::endl;
}



std::vector<std::pair<int,int>> Find3dIntersect(std::vector<Point3D> pts1, 
                                                std::vector<Point3D> pts2,
                                                unsigned int min_matches = 3) {
  std::vector<std::pair<int,int>> intersect;

  for (unsigned int i=0; i<pts1.size(); i++){

    std::vector<std::pair<int,int>> tr1(pts1[i].track_);
    std::sort(tr1.begin(), tr1.end(), [](auto &left, auto &right) {
        return left.second < right.second;
    });
        
    for (unsigned int j=0; j<pts2.size(); j++){
      std::vector<std::pair<int,int>> tr2(pts2[j].track_);

      std::sort(tr2.begin(), tr2.end(), [](auto &left, auto &right) {
        return left.second < right.second;
      });

      std::vector<std::pair<int,int>> inter;
      std::set_intersection(tr1.begin(), tr1.end(),
                            tr2.begin(), tr2.end(),
                            std::back_inserter(inter));
            
      if (inter.size() >= min_matches){
        intersect.push_back(std::make_pair(i, j));
      }
    } 
  }

  return intersect;
}





std::vector<int> IntersectFrames(std::vector<Image> &img1,
                                 std::vector<Image> &img2) {

    std::vector<int> intersect;

    std::vector<int> frames_base;
    for (int i = 0; i < img1.size(); i++) {
        frames_base.push_back(img1[i].image_id_);
    }

    std::vector<int> frames_float;
    for (int i = 0; i < img2.size(); i++) {
        frames_float.push_back(img2[i].image_id_);
    }

    std::sort(frames_base.begin(), frames_base.end());
    std::sort(frames_float.begin(), frames_float.end());

    std::set_intersection(frames_base.begin(), frames_base.end(),
                          frames_float.begin(), frames_float.end(),
                          back_inserter(intersect));

    return intersect;
}








#endif




