#ifndef VIEWER_HPP
#define VIEWER_HPP

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include <iostream>
#include <vector>

#include <boost/date_time.hpp>
#include <boost/thread.hpp>

#include "types.hpp"



class Viewer {

public:

  Viewer() {
    viewer_ = new pcl::visualization::PCLVisualizer("3D Viewer");
    viewer_->setBackgroundColor(0, 0, 0);
    viewer_->addCoordinateSystem(1.0);
    viewer_->initCameraParameters();
  }


  ~Viewer() {}

  void add_pc(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string name) {
    viewer_->addPointCloud(cloud, name);
  }


  void add_pc(std::vector<Point3D> &points, 
              std::string pc_name, Eigen::Vector3d color = Eigen::Vector3d(255, 255, 255)) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (int i = 0; i < points.size(); i++) {
      cloud->points.push_back(pcl::PointXYZRGB(points[i].xyz_[0], points[i].xyz_[1], points[i].xyz_[2], color[0], color[1], color[2]));
      //cloud->points.back().rgb = *reinterpret_cast<float*>(&rgb);
    }
    viewer_->addPointCloud(cloud, pc_name);
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, pc_name);
  }


  void add_pc(std::vector<std::vector<Point3D>> &points, 
              std::string pc_name, Eigen::Vector3d color = Eigen::Vector3d(255, 255, 255)) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (int i = 0; i < points.size(); i++) {
      cloud->points.push_back(pcl::PointXYZRGB(points[i].xyz_[0], points[i].xyz_[1], points[i].xyz_[2], color[0], color[1], color[2]));
      //cloud->points.back().rgb = *reinterpret_cast<float*>(&rgb);
    }
    viewer_->addPointCloud(cloud, pc_name);
    viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, pc_name);
  }


  void add_line(Eigen::Vector4d line, 
                std::string line_name, 
                Eigen::Vector3d color = Eigen::Vector3d(255, 255, 255)) {
                  
    viewer_->addLine(pcl::PointXYZ(line[0], line[1], line[2]), 
                     pcl::PointXYZ(line[0] + line[3], line[1] + line[4], line[2] + line[5]), 
                     color[0], color[1], color[2], 
                     line_name);
  }

  void add_line_coeffs(pcl::ModelCoefficients coeffs, 
                std::string line_name, 
                Eigen::Vector3d color = Eigen::Vector3d(255, 255, 255)) {
    viewer_->addLine(coeffs, 
                     line_name);
  }


  void spin() {
    while (!viewer_->wasStopped()) {
      viewer_->spin();
      //boost::this_thread::sleep(boost::posix_time::microseconds(100000));
    } 
  }


private:

  pcl::visualization::PCLVisualizer *viewer_;

};


#endif