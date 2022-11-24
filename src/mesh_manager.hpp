#ifndef MESH_MANAGER_HPP
#define MESH_MANAGER_HPP

#include <iostream>
#include <vector>
#include <math.h>
#include <fstream>
#include <thread>

#include "types.hpp"


// OpenCV Libraries
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>

// PCL Libraries
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/point_types.h>

#include <pcl/common/common.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/mls.h>
#include <pcl/surface/impl/mls.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/cloud_viewer.h>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <pcl/surface/gp3.h>

// Other Libraries
#include <Eigen/StdVector>
#include <algorithm>
#include <pcl/visualization/vtk.h>


#include <unordered_map>


class MeshManager {

  public:

    MeshManager() {}

    ~MeshManager() {}




  bool LoadMPsIntoCloud() {
    // Load top layer into cloud
    pc_t_0.width    = vMPsXYZN_.size(); 
    pc_t_0.height   = 1;
    pc_t_0.is_dense = false;
    pc_t_0.points.resize(pc_t_0.width * pc_t_0.height);

    pc0.width    = vMPsXYZN_.size(); 
    pc0.height   = 1;
    pc0.is_dense = false;
    pc0.points.resize(pc_t_0.width * pc_t_0.height);

    for (size_t i = 0; i < pc_t_0.points.size(); i++) {
      pc_t_0.points[i].x = vMPsXYZN_[i][0];
      pc_t_0.points[i].y = vMPsXYZN_[i][1];
      pc_t_0.points[i].z = vMPsXYZN_[i][2];

      pc0.points[i].x = vMPsXYZN_[i][0];
      pc0.points[i].y = vMPsXYZN_[i][1];
      pc0.points[i].z = vMPsXYZN_[i][2];
    }

    if (pc_t_0.width > 0 && pc0.width > 0)
      return true;
    else
      return false;
  }



  std::vector<std::vector<int>> Triangulate() {
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ> (pc_t_0));

    // Normal estimation*
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);
    n.setInputCloud (cloud);
    n.setSearchMethod (tree);
    n.setKSearch (20);
    n.compute (*normals);
    //* normals should not contain the point normals + surface curvatures

    // Concatenate the XYZ and normal fields*
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
    //* cloud_with_normals = cloud + normals

    // Create search tree*
    pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
    tree2->setInputCloud (cloud_with_normals);

    // Initialize objects
    pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
    pcl::PolygonMesh mesh_out;

    // Set the maximum distance between connected points (maximum edge length)
    gp3.setSearchRadius (500.0);

    // Set typical values for the parameters
    gp3.setMu (2.5);
    gp3.setMaximumNearestNeighbors (100);
    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);

    // Get result
    gp3.setInputCloud (cloud_with_normals);
    gp3.setSearchMethod (tree2);
    gp3.reconstruct (mesh_out);

    // Additional vertex information
    // To which part (cloud) does each vertex belong
    std::vector<int> parts = gp3.getPartIDs();
    // Whether the vertex status is [-1,0,1,2,3] = [NONE,FREE,FRINGE,BOUNDARY,COMPLETED]
    std::vector<int> states = gp3.getPointStates();

    std::unordered_map<int, int> map;
    int max_val = -1;
    int max_key = -1;
    for (unsigned int i=0; i<parts.size(); i++) {
      if (map.find(parts[i]) == map.end()) {
        map[parts[i]] = 1;
        if (max_val < 1) {
          max_val = 1;
          max_key = parts[i];
        }
      }
      else {
        map[parts[i]]++;
        if (map[parts[i]] > max_val) {
          max_val = map[parts[i]];
          max_key = parts[i];
        }
      }
    }

    std::vector<std::vector<int>> triangles;
    for (unsigned int i=0; i<mesh_out.polygons.size(); i++) {
      unsigned int nver0 = mesh_out.polygons[i].vertices[0];
      unsigned int nver1 = mesh_out.polygons[i].vertices[1];
      unsigned int nver2 = mesh_out.polygons[i].vertices[2];

      if (parts[nver0] == max_key && parts[nver1] == max_key && parts[nver2] == max_key) {
        std::vector<int> triangle;
        triangle.push_back(nver0);
        triangle.push_back(nver1);
        triangle.push_back(nver2);
        triangles.push_back(triangle);
      }
    }
    
    return triangles;
  }





  std::vector<Point3D*> vpMPs;
  std::vector<std::vector<float>> vMPsXYZN_;
  std::vector<MeshSet*> mesh_sets_;

  // Staring PointCloud
  pcl::PointCloud<pcl::PointNormal> pc0;
  pcl::PointCloud<pcl::PointXYZ> pc_t_0;

  // Smoothed PointCloud (MLS)
  pcl::PointCloud<pcl::PointNormal> pc_t_1;
  std::vector<int> vtindices;

  // Reconstructed mesh
  pcl::PolygonMesh mesh_t;
  std::vector<std::vector<int> > triangles_t;
  std::vector<std::vector<Point3D*> > vpMPs2Draw;
    
  // Set Points as active or not
  std::vector<bool> vbtMPsActive_;

  // Interface 
  float mls_search_radius_ = 1.0;
  int mls_polynomial_order_ = 3;






  float mesh_mu_ = 2.5;
  float mesh_search_radius_ = 1.0;
  int mesh_max_neighbours_ = 25;
  int mesh_surf_angle_ = 150;
  int mesh_min_angle_ = 5;
  int mesh_max_angle_ = 85;

};




















#endif