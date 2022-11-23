#include <iostream>
#include <string>

#include "data_handler.hpp"
#include "viewer.hpp"


#include "types.hpp"

#include "utils.hpp"
#include <unordered_map>

#include "img_vis.hpp"

#include "mesh_manager.hpp"


#include <pcl/ModelCoefficients.h>


int main() {

  std::vector<Model> models;
  std::vector<std::string> folders = {"global","16","38"};
  for (std::string label : folders) {
      std::string path_images = "../models/" + label + "/images.txt";
      std::string path_points = "../models/" + label + "/points3D.txt";
      std::vector<Image> images;
      std::vector<Point3D> points3D;
      ReadImages(path_images, images);
      ReadPoints3D(path_points, points3D);
      Model model;
      model.images_ = images;
      model.points3D_ = points3D;
      model.ComputeIndices();
      models.push_back(model);
  }

  int m1id = 0;
  int m2id = 1;

  std::vector<std::pair<int,int>> intersect_points = Find3dIntersect(models[m1id].points3D_, 
                                                                     models[m2id].points3D_);

  std::vector<int> intersect_images = IntersectFrames(models[m1id].images_, 
                                                      models[m2id].images_);

  std::cout << " - Points intersect = " << intersect_points.size() << std::endl;
  std::cout << " - Images intersect = " << intersect_images.size() << std::endl;

  int id_anchor = intersect_images[intersect_images.size() / 2];
  std::cout << " - Anchor image = " << id_anchor << std::endl;
  int img_idx = models[m2id].image_indices_[id_anchor];
  Image* pi = &models[m2id].images_[img_idx];
  std::string img_path = "../images/" + pi->name_;





  std::vector<std::vector<float>> points_2d;
  std::vector<Point3D*> points_3d;

  Model* pm = &models[m2id];
  for (unsigned int i=0; i<intersect_points.size(); i++) {
    int p3d_idx = intersect_points[i].second;
    Point3D* pp3d = &pm->points3D_[p3d_idx];

    for (auto tr : pp3d->track_) {
      int img_id = tr.first;
      int pt_id = tr.second;

      if (img_id == id_anchor) {
        int i_idx = pm->image_indices_[img_id];
        Image* pi = &pm->images_[i_idx];
        Eigen::Vector2d* p2d = &pi->points2D_[pt_id];
        points_2d.push_back({p2d->x(), p2d->y()});
        points_3d.push_back(pp3d);

        break;
      }
    }
  }


  std::cout << " - Points 2d = " << points_2d.size() << std::endl;

  MeshManager mm;
  for (unsigned int i=0; i<points_2d.size(); i++) {
    mm.vMPsXYZN_.push_back({points_2d[i][0], points_2d[i][1], 0.0f});
    mm.vbtMPsActive_.push_back(true);
  }

  std::cout << "Data loaded into MeshManager" << std::endl;

  std::vector<std::vector<int>> tri;
  if (mm.LoadMPsIntoCloud()) {
    tri = mm.Triangulate();
  }

  ShowPolygons(img_path, tri, points_2d);


  std::vector<std::vector<Point3D>> points_3d_vis;
  for (unsigned int i=0; i<tri.size(); i++) {
    std::vector<Point3D> p3d;
    for (unsigned int j=0; j<tri[i].size(); j++) {
      p3d.push_back(*points_3d[tri[i][j]]);
    }
    points_3d_vis.push_back(p3d);
  }

  Viewer v;
  v.add_pc(points_3d_vis, "mesh");

  



  std::cout << " [ OK ] " << std::endl;

  return 0;
}