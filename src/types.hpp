
#ifndef TYPES_HPP
#define TYPES_HPP



#include <eigen3/Eigen/Core>

class Point3D {
  public:
    Point3D() {
      xyz_ = Eigen::Vector3d::Zero();
    }

    Point3D(Eigen::Vector3d xyz) {
      xyz_ = xyz;
    }

    Point3D(double x, double y, double z) {
      xyz_ = Eigen::Vector3d(x, y, z);
    }
    
    int id_;
    Eigen::Vector3d xyz_;
};


#endif