
#ifndef TYPES_HPP
#define TYPES_HPP



#include <eigen3/Eigen/Core>
#include <unordered_map>

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
    Eigen::Vector3d color_;
    double error_;
    
    //std::vector<int> track_images;
    //std::vector<int> track_points;
    std::vector<std::pair<int,int>> track_;

    bool is_outlier_ = false;
    double repr_error_;

    std::pair<int,int> m2_target_id_;


};


class Image {

  public:

    Image() {
      qvec_ = Eigen::Vector4d::Zero();
      tvec_ = Eigen::Vector3d::Zero();
    }



    int image_id_;
    int camera_id_;
          
    double fx_;
    double fy_;
    double cx_;
    double cy_;
    double cam_w_;
    double cam_h_;

    std::string name_;

    Eigen::Vector4d qvec_;
    Eigen::Vector3d tvec_;

    std::vector<Eigen::Vector2d> points2D_;
    std::vector<int> num_correspondences_have_point3D_;
    std::vector<int> point2D_point3D_;
    std::vector<int> point2D_point3Dd_;

    

    int num_points3D_;
    int num_points3Dd_;

};




class Model {
  public:
    std::vector<Point3D> points3D_;
    std::vector<Image> images_;

    std::unordered_map<int,int> image_indices_;

    void ComputeIndices() {
      image_indices_.clear();
      for (unsigned int i=0; i<images_.size(); i++) {
        image_indices_[images_[i].image_id_] = i;
      }
    }


};



class MeshSet {
  public:

    MeshSet(int m_idx, int i_idx, int p3d_idx, int p2d_idx, 
            Model* m, Image* i, Point3D* p3d, Eigen::Vector2d* p2d) : 
            m_idx_(m_idx), i_idx_(i_idx), p3d_idx_(p3d_idx), p2d_idx_(p2d_idx),
            m_(m), i_(i), p3d_(p3d), p2d_(p2d) {}

    int m_idx_ = -1;
    int i_idx_ = -1;
    int p3d_idx_ = -1;
    int p2d_idx_ = -1;

    Model* m_;
    Image* i_;
    Point3D* p3d_;
    Eigen::Vector2d* p2d_;
};


#endif