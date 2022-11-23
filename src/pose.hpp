



#include <eigen3/Eigen/Eigenvalues>

#include "types.hpp"



Eigen::Vector4d NormalizeQuaternion(const Eigen::Vector4d& qvec) {
  const double norm = qvec.norm();
  if (norm == 0) {
    // We do not just use (1, 0, 0, 0) because that is a constant and when used
    // for automatic differentiation that would lead to a zero derivative.
    return Eigen::Vector4d(1.0, qvec(1), qvec(2), qvec(3));
  } else {
    return qvec / norm;
  }
}



Eigen::Matrix3d QuaternionToRotationMatrix(const Eigen::Vector4d& qvec) {
  const Eigen::Vector4d normalized_qvec = NormalizeQuaternion(qvec);
  const Eigen::Quaterniond quat(normalized_qvec(0), normalized_qvec(1),
                                normalized_qvec(2), normalized_qvec(3));
  return quat.toRotationMatrix();
}








std::pair<float, float> Project3DPoint(Point3D *p, Image *im) {
  Eigen::Vector3d P = p->xyz_;

  Eigen::Matrix3d mRcw = QuaternionToRotationMatrix(im->qvec_);
  Eigen::Vector3d mtcw = im->tvec_;

  // 3D in camera coordinates
  const Eigen::Vector3d Pc = mRcw*P+mtcw;
  const float &PcX = Pc(0);
  const float &PcY= Pc(1);
  const float &PcZ = Pc(2);

  // Check positive depth
  if(PcZ<0.0f) {
    return std::make_pair(-1, -1);
  }

  // Project in image and check it is not outside
  float invz = 1.0f/PcZ;
  float u = im->fx_ * PcX * invz + im->cx_;
  float v = im->fy_ * PcY * invz + im->cy_;

  return std::make_pair(u, v);
}



std::vector<std::vector<float>> Project3DPoints(std::vector<Point3D*> &points,
                                                Image *image) {

  std::vector<std::vector<float>> projected_points;

  for (Point3D *point : points) {
    std::pair<float, float> uv = Project3DPoint(point, image);
    std::vector<float> uv_vec = {uv.first, uv.second};
    projected_points.push_back(uv_vec);
  }

  return projected_points;
}
