#include <iostream>
#include <string>

#include "data_handler.hpp"
#include "viewer.hpp"


#include "types.hpp"

#include "utils.hpp"


#include <pcl/ModelCoefficients.h>


int main() {
    std::cout << "Hello, World!" << std::endl;

    std::string path_images = "../models/38/images.txt";
    std::string path_points = "../models/38/points3D.txt";

    std::vector<Point3D> images;
    ReadImages(path_images, images);

    std::vector<Point3D> points3D;
    ReadPoints3D(path_points, points3D);



    Eigen::Vector4d line = fit_3d_line(points3D);
    pcl::ModelCoefficients::Ptr coeffs = fit_3d_line_coeffs(points3D);

    //Viewer viewer;
    //viewer.add_pc(points3D, "points", Eigen::Vector3d(255, 0, 0));
    //viewer.add_pc(images, "images", Eigen::Vector3d(0, 255, 0));
    //viewer.add_line(line, "line", Eigen::Vector3d(0, 0, 255));
    //viewer.add_line_coeffs(*coeffs, "line2", Eigen::Vector3d(0, 255, 255));
    //viewer.spin();


    //std::cout << "[";
    //for (unsigned int i=0; i<points3D.size(); i++) {
    //    std::cout << points3D[i].xyz_.transpose() << ";" << std::endl;
    //}
    //std::cout << "];" << std::endl;

    Eigen::Vector3d p0 = Eigen::Vector3d(0, 0, 0);
    Eigen::Vector3d d0 = Eigen::Vector3d(0, 0, 0);
    std::make_pair(p0, d0) = ORD(points3D);


    return 0;
}