#ifndef DATA_HANDLER_HPP
#define DATA_HANDLER_HPP

#include <iostream>
#include <string>
#include <sstream>
#include <fstream>
#include <vector>
#include <algorithm>
#include <numeric>

#include "types.hpp"
#include <eigen3/Eigen/Core>


bool IsNotWhiteSpace(const int character) {
  return character != ' ' && character != '\n' && character != '\r' &&
         character != '\t';
}

void StringLeftTrim(std::string* str) {
  str->erase(str->begin(),
             std::find_if(str->begin(), str->end(), IsNotWhiteSpace));
}

void StringRightTrim(std::string* str) {
  str->erase(std::find_if(str->rbegin(), str->rend(), IsNotWhiteSpace).base(),
             str->end());
}

void StringTrim(std::string* str) {
  StringLeftTrim(str);
  StringRightTrim(str);
}



void ReadImages(std::string path, std::vector<Point3D>& points) {

  std::ifstream file(path);
  if (!file.is_open()) {
    return;
  }

  std::string line;
  std::string item;

  while (std::getline(file, line)) {
    StringTrim(&line);

    if (line.empty() || line[0] == '#') {
      continue;
    }

    std::stringstream line_stream1(line);

    // ID
    std::getline(line_stream1, item, ' ');
    int image_id = std::stoul(item);

    Eigen::Vector4d qvec;
    Eigen::Vector3d tvec;

    // QVEC (qw, qx, qy, qz)
    std::getline(line_stream1, item, ' ');
    qvec(0) = std::stold(item);

    std::getline(line_stream1, item, ' ');
    qvec(1) = std::stold(item);

    std::getline(line_stream1, item, ' ');
    qvec(2) = std::stold(item);

    std::getline(line_stream1, item, ' ');
    qvec(3) = std::stold(item);

    // TVEC
    std::getline(line_stream1, item, ' ');
    tvec(0) = std::stold(item);

    std::getline(line_stream1, item, ' ');
    tvec(1) = std::stold(item);

    std::getline(line_stream1, item, ' ');
    tvec(2) = std::stold(item);

    // CAMERA_ID
    std::getline(line_stream1, item, ' ');
    int CameraId = std::stoul(item);

    // NAME
    std::getline(line_stream1, item, ' ');
    std::string image_name = item;

    // POINTS2D
    if (!std::getline(file, line)) {
      break;
    }

    StringTrim(&line);
    std::stringstream line_stream2(line);

    std::vector<Eigen::Vector2d> points2D;
    std::vector<int> point3D_ids;
    std::vector<int> point3Dd_ids;    //IC05

    if (!line.empty()) {
      while (!line_stream2.eof()) {
        Eigen::Vector2d point;

        std::getline(line_stream2, item, ' ');
        point.x() = std::stold(item);

        std::getline(line_stream2, item, ' ');
        point.y() = std::stold(item);

        std::getline(line_stream2, item, ' ');
        std::getline(line_stream2, item, ' ');
      }
    }

    points.push_back(Point3D(tvec));
  }
}


void ReadPoints3D(std::string path, std::vector<Point3D>& points) {

  std::ifstream file(path);
  if (!file.is_open()) {
    std::cout << "Unable to open file " << path << std::endl;
    return;
  }

  std::string line;
  std::string item;

  while (std::getline(file, line)) {
    StringTrim(&line);

    if (line.empty() || line[0] == '#') {
      continue;
    }

    std::stringstream line_stream(line);

    // ID
    std::getline(line_stream, item, ' ');
    const int point3D_id = std::stoll(item);

    // Make sure, that we can add new 3D points after reading 3D points
    // without overwriting existing 3D points.
    //model.num_added_points3D_ = std::max(model.num_added_points3D_, point3D_id);

    // XYZ
    std::getline(line_stream, item, ' ');
    double x = std::stold(item);

    std::getline(line_stream, item, ' ');
    double y = std::stold(item);

    std::getline(line_stream, item, ' ');
    double z = std::stold(item);

    class Point3D point3D;
    point3D.id_ = point3D_id;
    point3D.xyz_ = Eigen::Vector3d(x, y, z);

    // Color
    std::getline(line_stream, item, ' ');
    double color0 = static_cast<uint8_t>(std::stoi(item));

    std::getline(line_stream, item, ' ');
    double color1 = static_cast<uint8_t>(std::stoi(item));

    std::getline(line_stream, item, ' ');
    double color2 = static_cast<uint8_t>(std::stoi(item));

    // ERROR
    std::getline(line_stream, item, ' ');
    auto berror = std::stold(item);


    // TRACK
    while (!line_stream.eof()) {
      //Track::TrackElement track_el;

      std::getline(line_stream, item, ' ');
      StringTrim(&item);
      if (item.empty()) {
        break;
      }
      int image_id = std::stoul(item);

      std::getline(line_stream, item, ' ');
      int point2D_idx = std::stoul(item);
    }

    points.push_back(point3D);
  }
  
}


#endif