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



void ReadImages(std::string path, std::vector<Image>& images) {

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

    Image image;

    // ID
    std::getline(line_stream1, item, ' ');
    image.image_id_ = std::stoul(item);

    // QVEC (qw, qx, qy, qz)
    std::getline(line_stream1, item, ' ');
    image.qvec_(0) = std::stold(item);

    std::getline(line_stream1, item, ' ');
    image.qvec_(1) = std::stold(item);

    std::getline(line_stream1, item, ' ');
    image.qvec_(2) = std::stold(item);

    std::getline(line_stream1, item, ' ');
    image.qvec_(3) = std::stold(item);

    // TVEC
    std::getline(line_stream1, item, ' ');
    image.tvec_(0) = std::stold(item);

    std::getline(line_stream1, item, ' ');
    image.tvec_(1) = std::stold(item);

    std::getline(line_stream1, item, ' ');
    image.tvec_(2) = std::stold(item);

    // CAMERA_ID
    std::getline(line_stream1, item, ' ');
    image.camera_id_ = std::stoul(item);

    // NAME
    std::getline(line_stream1, item, ' ');
    image.name_ = item;

    // POINTS2D
    if (!std::getline(file, line)) {
      break;
    }

    StringTrim(&line);
    std::stringstream line_stream2(line);

    if (!line.empty()) {
      while (!line_stream2.eof()) {
        Eigen::Vector2d point;

        std::getline(line_stream2, item, ' ');
        point.x() = std::stold(item);

        std::getline(line_stream2, item, ' ');
        point.y() = std::stold(item);

        image.points2D_.push_back(point);

        std::getline(line_stream2, item, ' ');
        if (item == "-1") {
          image.point2D_point3D_.push_back(-1);
        } else {
          image.point2D_point3D_.push_back(std::stoll(item));
        }

        std::getline(line_stream2, item, ' ');
        if (item == "-1") {
          image.point2D_point3Dd_.push_back(-1);
        } else {
          image.point2D_point3Dd_.push_back(std::stoll(item));
        }
      }
    }

    images.push_back(image);
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

    Point3D point;

    // ID
    std::getline(line_stream, item, ' ');
    point.id_ = std::stoll(item);

    // XYZ
    std::getline(line_stream, item, ' ');
    double x = std::stold(item);

    std::getline(line_stream, item, ' ');
    double y = std::stold(item);

    std::getline(line_stream, item, ' ');
    double z = std::stold(item);

    point.xyz_ = Eigen::Vector3d(x, y, z);

    // Color
    std::getline(line_stream, item, ' ');
    double color0 = static_cast<uint8_t>(std::stoi(item));

    std::getline(line_stream, item, ' ');
    double color1 = static_cast<uint8_t>(std::stoi(item));

    std::getline(line_stream, item, ' ');
    double color2 = static_cast<uint8_t>(std::stoi(item));

    point.color_ = Eigen::Vector3d(color0, color1, color2);

    // ERROR
    std::getline(line_stream, item, ' ');
    point.error_ = std::stold(item);


    // TRACK
    while (!line_stream.eof()) {
      //Track::TrackElement track_el;

      std::getline(line_stream, item, ' ');
      StringTrim(&item);
      if (item.empty()) {
        break;
      }
      int track_image = std::stoul(item);
      //point.track_images.push_back(std::stoul(item));

      std::getline(line_stream, item, ' ');
      int track_point = std::stoul(item);
      //point.track_points.push_back(std::stoul(item));

      point.track_.push_back(std::make_pair(track_image, track_point));
    }

    points.push_back(point);
  }
  
}


#endif