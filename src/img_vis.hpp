#include <iostream>
#include <string>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>




void ShowPolygons(std::string &img_path,  
                  std::vector<std::vector<int>> &tri, 
                  std::vector<std::vector<float>> &points_2d,
                  std::vector<int> color = {0, 255, 0}) {
    cv::Mat img = cv::imread(img_path, cv::IMREAD_COLOR);

    for (unsigned int i=0; i<tri.size(); i++) {
        for (unsigned int j=0; j<tri[i].size(); j++) {
            int idx = tri[i][j];
            cv::circle(img, cv::Point(points_2d[idx][0], points_2d[idx][1]), 2, cv::Scalar(color[0], color[1], color[2]), 2);
        }
        cv::line(img, cv::Point(points_2d[tri[i][0]][0], points_2d[tri[i][0]][1]), cv::Point(points_2d[tri[i][1]][0], points_2d[tri[i][1]][1]), cv::Scalar(color[0], color[1], color[2]), 2);
        cv::line(img, cv::Point(points_2d[tri[i][1]][0], points_2d[tri[i][1]][1]), cv::Point(points_2d[tri[i][2]][0], points_2d[tri[i][2]][1]), cv::Scalar(color[0], color[1], color[2]), 2);
        cv::line(img, cv::Point(points_2d[tri[i][2]][0], points_2d[tri[i][2]][1]), cv::Point(points_2d[tri[i][0]][0], points_2d[tri[i][0]][1]), cv::Scalar(color[0], color[1], color[2]), 2);
    }

    cv::imshow("Image", img);

    cv::waitKey(0);

    cv::destroyAllWindows();
}