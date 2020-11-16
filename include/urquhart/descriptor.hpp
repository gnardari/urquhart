#pragma once
#include <opencv2/opencv.hpp>
#include <utils.hpp>

namespace descriptor{
    PointVector samplePoints(PointVector points, double step);
    std::vector<double> centroidDist(PointVector points, PointVector sampledPoints);
    std::vector<double> invariantFourier(std::vector<double> centroidDesc);
    std::vector<double> compute(PointVector points);
}