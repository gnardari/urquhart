#pragma once
#include <opencv2/opencv.hpp>
#include <distance.hpp>

namespace descriptor{
    // Samples points around the perimeter of a polygon and returns a new set of points.
    // The size of the returned PointVector depends on step.
    // Smaller steps means more points will be sampled,
    // e.g. step=0.05 means a point every 5% of the perimeter, resulting in 20 points.
    PointVector samplePoints(PointVector points, double step);
    // Uses sampled points to compute a centroid distance descriptor
    std::vector<double> centroidDist(PointVector points, PointVector sampledPoints);
    // Uses OpenCV to compute the magnitude of the DFT of the centroid descriptor
    std::vector<double> invariantFourier(std::vector<double> centroidDesc);
    // Runs the entire descriptor computation pipeline
    std::vector<double> compute(PointVector points);
}