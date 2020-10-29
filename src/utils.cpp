#include <utils.hpp>

double euclideanDistance(PointT A, PointT B){
    double x = A[0] - B[0];
    double y = A[1] - B[1];
    return std::sqrt(std::pow(x,2) + std::pow(y,2));
}