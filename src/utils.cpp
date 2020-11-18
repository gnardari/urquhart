#include <utils.hpp>

double euclideanDistance(std::vector<double> A, std::vector<double> B){
    double d = 0;
    for(size_t i = 0; i < A.size(); ++i){
        d += std::pow(A[i] - B[i], 2);
    }
    return std::sqrt(d);
}