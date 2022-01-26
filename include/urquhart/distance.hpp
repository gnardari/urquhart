#pragma once

#include <iostream>
#include <vector>
#include <algorithm>
#include <numeric>
#include <tuple>
#include <utility>
#include <math.h>

using vecPtT = std::vector<double>;
using EdgeT = std::pair<int, int>;
using PointVector = std::vector<vecPtT>;

inline float pow_2(const float &x) { return x * x; }

inline double euclideanDistance(std::vector<double> A, std::vector<double> B)
{
  double d = 0;
  for (size_t i = 0; i < A.size(); ++i)
  {
    d += pow_2(A[i] - B[i]);
  }
  return std::sqrt(d);
}

inline double euclideanDistance2D(std::vector<double> A, std::vector<double> B)
{
  double d = 0;
  for (size_t i = 0; i < 2; ++i)
  {
    d += pow_2(A[i] - B[i]);
  }
  return std::sqrt(d);
}

inline size_t cantorPairing(size_t a, size_t b)
{
  // a is always the smallest number
  size_t aux = a;
  if(b < a)
  {
    a = b;
    b = aux;
  }
  return (a + b) * (a + b + 1) / 2 + a;
}
