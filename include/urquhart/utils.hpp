#pragma once

#include <vector>
#include <algorithm>
#include <tuple>
#include <utility>
#include<math.h>

using PointT  = std::vector<double>;
using EdgeT = std::pair<int,int>;
using PointVector = std::vector<PointT>;

double euclideanDistance(PointT A, PointT B);