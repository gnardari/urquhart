#pragma once 

#include <set>
#include <utils.hpp>
#include <polygon.hpp>
#include <observation.hpp>

namespace matching {
    void linePointMatching(const urquhart::Polygon& A, const urquhart::Polygon& B,
        std::vector<std::pair<PointT, PointT>>& pointMatches);

    void polygonMatching(
        urquhart::Observation& ref, std::vector<size_t> refIds,
        urquhart::Observation& targ, std::vector<size_t> targIds, double thresh,
        std::vector<std::pair<size_t, size_t>>& polygonMatches);

    std::vector<std::pair<PointT, PointT>> hierarchyMatching(urquhart::Observation& ref,
                                                             urquhart::Observation& targ, double thresh);
}