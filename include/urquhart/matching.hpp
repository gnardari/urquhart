#pragma once

#include <set>
#include <distance.hpp>
#include <polygon.hpp>
#include <observation.hpp>

namespace matching
{
    // Expands triangle matches into edge and point correspondences.
    // This function will test all possible combinations of edge assignments and use the one
    // with smallest euclidean distance between the edge lengths
    void linePointMatching(const urquhart::Polygon &A, const urquhart::Polygon &B,
                           std::vector<std::pair<vecPtT, vecPtT>> &pointMatches,
                           std::set<size_t> &unique_matches);

    // Matches polygons (works for triangles as well). This function does a greedy assignment
    // based on the euclidean distance of the DFT descriptors. The best match has to be under thresh to be accepted
    void polygonMatching(
        urquhart::Observation &ref, std::vector<size_t> refIds,
        urquhart::Observation &targ, std::vector<size_t> targIds, double thresh,
        std::vector<std::pair<size_t, size_t>> &polygonMatches);

    // Matches a pair of observations. Returns a vector of tuples of points that were considered matches.
    // pair.first refers to a point in ref, pair.second refers to a point in targ
    std::vector<std::pair<vecPtT, vecPtT>> hierarchyMatching(urquhart::Observation &ref,
                                                             urquhart::Observation &targ, double thresh);
}