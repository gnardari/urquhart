#include <matching.hpp>

namespace matching {

void polygonMatching(
    urquhart::Observation& ref, std::vector<size_t> refIds,
    urquhart::Observation& targ, std::vector<size_t> targIds, double thresh,
    std::vector<std::pair<size_t, size_t>>& polygonMatches){

     std::set<size_t> matched;
     for(auto rIdx : refIds){
         size_t bestMach = -1;
         size_t bestDist = 100000;
         urquhart::Polygon rp = ref.H->get_vertex(rIdx);
         for(auto tIdx : targIds){
             urquhart::Polygon tp = targ.H->get_vertex(tIdx);
             // if tIdx was not matched before and the difference of number of points is not larger than 3
             if(matched.find(tIdx) == matched.end() &&
                std::abs(int(rp.points.size() - tp.points.size())) <= 3){
                 double d = euclideanDistance(rp.descriptor, tp.descriptor);
                 if(d < bestDist){
                     bestDist = d;
                     bestMach = tIdx;
                 }
             }
         }
         if(bestDist < thresh){
             matched.insert(bestMach);
             std::pair<size_t, size_t> m = std::make_pair(rIdx, bestMach);
             polygonMatches.push_back(m);
         }
     }
 }

void linePointMatching(const urquhart::Polygon& A, const urquhart::Polygon& B,
        std::vector<std::pair<PointT, PointT>>& pointMatches){
    std::vector<size_t> chi = {0,1,2};
    std::vector<size_t> bestPermutation;
    double bestDist = 1000000;
    do {
        // std::cout << B.edgeLengths[chi[0]] << ' ' << B.edgeLengths[chi[1]] << ' ' << B.edgeLengths[chi[2]] << '\n';
        std::vector<double> permutation = {B.edgeLengths[chi[0]], B.edgeLengths[chi[1]], B.edgeLengths[chi[2]]};
        double d = euclideanDistance(A.edgeLengths, permutation);
        if(d < bestDist){
            bestDist = d;
            bestPermutation = chi;
        }
    } while (std::next_permutation(chi.begin(), chi.end()));

    for(size_t i = 0; i < 3; ++i){
        PointT pA = A.points[i];
        PointT pB = B.points[bestPermutation[i]];
        pointMatches.push_back(std::make_pair(pA, pB));
    }
}

std::vector<std::pair<PointT, PointT>> hierarchyMatching(urquhart::Observation& ref, urquhart::Observation& targ, double thresh){
    std::vector<size_t> refIds = ref.H->get_children(0);
    std::vector<size_t> targIds = targ.H->get_children(0);
    std::vector<std::pair<size_t, size_t>> polygonMatches;
    polygonMatching(ref, refIds, targ, targIds, thresh, polygonMatches);

    std::vector<std::pair<size_t,size_t>> triangleMatches;
    for(auto pMatch : polygonMatches){
        refIds = ref.H->get_children(pMatch.first);
        targIds = targ.H->get_children(pMatch.second);
        // TODO: ADD CHECK IF % OF TRIANGLES THAT MACTHED IS LARGER THAN 1/2
        polygonMatching(ref, refIds, targ, targIds, thresh, triangleMatches);
        // std::copy(matches.begin(), matches.end(), std::back_inserter(triangleMatches));
    }

    std::vector<std::pair<PointT,PointT>> pointMatches;
    for(auto tMatch : triangleMatches){
        urquhart::Polygon rT = ref.H->get_vertex(tMatch.first);
        urquhart::Polygon tT = targ.H->get_vertex(tMatch.second);
        linePointMatching(rT, tT, pointMatches);
        // std::copy(matches.begin(), matches.end(), std::back_inserter(pointMatches));
    }
    return pointMatches;
}

} // matching