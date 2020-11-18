#include <matching.hpp>
#include <observation.hpp>
#include <utils.hpp>

int main(){
    std::vector<urquhart::Polygon> triangles;
    PointVector points{{0,0}, {0,1.1}, {1,0}, {1,1}, {1,2}};
    urquhart::Observation obs(points);

    PointVector pointr{{0,0}, {-1.1,0}, {0,1}, {-1,1}, {-2,1}};
    urquhart::Observation obsr(pointr);

    auto matches = matching::hierarchyMatching(obs, obsr, 5);
    for(auto m : matches){
        std::cout << m.first[0] << "," << m.first[1] << " -> ";
        std::cout << m.second[0] << "," << m.second[1] << std::endl;
    }

    return 0;
}