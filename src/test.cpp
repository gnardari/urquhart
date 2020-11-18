#include <observation.hpp>
#include <utils.hpp>

int main(){
    std::vector<urquhart::Polygon> triangles;
    PointVector points{{0,0}, {0,1.1}, {1,0}, {1,1}, {1,2}};
    urquhart::Observation obs(points);

    PointVector pointr{{0,0}, {-1.1,0}, {0,1}, {-1,1}, {-2,1}};
    urquhart::Observation obsr(pointr);
    // obs.delaunayTriangulation_(points, triangles);
    // std::cout << "Num tris: " << triangles.size() << std::endl;

    return 0;
}