#include <observation.hpp>
#include <utils.hpp>

using namespace urquhart;

int main(){
    std::vector<Polygon> triangles;
    PointVector points{{0,0}, {0,1.1}, {1,0}, {1,1}, {1,2}};
    Observation obs;
    obs.delaunayTriangulation_(points, triangles);
    std::cout << "Num tris: " << triangles.size() << std::endl;

    return 0;
}