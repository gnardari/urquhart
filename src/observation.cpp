#include <observation.hpp>

namespace urquhart
{

Observation::Observation(){
};

void Observation::delaunayTriangulation_(std::vector<std::vector<double>>& points){
    std::vector<double> qhull_points_data(points.size() * 2);
    for (size_t pidx = 0; pidx < points.size(); ++pidx) {
        const auto& pt = points[pidx];
        qhull_points_data[pidx * 2 + 0] = pt[0];
        qhull_points_data[pidx * 2 + 1] = pt[1];
    }

    orgQhull::PointCoordinates qhull_points(2, "");
    qhull_points.append(qhull_points_data);

    orgQhull::Qhull q;
    q.runQhull(qhull_points.comment().c_str(), qhull_points.dimension(),
                   qhull_points.count(), qhull_points.coordinates(), "Qt");

    // Delaunay regions as a vector of vectors
    std::vector<std::vector<int>> regions;
    orgQhull::QhullFacetListIterator k(q.facetList());
    while(k.hasNext()){
        Polygon p();
        orgQhull::QhullFacet f = k.next();

        // get facet neighbors
        orgQhull::QhullFacetListIterator neighbors(f.neighborFacets());
        std::vector<int> neighIds;
        while(neighbors.hasNext()){
            auto n = k.next();
            neighIds.push_back(n.id());
        }

        // get facet vertices
        std::vector<int> vertices;
        orgQhull::QhullVertexSetIterator i(f.vertices());
        while(i.hasNext()){
            orgQhull::QhullVertex vertex= i.next();
            orgQhull::QhullPoint p = vertex.point();
            vertices.push_back(p.id());
        }
        regions.push_back(vertices);
    }
};

} // urquhart