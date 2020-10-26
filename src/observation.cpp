#include <observation.hpp>

namespace urquhart
{

Observation::Observation(){
};

void Observation::delaunayTriangulation_(std::vector<std::vector<double>>& points, std::vector<Polygon>& polygons){
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
    orgQhull::QhullFacetListIterator k(q.facetList());
    while(k.hasNext()){
        orgQhull::QhullFacet f = k.next();

        // get facet neighbors
        orgQhull::QhullFacetSet neighborsSet(f.neighborFacets());
        auto neighbors = neighborsSet.toStdVector();
        std::vector<int> neighIds;
        for (auto n : neighbors){
            neighIds.push_back(n.id());
        }

        // get facet vertices
        std::vector<std::vector<double>> vertices;
        std::vector<int> vtxIds;
        orgQhull::QhullVertexSetIterator i(f.vertices());
        while(i.hasNext()){
            orgQhull::QhullVertex vertex= i.next();
            orgQhull::QhullPoint p = vertex.point();
            vertices.push_back(points[p.id()]);
            vtxIds.push_back(p.id());
        }

        // get facet edges
        std::vector<std::pair<int,int>> edges;
        std::vector<double> edgeLengths;
        for (auto i = 0; i <= vtxIds.size(); ++i){
            int endIdx = i + 1;
            if(i == vtxIds.size())
                endIdx = 0;
            auto edge = std::make_pair(vtxIds[i], vtxIds[endIdx]);
            double len = euclideanDistance(points[edge.first], points[edge.second]);
            edges.push_back(edge);
            edgeLengths.push_back(len);
        }
        Polygon poly(vertices, neighIds, edges, edgeLengths);
        polygons.push_back(poly);
    }
};

} // urquhart