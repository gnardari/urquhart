#include <observation.hpp>

namespace urquhart
{

Observation::Observation(PointVector& landmarks){
    std::vector<Polygon> triangles;
    delaunayTriangulation_(landmarks, triangles);
    H = new Tree(triangles);
    urquhartTesselation_();
};

void Observation::urquhartTesselation_(){
    std::vector<size_t> leaves = H->traverse();
    for(size_t leaf : leaves){
        Polygon p = H->get_vertex(leaf);
        int neighId = -1;
        double longestEdgeLen = -1;
        for(int i=0; i < p.edgeLengths.size(); ++i){
            if(p.edgeLengths[i] > longestEdgeLen){
                longestEdgeLen = p.edgeLengths[i];
                neighId = p.neighbors[i];
            }
        }
        if(neighId == -1) continue;
        Polygon n = H->get_vertex(neighId);
        std::cout << "Poly neigh: " << neighId << std::endl;
        std::cout << p.edgeLengths[0] << " " << p.edgeLengths[1] << " " << p.edgeLengths[2] << std::endl;
        std::cout << p.neighbors[0] << " " << p.neighbors[1] << " " << p.neighbors[2] << std::endl;
        std::cout << "------------------" << std::endl;
        // size_t ancIdx = H->get_ancestor(neighId);
        // Polygon ancestor = H->get_vertex(ancIdx);
    }
}

void Observation::delaunayTriangulation_(PointVector& points, std::vector<Polygon>& polygons){
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
                   qhull_points.count(), qhull_points.coordinates(), "Qt Qbb Qc Qz Q12 d");

    std::cout << "Qhull dim " << q.hullDimension() << std::endl;
    std::cout << "N facets " << q.facetCount() << std::endl;

    // Delaunay regions as a vector of vectors
    orgQhull::QhullFacetListIterator k(q.facetList());

    // the facet ids are confusing, we want to map the good facets to order of appearance
    size_t fIdx = 0;
    std::map<size_t, size_t> id_map;
    for (auto e : q.facetList()){
        if(e.isGood()){
            id_map[e.id()] = fIdx;
            fIdx++;
        }
    }

    while(k.hasNext()){
        orgQhull::QhullFacet f = k.next();
        if (!f.isGood()) continue;

        // get neighbors for each edge of the triangle
        orgQhull::QhullFacetSet neighborsSet(f.neighborFacets());
        // this will only list neighbors that are also a good facet, so we complete with -1
        std::vector<int> auxNeighIds;
        std::vector<int> neighIds(3);
        for (auto e : neighborsSet){
            if(e.isGood())
                auxNeighIds.push_back(id_map[e.id()]);
            else
                auxNeighIds.push_back(-1);
        }
    
        // shifting neighbor order to align with the edges (edge[i] will be shared with neighIds[i])
        for(size_t i=1; i <= auxNeighIds.size(); ++i)
            neighIds[i%auxNeighIds.size()] = auxNeighIds[i-1];

        // get facet vertices
        PointVector vertices;
        std::vector<int> vtxIds;
        orgQhull::QhullVertexSetIterator i(f.vertices());
        while(i.hasNext()){
            orgQhull::QhullVertex vertex= i.next();
            orgQhull::QhullPoint p = vertex.point();
            vertices.push_back(points[p.id()]);
            vtxIds.push_back(p.id());
        }

        // get facet edges
        std::vector<EdgeT> edges;
        std::vector<double> edgeLengths;
        for (auto i = 0; i < vtxIds.size(); ++i){
            int endIdx = i + 1;
            if(endIdx == vtxIds.size())
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