#include <observation.hpp>

namespace urquhart{

Observation::Observation(PointVector& points){
    landmarks = points;
    std::vector<Polygon> triangles;
    delaunayTriangulation_(points, triangles);
    H = new Tree(triangles);
    urquhartTesselation_();
    // H->view();
};

void Observation::urquhartTesselation_(){
    std::vector<size_t> leaves = H->traverse();
    for(size_t leaf : leaves){
        Polygon p = H->get_vertex(leaf);
        int neighId = -1;
        int neighPos = -1;
        double longestEdgeLen = -1;
        for(int i=0; i < p.edgeLengths.size(); ++i){
            if(p.edgeLengths[i] > longestEdgeLen){
                longestEdgeLen = p.edgeLengths[i];
                neighId = p.neighbors[i];
                neighPos = i;
            }
        }
        if(neighId == -1) continue;

        // we drop edges based on triangles,
        // but we must merge the highest available polygon that leaf is part of
        // (could be itself if it wasn't part of a merge operation before)
        size_t leafAncIdx = H->get_ancestor(leaf);
        size_t neighAncIdx = H->get_ancestor(neighId);
        if(leafAncIdx != neighAncIdx){
            Polygon n = H->get_vertex(neighId);
            if(neighAncIdx != neighId)
                n = H->get_vertex(neighAncIdx);

            Polygon merged = mergePolygons_(p, n, neighPos);
            H->merge_op(leafAncIdx, neighAncIdx, merged);
        }
    }
}

Polygon Observation::mergePolygons_(Polygon p, Polygon n, size_t commonEdgeIdx){
    /*
    (0,1), (1,2), (2,0)
                        => (0,1), (1,2), (2,0), (4,0)
    (2,0), (0,4), (4,2)
    */
   // rotates puts idx as first element, for p we want the common edge to be the last elem
   p.rotate((commonEdgeIdx+1)%3);
   EdgeT commonEdge = p.edges[commonEdgeIdx];

   std::vector<EdgeT>::iterator it = std::find_if(n.edges.begin(), n.edges.end(),
        [&commonEdge](EdgeT e){
            if(e.first == commonEdge.first && e.second == commonEdge.second) return True;
            if(e.second == commonEdge.first && e.first == commonEdge.second) return True;
            return False;
        });

    n.rotate(it - n.edges.begin());

    size_t mergedSize = p.points.size() + n.points.size();
    std::vector<int> neighbors;
    neighbors.reserve(mergedSize-1);
    neighbors.insert(neighbors.end(), p.neighbors.begin(), p.neighbors.end()-1);
    neighbors.insert(neighbors.end(), n.neighbors.begin()+1, n.neighbors.end());

    std::vector<EdgeT> edges;
    edges.reserve(mergedSize-1);
    edges.insert(edges.end(), p.edges.begin(), p.edges.end()-1);
    edges.insert(edges.end(), n.edges.begin()+1, n.edges.end());

    std::vector<double> edgeLengths;
    edgeLengths.reserve(mergedSize-1);
    edgeLengths.insert(edgeLengths.end(), p.edgeLengths.begin(), p.edgeLengths.end()-1);
    edgeLengths.insert(edgeLengths.end(), n.edgeLengths.begin()+1, n.edgeLengths.end());

    PointVector points;
    for(auto e : edges){
        points.push_back(landmarks[e.first]);
    }
    Polygon merged(points, neighbors, edges, edgeLengths);

    return merged;
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
    // start at one because the root is 0
    size_t fIdx = 1;
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