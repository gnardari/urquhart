#include <observation.hpp>

namespace urquhart{

Observation::Observation(PointVector& points){
    landmarks = points;
    std::vector<Polygon> triangles;
    delaunayTriangulation_(points, triangles);
    H = new Tree(triangles);
    urquhartTesselation_();
};

void Observation::view()
{
    H->view_h2_polygons();
}

void Observation::urquhartTesselation_(){
    std::vector<size_t> leaves = H->traverse();
    for(size_t leaf : leaves){
        Polygon p = H->get_vertex(leaf);
        int neighId = -1;
        int neighPos = -1;
        double longestEdgeLen = -1;
        for(int i=0; i < p.edgeLengths.size(); ++i){
            auto ll = p.edgeLengths[i];
            if(p.edgeLengths[i] > longestEdgeLen){
                longestEdgeLen = p.edgeLengths[i];
                neighId = p.neighbors[i];
                neighPos = i;
            }
        }
        if(neighId == -1) continue;

        EdgeT commonEdge = p.edges[neighPos];
        // we drop edges based on triangles,
        // but we must merge the highest available polygon that leaf is part of
        // (could be itself if it wasn't part of a merge operation before)
        size_t leafAncIdx = H->get_ancestor(leaf);
        size_t neighAncIdx = H->get_ancestor(neighId);
        if(leafAncIdx != neighAncIdx){
            Polygon n = H->get_vertex(neighAncIdx);

            if(leafAncIdx != leaf)
                p = H->get_vertex(leafAncIdx);

            Polygon merged = mergePolygons_(p, n, commonEdge);
            if (merged.points.size() != 0)
            {
                H->merge_op(leafAncIdx, neighAncIdx, merged);
            } 
        }
    }
}

Polygon Observation::combinePolygonData_(const Polygon& p, const Polygon& n)
{
    bool same_direction = n.edges[0].first == p.edges[p.edges.size()-1].first;
    std::vector<int> neighbors;
    std::vector<EdgeT> edges;
    std::vector<double> edgeLengths;
    if(same_direction)
    {
        neighbors.insert(neighbors.end(), p.neighbors.begin(), p.neighbors.end()-1);
        neighbors.insert(neighbors.end(), n.neighbors.rbegin(), n.neighbors.rend()-1);

        edges.insert(edges.end(), p.edges.begin(), p.edges.end()-1);
        // edges.insert(edges.end(), n.edges.rbegin(), n.edges.rend()-1);
        for(int i = n.edges.size()-1; i != 0; --i)
        {
            const EdgeT& e = n.edges[i];
            edges.push_back(std::make_pair(e.second, e.first));
        }
        
        edgeLengths.insert(edgeLengths.end(), p.edgeLengths.begin(), p.edgeLengths.end()-1);
        edgeLengths.insert(edgeLengths.end(), n.edgeLengths.rbegin(), n.edgeLengths.rend()-1);
        
    } else 
    {
        neighbors.insert(neighbors.end(), p.neighbors.begin(), p.neighbors.end()-1);
        neighbors.insert(neighbors.end(), n.neighbors.begin()+1, n.neighbors.end());
        
        edges.insert(edges.end(), p.edges.begin(), p.edges.end()-1);
        edges.insert(edges.end(), n.edges.begin()+1, n.edges.end());
        
        edgeLengths.insert(edgeLengths.end(), p.edgeLengths.begin(), p.edgeLengths.end()-1);
        edgeLengths.insert(edgeLengths.end(), n.edgeLengths.begin()+1, n.edgeLengths.end());
    }

    PointVector points;
    for(auto e : edges){
        points.push_back(landmarks[e.first]);
    }

    Polygon merged(points, neighbors, edges, edgeLengths);
    return merged;
}

std::vector<EdgeT>::iterator Observation::findEdge_(Polygon& x, EdgeT commonEdge)
{
   std::vector<EdgeT>::iterator it = std::find_if(x.edges.begin(), x.edges.end(),
        [&commonEdge](EdgeT e){
            if(e.first == commonEdge.first && e.second == commonEdge.second) return True;
            if(e.second == commonEdge.first && e.first == commonEdge.second) return True;
            return False;
        });    
    return it;
}

Polygon Observation::mergePolygons_(Polygon& p, Polygon& n, EdgeT commonEdge){
    /*
    (0,1), (1,2), (2,0)
                        => (0,1), (1,2), (2,4), (4,0)
    (2,0), (0,4), (4,2)
    */

   // rotates puts idx as first element, for p we want the common edge to be the last elem
   // we want the element after commonEdge to be the first, i.e. commonEdge is the last element
   
   auto p_it = findEdge_(p, commonEdge);
   auto commonEdgeIdx = p_it - p.edges.begin();
   p.rotate((commonEdgeIdx+1)%p.edges.size());

   auto n_it = findEdge_(n, commonEdge);
   if(n_it == n.edges.end())
       return Polygon();

   n.rotate(n_it - n.edges.begin());
   Polygon merged = combinePolygonData_(p, n);
    
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
            double len = euclideanDistance2D(points[edge.first], points[edge.second]);
            edges.push_back(edge);
            edgeLengths.push_back(len);
        }
        Polygon poly(vertices, neighIds, edges, edgeLengths);
        polygons.push_back(poly);
    }
};

} // urquhart