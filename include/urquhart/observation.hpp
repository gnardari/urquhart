#pragma once

#include <utils.hpp>
#include <polygon.hpp>
#include <tree.hpp>
#include <map>

#include "libqhullcpp/RboxPoints.h"
#include "libqhullcpp/QhullError.h"
#include "libqhullcpp/QhullQh.h"
#include "libqhullcpp/QhullFacet.h"
#include "libqhullcpp/QhullFacetList.h"
#include "libqhullcpp/QhullFacetSet.h"
#include "libqhullcpp/QhullLinkedList.h"
#include "libqhullcpp/QhullPoint.h"
#include "libqhullcpp/QhullVertex.h"
#include "libqhullcpp/QhullVertexSet.h"
#include "libqhullcpp/Qhull.h"

namespace urquhart {
    class Observation {
        public:
            explicit Observation(PointVector& landmarks);
            // H stores all polygons in a tree structure, where each vertex represents a polygon.
            // The childs of a polygon are the triangles that were merged to compose it.
            // Triangles are the leaves of the tree.
            // The vertex 0 (root) is an empty polygon that is only used to connect all polygons
            // TODO: H should be private and have accessors
            Tree* H;
        private:
            PointVector landmarks;
            // Computes a Delaunay triangulation using QHull from a set of landmarks.
            void delaunayTriangulation_(PointVector& points, std::vector<Polygon>& polygons);
            // Uses the triangles of the delaunay triangulation to compute an urquhart tessellation
            void urquhartTesselation_();
            // Merges two polygons into a new one, this is used inside the urquhart tessellation computation
            // p will be merged with n, which shares the longest edge of p, indexed (relative to p) by commonEdgeIdx
            Polygon mergePolygons_(Polygon p, Polygon n, size_t commonEdgeIdx);
    };
}