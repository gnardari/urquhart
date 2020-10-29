#include <utils.hpp>
#include <polygon.hpp>

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
            explicit Observation();
            void delaunayTriangulation_(PointVector& points, std::vector<Polygon>& polygons);
        private:
            void processObservation_();
            void computeDescriptor_();
            void urquhartTesselation_();
            void mergeShapes_();
    };
}