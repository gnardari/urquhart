#include <utils.hpp>

namespace urquhart {
    class Polygon {
        public:
            explicit Polygon(PointVector pp, std::vector<int> nn,
                        std::vector<EdgeT> ee, std::vector<double> el){
                points = pp;
                neighbors = nn;
                edges = ee;
                edgeLengths = el;
            }
            PointVector points;
            std::vector<int> neighbors;
            // edges (x,y), undirected and their respective lengths
            std::vector<EdgeT> edges;
            std::vector<double> edgeLengths;
    };
}