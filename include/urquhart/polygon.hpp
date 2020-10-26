#include <utils.hpp>

namespace urquhart {
    class Polygon {
        public:
            explicit Polygon(std::vector<std::vector<double>> pp, std::vector<int> nn,
                        std::vector<std::pair<int,int>> ee, std::vector<double> el){
                points = pp;
                neighbors = nn;
                edges = ee;
                edgeLengths = el;
            }
            std::vector<std::vector<double>> points;
            std::vector<int> neighbors;
            // edges (x,y), undirected and their respective lengths
            std::vector<std::pair<int,int>> edges;
            std::vector<double> edgeLengths;
    };
}