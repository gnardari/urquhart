#include <utils.hpp>

namespace urquhart {
    class Polygon {
        public:
            std::vector<std::vector<double>> points;
            std::vector<int> neighbors;
            // edges (x,y), undirected and their respective lengths
            std::vector<std::vector<int>> edges;
            std::vector<double> edges;
    };
}