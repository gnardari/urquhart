#pragma once
#include <descriptor.hpp>
#include <utils.hpp>

namespace urquhart {
    class Polygon {
        public:
            explicit Polygon(){
                points = {};
                neighbors = {};
                edges = {};
                edgeLengths = {};
                descriptor = {};
            }

            explicit Polygon(PointVector pp, std::vector<int> nn,
                        std::vector<EdgeT> ee, std::vector<double> el){
                points = pp;
                neighbors = nn;
                edges = ee;
                edgeLengths = el;
                descriptor = descriptor::compute(pp);
            }

            void rotate(const size_t idx){
                std::rotate(neighbors.begin(), neighbors.begin()+idx, neighbors.end());
                std::rotate(points.begin(), points.begin()+idx, points.end());
                std::rotate(edges.begin(), edges.begin()+idx, edges.end());
                std::rotate(edgeLengths.begin(), edgeLengths.begin()+idx, edgeLengths.end());
            }

            PointVector points;
            std::vector<int> neighbors;
            // edges (x,y), undirected and their respective lengths
            std::vector<EdgeT> edges;
            std::vector<double> edgeLengths;
            std::vector<double> descriptor;
    };
}